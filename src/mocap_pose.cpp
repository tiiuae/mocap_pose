/// @file mopcap_pose.cpp : Implementation Qualisys Motion Capture convertor

#include "mocap_pose/mocap_pose.hpp"
#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include <Eigen/Geometry>
#include <RTPacket.h>
#include <RTProtocol.h>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <chrono>
#include <sstream>
#include <thread>
using namespace std::chrono;

struct MocapPose::Impl
{
    std::string serverAddress = "";
    unsigned short basePort = 22222;
    int sdkMajorVersion = 1;
    int sdkMinorVersion = 19;
    bool bigEndian = false;
    std::string bodyName = "drone";
    int velocity_type = 1;

    // "Home" point on Earth, from where we "linearize" our coordinates
    geodesy::UTMPoint home;
    // step in [sec] between publishing (setup with frequency/FPS parameter)
    double publishing_timestep = 0.0;

    rclcpp::Time last_timestamp{};
    rclcpp::Time last_published_timestamp{};
    Eigen::Vector3f last_position{};
    Eigen::Vector3f last_velocity{};

    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr publisher;
    std::thread worker_thread;
    std::atomic_bool worker_thread_running{};

    Eigen::Vector3f FixNans(Eigen::Vector3f vec)
    {
        if (std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[0]))
        {
            return Eigen::Vector3f(0, 0, 0);
        }
        else
        {
            return vec;
        }
    }

    px4_msgs::msg::SensorGps PrepareGpsMessage(const Eigen::Vector3f& position,
                                               const Eigen::Quaternionf& q,
                                               const rclcpp::Time& timestamp)
    {
        px4_msgs::msg::SensorGps sensor_gps{};

        // Position
        geodesy::UTMPoint utm = geodesy::UTMPoint(home);

        utm.easting += position[0];
        utm.northing += position[1];
        utm.altitude += position[2];

        const geographic_msgs::msg::GeoPoint point = toMsg(utm);

        Eigen::Vector3f velocity{};  // meters per second

        // Velocity
        if (last_timestamp.nanoseconds() > 0)
        {
            velocity = (position - last_position) / (timestamp.seconds() - last_timestamp.seconds());
        }
        velocity = FixNans(velocity);

        if (velocity_type > 1)
        {
            // filter velocity
            const float avg_factor = 0.1F;
            velocity = velocity * avg_factor + last_velocity * (1.F - avg_factor);
            velocity = FixNans(velocity);
        }

        last_timestamp = timestamp;
        last_position = position;
        last_velocity = velocity;

        // PX4 uses "steady_clock" timer, while Qualisys Mocap data comes with "system_clock" timestamps
        // Here we convert one to another, such that PX4 could use our message
        const auto system_now = time_point_cast<microseconds>(system_clock::now()).time_since_epoch().count();
        const auto steady_now = time_point_cast<microseconds>(steady_clock::now()).time_since_epoch().count();
        steady_clock::rep steady_stamp = timestamp.nanoseconds() / 1000ULL - system_now + steady_now;

        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        const double heading_rad = -atan2(siny_cosp, cosy_cosp);

        sensor_gps.timestamp = steady_stamp;
        sensor_gps.lat = (uint32_t)std::round(point.latitude * 10000000);
        sensor_gps.lon = (uint32_t)std::round(point.longitude * 10000000);
        sensor_gps.alt = (uint32_t)std::round(point.altitude * 1000);
        sensor_gps.s_variance_m_s = 0.2f;

        sensor_gps.fix_type = 3;
        sensor_gps.eph = 0.5f;
        sensor_gps.epv = 0.8f;
        sensor_gps.hdop = 0.0f;
        sensor_gps.vdop = 0.0f;

        sensor_gps.time_utc_usec = timestamp.nanoseconds() / 1000ULL;  // system time (UTF) in millis
        sensor_gps.satellites_used = 16;
        sensor_gps.heading = heading_rad;
        sensor_gps.heading_offset = 0.0f;

        if (velocity_type > 0)
        {
            sensor_gps.vel_m_s = velocity.norm();
            sensor_gps.vel_n_m_s = velocity[1];
            sensor_gps.vel_e_m_s = velocity[0];
            sensor_gps.vel_d_m_s = -velocity[2];
            sensor_gps.cog_rad = atan2(velocity[0], velocity[1]);
            sensor_gps.vel_ned_valid = true;
        }
        else  // do not send velocity if velocity_type parameter == 0
        {
            sensor_gps.vel_m_s = NAN;
            sensor_gps.vel_n_m_s = NAN;
            sensor_gps.vel_e_m_s = NAN;
            sensor_gps.vel_d_m_s = NAN;
            sensor_gps.cog_rad = NAN;
            sensor_gps.vel_ned_valid = false;
        }

        return sensor_gps;
    }
};

MocapPose::MocapPose() : Node("MocapPose"), impl_(new MocapPose::Impl())
{
    RCLCPP_INFO(this->get_logger(), "MocapPose (Motion Capture Positioning Service)");

    declare_parameter<double>("home_lat", 61.50341);
    declare_parameter<double>("home_lon", 23.77509);
    declare_parameter<double>("home_alt", 110.0);
    declare_parameter<double>("frequency", 10.0);
    declare_parameter<int>("velocity_type", 1);
    declare_parameter<std::string>("server_address", "172.18.32.20");
    declare_parameter<std::string>("body_name", "sad");

    double frequency = 0.0;
    auto point = geographic_msgs::msg::GeoPoint();
    get_parameter("home_lat", point.latitude);
    get_parameter("home_lon", point.longitude);
    get_parameter("home_alt", point.altitude);
    get_parameter("server_address", impl_->serverAddress);
    get_parameter("body_name", impl_->bodyName);
    get_parameter("frequency", frequency);
    get_parameter("velocity_type", impl_->velocity_type);
    impl_->home = geodesy::UTMPoint(point);
    impl_->publishing_timestep = (frequency > 0.0) ? 1.0 / frequency : 0.0;

    RCLCPP_INFO(get_logger(), "Looking for body with name: %s", impl_->bodyName.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Frequency: %lf, delay: %lf, Velocity type: %d",
                frequency,
                impl_->publishing_timestep,
                impl_->velocity_type);
    RCLCPP_INFO(get_logger(),
                "Initial Home coordinates: lat: %lf, lon: %lf, alt: %lf",
                point.latitude,
                point.longitude,
                point.altitude);

    RCLCPP_INFO(get_logger(),
                "east: %lf north: %lf alt: %lf  -  zone: %u%c",
                impl_->home.easting,
                impl_->home.northing,
                impl_->home.altitude,
                impl_->home.zone,
                impl_->home.band);

    impl_->publisher = create_publisher<px4_msgs::msg::SensorGps>("SensorGps_PubSubTopic", 10);
    impl_->worker_thread_running = true;
    impl_->worker_thread = std::thread(&MocapPose::WorkerThread, this);
}

void MocapPose::Stop()
{
    impl_->worker_thread_running = false;
    if(impl_->worker_thread.joinable())
    {
        impl_->worker_thread.join();
    }
}

void MocapPose::WorkerThread()
{
    while (impl_->worker_thread_running)
    {
        auto latest_succesfull_receive = std::chrono::system_clock::now();
        CRTProtocol rtProtocol;

        bool dataAvailable = false;
        bool streamFrames = false;
        unsigned short udpPort = 6734;
        bool very_first_message = true;
        while (impl_->worker_thread_running)
        {
            auto now = std::chrono::system_clock::now();
            if (now > latest_succesfull_receive + std::chrono::seconds(10))
            {
                RCLCPP_WARN(get_logger(),
                            "Have not succesfully received for more than 10 secs - restarting receiver");
                break;
            }

            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(impl_->serverAddress.c_str(),
                                        impl_->basePort,
                                        &udpPort,
                                        impl_->sdkMajorVersion,
                                        impl_->sdkMinorVersion,
                                        impl_->bigEndian))
                {
                    RCLCPP_WARN(get_logger(),
                                "Trying to connect to %s:%d : %s",
                                impl_->serverAddress.c_str(),
                                impl_->basePort,
                                rtProtocol.GetErrorString());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                else
                {
                    RCLCPP_INFO(
                        get_logger(), "Succesfully connected to %s:%d", impl_->serverAddress.c_str(), impl_->basePort);
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    RCLCPP_WARN(get_logger(), "6DoF data is not available: %s", rtProtocol.GetErrorString());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "6DoF data is available (server configured well)");
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(
                        CRTProtocol::RateAllFrames, 0, udpPort, nullptr, CRTProtocol::cComponent6d))
                {
                    RCLCPP_WARN(get_logger(), "Cannot start data streaming: %s", rtProtocol.GetErrorString());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Data streaming succesfully started");
                }
                streamFrames = true;
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.Receive(packetType, true) == CNetwork::ResponseType::success)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotation[9];

                    CRTPacket* rtPacket = rtProtocol.GetRTPacket();
                    bool body_found = false;

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotation))
                        {
                            auto name = std::string(rtProtocol.Get6DOFBodyName(i));

                            if (name == impl_->bodyName)
                            {
                                body_found = true;

                                // convert millimeters into meters
                                const Eigen::Vector3f Pos = Eigen::Vector3f(fX, fY, fZ) / 1000.F;
                                const Eigen::Map<Eigen::Matrix3f> R(rotation);
                                const Eigen::Quaternionf Q(R);

                                RCLCPP_INFO(this->get_logger(),
                                            "Position: [%6.2f %6.2f %6.2f]\t Quaternion: [%6.3f %6.3f %6.3f %6.3f]",
                                            Pos[0],
                                            Pos[1],
                                            Pos[2],
                                            Q.w(),
                                            Q.x(),
                                            Q.y(),
                                            Q.z());

                                const auto timestamp = rclcpp::Clock().now();
                                const auto gps_msg = impl_->PrepareGpsMessage(Pos, Q, timestamp);
                                const bool time_to_publish = (impl_->last_published_timestamp.seconds() +
                                                              impl_->publishing_timestep) <= timestamp.seconds();

                                const bool translation_is_valid =
                                    !std::isnan(Pos[0]) && !std::isnan(Pos[1]) && !std::isnan(Pos[2]);
                                const bool rotation_is_valid = !std::isnan(Q.w()) && !std::isnan(Q.x()) &&
                                                               !std::isnan(Q.y()) && !std::isnan(Q.z());
                                const bool data_is_valid = translation_is_valid && rotation_is_valid;

                                latest_succesfull_receive = std::chrono::system_clock::now();

                                if (data_is_valid)
                                {
                                    if (very_first_message || time_to_publish)
                                    {
                                        RCLCPP_INFO(this->get_logger(),
                                                    "Publish GPS at time %lf, previous_time: %lf",
                                                    timestamp.seconds(),
                                                    impl_->last_published_timestamp.seconds());
                                        impl_->publisher->publish(gps_msg);
                                        impl_->last_published_timestamp = timestamp;
                                        very_first_message = false;
                                    }
                                }
                                else
                                {
                                    RCLCPP_WARN(get_logger(), "Data is full of NaNs and thus NOT PUBLISHED!");
                                }
                            }
                        }
                    }

                    if (!body_found)
                    {
                        RCLCPP_WARN(get_logger(),
                                    "Cannot find body named \"%s\" in Qualisys Data Stream",
                                    impl_->bodyName.c_str());
                    }
                }
            }
        }
        rtProtocol.StopCapture();
        rtProtocol.Disconnect();
    }
}

MocapPose::~MocapPose()
{
    impl_->worker_thread_running = false;
    if (impl_->worker_thread.joinable())
    {
        impl_->worker_thread.join();
    }
}
