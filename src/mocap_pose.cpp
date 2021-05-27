/*
Qualisys Mocap system
*/

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

struct MocapPose::Impl
{
    std::string serverAddress = "192.168.43.98";
    unsigned short basePort = 22222;
    int sdkMajorVersion = 1;
    int sdkMinorVersion = 19;
    bool bigEndian = false;
    std::string bodyName = "drone";

    // int _update_freq;
    geodesy::UTMPoint home;
    double north_offset = 0.0;
    double publishing_timestep = 1.0;

    rclcpp::Time last_timestamp{};
    rclcpp::Time last_published_timestamp{};
    Eigen::Vector3f last_position{};
    Eigen::Vector3f last_velocity{};

    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr publisher;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  _control_sub;
    std::thread worker_thread;
    std::atomic_bool worker_thread_running{};
    void WorkerThread();

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
                                               const rclcpp::Time timestamp)
    {
        // Position
        geodesy::UTMPoint utm = geodesy::UTMPoint(home);
        const Eigen::AngleAxisf north_correction(north_offset, Eigen::Vector3f::UnitZ());
        const auto position_corrected = north_correction * position;

        utm.easting += position_corrected[0];
        utm.northing += position_corrected[1];
        utm.altitude += position_corrected[2];

        const geographic_msgs::msg::GeoPoint point = toMsg(utm);

        // Velocity
        Eigen::Vector3f velocity{};  // meters per second
        if (last_timestamp.nanoseconds() > 0)
        {
            velocity = (position - last_position) / (timestamp.seconds() - last_timestamp.seconds());
        }
        velocity = FixNans(velocity);

        // filter velocity, as it's too noisy
        const float avg_factor = 0.1F;
        velocity = velocity * avg_factor + last_velocity * (1.F - avg_factor);
        velocity = FixNans(velocity);

        last_timestamp = timestamp;
        last_position = position;
        last_velocity = velocity;

        const auto velocity_corrected = north_correction * velocity;
        const uint64_t timecode =
            std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now())
                .time_since_epoch()
                .count();

        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());

        // ToDo: not corrected for north!!
        const double heading_rad = -atan2(siny_cosp, cosy_cosp);

        px4_msgs::msg::SensorGps sensor_gps;
        sensor_gps.timestamp = timecode;
        sensor_gps.lat = (uint32_t)std::round(point.latitude * 10000000);
        sensor_gps.lon = (uint32_t)std::round(point.longitude * 10000000);
        sensor_gps.alt = (uint32_t)std::round(point.altitude * 1000);
        sensor_gps.s_variance_m_s = 0.2f;

        sensor_gps.fix_type = 3;
        sensor_gps.eph = 0.5f;
        sensor_gps.epv = 0.8f;
        sensor_gps.hdop = 0.0f;
        sensor_gps.vdop = 0.0f;

        sensor_gps.vel_m_s = velocity_corrected.norm();
        sensor_gps.vel_n_m_s = velocity_corrected[1];
        sensor_gps.vel_e_m_s = velocity_corrected[0];
        sensor_gps.vel_d_m_s = -velocity_corrected[2];
        sensor_gps.cog_rad = atan2(velocity_corrected[0], velocity_corrected[1]);
        sensor_gps.vel_ned_valid = 1;

        sensor_gps.time_utc_usec = timestamp.nanoseconds() / 1000ULL;  // system time (UTF) in millis
        sensor_gps.satellites_used = 16;
        sensor_gps.heading = heading_rad;
        sensor_gps.heading_offset = 0.0f;

        return sensor_gps;
    }
};

MocapPose::MocapPose() : Node("MocapPose"), impl_(new MocapPose::Impl())
{
    RCLCPP_INFO(this->get_logger(), "MocapPose (Motion Capture Positioning Service)");

    this->declare_parameter<double>("home_lat", 61.50341);
    this->declare_parameter<double>("home_lon", 23.77509);
    this->declare_parameter<double>("home_alt", 110.0);
    this->declare_parameter<double>("north_offset", 0.0);
    this->declare_parameter<double>("frequency", 10.0);
    this->declare_parameter<std::string>("server_address", "192.168.43.89");
    this->declare_parameter<std::string>("body_name", "sad04");

    double n_off = 0.0, frequency = 0.0;
    auto point = geographic_msgs::msg::GeoPoint();
    this->get_parameter("home_lat", point.latitude);
    this->get_parameter("home_lon", point.longitude);
    this->get_parameter("home_alt", point.altitude);
    this->get_parameter("north_offset", n_off);
    this->get_parameter("server_address", impl_->serverAddress);
    this->get_parameter("body_name", impl_->bodyName);
    this->get_parameter("frequency", frequency);

    if (frequency > 0.0)
    {
        impl_->publishing_timestep = 1.0 / frequency;
    }

    RCLCPP_INFO(this->get_logger(), "Looking for body with name: %s", impl_->bodyName);
    RCLCPP_INFO(this->get_logger(), "Frequency: %lf, frame_delay: %lf", frequency, impl_->publishing_timestep);

    impl_->north_offset = M_PI * (n_off / 180.0);

    RCLCPP_INFO(this->get_logger(),
                "Home coordinates: lat: %lf, lon: %lf, alt: %lf",
                point.latitude,
                point.longitude,
                point.altitude);

    impl_->home = geodesy::UTMPoint(point);

    RCLCPP_INFO(this->get_logger(),
                "east: %lf north: %lf alt: %lf  -  zone: %u%c",
                impl_->home.easting,
                impl_->home.northing,
                impl_->home.altitude,
                impl_->home.zone,
                impl_->home.band);

    //    impl_->_control_sub = this->create_subscription<std_msgs::msg::String>(
    //        "IndoorPos_ctrl", 10, std::bind(&IndoorPos::Control, this, _1));

    impl_->publisher = this->create_publisher<px4_msgs::msg::SensorGps>("SensorGps_PubSubTopic", 10);

    if (!impl_->worker_thread_running && !impl_->worker_thread.joinable())
    {
        impl_->worker_thread_running = true;
        impl_->worker_thread = std::thread(&MocapPose::WorkerThread, this);
    }
}

void MocapPose::WorkerThread()
{
    CRTProtocol rtProtocol;

    bool dataAvailable = false;
    bool streamFrames = false;
    unsigned short udpPort = 6734;

#if 0 == 1  // ToDo: Do we need automatic server discovery?
        if (impl_->serverAddress.empty())
        {
            // This code suppose to discover Qualisys Server automatically, it even works when running from sdk sample app,
            // However, it crushes when run under ROS node. Something connected to socket buffer overflow.

            // Try discovering Mocap Server automatically
            while (impl_->worker_thread_running && impl_->serverAddress.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Trying to discover Mocap Server ...");

                if (rtProtocol.DiscoverRTServer(4534, true))
                {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    const auto numberOfResponses = rtProtocol.GetNumberOfDiscoverResponses();
                    for (auto index = 0; index < numberOfResponses; index++)
                    {
                        unsigned int addr;
                        unsigned short port;
                        std::string message;
                        if (rtProtocol.GetDiscoverResponse(index, addr, port, message) && addr > 0)
                        {
                            std::stringstream ip_address_steam;
                            ip_address_steam << int(0xFFU & addr) << ".";
                            ip_address_steam << int(0xFFU & (addr >> 8U)) << ".";
                            ip_address_steam << int(0xFFU & (addr >> 16U)) << ".";
                            ip_address_steam << int(0xFFU & (addr >> 24U));
                            impl_->basePort = port;
                            std::getline(ip_address_steam, impl_->serverAddress);

                            RCLCPP_INFO(this->get_logger(),
                                        "Discovered Mocap Server: %s:%d - %s",
                                        impl_->serverAddress.c_str(),
                                        impl_->basePort,
                                        message.c_str());
                        }
                    }
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }
#endif

    while (impl_->worker_thread_running)
    {
        if (!rtProtocol.Connected())
        {
            if (!rtProtocol.Connect(impl_->serverAddress.c_str(),
                                    impl_->basePort,
                                    &udpPort,
                                    impl_->sdkMajorVersion,
                                    impl_->sdkMinorVersion,
                                    impl_->bigEndian))
            {
                RCLCPP_WARN(this->get_logger(), "rtProtocol.Connect: %s", rtProtocol.GetErrorString());
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }

        if (!dataAvailable)
        {
            if (!rtProtocol.Read6DOFSettings(dataAvailable))
            {
                RCLCPP_WARN(this->get_logger(), "rtProtocol.Read6DOFSettings: %s", rtProtocol.GetErrorString());
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }

        if (!streamFrames)
        {
            if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, nullptr, CRTProtocol::cComponent6d))
            {
                RCLCPP_WARN(this->get_logger(), "rtProtocol.StreamFrames: %s", rtProtocol.GetErrorString());
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            streamFrames = true;
        }

        CRTPacket::EPacketType packetType;
        bool very_first_message = false;
        if (rtProtocol.Receive(packetType, true) == CNetwork::ResponseType::success)
        {
            if (packetType == CRTPacket::PacketData)
            {
                float fX, fY, fZ;
                float rotation[9];

                CRTPacket* rtPacket = rtProtocol.GetRTPacket();

                for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                {
                    if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotation))
                    {
                        auto name = std::string(rtProtocol.Get6DOFBodyName(i));

                        if (name == impl_->bodyName)
                        {
                            // convert millimeters into meters
                            const Eigen::Vector3f Pos = Eigen::Vector3f(fX, fY, fZ) / 1000.F;
                            const Eigen::Map<Eigen::Matrix3f> R(rotation);
                            const Eigen::Quaternionf Q(R);

                            RCLCPP_INFO(this->get_logger(),
                                        "Pos: %9.3f %9.3f %9.3f  Quat: %6.3f %6.3f %6.3f %6.3f",
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
                            const bool rotation_is_valid =
                                !std::isnan(Q.w()) && !std::isnan(Q.x()) && !std::isnan(Q.y()) && !std::isnan(Q.z());
                            const bool data_is_valid = translation_is_valid && rotation_is_valid;

                            if ((data_is_valid) && (very_first_message || time_to_publish))
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
                    }
                }
            }
        }
    }
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
}

MocapPose::~MocapPose()
{
    impl_->worker_thread_running = false;
    if (impl_->worker_thread.joinable())
    {
        impl_->worker_thread.join();
    }
}

//
// void MocapPose::Control(const std_msgs::msg::String::SharedPtr msg) const
//{
//    RCLCPP_INFO(this->get_logger(), "Request command: '%s'", msg->data.c_str());
//    if (strcmp(msg->data.c_str(), "calibrate") == 0) {
//        if (impl_->_node_state == IndoorPosPrivate::IndoorNodeState::Idle) {
//            RCLCPP_INFO(this->get_logger(), "Request Calibration");
//            impl_->_node_state = IndoorPosPrivate::IndoorNodeState::ReqCalibrate;
//        } else {
//            RCLCPP_WARN(this->get_logger(), "Calibrate request not allowed");
//        }
//    }
//    else if (strcmp(msg->data.c_str(), "restart") == 0) {
//        if (impl_->_node_state == IndoorPosPrivate::IndoorNodeState::Idle) {
//            RCLCPP_INFO(this->get_logger(), "Request Restart");
//            impl_->_node_state = IndoorPosPrivate::IndoorNodeState::ReqRestart;
//        } else {
//            RCLCPP_WARN(this->get_logger(), "Restart request not allowed");
//        }
//    }
//    else {
//        RCLCPP_INFO(this->get_logger(), "Unknown command: '%s'", msg->data.c_str());
//    }
//}
