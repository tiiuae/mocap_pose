/// @file mopcap_pose.cpp : Implementation Qualisys Motion Capture convertor

#include "mocap_pose/mocap_pose.hpp"
#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include <Eigen/Geometry>
#include <RTPacket.h>
#include <RTProtocol.h>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <time.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <prometheus/counter.h>
#include <prometheus/exposer.h>
#include <prometheus/registry.h>
#include <chrono>
#include <sstream>
#include <thread>
#include <mutex>
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

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_lat;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_lon;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_alt;

    std::thread worker_thread;
    std::atomic_bool worker_thread_running{};
    std::mutex home_coord_mutex;

    std::shared_ptr<prometheus::Registry> metrics_registry = std::make_shared<prometheus::Registry>();
    prometheus::Counter* locationUpdateCount;

    Eigen::Vector3f FixNans(Eigen::Vector3f vec)
    {
        if (std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]))
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
                                               const rclcpp::Time& timestamp,
                                               const rclcpp::Time& utc_timestamp)
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

        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        const double heading_rad = -atan2(siny_cosp, cosy_cosp);

        sensor_gps.timestamp = timestamp.nanoseconds() / 1000ULL;
        sensor_gps.lat = (uint32_t)std::round(point.latitude * 10000000);
        sensor_gps.lon = (uint32_t)std::round(point.longitude * 10000000);
        sensor_gps.alt = (uint32_t)std::round(point.altitude * 1000);
        sensor_gps.s_variance_m_s = 0.2f;

        sensor_gps.fix_type = 3;
        sensor_gps.eph = 0.5f;
        sensor_gps.epv = 0.8f;
        sensor_gps.hdop = 0.0f;
        sensor_gps.vdop = 0.0f;

        sensor_gps.time_utc_usec = utc_timestamp.nanoseconds() / 1000ULL;  // system time (UTF) in millis
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

MocapPose::MocapPose() : Node("MocapPose"), impl_(new MocapPose::Impl()), minTimestampDiff(INT64_MAX)
{
    RCLCPP_INFO(this->get_logger(), "MocapPose (Motion Capture Positioning Service)");

    srand(time(NULL));

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
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

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

    auto callback = [this](const rclcpp::Parameter & p) {
        double value = NAN;
        if (p.get_type_name().compare("double") == 0) {
            value = p.as_double();
        }
        RCLCPP_INFO(this->get_logger(), "Param set cb: Parameter updated \"%s\": %f",
            p.get_name().c_str(),
            value
        );

        std::unique_lock<std::mutex> lock(this->impl_->home_coord_mutex);
        auto point = geodesy::toMsg(this->impl_->home);
        if (p.get_name().compare("home_lat") == 0) {
            point.latitude = p.as_double();
        } else if (p.get_name().compare("home_lon") == 0) {
            point.longitude = p.as_double();
        } else if (p.get_name().compare("home_alt") == 0) {
            point.altitude = p.as_double();
        }
        this->impl_->home = geodesy::UTMPoint(point);
    };

    impl_->locationUpdateCount = &(prometheus::BuildCounter()
        .Name("location_update_count")
        .Help("Number of location updates received from Mocap server")
        .Register(*impl_->metrics_registry).Add({}));

    impl_->param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    impl_->cb_handle_lat = impl_->param_subscriber->add_parameter_callback("home_lat", callback);
    impl_->cb_handle_lon = impl_->param_subscriber->add_parameter_callback("home_lon", callback);
    impl_->cb_handle_alt = impl_->param_subscriber->add_parameter_callback("home_alt", callback);

    impl_->publisher = create_publisher<px4_msgs::msg::SensorGps>(kGpsSensorTopic, 10);
    impl_->worker_thread_running = true;
    impl_->worker_thread = std::thread(&MocapPose::WorkerThread, this);
}

void MocapPose::Stop()
{
    RCLCPP_INFO(get_logger(), "Stopping mocap_pose threads.");
    impl_->worker_thread_running = false;
    if(impl_->worker_thread.joinable())
    {
        impl_->worker_thread.join();
    }
}

rclcpp::Time MocapPose::QualisysToRosTimestamp(unsigned long long ts) {

    uint64_t now = time_point_cast<microseconds>(steady_clock::now()).time_since_epoch().count();

    // Just assume minimum communication latency of 0, it should be neglible
    const uint64_t min_latency = 0;
    int64_t diff = now - ts;

    if (diff < minTimestampDiff) {
        minTimestampDiff = diff;
    }

    return rclcpp::Time((ts + minTimestampDiff - min_latency) * 1000);
}

void MocapPose::WorkerThread()
{
    std::shared_ptr<prometheus::Exposer> metrics_exposer;

    auto metrics_port = getenv("METRICS_PORT");
    if (metrics_port != nullptr) // start HTTP endpoint only if requested
    {
        metrics_exposer = std::make_shared<prometheus::Exposer>("0.0.0.0:" + std::string(metrics_port));

        metrics_exposer->RegisterCollectable(this->impl_->metrics_registry);
    }

    // randomize port between 6734 and maximum port.
    // this is because if we're behind (and we should assume so) a NAT gateway, the port must not
    // be ambiguous because there might be multiple mocap-pose clients behind the same gateway.
    //
    // using the same port across re-tries so QTM server in theory can consolidate or cleanup resources.
    unsigned short udpPort = 6734 + (rand() % (65535 - 6734));
    RCLCPP_INFO(get_logger(), "UDP port used: %u.", udpPort);

    while (impl_->worker_thread_running)
    {
        auto latest_succesfull_receive = clock_->now();
        CRTProtocol rtProtocol;

        bool dataAvailable = false;
        bool streamFrames = false;

        bool very_first_message = true;

        if (sendNATHolepunchPacket(udpPort) != 0)
        {
            RCLCPP_WARN(get_logger(), "sendNATHolepunchPacket() failed");
        }

        while (impl_->worker_thread_running)
        {
            auto now = clock_->now();
            if ((now > (latest_succesfull_receive + rclcpp::Duration(10,0))) && rtProtocol.Connected())
            {
                RCLCPP_WARN(get_logger(),
                            "Have not succesfully received for more than 10 secs - restarting receiver");
                // Stop this process and start a new container so we can get a new connection.
                // This scenario happens when we can get a conntection but for some reason data
                // is not arriving to the client.
                impl_->worker_thread_running = false;
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
                        auto name = std::string(rtProtocol.Get6DOFBodyName(i));

                        if (name == impl_->bodyName)
                        {
                            body_found = true;
                            if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotation))
                            {
                                const auto timestamp = clock_->now();
                                const bool time_to_publish = (impl_->last_published_timestamp.seconds() +
                                                              impl_->publishing_timestep) <= timestamp.seconds();
                                if (!time_to_publish && !very_first_message){
                                    continue;
                                }

                                // convert millimeters into meters
                                const Eigen::Vector3f Pos = Eigen::Vector3f(fX, fY, fZ) / 1000.F;
                                const Eigen::Map<Eigen::Matrix3f> R(rotation);
                                const Eigen::Quaternionf Q(R);
#if 0
                                RCLCPP_INFO(this->get_logger(),
                                            "Position: [%6.2f %6.2f %6.2f]\t Quaternion: [%6.3f %6.3f %6.3f %6.3f]",
                                            Pos[0],
                                            Pos[1],
                                            Pos[2],
                                            Q.w(),
                                            Q.x(),
                                            Q.y(),
                                            Q.z());
#endif
                                const auto gps_timestamp = QualisysToRosTimestamp(rtPacket->GetTimeStamp());
                                const auto gps_msg = impl_->PrepareGpsMessage(Pos, Q, timestamp, timestamp);

                                const bool translation_is_valid =
                                    !std::isnan(Pos[0]) && !std::isnan(Pos[1]) && !std::isnan(Pos[2]);
                                const bool rotation_is_valid = !std::isnan(Q.w()) && !std::isnan(Q.x()) &&
                                                               !std::isnan(Q.y()) && !std::isnan(Q.z());
                                const bool data_is_valid = translation_is_valid && rotation_is_valid;

                                latest_succesfull_receive = clock_->now();

                                if (data_is_valid)
                                {
                                    if (very_first_message || time_to_publish)
                                    {
                                        impl_->locationUpdateCount->Increment();

                                        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000,
                                                    "Publish GPS at time %lf, previous_time: %lf , gps ts %lf",
                                                    timestamp.seconds(),
                                                    impl_->last_published_timestamp.seconds(),
                                                    gps_timestamp.seconds());
                                        impl_->publisher->publish(gps_msg);
                                        impl_->last_published_timestamp = timestamp;
                                        very_first_message = false;
                                    }
                                }
                                else
                                {
                                    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
                                                "body found but location null - not publishing");
                                }
                            }
                        }
                    }

                    if (!body_found)
                    {
                        RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
                                    "Cannot find body named \"%s\" in Qualisys Data Stream",
                                    impl_->bodyName.c_str());
                    }
                }
            }
        }
        rtProtocol.StopCapture();
        rtProtocol.Disconnect();
    }
    exit(1);
}

MocapPose::~MocapPose()
{
    impl_->worker_thread_running = false;
    if (impl_->worker_thread.joinable())
    {
        impl_->worker_thread.join();
    }
}

/* connection to QTM server works the following way:

    - client opens TCP control connection, on which it negotiates start of UDP stream back to client
    - if any NAT gateways stand between QTM server and the client, from gateway perspective it looks
      like the server opens an unsolicited connection to client (server sends first packet, which is NOT OK)
    - therefore we send a dummy packet to server so from NAT gw perspective the server-initiated packets
      look like packets that initiated by the client (client sent first packet, which is OK).

    for extra details see https://tailscale.com/blog/how-nat-traversal-works/

    an alternative would have been to use full TCP connection (data plane uses the same TCP connection
    as the control plane), but network unreliability issues make mocap-pose stuck on TCP-only, as
    the client doesn't realize the connection is broken because after handshake + stream starting,
    the client doesn't send any data towards the server.

    TCP keepalives could have been a solution, but the latency of this data is important and I think
    UDP based stream (considering we only care about the newest data) recovers much faster than even
    tuned TCP keepalive parameters could.

    from tcpdump looks like this (the first line is our holepunch packet, from second on its QTM data stream):

    15:47:39.883615 IP worklaptop.17535 > 172.18.32.20.22225: UDP, length 0
    15:47:39.923224 IP 172.18.32.20.22225 > worklaptop.17535: UDP, length 712
*/
int MocapPose::sendNATHolepunchPacket(unsigned short receivingUDPPort) {
    // +3 is for "QTM RT-protocol over OSC" on top of base port, documented at:
    //   https://docs.qualisys.com/qtm-rt-protocol/#ip-port-numbers
    uint16_t qtmServerPort = impl_->basePort + 3; // usually 22225

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1) {
        return -1;
    }

    sockaddr_in qtmServerAddr;
    qtmServerAddr.sin_family = AF_INET;
    qtmServerAddr.sin_port = htons(qtmServerPort);
    qtmServerAddr.sin_addr.s_addr = inet_addr(impl_->serverAddress.c_str());

    sockaddr_in ourAddress;
    ourAddress.sin_family = AF_INET;
    ourAddress.sin_port = htons(receivingUDPPort);
    ourAddress.sin_addr.s_addr = inet_addr("0.0.0.0");

    // needed for outgoing packets to have predefined port (this is important)
    if (bind(sock, reinterpret_cast<sockaddr*>(&ourAddress), sizeof(ourAddress)) == -1) {
        return -1;
    }

    std::string emptyPayload = ""; // doesn't need to have any content
    int n_bytes = ::sendto(sock, emptyPayload.c_str(), emptyPayload.length(), 0, reinterpret_cast<sockaddr*>(&qtmServerAddr), sizeof(qtmServerAddr));
    ::close(sock);

    return 0;
}
