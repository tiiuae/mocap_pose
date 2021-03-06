#ifndef FOG_SW_MOCAP_POSE_HPP
#define FOG_SW_MOCAP_POSE_HPP

#include <cstdint>
#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/string.hpp>

/// ROS2 Driver for Qualisys Motion Capture (Mocap) system
/// Provides "fake GPS" messages for selected Mocap object (set up with ROS2 parameters)
class MocapPose : public rclcpp::Node
{
public:
    MocapPose();
    ~MocapPose();
    void Stop();

private:

    rclcpp::Time QualisysToRosTimestamp(unsigned long long ts);
    void WorkerThread();
    struct Impl;
    std::unique_ptr<Impl> impl_;
    int64_t minTimestampDiff;
};

#endif // FOG_SW_MOCAP_POS_HPP
