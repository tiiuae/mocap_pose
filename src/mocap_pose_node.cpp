/// @file mopcap_pose_node.cpp : ROS2 node running Mocap -> PX4 GPS convertor

#include "mocap_pose/mocap_pose.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MocapPose> node = std::make_shared<MocapPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
