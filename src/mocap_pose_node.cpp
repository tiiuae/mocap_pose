#include "mocap_pose/mocap_pose.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MocapPose> node = std::make_shared<MocapPose>();
    //std::thread w_thread(indoor_pos_worker, node);
    rclcpp::spin(node);
    //w_thread.join();
    rclcpp::shutdown();
    return 0;
}
