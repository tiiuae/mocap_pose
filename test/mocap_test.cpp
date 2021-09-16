#include "mocap_pose/mocap_pose.hpp"
#include "gtest/gtest.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/// Basic Test to check that Mocap does not crush
TEST(MocapTest, BasicTest)
{
    std::shared_ptr<MocapPose> mocap = std::make_shared<MocapPose>();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mocap->Stop();
}
