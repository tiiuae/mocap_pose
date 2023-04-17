cmake_minimum_required(VERSION 3.5)
project(mocap_pose)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geodesy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(prometheus-cpp CONFIG REQUIRED)

add_subdirectory(qualisys_cpp_sdk)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(mocap_pose STATIC src/mocap_pose.cpp)
target_link_libraries(mocap_pose qualisys_cpp_sdk prometheus-cpp::pull)
ament_target_dependencies(mocap_pose rclcpp geodesy px4_msgs Eigen3)

add_executable(mocap_pose_node src/mocap_pose_node.cpp)
ament_target_dependencies(mocap_pose_node rclcpp geodesy px4_msgs Eigen3)
target_link_libraries(mocap_pose_node qualisys_cpp_sdk mocap_pose)

target_include_directories(mocap_pose PUBLIC include)
target_include_directories(mocap_pose_node PUBLIC include)

install(TARGETS
        mocap_pose
        mocap_pose_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )


if(BUILD_TESTING)
  enable_testing()

  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  add_executable(mocap_test test/mocap_test.cpp)
  target_link_libraries(mocap_test gtest pthread mocap_pose)
  add_test(NAME mocap_test COMMAND mocap_test)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
