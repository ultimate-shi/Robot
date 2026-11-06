# 作用：构建双目拆分和 RGB-D 高频处理；用法：colcon build --packages-select robot_stereo_components。
cmake_minimum_required(VERSION 3.8)
project(robot_stereo_components)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rgbd_throttle_cpp src/rgbd_throttle_node.cpp)
ament_target_dependencies(rgbd_throttle_cpp rclcpp sensor_msgs cv_bridge rtabmap_msgs)
target_link_libraries(rgbd_throttle_cpp ${OpenCV_LIBRARIES})
target_compile_features(rgbd_throttle_cpp PUBLIC cxx_std_17)
install(TARGETS rgbd_throttle_cpp DESTINATION lib/${PROJECT_NAME})

add_executable(stereo_splitter_cpp src/stereo_splitter_node.cpp)
ament_target_dependencies(
  stereo_splitter_cpp
  camera_info_manager
  rclcpp
  sensor_msgs
  std_msgs
)
target_compile_features(stereo_splitter_cpp PUBLIC cxx_std_17)

install(TARGETS stereo_splitter_cpp DESTINATION lib/${PROJECT_NAME})

ament_package()
