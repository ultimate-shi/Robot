# 使用方法：由 colcon 构建 RobotSerialSystem ros2_control 硬件插件。
cmake_minimum_required(VERSION 3.16)
project(robot_hardware)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(robot_serial_system SHARED src/robot_serial_system.cpp)
target_compile_features(robot_serial_system PUBLIC cxx_std_17)
target_include_directories(robot_serial_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_serial_system
  diagnostic_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle
  robot_interfaces sensor_msgs std_msgs std_srvs yaml_cpp_vendor)
target_link_libraries(robot_serial_system yaml-cpp)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS robot_serial_system
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES robot_hardware_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_protocol test/test_protocol.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_serial_system)
ament_export_dependencies(hardware_interface pluginlib)
ament_package()
