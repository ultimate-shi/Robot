# 使用方法：由工作区 colcon build 生成跨 Package 的消息、服务和 Action 接口。
cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticDetection.msg"
  "msg/SemanticDetectionArray.msg"
  "msg/TerrainState.msg"
  "msg/MissionState.msg"
  "srv/DetectObjects.srv"
  "srv/SetDetectionMode.srv"
  "srv/CaptureSample.srv"
  "srv/ConfirmMission.srv"
  "action/PlanMission.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
