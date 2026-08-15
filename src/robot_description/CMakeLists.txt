# 使用方法：colcon 构建后安装 URDF、mesh 和 description launch 静态资源。
cmake_minimum_required(VERSION 3.8)
project(robot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch meshes urdf rviz DESTINATION share/${PROJECT_NAME})
ament_package()
