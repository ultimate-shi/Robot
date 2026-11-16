<!-- 作用：介绍独立 ROS 工作区入口；使用方法：进入对应子目录构建，勿在本目录统一构建。 -->
# ROS 工作区

- [robot_car](robot_car/README.md)：小车工作区，包含原有全部 ROS 代码、脚本及容器配置。
- [robot_arm](robot_arm/README.md)：机械臂独立工作区。

分别进入子目录构建并在独立终端加载各自的 install/setup.bash。不要在 ROS2/ 执行统一构建。
原有容器须在停止后从 robot_car/scripts/docker/run_jazzy_container.sh 重新创建，以更新宿主机挂载；容器内仍为 /workspace。
