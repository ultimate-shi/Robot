<!-- 作用：说明机械臂独立工作区；使用方法：在已安装 ROS 2 Jazzy 的终端按下列步骤构建。 -->
# 机械臂工作区

当前只提供 robot_arm 的 ament_python 包骨架，不依赖小车包，尚未实现硬件驱动或运动控制。
机械臂固件、协议与电路资料仍位于仓库根目录的 `机械臂/`。

从仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ROS2/robot_arm
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix robot_arm
```

`src/robot_arm/robot_arm/` 放 Python 模块；后续节点须在 setup.py 注册。
`launch/` 放启动文件，`config/` 放与启动文件同名的 YAML 参数，`test/` 放测试。
新增模型或运动规划时可按职责增设包，独立构建和验证。
