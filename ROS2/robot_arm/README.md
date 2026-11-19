<!-- 作用：说明机械臂独立工作区；使用方法：在已安装 ROS 2 Jazzy 的终端按下列步骤构建。 -->
# 机械臂工作区

当前只提供 robot_arm 的 ament_python 包骨架，不依赖小车包，尚未实现硬件驱动或运动控制。
ESP32-C3关节固件位于[ESP32/arm_joint](../../ESP32/arm_joint/README.md)，支持master/slave、板端PID角度控制和直接PWM控制。
接口和标定说明见[硬件API](docs/hardware_api.md)；电路、CAD及设计历史位于仓库根目录的`机械臂/`。
ROS包仍为骨架，尚未实现ROS串口驱动；固件与ROS工作区分别构建。

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
