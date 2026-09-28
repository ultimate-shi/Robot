<!-- 作用：说明 robot_hardware 的串口协议、ros2_control 插件和实机接口。 -->
<!-- 使用方法：修改 DMC0、DMC1、SE2、编码器、舵机或超声波硬件接入前先阅读本文件。 -->
# robot_hardware 功能包

## 职责与边界

`robot_hardware` 实现 `robot_hardware/RobotSerialSystem` ros2_control SystemInterface，连接四腿、四个转向、四个驱动轮和双轴头部与 DMC0、DMC1、SE2 串口板，并发布硬件健康和 8 路超声波。

本包与 `robot_control` 一起归 `control_hardware_owner` 管理。设备、协议、看门狗、限位和标定参数必须放在 YAML 或 launch 参数中。

## 入口和配置

- `hardware_bringup.launch.py`：加载模型、controller manager 和控制器。
- `config/hardware_bringup.yaml`：串口候选、超时、重试、几何参数、超声波滤波范围。
- `config/servo_calibration.yaml`：头部和转向舵机零点、方向、限位、PID 与死区。
- 原生串口命令与独占要求见 `docs/hardware_serial_commands.md`。

## ROS 接口

- 发布 `/hardware/chassis_state`、`/diagnostics` 和 8 路 `/ultrasonic/*` `Range`。
- 订阅 `/hardware/command_heartbeat` 和 `/wheel_motion/active`；T 定距期间任一心跳超时会发 DMC1 停车命令。
- 服务 `/hardware/recover`、内部 `/hardware/execute_encoder_motion` 和 `/hardware/stop_encoder_motion`。
- 导出腿、转向、车轮和头部的 ros2_control 命令/状态接口。

## 验证

```bash
colcon build --symlink-install --packages-select robot_interfaces robot_description robot_hardware
colcon test --packages-select robot_hardware
colcon test-result --verbose
source install/setup.bash
ros2 launch robot_hardware hardware_bringup.launch.py --show-args
git diff --check
```

当前 `hardware_bringup.launch.py` 会包含 `robot_control/controllers.launch.py`，但 `package.xml` 尚未声明 `robot_control` 运行依赖；后续依赖整理时应补充并复测。
