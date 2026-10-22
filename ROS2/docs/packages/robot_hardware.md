<!-- 作用：说明 robot_hardware 的串口协议、ros2_control 插件和实机接口。 -->
<!-- 使用方法：修改 DMC0、DMC1、SE2、编码器、舵机或超声波硬件接入前先阅读本文件。 -->
# robot_hardware 功能包

## 职责与边界

`robot_hardware` 实现 `robot_hardware/RobotSerialSystem` ros2_control SystemInterface，连接四腿、四个转向、四个驱动轮和双轴头部与 DMC0、DMC1、SE2 串口板，并发布硬件健康和 8 路超声波。

本包与 `robot_control` 共同组成控制和实机执行链。设备、协议、看门狗、限位和标定参数必须放在 YAML 或 launch 参数中。

## 入口和配置

- `hardware_bringup.launch.py`：加载模型、controller manager 和控制器。
- `config/hardware_bringup.yaml`：串口候选、超时、重试、几何参数、超声波滤波范围。
- `config/servo_calibration.yaml`：头部和转向舵机零点、方向、限位、PID 与死区。
- `config/wheel_calibration.yaml`：四轮关节到 DMC1 轴映射、反馈和命令方向、CPR、逐轮 D 模式速度曲线。换电机后更新该轮位并重测。
- 原生串口命令与独占要求见 `docs/hardware_serial_commands.md`。

## ROS 接口

- 发布 `/hardware/chassis_state`、`/diagnostics` 和 8 路 `/ultrasonic/*` `Range`。
- 订阅 `/hardware/command_heartbeat` 和 `/wheel_motion/active`；T 定距期间任一心跳超时会发 DMC1 停车命令。
- 服务 `/hardware/recover`、内部 `/hardware/execute_encoder_motion` 和 `/hardware/stop_encoder_motion`。
- 导出腿、转向、车轮和头部的 ros2_control 命令/状态接口。
- 未验证的 S 轮速模式会拒绝整组非零轮速并发送停车命令，同时通过 `/hardware/chassis_state.fault` 和 `/diagnostics` 提示；停车写入失败、通信及协议错误仍会锁定输出。锁定时立即发布故障状态，供网页在后续反馈中断时显示最后原因。

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

右后轮的 `encoder_sign`、`speed_command_sign` 和 D 模式驱动方向在轮位 YAML 中配置。
S7 正负指令均曾超速，当前默认与诊断配置均禁用该模式；修改方向字段后仍须架空
核对轮胎实际方向、目标速度、原始反馈、电流和停车，不能仅凭符号变化判定闭环通过。
