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
- `config/wheel_calibration.yaml`：四轮关节到 DMC1 轴映射、反馈和命令方向、CPR、逐轮 D 模式速度曲线。反向取正向占空比的负数，换电机后更新该轮位并重测。
- 右后轮实物前进使用 D7 正 PWM、原始正编码器计数；后退使用 D7 负 PWM。2026-10-22 全轮 D 命令切换后，已完成 ±0.06/0.08/0.1 m/s 六段架空短测；接地负载和长期跟速仍待验证。
- 原生串口命令与独占要求见 `docs/hardware_serial_commands.md`。

## ROS 接口

- 发布 `/hardware/chassis_state`、`/diagnostics` 和 8 路 `/ultrasonic/*` `Range`。
- SE2 返回 0 或超出 `sonar_min_valid_mm`～`sonar_max_valid_mm` 时跳过该通道本次 `Range` 发布；接收者按最近有效样本的超时判定失联，不会收到伪造的远距读数。
- 订阅 `/hardware/command_heartbeat` 和 `/wheel_motion/active`；T 定距期间任一心跳超时会发 DMC1 停车命令。
- 服务 `/hardware/recover`、内部 `/hardware/execute_encoder_motion` 和 `/hardware/stop_encoder_motion`。
- 导出腿、转向、车轮和头部的 ros2_control 命令/状态接口。
- 四轮速度及停车均用 D4～D7，零速占空比为 0；反馈由 D 状态查询的编码器差分计算。停车写入失败、通信及协议错误会锁定输出，并发布故障状态。
- SE2 六路舵机由独立刷新线程按 `servo_keepalive_period_ms` 发送最新 PWM，避免 ros2_control 的串口读写周期超过固件 100 ms 回中期限。上游 PWM 超过 `servo_command_timeout_ms` 或硬件命令心跳过期时仅发送中性值；刷新失败锁定硬件。停用时先结束刷新，再发送中性 PWM。
- 转向舵机在误差不大于 2.5° 时进入中性输出，到位后在误差超过 4° 时重新调整；两个阈值由 `servo_calibration.yaml` 的 `brake_deadzone`、`resume_deadzone` 加上固定 0.5° 决定。重新调整阈值必须小于底盘轮速放行阈值（当前约 4.58°），防止舵机保持中性而轮子被门控停车。目标改变时立即解除到位状态。头部两轴保留原有 2.5° 停止阈值。

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

四轮 `encoder_sign`、`drive_sign` 与 D 曲线在轮位 YAML 中配置。反向使用相同幅值的负占空比；更换电机或接线后，重新架空核对四轮正反实物方向、目标速度、编码器反馈、电流和停车。
