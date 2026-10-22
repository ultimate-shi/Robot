<!-- 作用与用法：任一轮电机更换、接线调整或速度异常时，按本文架空采集双向曲线、更新轮位配置并用 /cmd_vel_safe 验收。 -->
# 四轮方向标定、PWM 曲线与 ROS 速度验收

## 适用范围与控制链

本流程适用于 FL、FR、RL、RR 任一轮位。`/cmd_vel_safe` 经底盘运动学输出标准 ROS 关节速度，`ros2_control` 转交硬件接口；只有 `RobotSerialSystem` 负责关节到 DMC1 轴的映射、编码器方向、CPR、PWM 曲线、限幅和故障处理。标定写入 [`wheel_calibration.yaml`](../src/robot_hardware/config/wheel_calibration.yaml)，不在底盘运动学或控制器中补偿单个电机。

所有驱动测试前须架空并固定四轮，确认周围无人、急停可用；检查该轮接线与机构没有干涉。停止 `vctrl.service` 和已有的实机 ROS 底盘进程，并用 `fuser /dev/ttyACM*` 确认串口只由当前测试进程占用。直接串口测试绕过导航安全速度层，只能在上述条件下进行。下文路径按 Jazzy 容器内 `/workspace` 编写，板端原生命令格式见[硬件串口指令集](hardware_serial_commands.md)。

## 1. 确认轮位、方向与现场阈值

先从 `wheel_calibration.yaml` 查目标轮位的 `axis`、`encoder_sign`、`cpr`，并核对控制板 `$info`、`I<轴号>`、`J<轴号>`、`P<轴号>`。目前轮位和轴的对应关系是 FL→4、FR→5、RL→6、RR→7；更换接线后以实测和更新后的配置为准。`I` 返回的是控制板编码器设置，**不能从 `I` 的方向字段直接推出 ROS `encoder_sign`**。

用小幅、短时的正负 PWM 先做方向检查。记录 `S` 状态中的原始 `tps` 和编码器增量，二者符号应一致；用候选 `encoder_sign` 换算：

```text
ROS 关节速度 rad/s = 原始 tps × encoder_sign × 2π / 实测 CPR
```

再按 URDF 和车体运动学检查轮胎接地点的实际前进方向：轮胎驱动车身前进时应为 ROS 正速度。仅靠同一编码器的 `tps` 与计数，无法独立识别编码器是否装反。确定物理方向后，才固定 `encoder_sign`。正 PWM 对应的 ROS 方向写入 `drive_sign`（正向为 `+1`、反向为 `-1`）；负 PWM 对应相反方向，写入 `negative_drive_sign`。S 模式另有 `speed_command_sign`，只有实际验证 S 目标与反馈后才能设置并启用。

按该轴已知的机械、电机和驱动能力确定 `--current-stop-ma`。采样工具要求显式传入该值，允许范围为 500～3200 mA；当前统一测试门槛为 3200 mA。先小 PWM、短时确认，再逐步增加；达到阈值、读数矛盾、出现异常声响或振动时停车排查。自动停车不能代替人工急停。

## 2. 逐轴采集正反 PWM 曲线

[`wheel_pwm_curve.py`](../scripts/serial/wheel_pwm_curve.py) 只向指定的 DMC1 电机轴发送非零 D 指令，逐点保存原始 `S` 状态、后半段平均速度、峰值电流和编码器增量；异常及退出时向四轴发送停车。先把下面四个变量改为**当前轮位**的实测值和报告名：

```bash
MOTOR=4                 # DMC1 轴号：4、5、6 或 7
ENCODER_SIGN=1          # 按第 1 步确认的 ROS 编码器符号：1 或 -1
CURRENT_STOP_MA=3200    # 当前统一的软件停车阈值，单位 mA
REPORT_PREFIX=fl        # 当前轮位：fl、fr、rl 或 rr

# 正 PWM：先短时逐点检查，再采集 3 秒稳态。
python3 scripts/serial/wheel_pwm_curve.py --motor "$MOTOR" \
  --encoder-sign "$ENCODER_SIGN" --current-stop-ma "$CURRENT_STOP_MA" \
  --values=10,20,30,40,50,60,70,80,90,100 --duration-ms 1000 \
  --output "calibration_reports/wheel/${REPORT_PREFIX}_positive_short.json"
python3 scripts/serial/wheel_pwm_curve.py --motor "$MOTOR" \
  --encoder-sign "$ENCODER_SIGN" --current-stop-ma "$CURRENT_STOP_MA" \
  --values=10,20,30,40,50,60,70,80,90,100 --duration-ms 1000 \
  --hold-ms 3000 --renew-ms 400 \
  --output "calibration_reports/wheel/${REPORT_PREFIX}_positive_steady.json"

# 负 PWM：按同样顺序独立采集，勿将正向曲线直接反号套用。
python3 scripts/serial/wheel_pwm_curve.py --motor "$MOTOR" \
  --encoder-sign "$ENCODER_SIGN" --current-stop-ma "$CURRENT_STOP_MA" \
  --values=-10,-20,-30,-40,-50,-60,-70,-80,-90,-100 --duration-ms 1000 \
  --output "calibration_reports/wheel/${REPORT_PREFIX}_negative_short.json"
python3 scripts/serial/wheel_pwm_curve.py --motor "$MOTOR" \
  --encoder-sign "$ENCODER_SIGN" --current-stop-ma "$CURRENT_STOP_MA" \
  --values=-10,-20,-30,-40,-50,-60,-70,-80,-90,-100 --duration-ms 1000 \
  --hold-ms 3000 --renew-ms 400 \
  --output "calibration_reports/wheel/${REPORT_PREFIX}_negative_steady.json"
```

测试点和阈值应根据现场情况逐步增加，不必一次运行完整的 10～100% 列表。`--values` 接受逗号分隔的非零整数，范围为 −100～100；可从少量低幅点开始，下一次只测未覆盖的点。板端单条命令最长由 `J<轴号>` 决定，脚本单条最多 1000 ms；3 秒稳态通过周期续发实现。每点间停车、到阈值停车、错误时停车。JSON 报告保留全部原始样本和错误，属于本机临时标定数据，不提交仓库。

对每个有效点检查：无 `error`、`mean_tps` 与 `encoder_delta_ticks` 同号、电流未达到阈值、后半段速度较稳定。按 PWM 绝对值递增整理两条曲线；同一方向的速度绝对值应严格递增。若不单调、幅值异常或方向反复变化，先检查机构、驱动和编码器，不应直接写入配置。

## 3. 更新硬件配置并构建

在目标轮位下写入 `axis`、`encoder_sign`、`cpr`、`drive_sign` 和正 PWM 的 `pwm`/`speed_rad_s`。若已实测负 PWM，设置 `negative_pwm_enabled: true`，写入 `negative_drive_sign`、`negative_pwm`/`negative_speed_rad_s`。数组中 PWM 使用正的**绝对值**，速度也使用正的**绝对值**；方向单独由符号字段表达。`feedback_gain` 和 `max_pwm_correction` 只在 ROS 实测后按该轮位调整。`speed_mode_enabled` 只对已通过 S 模式实测的轴启用；未实测的低速或方向不能靠改标志假设可用。

硬件接口会拒绝重复轴号、无效符号和非单调曲线。对于禁用 S 模式的轮位，低于相应曲线首个速度的非零目标会停车并报告故障；高于末点时 PWM 钳在最大实测点，可能达不到目标。更换电机、编码器、减速比或接线后，重新核对对应轮位的这些字段和两条曲线。

`wheel_calibration_s7_diagnostic.yaml` 当前也禁用 S7。2026-10-22 独占串口短测发现
`S7 500 350` 的原始反馈为正，板端速度环按负向反馈计算，存在正反馈超速风险。
STM32 固件按后续要求保持原样；ROS 方向字段可在轮位标定中设置，但不能代替
四轮稳固架空、现场可立即断电条件下的 S7 正反向跟速、电流、停车及方向验收。
验收前不打开诊断配置的 `speed_mode_enabled`。D7 PWM 曲线维持已测值；接地驾驶和导航仍不使用 S7。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_hardware
source install/setup.bash
python3 -m pytest -q src/robot_hardware/test
```

## 4. 经 `/cmd_vel_safe` 验收速度和转向

确认采样脚本退出、串口空闲、没有其他 `/cmd_vel_safe` 发布者。分别启动硬件与底盘控制，再检查控制器和话题：

```bash
ros2 launch robot_hardware hardware_bringup.launch.py
ros2 launch robot_control chassis_control.launch.py publish_odometry:=false
ros2 control list_controllers
ros2 topic info /cmd_vel_safe -v
```

上述两个 launch 分别保持在独立终端。`wheel_controller`、`steering_controller`、`joint_state_broadcaster` 应为 `active`，测试前 `/cmd_vel_safe` 应无其他发布者。另开终端记录 `/wheel_controller/commands`、`/steering_controller/commands`、`/joint_states`、`/hardware/chassis_state`。按现场已验证的安全速度，分别发送短时直行、后退、转向，并在**每段之后**发零速、确认四轮反馈回零。以下速度只是架空示例，必须落在待测轮位已实测的曲线范围内：

```bash
timeout --signal=INT 5s ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.06}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'

timeout --signal=INT 5s ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: -0.06}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'

timeout --signal=INT 5s ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.06}, angular: {z: 0.02}}'
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

逐轮按同一时间窗对比 `/wheel_controller/commands` 的关节速度目标与 `/joint_states` 的反馈，计算 `|反馈−目标|/|目标|`，同时检查方向、峰值电流、`fault` 和最终停车。转向时还要比较 `/steering_controller/commands` 与转向关节位置。分析稳定段且目标非零的采样；转向到位保护可能临时把四轮目标置零，不能把这段算作电机跟速误差。记录测试速度、目标和反馈、误差、异常、电流与环境条件。架空合格后，还需在受控场地验证装车负载和导航闭环。

结束时先发零速、确认反馈回零，再关闭底盘控制和硬件 launch。每次换轮位或更换电机，均从轴映射和方向检查重新执行本流程。
