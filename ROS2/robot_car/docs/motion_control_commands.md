<!-- 作用：汇总 ROS 2 底盘运动控制与排障命令；使用方法：核对现场安全条件后按相应章节执行。 -->
# ROS 2 运动控制指令速查

命令默认在 `robot-jazzy` 容器内执行：

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

底层 DMC0、DMC1、SE2 串口命令见
[`hardware_serial_commands.md`](hardware_serial_commands.md)。

实机控制器会把每条实际写入三块控制板的原始命令发布到 `/hardware/serial_tx`。
日常 MCAP 默认录制该话题，可在 9100 端口的 `/log/` 页面按话题筛选，
或用 `ros2 topic echo /hardware/serial_tx` 实时查看；`result` 区分完整回包、
写入失败、部分写入和超时。`UNIDENTIFIED` 表示身份查询时尚未识别板卡，
此时用 `device` 区分设备。`response_ok` 仅说明收到协议行，不能代替上层对回包字段、
错误码和实物动作的核对。直接运行独占串口脚本的指令不经过该 ROS 硬件插件。

架空复测使用 `test/chassis_mode_hardware_probe.py`，默认参数位于
`config/chassis_mode_hardware_probe.yaml`。探针除轮速、转角和电流外，还核对安全层
是否大幅改变输入命令，并在头部反馈相对每段起点偏移超过配置门槛时立即停测。
若超声波因近处物体缩减弧线或斜行速度，该段不算三模式验收通过。

## 架空验证结果

2026-10-20 修复 SE2 后再次架空复测：先确认三板身份、四轮零速与日志录制，
再经安全层执行 `four_ws` 8 段、`crab` 4 段、`ackermann` 4 段，每段 10 秒，
16 段均通过。有效轮动 6.53～7.59 秒，最大单帧转角误差 0.072 rad，
轮电流峰值 2643 mA，测试期间头部最大反馈偏移约 0.14°，最后四轮零速且无锁定故障。
左侧超声波起初测得约 0.16 m，测试开始时加载的安全层参数会削减左转命令，故使用仅存在于 `/tmp` 的
架空试验参数将侧向停车/警戒距离暂设为 0.08/0.13 m；安全过滤器仍全程运行，
测试进程退出后该参数不再生效。探针同时核对安全层输出与输入，避免把缩减后的路线
误判为三模式正常。MCAP 保存本次 36441 条实际串口发送记录，可在 9100 日志页按
`/hardware/serial_tx` 查询；原始逐帧报告保存在忽略目录
`calibration_reports/chassis_mode_full_20261020.json`。
测试期间工作区的 `obstacle_avoidance.yaml` 另有改动，当前默认前方停车/警戒为
0.10/0.20 m、侧向为 0.10/0.15 m；本次实测进程未加载该改动后的文件。
本次只验收架空轮速和转向反馈，不能证明接地位移或 Nav2 路线精度。

2026-10-10 反馈修正后的架空复测：测试脚本电流停止门槛为 3100mA，三模式共 16 段，
每段 10 秒。`four_ws` 前后、左右弧线、连续 S 弯和倒退弧线 8 段，`crab` 四方向斜行
4 段，`ackermann` 前后左右转弯 4 段全部达到至少 5 秒有效轮动；实际范围
6.27～7.58 秒，最高轮电流 2664mA，最大单帧舵角误差 0.116rad，最后无硬件故障。
控制器先等待连续两帧转向反馈进入误差范围，再依据实时误差降速或停车；安全层否决
复合速度任一分量时整条命令停车。原始汇总见忽略的
`calibration_reports/chassis_mode_final_20261010.json`。架空测试不能证明真实车体位移
达到 1 米，也不能验收装车负载、Nav2 定位和闭环路线精度。

2026-10-10 修改前基线：在 Jazzy 容器内启动现有机器人模型、实机控制器、
`chassis_control.launch.py` 和 `safety.launch.py`，用
`robot_control/test/chassis_mode_hardware_probe.py` 经 `/cmd_vel_nav_smoothed` 输入速度。
`four_ws` 前后、左右弧线、连续 S 弯和倒退弧线，`crab` 四个前后斜行方向，以及
`ackermann` 前后左右转弯各运行 8 秒。单独的 `four_ws` 前进一次达到 5.27 秒
有效轮动，四轮目标 +1.455rad/s、反馈中位数 +1.441/+1.356/+1.458/+1.349rad/s；
随后完整 16 段均未累计达到 5 秒有效轮动；
转向反馈总体接近目标，但转向到位门控反复输出四轮零速。连续 S 弯复测的非零轮速命令
占比为 40.5%/33.3%，连续倒退左右转弯为 45.2%/28.6%。原始汇总在忽略的
`calibration_reports/chassis_mode_{,route_}probe_20261010.json`。最后确认四轮命令全零，
控制板在线、无锁定故障。因此三模式的运动学指令已发出并有方向正确的反馈，但当前不能
验收为稳定运行，也未完成真实 1 米位移验证；架空反馈只对应轮子等效位移。

| 命令组 | 结果 |
|---|---|
| 硬件启动、八个控制器、`/hardware/chassis_state`、`/joint_states` | 通过；三块板连接、输出使能，启动独立轮速看门狗后心跳正常 |
| 零距离 5° 转向与回中 Action | 通过；四个转向角反馈到位，轮速保持零 |
| 2cm 四轮定距 Action | 未通过；右后轮仅约 0.35cm，其余轮约 2.6~2.7cm，Action 中止并停车 |
| `/hardware/stop_encoder_motion`、无故障时 `/hardware/recover` | 服务返回成功；停车后轮速反馈为零 |
| Twist 前进、后退、弧线、原地转、斜行；0.20m 定距 | 右后轮异常后停止测试，尚未实机验证 |
| Nav2 目标、车身升降/平衡、头部运动 | 架空底盘测试不能验证导航路径和车身稳定性，本次未执行 |

右后轮故障排除前不得执行整车带速命令。重新测试时先复查机械阻力、电机与编码器线束、驱动通道，
再从四轮独立低速、短距离测试开始。

## 四轮独立速度曲线工具

2026-10-10 新一轮架空测试已对 D4～D7 的正 PWM 10%～100% 各取 10 点，
原始采样在 `calibration_reports/wheel/{fl,fr,rl,rr}_d_manual.json`。
此轮 D100% 实测速度幅值分别为 1.92590、1.86307、1.94010、1.99849rad/s；
峰值电流和时间序列以 JSON 为准。以下 2026-09-27/28 段落仅记录当时测试状态。
当前 ROS 硬件接口按 `robot_hardware/config/wheel_calibration.yaml` 读取各轮正 PWM 曲线，
所有轮位和方向均发送逐轴 D 命令；反向取同幅值的负占空比，低于首点时从零点插值。
2026-10-22 已完成全轮 D 模式 ±0.06/0.08/0.1 m/s 六段架空短测，接地负载仍待验收。
更换轮电机后应重新测相应轮位，并核对 `/joint_states` 的位置和速度正方向。

2026-10-10 `/cmd_vel_safe` 架空测试：直行 0.04m/s 时四轮 ROS 目标均为
+0.727rad/s；FL/FR/RL 反馈中位数约 +0.731/+0.707/+0.707rad/s，
RR 在 `speed_command_sign: 1` 时为 -1.922rad/s，改为 -1 后为 +1.978rad/s。
因此右后 S7 两种符号都不能跟随目标；这是历史 S 模式测试，当前 ROS 插件不发送 S 命令。
0.05m/s、0.02rad/s 小角度转弯的四轮目标约
+0.860/+0.962/+0.860/+0.962rad/s，转向目标约
+0.066/+0.059/-0.066/-0.059rad；转向反馈摆动使轮速命令反复置零，
该段不能作为稳态轮速验收。原始 ROS 目标、反馈和电流时序保存在
`calibration_reports/wheel/cmd_vel_safe_probe*_20261010.json`（运行产物，不提交）。

2026-09-27 追加直接串口复测：`D4/D5/D6` 在 15% PWM、600ms 的已测试范围内分别达到
约 0.716/0.748/0.735rad/s 平稳轮速；这是已测安全上限，尚非电机物理最高速。
`D7 10%` 几乎不转且峰值 2167mA，未取得安全稳态速度。多轴 `S` 对右前、左后无效，
单轴 `S4/S5/S6` 可驱动前三轮；右后 `S7 400tick/s` 短脉冲触发 2200mA 软件停车。
四轮 2×2 速度与电流图为
`calibration_reports/2026-09-27/dmc1_s_speed_current_4w.png`，开环 PWM 图为同目录
`dmc1_s_speed_current_4w_d.png`，原始合并数据为 `dmc1_final_report.json`。
本机 `I4..I7` 返回的编码器方向为 `[-1,+1,-1,+1]`；曲线展示 ROS 正轮速，
由板端 tps 乘 `-encoder_direction`，因此右侧两轮在图中反号。右后面板留空表示未取得
安全的 S 稳态样本，并非测定它的输出恒为零。
报告无完整四轮安全曲线，逐轮等速补偿继续关闭。

2026-09-28 扩展 D 测试：左前 D30%/600ms 稳态约 1.335rad/s（2465tick/s），
右前 D20%/600ms 约 1.041rad/s（1923tick/s），左后 D20%/600ms 约
0.964rad/s（1780tick/s）。相应已测峰值约 2610/2567/1972mA。这些是当前
**已测范围内**的上限，不是 D100% 的最高转速。右后继续测试 D12%～30%：
D20%/600ms 曾达到约 0.411rad/s，但波动比约 1.696，不能视为稳态；
D30%/300ms 测得 2875mA，达到本轮 2800mA 软件停机门槛并立即停车。
给定 3000mA 最长 2s 的边界，无法据此保证 D100% 的电流安全，所以没有执行 D100%。

以前三轮 D 已测稳态速度为上限，逐轴 `S4/S5/S6` 的 600ms 正反向点扩展至约
±1.4/±1.0/±0.9rad/s（左后负向到约 -0.77rad/s，稳态电流已超过额定
1000mA）。右后的 S 曲线仍缺有效点。更新后的四轮 2×2 S 速度、电流图为
`calibration_reports/2026-09-28/dmc1_s_speed_current_4w.png`，D PWM 图为同目录的
`dmc1_s_speed_current_4w_d.png`，134 个采样点与各点原始时序见
`dmc1_final_report.json`。两图中的峰值电流为该指令已观测到的最大值，采样间隙内
可能存在更高的瞬态。S 图的横轴是板端 S 目标速度，纵轴是实测 tps 乘
`-encoder_direction` 后的 ROS 轮速；板端右轮的 S 正目标会返回负 tps。

四轮原始采集程序已退役；上述实测数据仅保留作硬件诊断记录。
当时改为逐轴 `S4..S7` 后，硬件串口单周期约 115ms，控制器管理器调至 7Hz；
零速架空启动没有再次出现 10Hz 下持续的周期超时。ROS 轮速复验被“转向关节未回中”
保护拦截，未发送轮速；只读 `/joint_states` 当时四个转向角约为
`[-0.119,-0.034,-0.077,-0.179]rad`，超过标定工具 0.06rad 限值的有三轮。
需排查转向反馈和零位后再验收完整运动链。

### 简化单电机记录

`scripts/serial/single_motor_curve.py` 每次只测试指定的一个 DMC1 轮电机。显式填写 D 占空比
或 S 目标 tick/s，工具按填写顺序逐点发送、记录每次 `S ` 查询的四轮 tps、编码器和电流，
每点结束逐轴停车；运行结束自动生成同名 JSON 和 PNG。它不根据电流、速度、稳定性或前一点
结果筛选下一点，也不会自动增加占空比。命令时长仍须满足板端 `J<id>` 返回的上限。

停止 `vctrl.service`、ROS 硬件进程并确保四轮架空、DMC1 串口独占后，例如测左前：

```bash
python3 scripts/serial/single_motor_curve.py record \
  --motor 4 --mode D --values 5,10,15 --duration-ms 600 \
  --output calibration_reports/fl_d_manual.json
python3 scripts/serial/single_motor_curve.py record \
  --motor 4 --mode S --values=-400,400,800 --duration-ms 600 \
  --output calibration_reports/fl_s_manual.json
```

`--motor 4/5/6/7` 分别是左前、右前、左后、右后；`--device` 可指定已由 `$info` 确认的
DMC1 路径，省略时扫描 `/dev/ttyACM0..8`。JSON 保存每个测试点的原始响应、采样时序及
汇总轮速/电流，PNG 的上半部分是输入与实测轮速，下半部分是峰值及后半段平均电流。
只重画图片可运行：

```bash
python3 scripts/serial/single_motor_curve.py plot \
  calibration_reports/fl_d_manual.json --output calibration_reports/fl_d_manual.png
```

本车右后轮 D30%/300ms 已测到 2875mA 且轮速不稳定；这个简化工具不会代替现场观察、
急停或驱动器自身的保护，不应据此直接对右后轮发送 D100%。

ROS 四轮曲线采集入口已退役，历史测量结论仅供排障参考。

2026-09-27 架空扫描：电机额定 1000mA，用户确认峰值 2300mA 可持续 2~3s；工具采用
1.0s 稳态脉冲、0.4s 缓升速和 2200mA 软件停机值。左前轮 ±0.2~±0.4rad/s 的反馈大致跟随
命令，但 +0.8rad/s 与 -0.5rad/s 的启动瞬态分别测到 2475mA 和 2567mA，超过当时的
2300mA 软件门槛；用户随后补充允许 3000mA 最多 2s，
工具自动停车。右前、左后、右后在 ±0.2~±0.6rad/s 范围的编码器速度和位置差分均接近零，
电流也接近空闲值。四张输入输出曲线及原始轨迹见
`calibration_reports/2026-09-27/final/`（该运行产物被 Git 忽略）。这是无效标定报告，
不能导出或启用补偿。先排查三轮速度链和左前启动过流，再确定安全的重测范围；
软件 10Hz 电流采样无法保证瞬态永不超过设定值。只读诊断确认 DMC1 四轮编码器均为
11600 CPR、四路速度命令接口均已声明并被控制器占用；独立 ROS 通道探针也确认四路命令
均能穿过轮速看门狗。故仍需排查 DMC1 速度模式、驱动器输出和电机/编码器线束，不能把
零响应当作普通曲线差异并通过加大命令补偿。

## 1. 启动

```bash
# 实机
ros2 launch robot_main robot.launch.py mode:=real

# 虚拟机器人
ros2 launch robot_main robot.launch.py mode:=virtual \
  map_yaml_file:=/workspace/maps/studyroom/studyroom.yaml
```

实机启动后先检查：

```bash
ros2 control list_controllers
ros2 topic echo /hardware/chassis_state --once
ros2 topic echo /wheel/odometry_status --once
```

八个控制器（关节状态、转向、轮速、头部和四个腿关节）应为 `active`，`dmc0_connected`、`dmc1_connected`、`se2_connected`、
`outputs_enabled` 和 `watchdog_ok` 应为 `true`。

实机转向前先确认四轮架空稳固、周围无人员接触轮子和转向机构，并备好断电手段。
先观察 `/joint_states` 中四个转向关节的实际角度与
`/hardware/chassis_state` 中的故障、驱动轮及腿电流；SE2 不提供转向舵机电流反馈，
还要现场观察机械干涉、异响和发热。任一关节反馈缺失或电流异常时不要发转向命令。
完整运动链启动后还需确认 `/hardware/command_heartbeat` 持续、
`/wheel_controller/commands` 四个值均为零。先做下文的零距离小角度 Action，再做转弯速度测试。

## 2. 底盘速度

整车导航速度链：

```text
/cmd_vel_nav_raw -> 平滑 -> /cmd_vel_nav -> robot_web /cmd_vel_console
-> 超声波安全层 -> /cmd_vel_nav_safe -> robot_web /cmd_vel_safe
-> 运动学 -> ros2_control
```

当前代码启动时软件停止锁初值为未锁定；
若软件停止锁已锁定，在 `/robot/` 首页显式解除后，Nav2 或人工速度才会被转发。
网页已取消独占控制权，任一页面均可操作。
手动速度由控制台直接发布到 `/cmd_vel_safe`，绕过超声波避障；导航速度经
`/cmd_vel_console` 和避障过滤后，再由控制台转发到 `/cmd_vel_safe`。
单独启动 `safety.launch.py` 的测试链路不包含该仲裁节点，话题路径以其 launch 参数为准。

| 字段 | 正值 | 负值 | 单位 |
|---|---|---|---|
| `linear.x` | 前进 | 后退 | m/s |
| `linear.y` | 向左 | 向右 | m/s |
| `angular.z` | 左转/逆时针 | 右转/顺时针 | rad/s |

前进 0.08m/s，持续约 5 秒：

```bash
ros2 topic pub -r 10 -t 50 /cmd_vel_nav_raw geometry_msgs/msg/Twist \
  '{linear: {x: 0.08}, angular: {z: 0.0}}'
```

后退：

```bash
ros2 topic pub -r 10 -t 30 /cmd_vel_nav_raw geometry_msgs/msg/Twist \
  '{linear: {x: -0.05}, angular: {z: 0.0}}'
```

左转弧线：

```bash
ros2 topic pub -r 10 -t 30 /cmd_vel_nav_raw geometry_msgs/msg/Twist \
  '{linear: {x: 0.06}, angular: {z: 0.15}}'
```

原地左转：

```bash
ros2 topic pub -r 10 -t 30 /cmd_vel_nav_raw geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.15}}'
```

### 运动模式

```bash
ros2 param get /chassis_controller motion_mode
ros2 param set /chassis_controller motion_mode four_ws
```

| 模式 | 用途 |
|---|---|
| `four_ws` | 默认模式；Nav2、转弯和原地旋转 |
| `crab` | 四轮同向斜行，使用 `linear.x + linear.y` |
| `ackermann` | 前轮近似阿克曼，不能真正原地旋转 |

左前 15°斜行：

```bash
ros2 param set /chassis_controller motion_mode crab
ros2 topic pub -r 10 -t 30 /cmd_vel_nav_raw geometry_msgs/msg/Twist \
  '{linear: {x: 0.096593, y: 0.025882}, angular: {z: 0.0}}'
ros2 param set /chassis_controller motion_mode four_ws
```

使用整车入口测试导航时，不要直接发布 `/cmd_vel_nav`、`/cmd_vel_safe` 或最终轮速话题。它们会与现有节点竞争，或者绕过安全层。控制台须处于导航、探索或跟随模式，才会把导航速度送入避障链；手动模式会持续输出零导航速度。侧向超声波无效时，安全层会禁止斜行。

## 3. 停车和恢复

先停止正在运行的命令发布器，再发零速：

```bash
ros2 topic pub -r 20 -t 10 /cmd_vel_nav_raw geometry_msgs/msg/Twist '{}'
```

取消全部 Nav2 单目标导航：

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  '{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
                stamp: {sec: 0, nanosec: 0}}}'
```

控制板通信故障排除后恢复硬件：

```bash
ros2 service call /hardware/recover std_srvs/srv/Trigger '{}'
```

停止 launch 或命令心跳消失后，轮速看门狗会在 500ms 后输出零速。
正在执行 `/execute_wheel_motion` 的 DMC1 定距动作需用 Action 取消；
服务端会调用 `/hardware/stop_encoder_motion`，硬件也会在动作租约或心跳失效时发送停车命令。

## 4. 四轮定距

数组顺序统一为：`[左前, 右前, 左后, 右后]`。

先进行零距离、5° 转向测试（轮速应始终为零）：

```bash
ros2 action send_goal --feedback /execute_wheel_motion \
  robot_interfaces/action/ExecuteWheelMotion \
  '{steering_angle_rad: [0.087266, 0.087266, 0.087266, 0.087266],
    distance_m: [0.0, 0.0, 0.0, 0.0],
    steering_tolerance_rad: 0.03,
    steering_timeout: {sec: 5, nanosec: 0},
    motion_timeout: {sec: 0, nanosec: 0}}'
```

确认四个转向关节实际角度方向正确、没有异常电流或机械干涉后，再逐步增大角度。

直行 0.20m（仅在转向、停车、超声波和右后轮启动迟滞检查通过后执行）：

```bash
ros2 action send_goal --feedback /execute_wheel_motion \
  robot_interfaces/action/ExecuteWheelMotion \
  '{steering_angle_rad: [0.0, 0.0, 0.0, 0.0],
    distance_m: [0.20, 0.20, 0.20, 0.20],
    steering_tolerance_rad: 0.03,
    steering_timeout: {sec: 5, nanosec: 0},
    motion_timeout: {sec: 15, nanosec: 0}}'
```

只转到左前 15°，不行驶：

```bash
ros2 action send_goal --feedback /execute_wheel_motion \
  robot_interfaces/action/ExecuteWheelMotion \
  '{steering_angle_rad: [0.261799, 0.261799, 0.261799, 0.261799],
    distance_m: [0.0, 0.0, 0.0, 0.0],
    steering_tolerance_rad: 0.03,
    steering_timeout: {sec: 5, nanosec: 0},
    motion_timeout: {sec: 0, nanosec: 0}}'
```

四个距离必须全部为零，或者全部非零。不要手工调用内部服务
`/hardware/execute_encoder_motion`。

## 5. Nav2 导航

```bash
ros2 action send_goal --feedback /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map},
           pose: {position: {x: 1.0, y: 0.0, z: 0.0},
                  orientation: {z: 0.0, w: 1.0}}}}'
```

目标 yaw 对应四元数：`z=sin(yaw/2)`，`w=cos(yaw/2)`。

## 6. 车身和头部

车身自动水平：

```bash
ros2 service call /body_balance/set robot_interfaces/srv/SetBodyBalance \
  '{enabled: true, level: mid, target_roll_deg: 0.0, target_pitch_deg: 0.0}'
```

升高或降低：

```bash
ros2 service call /body_balance/set robot_interfaces/srv/SetBodyBalance \
  '{enabled: true, level: high, target_roll_deg: 0.0, target_pitch_deg: 0.0}'

ros2 service call /body_balance/set robot_interfaces/srv/SetBodyBalance \
  '{enabled: true, level: low, target_roll_deg: 0.0, target_pitch_deg: 0.0}'
```

关闭自动平衡：

```bash
ros2 service call /body_balance/set robot_interfaces/srv/SetBodyBalance \
  '{enabled: false, level: mid, target_roll_deg: 0.0, target_pitch_deg: 0.0}'
```

头部 `[yaw, pitch]`，单位 rad：

```bash
# 左转 10°、抬头 5°
ros2 topic pub -r 2 -t 10 /head_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [0.174533, 0.087266]}'

# 回中
ros2 topic pub -r 2 -t 10 /head_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0]}'
```

建图时 `head_mapping_lock` 会持续回中，不能同时手工控制头部。

## 7. 快速调试

### 车不动

```bash
ros2 topic echo /cmd_vel_nav_raw
ros2 topic echo /cmd_vel_nav_smoothed
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_console
ros2 topic echo /cmd_vel_nav_safe
ros2 topic echo /cmd_vel_safe
ros2 topic echo /obstacle_avoidance/status
```

- 上游有速度、`cmd_vel_safe` 为零：检查控制台模式、软件停止锁、超声波和安全层状态。
- `cmd_vel_safe` 正确、控制器没命令：检查 `chassis_controller`。
- 控制器命令正确、实车不动：检查硬件输出、电机和编码器。

### 路径偏差大

```bash
ros2 topic echo /steering_controller/commands
ros2 topic echo /wheel_controller/commands
ros2 topic echo /joint_states
ros2 topic echo /wheel/odom
ros2 topic echo /visual_odom
ros2 topic echo /odom
ros2 topic echo /wheel/odometry_status
ros2 topic echo /hardware/chassis_state --field wheel_current_ma
```

- 目标舵角、轮速错误：检查 `motion_mode` 和 Twist 解算。
- 目标正确、某轮反馈偏低且电流高：检查机械堵转、电机、驱动器和线束。
- 轮侧、视觉、IMU 航向正负不一致：先修坐标轴和符号，再调 Nav2。
- 全局规划线本身错误：检查地图、TF、代价地图和 footprint。

当前实车曾出现右后轮启动迟滞：相同命令下起始约 `0.05 rad/s`，数秒后才升速，峰值电流约
`1.64A`。复测导航前应先排除此硬件问题。

2026-09-27 架空复测：零距离 5° 转向并回中均到位，四轮速度保持零；随后 2cm 四轮定距中，
左前、右前、左后约走 2.6~2.7cm，右后仅约走 0.35cm，Action 因轮编码器反馈超时中止。
停车服务返回成功，事后轮速为零、硬件无锁定故障。右后驱动或编码器问题解决前，
不要继续执行本页的 0.20m 定距、Twist 带速转弯、平移或 Nav2 导航命令。

### 超声波

```bash
ros2 topic echo /ultrasonic/front_left --qos-profile sensor_data
ros2 topic echo /ultrasonic/front_center --qos-profile sensor_data
ros2 topic echo /ultrasonic/front_right --qos-profile sensor_data
ros2 topic echo /obstacle_avoidance/status
```

`NaN` 表示无效回波或超量程。安全层默认失效关闭：当前运动方向没有新鲜有效距离时停止。

## 8. 接口查询

```bash
ros2 topic list -t
ros2 service list -t
ros2 action list -t
ros2 topic info /cmd_vel_safe -v
ros2 node info /chassis_controller
```
