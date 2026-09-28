<!-- 使用方法：从本文件选择运行模式，在 /home/radxa/Robot 统一构建和启动。 -->
# Robot

本仓库是机器人小车的 ROS 2 Jazzy 工作区，用于维护实机与虚拟机器人的统一模型、双目视觉、
SLAM、Nav2 路径规划、局部避障和底盘控制。

## 项目结构

```text
Robot/
├── .codex/                         # 项目级专业 Agent 与并发配置
├── AGENTS.md                      # 仓库开发约定和验证要求
├── docker/                         # ROS 2 Jazzy ARM64 容器镜像
├── docs/                           # 包说明、多 Agent、标定和验收文档
├── scripts/                        # 环境安装、镜像构建、相机配置和启动脚本
├── src/
│   ├── robot_interfaces/           # 感知、任务与控制的 msg/srv/action
│   ├── robot_main/                 # 实机与虚拟模式唯一整车启动入口
│   ├── robot_hardware/             # DMC0/DMC1/SE2 ros2_control 串口插件
│   ├── robot_brain/                # Qwen 决策核心、动作白名单和任务工具
│   ├── robot_perception/           # 双目、YOLO、深度、地形和虚拟传感器
│   ├── robot_navigation/           # RTAB-Map、Nav2、停靠点、跟随和探索预演
│   ├── robot_control/              # 四轮转向、里程计、速度门控和最终安全层
│   ├── robot_description/          # URDF、ros2_control 描述和 mesh
│   └── robot_stereo_components/    # 高带宽双目处理 C++ 节点
├── progress.md                     # 按日期记录开发过程、卡点和踩坑
└── README.md
```

## 多 Agent 协作

项目在 `.codex/agents/` 定义了跨包协调、本地大脑、感知、导航、控制硬件和平台集成 Agent。
复杂问题可以让相关 Agent 并行做只读排查，再按文件划分唯一写入者并统一验证。职责表、跨包
协作流程、调用示例和每个功能包的中文文档见
[`docs/multi_agent_workflow.md`](docs/multi_agent_workflow.md)。

## 多 Package 架构

工作区按领域拆分，但仍使用同一个 `src/`、`build/` 和 `install/`。所有 Package 统一
通过根目录 `colcon build --symlink-install` 构建。旧 `robot` 兼容包已经退役，启动和单独
运行节点时应直接使用对应的领域 Package。

依赖方向固定为：`robot_brain` 保留独立决策核心，ROS 任务入口待重新接入；`robot_navigation` 通过类型化消息消费感知结果；`robot_control` 不依赖 Qwen、相机
或导航实现。Qwen 只能建议 `goto_object`、`follow_person`、`explore`，不能发布 Topic、
生成坐标或取得控制权。

关键类型化接口：

- `/perception/semantic_detections`：`robot_interfaces/msg/SemanticDetectionArray`。
- `/perception/detect_objects`、`/perception/set_detection_mode`：按需识别与持续识别模式。
- `/mission/plan`：只生成目标和路径的 `PlanMission` Action。
- `/mission/confirm`：确认已缓存预览，只发布 `/mission/navigation_goal`。
- `/mission/state`：类型化任务状态；旧 `/mission/status` JSON 暂时兼容。
- `/perception/terrain_state`：感知到控制的类型化地形约束。
- `/hardware/chassis_state`：三块控制板、看门狗、输出锁定和电流状态。
- `/wheel/odometry_status`：逐轮残差、权重、疑似故障和动态协方差状态。
- `/execute_wheel_motion`：先转向后执行四轮定距的 `ExecuteWheelMotion` Action。
- 单轮串口采样：`scripts/serial/single_motor_curve.py record` 按显式指定的 D/S 测试点记录 JSON 和速度、电流 PNG；运行前须独占 DMC1 串口并架空车轮。具体命令见 `docs/motion_control_commands.md`。
- `/hardware/stop_encoder_motion`：取消或超时后停止 DMC1 定距目标；定距期间还监测 `/wheel_motion/active` 租约。
- `/body_balance/set`、`/body_balance/status`：姿态平衡控制与状态。

## Python 功能分层

Python 节点按职责分别位于 `robot_perception`、`robot_navigation`、`robot_control` 和
`robot_brain`，不再提供旧 `robot` 包的运行入口。

### sensing：传感器接入层

- `stereo_splitter.py`：Python 双目拼接图拆分实现，作为 C++ 实现的回退方案。
- `stereo_pair_throttle.py`：选择同时间戳左右图像对，并限制立体匹配频率。
- `virtual_imu.py`：根据虚拟底盘里程计生成模拟 IMU。
- `virtual_ultrasonic.py`：根据环境点云和传感器 TF 生成 8 路模拟超声波。

正常情况下，双目拆分优先使用
`src/robot_stereo_components/src/stereo_splitter_node.cpp`。需要调试 Python 实现时，在相机
launch 后追加：

```bash
splitter_backend:=python
```

### perception：感知处理层

- `stereo_depth.py`：将视差转换为米制深度和 8 位预览图。
- `stereo_pointcloud_filter.py`：过滤真实双目点云，输出 Nav2 使用的局部障碍点。
- `pointcloud_obstacle_filter.py`：过滤离线 PLY 环境点云。
- `snapshot_local_observer.py`：从保存的全局点云裁剪虚拟机器人当前可见的局部障碍。
- `terrain_heightmap.py`：从点云生成地形高度网格。
- `terrain_physics.py`：计算坡度、台阶、坑洼和可通行性。
- `terrain_analyzer.py`：订阅点云并发布统一的地形状态。

### localization：定位层

真实建图由 `rtabmap_odom/stereo_odometry` 输出 `/visual_odom_raw`，经过协方差下限保护后
得到 `/visual_odom`，再由
`robot_localization` 二维 EKF 统一输出 `/odom` 和 `odom -> base_link`。当前融合 GY95T 的
yaw 角速度、四轮实际反馈生成的 `/wheel/odom` 和双目视觉位姿。
RTAB-Map 继续负责 `map -> odom`，视觉里程计不再单独发布冲突 TF。

本次传感器融合功能按 ROS Package 分工如下：

| 功能 | 所属 Package | 主要输入与输出 |
| --- | --- | --- |
| GY95T 串口驱动、零偏/中值/低通、Madgwick | `robot_perception` | `/dev/gy95t` → `/sensors/imu/data` |
| 双目伪障碍点过滤 | `robot_perception` | `/stereo/points2` → `/mapping/stereo_obstacle_points` |
| 视觉、IMU、轮速 EKF 与 RTAB-Map 编排 | `robot_navigation` | `/visual_odom`、IMU、`/wheel/odom` → `/odom` |
| 建图头部归中、四轮运动学 | `robot_control` | 头部位置命令；`/joint_states` → `/wheel/odom` |
| 现有 IMU/相机 TF 与头部 ros2_control 接口 | `robot_description` | URDF、`imu_link`、头部关节硬件接口 |

`RobotSerialSystem` 通过标准 `sensor_msgs/JointState` 提供四个实际转角、连续轮编码器位置和
轮速。四轮里程计默认启用，只在至少三个轮子的反馈一致时发布 `/wheel/odom`，不使用命令值
冒充反馈。

### mapping：建图和地图层

- `snapshot_manager.py`：缓存 RTAB-Map 的二维地图和三维点云，并按服务请求保存快照。
- `ply_publisher.py`：读取离线 PLY 文件，发布 Foxglove 显示和仿真感知使用的点云。

地图快照服务：

```bash
# 创建临时导航预演快照
ros2 service call /mapping/create_preview_snapshot std_srvs/srv/Trigger '{}'

# 创建长期保存快照
ros2 service call /mapping/save_snapshot std_srvs/srv/Trigger '{}'
```

临时快照位于 `/tmp/robot_preview/current.{yaml,pgm,ply,json}`，长期快照默认位于
`/workspace/maps/map_YYYYMMDD_HHMMSS/`。长期目录名和 `map.json` 的 `created_at`
默认按 `mapping_snapshot.yaml` 中的 `snapshot_timezone: Asia/Shanghai` 生成，
不受容器本地时区影响。

### mission：任务层

- `goal_manager.py`：接收 Foxglove 发布的 `/goal_pose`，校验后发送给 Nav2
  `NavigateToPose` action。

相关接口：

- `/goal_pose`：输入 `geometry_msgs/msg/PoseStamped`，frame 必须是 `map`。
- `/mission/status`：任务状态。
- `/mission/cancel`：取消当前任务的 `std_srvs/srv/Trigger` 服务。

后续的人体跟随、物体目标和自主探索，应先在任务层生成导航目标，再交给 Nav2，不直接控制底盘。

### safety：安全层

- `range_to_scan.py`：把 8 路超声波 Range 合成为 LaserScan。
- `obstacle_avoidance.py`：底盘前最后一级安全过滤；无有效量程、量程超时或距离过近时
  限速或停车。

导航预演的速度链：

```text
Nav2
  -> /cmd_vel_nav
  -> 双目 Collision Monitor
  -> /cmd_vel_stereo_safe
  -> 超声波最终安全层
  -> /cmd_vel_safe
  -> 虚拟底盘
```

### control：控制层

- `nav_controller.py`：连接 Nav2 速度平滑器和底盘安全链。
- `chassis_controller.py`：将安全速度转换为轮速和转向控制，并发布虚拟里程计。
- `chassis_feedback.py`：将 `/joint_states` 整理为底盘反馈。

实机和虚拟模式都保持上游 `/cmd_vel_safe` 接口不变，差异只存在于 ros2_control 插件。

### diagnostics：诊断层

- `stereo_pipeline_benchmark.py`：统计双目链路各阶段频率、消息年龄和处理延迟。

## 关键启动文件

启动文件按领域位于各 Package 的 `launch/` 目录。
每个对外 launch 参数都带有中文说明，可在启动前查看参数、用途和默认值：

```bash
ros2 launch <包名> <launch文件> --show-args
# 示例
ros2 launch robot_perception stereo_camera.launch.py --show-args
```

### Launch 分层

复杂场景入口只负责组合独立功能 launch 和传递参数，不直接重复创建节点。当前分层如下：

- 基础与可视化：`description.launch.py`、`joint_states.launch.py`、
  `foxglove.launch.py`。
- 控制功能：`controllers.launch.py`、`chassis_control.launch.py`、
  `head_mapping_lock.launch.py`、`wheel_odometry.launch.py`、
  `nav_velocity_gate.launch.py`、`obstacle_avoidance.launch.py`。
- 实机感知：`stereo_camera.launch.py`、`imu.launch.py`、
  `stereo_pointcloud_filter.launch.py`、`semantic_detection.launch.py`、
  `acceptance_sampler.launch.py`。
- 虚拟感知：`pointcloud_obstacle.launch.py`、`terrain_analysis.launch.py`、
  `virtual_ultrasonic.launch.py`、`range_to_scan.launch.py`、
  `virtual_imu.launch.py`。
- 导航与建图功能：`nav2.launch.py`、`stereo_odometry.launch.py`、
  `state_estimation.launch.py`、`rtabmap_mapping.launch.py`、
  `mapping_snapshot.launch.py`、`ply_map.launch.py`。
- 组合入口：`control.launch.py`、`safety.launch.py`、
  `virtual_sensors.launch.py`、`stereo_perception.launch.py`、
  `robot.launch.py`、`stereo_mapping.launch.py` 和 `stereo_robot.launch.py`。

独立功能入口均可用 `ros2 launch <包名> <文件名> --show-args` 查看输入话题、输出话题、
设备、配置文件和日志参数。组合入口已有同一基础功能时，应通过其 `start_*` 参数关闭重复实例。

ROS 2 常用运动命令和简化排障流程见
[`docs/motion_control_commands.md`](docs/motion_control_commands.md)；绕过 ROS、直接访问
DMC0/DMC1/SE2 的原生串口指令见
[`docs/hardware_serial_commands.md`](docs/hardware_serial_commands.md)。人工速度测试应从
`/cmd_vel_nav_raw` 或 `/cmd_vel_nav_smoothed` 进入安全链，不要直接发布 `/cmd_vel_safe` 或
最终轮速控制器话题。

### 统一整车入口

完整启动统一由 `robot_main` 编排。`mode` 只选择 ros2_control 插件，模型、关节、运动学、
安全链、里程计、EKF 和 Nav2 配置保持共用：

```bash
# 纯虚拟调试
ros2 launch robot_main robot.launch.py mode:=virtual \
  map_yaml_file:=/workspace/maps/studyroom/studyroom.yaml

# 实机控制，同时由实机 /joint_states 和 /odom 驱动同一数字孪生模型
ros2 launch robot_main robot.launch.py mode:=real

# 使用已有地图时才显式传入；此时不启动在线 RTAB-Map 建图
ros2 launch robot_main robot.launch.py mode:=real map_yaml_file:=/workspace/maps/example/map.yaml
```

`mode` 的安全默认值是 `virtual`；实机启动时不可省略 `mode:=real`。启动日志会打印最终选择的
模式和 ros2_control 插件，实机还应存在 `/hardware/chassis_state` 话题。`map_yaml_file` 默认
留空：实机启动在线 RTAB-Map 建图和 Nav2；只有显式传入 YAML 才加载已有地图。虚拟模式没有
实时双目建图源，因此必须显式指定地图。

虚拟模式加载 `mock_components/GenericSystem`；实机模式加载
`robot_hardware/RobotSerialSystem`，自动在 `/dev/ttyACM0..8` 中识别 DMC0、DMC1 和 SE2。
串口候选、发现重试、事务/命令超时、轮径、轴距、轮距和腿长以
`robot_hardware/config/hardware_bringup.yaml` 为统一启动默认值，并由 `robot_main` 同时传给
硬件插件、运动学、轮速里程计和平衡节点；现场可用同名 launch 参数覆盖。
旧 `robot_navigation/robot.launch.py` 与 `stereo_robot.launch.py` 仅作为兼容转发入口。

实机模式任一控制板通信失败时立即停止并锁定输出。排除故障后调用：

```bash
ros2 service call /hardware/recover std_srvs/srv/Trigger '{}'
```

容器启动脚本会逐条打印双目相机、GY95T 和 `/dev/ttyACM0..8` 的宿主机到容器映射。底盘
串口通过只放行 USB CDC ACM 主设备号的动态 `/host-dev` 视图接入，因此控制板 USB 重枚举后
仍使用容器内同名 `/dev/ttyACM*` 路径，无需再次重建容器。若旧 `vctrl.service` 正在运行，
脚本会警告它与 ROS 2 争用控制板；实机启动前必须执行
`sudo systemctl stop vctrl.service`，禁止两个硬件驱动同时连接串口。控制链为
`Nav2 -> /cmd_vel_nav -> 超声波安全层 -> /cmd_vel_safe -> 三模式运动学 -> ros2_control`。
轮速命令经过独立 500 ms 看门狗，转向未到位时轮速保持零；定距 Action 的 T 指令有独立停车服务和动作租约。8 路超声波统一为前3、后3、左1、右1。SE2 的 Range
话题使用传感器 Best Effort QoS，避障和 `/scan` 合成节点使用相同 QoS 接收。最终安全层默认
失效关闭：当前运动方向没有新鲜测距就输出零速；遇障默认停车，不自动倒车。SE2 原始测距
默认只接受 30~2500mm，低于 30mm 的探头振铃/无回波异常值发布为 `NaN`，不会伪装成
1~2cm 的真实障碍；上下限由 `hardware_bringup.yaml` 配置。

### 兼容虚拟机器人入口

```bash
ros2 launch robot_navigation robot.launch.py
```

该命令只转发到 `robot_main mode:=virtual`。

### 单独启动真实双目相机

```bash
ros2 launch robot_perception stereo_camera.launch.py video_device:=/dev/video0
```

主要输出左右原图、校正图、视差、深度和点云。常用参数：

- `calibration_mode:=true`：只保留标定需要的左右原图。
- Foxglove Bridge 默认随启动文件运行，使用 `foxglove_port` 修改监听端口。
- `splitter_backend:=cpp|python`：选择双目拆分实现。
- `navigation_processing_enabled:=false`：关闭视差、深度和导航点云处理。

### 真实双目建图

```bash
ros2 launch robot_navigation stereo_mapping.launch.py video_device:=/dev/video0
```

建图入口默认启动 GY95T，并把 `/sensors/imu/data` 同时送入双目视觉里程计和二维 EKF：

```bash
ros2 launch robot_navigation stereo_mapping.launch.py \
  video_device:=/dev/video0 imu_device:=/dev/gy95t
```

现场暂未接入 IMU 时显式传入 `use_imu:=false`，视觉里程计和 EKF 会保持纯视觉降级运行。

推荐在宿主机为 USB 转串口创建稳定的 `/dev/gy95t` 别名；首次调试也可直接传入实际设备，
例如 `imu_device:=/dev/ttyUSB0`。容器必须在创建时映射该设备，运行中的旧容器不能追加设备；
`run_jazzy_container.sh` 会依次检测 `/dev/gy95t`、当前 CH340 的稳定 by-id 和
`/dev/ttyUSB0`，统一映射为容器内 `/dev/gy95t`。存在多个 USB 串口时，用
`GY95T_DEVICE=/dev/ttyUSBx bash scripts/docker/run_jazzy_container.sh` 显式指定。

若启动时报 `XML or text declaration not at start of entity`，请确认已重新构建
`robot_description`，且 `robot.xacro` 的 `<?xml ...?>` 声明位于文件第一行。

该入口启动真实相机、双目视觉里程计、二维 EKF、过滤后的建图障碍点云、RTAB-Map、
头部建图归中、机器人模型、快照管理和 Foxglove，不启动虚拟底盘和 Nav2。默认
`require_head_feedback:=false` 适合摄像头机械固定向前的手持验证；接入实机头部闭环后应改为
`true`，并确保 `/head_controller/commands` 有控制器订阅且 `/joint_states` 返回实际角度。
`wait_imu_to_init` 默认保持 `false`：IMU 正常时仍会参与视觉旋转估计和 EKF，但串口短时掉线
不会阻塞纯视觉 `/visual_odom`。只有明确要求“没有 IMU 就不允许建图”时才设为 `true`。

没有 IMU 和轮式里程计时仍可依靠双目视觉里程计和 EKF 建图。手持验收时应把摄像头和
IMU 固定在同一刚性支架上，摄像头正前、IMU 水平，启动后先静止至少 2 秒完成陀螺仪零偏
估计，再缓慢移动整个支架；不能让摄像头相对 IMU 单独转动。

GY95T 驱动读取附件协议的 `0x08-0x2A` 寄存器，发布
`/sensors/imu/raw_unfiltered`、`/sensors/imu/data_raw` 和内部姿态对照
`/sensors/imu/vendor_rpy`。原始角速度和加速度经过三点中值、10 Hz 一阶低通及静止零偏
校正，随后由无磁 Madgwick 输出 `/sensors/imu/data`。首版 EKF 只使用 yaw 角速度，不使用
线加速度积分或易受电机干扰的磁航向。装车后必须按前倾、左倾和逆时针转动检查
`axis_permutation` 与 `axis_sign`。

二维地图由 RTAB-Map 从已同步的双目深度生成，并通过 `Grid/NoiseFilteringRadius` 与
`Grid/NoiseFilteringMinNeighbors` 删除缺少邻域支持的孤立误匹配，避免空白区域形成黑色点。
建图默认以最高 1 Hz 保留新观测，即使底盘位姿没有变化也会刷新全局栅格；这样固定观察时
新出现或消失的结构仍能反映到 `/map`，同时避免以相机帧率无限增长数据库。人员等短时动态
障碍仍应以 `/mapping/stereo_obstacle_points` 和 Nav2 局部代价地图为准，不应依赖静态地图
长期保存。实机头部在建图时归中，头部 PID 使用约 2.5° 停止死区，防止最小 PWM 导致相机在
零位附近持续左右、上下往返修正。
同时保留 `/mapping/stereo_obstacle_points`：它经过距离、高度、视场、每体素最少点数和相邻
体素支持过滤，供 Foxglove 检查和未来 Nav2 使用，但不再作为 RTAB-Map 必须同步的第五路输入，
避免过滤点云短时缺失导致整张二维地图无法生成。

建图入口默认在 `usb_cam` 占用设备前，通过相机真实的 V4L2 控制名开启自动曝光和自动
白平衡，并在终端回读控制值。若现场需要固定曝光和色温，可先运行
`scripts/stereo/configure_stereo_camera.sh`，再以
`apply_auto_camera_controls:=false` 启动建图。双目标定必须保持自动控制关闭，避免采样中
亮度和颜色漂移。

所有正常双目入口的 `apply_auto_camera_controls` 默认值均为 `true`；标定脚本会显式传入
`false`，且 `calibration_mode:=true` 时相机入口也会自动跳过控制写入。左右校正图的
`/compressed` 由 `rectify_node` 的 image_transport 插件按订阅需求发布，不再额外启动左右
压缩转发器，避免同一话题出现两个发布者和重复帧；显式压缩转发仅保留给深度预览。

`/visual_odom` 只表示双目视觉估计，`/odom` 才是 EKF 融合结果并负责
`odom -> base_link` TF。二者都不会发布速度命令或直接驱动底盘；真实小车只有在底盘控制链
收到 `/cmd_vel` 后才会运动。

Foxglove 常用话题：

- `/map`：二维占据栅格。
- `/mapping/cloud_map`：三维地图点云。
- `/visual_odom`：视觉里程计。
- `/odom`：视觉、IMU 和四轮反馈里程计的统一融合结果。
- `/sensors/imu/data`：无磁姿态滤波后的 IMU。
- `/mapping/stereo_obstacle_points`：生成二维地图前的过滤障碍点云。
- `/mapping/head_lock_status`：建图期间头部归中状态。
- `/tf`、`/robot_description`：机器人模型和轨迹。
- `/mapping/snapshot_status`：快照状态。

### 完整在线双目机器人（兼容入口）

`stereo_robot.launch.py` 现仅把兼容参数转发到 `robot_main mode:=real`。新部署和调试应直接使用
统一入口，避免继续扩展旧包装文件。

不传 `map_yaml_file` 时，同时启动 RTAB-Map 和 Nav2，使用实时 `/map` 边建图边导航：

```bash
ros2 launch robot_main robot.launch.py mode:=real
```

加载工作区 `maps/` 中已有地图时传入对应 YAML；PGM 使用 YAML 中的相对路径自动加载，实时
双目点云继续负责局部动态避障：

```bash
ros2 launch robot_navigation stereo_robot.launch.py \
  map_yaml_file:=/workspace/maps/map_20260827_202256/map.yaml \
  initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

只要 `map_yaml_file` 非空，就不会启动 RTAB-Map，也不会修改已有地图；地图服务器发布 `/map`，
给定的 `initial_x`、`initial_y`、`initial_yaw` 用于发布 `map -> odom` 初始关系。不传该参数时，
不会启动静态地图服务器或静态 `map -> odom`，两者均由在线 RTAB-Map 提供，避免重复发布。

两种模式都启动同一套 GY95T、双目视觉里程计和二维 EKF：`/visual_odom` 与 IMU yaw 角速度
融合为 `/odom`，并由 EKF 发布 `odom -> base_link`。默认 `use_imu:=true`；没有映射 GY95T
设备时应传入 `use_imu:=false`。已有地图模式当前不启动 AMCL，必须让机器人从命令给定的已知
地图位姿开始；如果启动位置不确定，需要后续接入 RTAB-Map localization 或 AMCL，而不能靠
IMU 推断地图中的绝对位置。

完整双目入口会覆盖 `chassis_controller.publish_odometry=false`，禁止旧底盘积分同时发布
`/odom` 和 `odom -> base_link`；真实轮侧反馈应通过 `/wheel/odom` 进入 EKF。单独运行旧控制
入口时该参数默认保持 `true`，兼容原有仿真和调试方式。

当前启动编排不再用 16、31、41 秒的长固定等待。入口先启动机器人模型、Foxglove、IMU、
视觉里程计和 EKF；传感器与控制链默认在第 3 秒开始，语义
感知在第 8 秒、Nav2 在第 14 秒开始。顶层功能入口以及双目相机和 Nav2 内部节点仍以默认
0.8 秒间隔错峰创建，避免 RK3588 在同一时刻初始化大量高负载节点。ros2_control 的
controller spawner 不再按定时器并发启动，
而是严格等待前一个退出后再启动下一个，避免多个 spawner 争抢进程锁。可用
`node_start_interval` 及四个阶段参数按现场负载调整；`navigation_start_delay` 应晚于双目
里程计和在线地图首次输出。Foxglove 默认保持 WARN，排查监听状态时传入
`foxglove_log_level:=info`，并确认终端出现 `Server listening on port 8765`。
所有带错峰定时器的独立入口都会先在当前作用域解析延迟；相机自动控制退出回调和控制器串行
生成链也会在注册时固化所需参数。因此这些入口被上层延时 include 后，不会因子 launch
作用域结束而丢失配置。所有组合入口还会为每个子 launch 建立独立配置作用域，避免多个功能
共用 `config_file`、`log_level` 等参数名时互相继承错误值。
GY95T 开始发布滤波数据前还会静止估计约 2 秒
陀螺仪零偏，期间出现一次
`Still waiting for data on topic imu/data_raw` 属于正常初始化；持续超过 5 秒则应检查是否有
重复 `gy95t_driver` 占用串口。四个阶段可用 `sensor_start_delay`、`control_start_delay`、
`navigation_start_delay` 和 `perception_start_delay` 调整，不建议全部设为 0。

该入口用于相机和机器人运动链已经处于同一坐标关系时的在线模式。首次接入实机应先架空车轮
检查方向、限位、看门狗和急停，再在室内平地低速验收；固定在环境中的相机不能与运动底盘混用。
容器启动脚本将宿主机 `/dev/stereo_camera` 映射为容器内 `/dev/video0`，因此该入口默认使用
`/dev/video0`；只有自定义容器设备映射时才需要传入 `video_device:=...`。
该入口默认以 `log_level:=warn` 启动各 ROS 节点，需要调试时可显式传入
`log_level:=info`。launch 框架输出的进程启动与退出提示不受该参数控制。

### 本地推理与任务代码

`robot_brain` 当前保留 Qwen 文字决策、动作白名单、场景与任务状态等 Python 核心代码，
不提供网页服务或 ROS 启动入口。任务规划器与 Nav2 的正式执行衔接仍需在整体软件阶段完成。

独立推理网关仍供语义感知使用；先准备纯文本 Qwen 模型，再启动网关：

```bash
./scripts/inference/download_rk3588_models.sh qwen
./scripts/inference/start_yolo_gateway.sh
```

网关在 9100 端口提供检测、分割和纯文本推理接口，使用同一把 NPU 锁串行调度。
需要外部语言模型时可设置 `ROBOT_LLM_ENDPOINT` 和 `ROBOT_LLM_FALLBACK_ENDPOINT`。
右目校正图、右目对齐深度和语义结果仍由 `robot_perception` 的 ROS 节点提供；验收采样
可通过 `acceptance_sampler.launch.py` 独立启动。

## 关键配置文件

配置已随职责拆到 `src/robot_{control,perception,navigation}/config/`。

| 文件 | 用途 |
| --- | --- |
| `stereo_camera.yaml` | UVC 相机格式、分辨率、帧率和拆分参数 |
| `cameras/*/left.yaml`、`right.yaml` | 按相机型号保存的左右目标定参数 |
| `stereo_pointcloud_filter.yaml` | 视差、深度范围和双目点云过滤参数 |
| `imu.yaml` | GY95T 串口、轴向、零偏、低通和无磁姿态滤波参数 |
| `stereo_odometry.yaml` | 双目视觉里程计参数 |
| `rtabmap_mapping.yaml` | RTAB-Map 在线建图参数 |
| `mapping_snapshot.yaml` | 二维地图和三维点云快照参数 |
| `state_estimation.yaml` | 视觉、IMU 和四轮反馈里程计的二维 EKF 参数 |
| `nav2.yaml` | Nav2 控制器、规划器、代价地图和行为树参数 |
| `stereo_robot.yaml` | 在线双目及独立 SegFormer 语义层的 Nav2 覆盖参数 |
| `controllers.yaml` | ros2_control 管理器以及轮子、转向和关节控制器参数 |
| `chassis_control.yaml`、`nav_velocity_gate.yaml`、`obstacle_avoidance.yaml` | 底盘控制、速度门控和最终避障参数 |
| `head_mapping_lock.yaml`、`wheel_odometry.yaml` | 建图头部归中与四轮里程计参数 |
| `terrain_analysis.yaml`、`pointcloud_obstacle.yaml` | 地形分析与环境点云障碍参数 |
| `virtual_imu.yaml`、`virtual_ultrasonic.yaml`、`range_to_scan.yaml` | 数字孪生传感器参数 |
| `semantic_detection.yaml`、`acceptance_sampler.yaml` | YOLO、SegFormer、深度融合和验收采样参数 |

设备路径、网络端口和现场参数应通过 YAML 或 launch 参数修改，不要写死在 Python 节点中。

## 模型、地图和世界

- `src/robot_description/urdf/robot.xacro`：机器人模型总入口。
- `src/robot_description/urdf/head.xacro`：两自由度头部和双目相机安装结构。
- `src/robot_description/urdf/hardware.xacro`：ros2_control 硬件接口。
- `src/robot_description/meshes/`：车体、轮子、头部和传感器网格。
- `maps/studyroom/studyroom.*`：默认二维地图和三维 PLY 点云。
- `maps/obstacle_test/obstacle_test.*`：避障测试地图。
- `maps/blank/blank.*`：基础运动链调试使用的空白地图。

虚拟模式启动前须提供有效 PLY 并安装 `plyfile`；点云缺失时节点会报错，不再发布调试用的虚构点。单独调试 `ply_map.launch.py` 时才可显式设置 `allow_fallback:=true`。

二维地图的 `.yaml` 和 `.pgm` 必须配套；需要三维局部观察或虚拟超声波时，还要提供同一
坐标系下的 `.ply`。容器把工作区根目录挂载为 `/workspace`，因此 launch 默认从
`/workspace/maps/studyroom/` 加载；宿主机直接运行时应通过 `map_yaml_file` 和 `ply_file`
传入宿主机绝对路径。

## 构建与容器

RK3588 的 Debian 12 主机通过 Ubuntu 24.04 ARM64 容器运行 ROS 2 Jazzy。

首次安装 Docker：

```bash
sudo bash scripts/docker/install_jazzy_docker.sh
```

构建包含 Nav2、RTAB-Map 和双目依赖的镜像：

```bash
bash scripts/docker/build_jazzy_image.sh
```

RTAB-Map 会安装 PCL、VTK 等较大依赖，构建前建议至少预留约 8 GB。

日常构建并启动默认虚拟机器人：

```bash
bash scripts/docker/run_robot_jazzy.sh
```

只启动容器，不自动构建和运行 launch：

```bash
bash scripts/docker/run_jazzy_container.sh
docker exec -it robot-jazzy bash
```

该脚本默认使用宿主机 IPC，让容器与宿主机上的 Fast DDS 进程共用 `/dev/shm`；当前不限制
容器共享内存为 1 GB。停止容器后若异常残留影响下一次 ROS 图发现，应先确认宿主机没有其他
ROS 2 进程，再处理对应的 Fast DDS 共享内存运行文件。

进入容器后手动构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

双目标定入口：

```bash
bash scripts/stereo/run_stereo_calibration.sh preview
bash scripts/stereo/run_stereo_calibration.sh calibrate 0.030
```

相机配置和标定的详细步骤见 `docs/stereo_calibration_guide.md`。

## 测试

```bash
colcon build --symlink-install
source install/setup.bash


更完整但保持精简的开发约定见 `AGENTS.md`。
