# Robot

本仓库是机器人小车的 ROS 2 Jazzy 工作区，用于维护实机与虚拟机器人的统一模型、双目视觉、
SLAM、Nav2 路径规划、局部避障和底盘控制。

## 项目结构

```text
Robot/
├── docker/                         # ROS 2 Jazzy ARM64 容器镜像
├── docs/                           # 双目标定和验收文档
├── scripts/                        # 环境安装、镜像构建、相机配置和启动脚本
├── src/
│   ├── robot/                      # 主 ROS 2 Python 功能包
│   │   ├── config/                 # Nav2、相机、SLAM、控制器和避障参数
│   │   ├── launch/                 # 各运行模式的启动文件
│   │   ├── map/                    # 二维地图和配套 PLY 点云
│   │   ├── meshes/                 # 机器人三维模型
│   │   ├── robot/                  # 按功能分层的 Python 节点
│   │   ├── test/                   # Python 单元测试和 lint 测试
│   │   ├── urdf/                   # 机器人 xacro 模型
│   │   ├── world/                  # 仿真世界
│   │   ├── package.xml             # ROS 包依赖
│   │   └── setup.py                # Python 包安装和 ros2 run 入口
│   └── robot_stereo_components/    # 高带宽双目处理 C++ 节点
├── progress.md                     # 按日期记录开发过程、卡点和踩坑
└── README.md
```

## Python 功能分层

Python 实现全部位于 `src/robot/robot/robot/` 的功能目录中，根目录不再放置具体节点。

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

该目录用于放置定位实现。目前双目视觉里程计直接使用
`rtabmap_odom/stereo_odometry`，由 `stereo_mapping.launch.py` 组合启动。后续接入
IMU、轮式里程计或 robot_localization 时，应在这一层增加融合节点，不需要改动感知和任务层。

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
`/workspace/maps/map_YYYYMMDD_HHMMSS/`。

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

真实运动控制接入后，可以替换本层底盘实现，同时保持上游 `/cmd_vel_safe` 接口不变。

### diagnostics：诊断层

- `stereo_pipeline_benchmark.py`：统计双目链路各阶段频率、消息年龄和处理延迟。

## 关键启动文件

所有启动文件位于 `src/robot/launch/`。

### 默认虚拟机器人

```bash
ros2 launch robot robot.launch.py
```

加载仓库内的二维地图、PLY 点云、Nav2、虚拟传感器和虚拟底盘，适合调试现有数字孪生链路。

### 单独启动真实双目相机

```bash
ros2 launch robot stereo_camera.launch.py video_device:=/dev/video0
```

主要输出左右原图、校正图、视差、深度和点云。常用参数：

- `calibration_mode:=true`：只保留标定需要的左右原图。
- Foxglove Bridge 默认随启动文件运行，使用 `foxglove_port` 修改监听端口。
- `splitter_backend:=cpp|python`：选择双目拆分实现。
- `navigation_processing_enabled:=false`：关闭视差、深度和导航点云处理。

### 真实双目建图

```bash
ros2 launch robot stereo_mapping.launch.py video_device:=/dev/video0
```

该入口启动真实相机、双目视觉里程计、RTAB-Map、地图点云、机器人模型、快照管理和
Foxglove，不启动虚拟底盘和 Nav2。

当前没有 IMU 和轮式里程计时仍可依靠双目视觉里程计建图。扩大地图时应保持相机安装在
机器人上并缓慢移动整车；相机完全静止只能建立当前视野附近的局部地图。

建图入口默认在 `usb_cam` 占用设备前，通过相机真实的 V4L2 控制名开启自动曝光和自动
白平衡，并在终端回读控制值。若现场需要固定曝光和色温，可先运行
`scripts/configure_stereo_camera.sh`，再以
`apply_auto_camera_controls:=false` 启动建图。双目标定必须保持自动控制关闭，避免采样中
亮度和颜色漂移。

`/visual_odom` 只表示双目视觉估计的机器人位姿，并通过 `odom -> base_link` TF 让
Foxglove/RViz 中的 `robot_description` 随估计轨迹移动；它不会发布速度命令，也不会直接
驱动真实底盘。真实小车只有在底盘控制链收到 `/cmd_vel` 后才会运动。

Foxglove 常用话题：

- `/map`：二维占据栅格。
- `/mapping/cloud_map`：三维地图点云。
- `/visual_odom`：视觉里程计。
- `/tf`、`/robot_description`：机器人模型和轨迹。
- `/mapping/snapshot_status`：快照状态。

### 已保存地图导航预演

先在建图模式调用临时快照服务，停止建图 launch，然后启动：

```bash
ros2 launch robot stereo_navigation_preview.launch.py
```

该入口不读取位置固定的真实相机，而是加载
`/tmp/robot_preview/current.yaml` 和 `current.ply`，让虚拟机器人在已保存环境中规划、
运动和避障。也可以指定其他地图：

```bash
ros2 launch robot stereo_navigation_preview.launch.py \
  map_yaml_file:=/workspace/src/robot/map/studyroom.yaml \
  ply_file:=/workspace/src/robot/map/studyroom.ply
```

### 完整在线双目机器人

```bash
ros2 launch robot stereo_robot.launch.py video_device:=/dev/video0
```

用于相机和机器人运动链已经处于同一坐标关系时的在线模式。真实底盘、IMU 和超声波尚未接入
ROS 前，不应把固定在现实环境中的相机点云与正在移动的虚拟机器人混合使用。

`common_bringup.launch.py` 是其他入口复用的公共组合文件，负责机器人模型、地图、Nav2、
ros2_control 和 Foxglove；通常不直接单独启动。

## 关键配置文件

配置文件位于 `src/robot/config/`。

| 文件 | 用途 |
| --- | --- |
| `stereo_camera.yaml` | UVC 相机格式、分辨率、帧率和拆分参数 |
| `cameras/*/left.yaml`、`right.yaml` | 按相机型号保存的左右目标定参数 |
| `stereo_pointcloud.yaml` | 视差、深度范围和双目点云过滤参数 |
| `rtabmap_stereo_mapping.yaml` | 双目视觉里程计、RTAB-Map 和地图点云参数 |
| `navigation_preview.yaml` | 静态点云局部观察和目标管理参数 |
| `stereo_collision_monitor.yaml` | 双目局部减速、停车区域和传感器超时 |
| `nav2_params.yaml` | Nav2 控制器、规划器、代价地图和行为树参数 |
| `nav2_stereo_overrides.yaml` | 在线双目模式的 Nav2 覆盖参数 |
| `controllers.yaml` | 轮子、转向和关节控制器参数 |
| `controller_manager.yaml` | ros2_control 管理器和硬件插件参数 |
| `terrain_params.yaml` | 地形分析、虚拟传感器和最终避障阈值 |

设备路径、网络端口和现场参数应通过 YAML 或 launch 参数修改，不要写死在 Python 节点中。

## 模型、地图和世界

- `src/robot/urdf/robot.xacro`：机器人模型总入口。
- `src/robot/urdf/head.xacro`：两自由度头部和双目相机安装结构。
- `src/robot/urdf/hardware.xacro`：ros2_control 硬件接口。
- `src/robot/meshes/`：车体、轮子、头部和传感器网格。
- `src/robot/map/studyroom.*`：默认二维地图和三维 PLY 点云。
- `src/robot/map/obstacle_test.*`：避障测试地图。
- `src/robot/map/blank.*`：基础运动链调试使用的空白地图。
- `src/robot/world/`：仿真世界文件。

二维地图的 `.yaml` 和 `.pgm` 必须配套；需要三维局部观察或虚拟超声波时，还要提供同一
坐标系下的 `.ply`。

## 构建与容器

RK3588 的 Debian 12 主机通过 Ubuntu 24.04 ARM64 容器运行 ROS 2 Jazzy。

首次安装 Docker：

```bash
sudo bash scripts/install_jazzy_docker.sh
```

构建包含 Nav2、RTAB-Map 和双目依赖的镜像：

```bash
bash scripts/build_jazzy_image.sh
```

RTAB-Map 会安装 PCL、VTK 等较大依赖，构建前建议至少预留约 8 GB。

日常构建并启动默认虚拟机器人：

```bash
bash scripts/run_robot_jazzy.sh
```

只启动容器，不自动构建和运行 launch：

```bash
bash scripts/run_jazzy_container.sh
docker exec -it robot-jazzy bash
```

进入容器后手动构建：

```bash
colcon build --packages-up-to robot --symlink-install
source install/setup.bash
```

双目标定入口：

```bash
bash scripts/run_stereo_calibration.sh preview
bash scripts/run_stereo_calibration.sh calibrate 0.030
```

相机配置和标定的详细步骤见 `docs/stereo_calibration_guide.md`。

## 测试

```bash
colcon build --packages-up-to robot --symlink-install
source install/setup.bash
python3 -m pytest -q \
  src/robot/test/test_mapping_snapshot.py \
  src/robot/test/test_stereo_processing.py
```

新增节点后，需要同时：

1. 放入正确的功能分层目录。
2. 在 `src/robot/setup.py` 的 `console_scripts` 中声明运行入口。
3. 将可调参数放入 `src/robot/config/` 或 launch 参数。
4. 更新 README 和当天的 `progress.md`。
