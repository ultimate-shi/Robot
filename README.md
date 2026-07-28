# Robot

机器人小车 ROS 2 仿真与导航工作区。当前主启动入口是：

```bash
ros2 launch robot robot.launch.py
```

本项目目标是按现实小车模型搭建 ROS 2 数字孪生，逐步还原车体结构、传感器、底盘运动控制、反馈状态、地图和 Nav2 路径规划能力。改底盘、轮子、传感器、地图和导航参数时，应优先考虑是否仍然和现实小车一致。

## 当前导航链路

Nav2 负责全局/局部规划、速度生成和平滑以及 Nav2 自身恢复。超声波一方面通过 `range_to_scan` 生成稀疏 `/scan` 并进入 Nav2 local costmap，另一方面仍作为底盘前最后一层近距离安全过滤；当前安全层在前方持续低于 stop 距离且后方安全时，会结合最近前进命令或 Nav2 目标刚活跃的状态，或目标失败后一段独立脱困窗口内的状态，短暂低速后退，避免小车一直顶在障碍前。

启用超声波避障时：

```text
Nav2 controller/behavior
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel_nav_smoothed
  -> nav_controller_node
  -> /cmd_vel_nav
  -> obstacle_avoidance
  -> /cmd_vel_safe
  -> chassis_controller_node
```

关闭超声波避障时：

```text
Nav2 controller/behavior
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel_nav_smoothed
  -> nav_controller_node
  -> /cmd_vel_nav
  -> chassis_controller_node
```

`robot.launch.py` 直接启动本项目需要的 Nav2 节点：`controller_server`、`planner_server`、`smoother_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` 和 `lifecycle_manager_navigation`。不再启动 `collision_monitor`、`route_server`、`docking_server` 和 `reverse_node`。

## 启动模式

默认点云地图模式：

```bash
ros2 launch robot robot.launch.py
```

默认使用 `studyroom.yaml` 作为 2D 地图，同时启动 `studyroom.ply` 点云、点云障碍物过滤、虚拟超声波、地形分析、`range_to_scan` 和 `obstacle_avoidance`。`/nav/obstacle_points` 和 `/scan` 都进入 Nav2 local costmap，`/ultrasonic/*` 同时进入 `range_to_scan` 和 `obstacle_avoidance`。

关闭超声波最终安全层：

```bash
ros2 launch robot robot.launch.py enable_ultrasonic_avoidance:=false
```

此时仍可使用点云地图和 Nav2 costmap，但不启动虚拟超声波和 `obstacle_avoidance`，底盘直接订阅 `/cmd_vel_nav`。

空白 2D 调试地图：

```bash
ros2 launch robot robot.launch.py \
  use_pointcloud_map:=false \
  map_yaml_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot/map/blank.yaml
```

空白模式用于回退确认 `/map`、TF、`/odom`、Nav2 和底盘链路能让小车动起来；不启动 PLY 点云、点云过滤、虚拟超声波、地形分析、`range_to_scan` 和 `obstacle_avoidance`。

## 启动参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `initial_x` | `0.0` | 设置静态 TF `map -> odom` 的 x 平移，单位 m。 |
| `initial_y` | `0.0` | 设置静态 TF `map -> odom` 的 y 平移，单位 m。 |
| `initial_yaw` | `0.0` | 设置静态 TF `map -> odom` 的 yaw，单位 rad。 |
| `use_pointcloud_map` | `true` | `true` 启动 PLY 点云、点云障碍过滤、地形和调试扫描链路；`false` 关闭这些点云相关节点。 |
| `enable_ultrasonic_avoidance` | `true` | `true` 在点云模式下启动虚拟超声波和最终安全过滤；`false` 底盘直接执行 `/cmd_vel_nav`。 |
| `nav2_params_file` | `install/robot/share/robot/config/nav2_params.yaml` | 传给本项目显式启动的 Nav2 节点。 |
| `map_yaml_file` | `install/robot/share/robot/map/studyroom.yaml` | 传给 `map_server` 的 2D 地图 YAML。 |
| `ply_file` | `install/robot/share/robot/map/studyroom.ply` | 点云模式下传给 `publish_ply` 的 PLY 文件。 |
| `log_level` | `warn` | 传给本 launch 启动节点的 ROS 日志级别。 |

## 构建和环境

```bash
cd /home/shijiahao/Downloads/ros2/robot_ws
colcon build --packages-select robot
source install/setup.bash
ros2 launch robot robot.launch.py
```

每次新开终端都需要重新 `source install/setup.bash`。Foxglove Bridge 默认监听：

```text
ws://<虚拟机IP>:8765
```

## 节点职责

两种模式都会启动：

- `robot_state_publisher`：发布 `/robot_description`，根据 URDF 发布机器人模型相关 TF。
- `map_server`：发布 `/map`。
- `static_transform_publisher`：发布 `map -> odom`，使用 `initial_x`、`initial_y`、`initial_yaw`。
- `ros2_control_node` 和 controller spawner：加载转向、轮速、腿部和 joint state 控制器。
- `chassis_feedback_node`：从 `/joint_states` 提取轮子转角和轮速，发布 `/wheel_states`。
- `nav_controller_node`：订阅 `/cmd_vel_nav_smoothed` 和 Nav2 action status；速度新鲜时发布 `/cmd_vel_nav`，目标成功、失败、取消或速度超时时发布零速；默认不依赖 action status 持续放行，并在 `/nav_controller/status` 输出 JSON 状态。
- `chassis_controller_node`：接收 `/cmd_vel_nav` 或 `/cmd_vel_safe`，输出转向和轮速命令，发布 `/odom` 和 `odom -> base_link` TF。
- `virtual_imu`：根据 `/odom` 发布 `/imu/data`。
- Nav2 精简导航栈：提供 `/navigate_to_pose` 和 `/navigate_through_poses`。
- `foxglove_bridge`：给 Foxglove 前端连接。

只在 `use_pointcloud_map:=true` 时启动：

- `publish_ply`：读取 PLY，发布 `/pointcloud` 和 `/perception/points`。
- `pointcloud_obstacle_filter`：过滤 `/perception/points`，发布 `/nav/obstacle_points` 给 Nav2 local costmap。
- `terrain_analyzer`：基于点云和 `/odom` 发布 `/terrain_status`。
- `range_to_scan`：把 8 路超声波转换为稀疏 `/scan`，供 Nav2 local costmap 的 `ultrasonic_scan_layer` 使用，也可用于 Foxglove/RViz 调试。

只在 `use_pointcloud_map:=true` 且 `enable_ultrasonic_avoidance:=true` 时启动：

- `virtual_ultrasonic`：基于 `/perception/points` 和每个超声波传感器自身 TF 朝向发布 8 路 `/ultrasonic/*`。
- `obstacle_avoidance`：根据未超时的 `/ultrasonic/*` 和 `/terrain_status` 把 `/cmd_vel_nav` 过滤成 `/cmd_vel_safe`，检测到障碍时在后台打印具体传感器和距离，并发布 `/obstacle_avoidance/status`；前方持续被挡且后方安全时，会在最近有前进命令或 Nav2 目标刚活跃，或目标失败后的 `nav_goal_escape_timeout` 窗口仍有效时，按参数短暂发布低速后退命令脱离障碍。

## 参数文件

`src/robot/config/nav2_params.yaml` 只配置当前实际启动的 Nav2 节点。它包含 planner、controller、local/global costmap、behavior、BT navigator、waypoint follower 和 velocity smoother 参数，不包含 `collision_monitor`、`route_server`、`docking_server`。local costmap 同时订阅 `/nav/obstacle_points` 和稀疏 `/scan`，global costmap 叠加 `/nav/obstacle_points` 供 planner 绕开 PLY 障碍。local/global `inflation_radius` 应大于车体内切半径；当前按 `robot_radius=0.25m` 设置为 `0.35m`，避免只规划中心线贴障碍而让车体边缘擦碰。RPP 碰撞预测开启后可根据局部障碍触发停止、恢复或重新规划。

`src/robot/config/terrain_params.yaml` 只保留项目运行时需要外部调整的参数：底盘尺寸/模式、地形阈值、点云过滤、虚拟超声波、安全距离、超声波过期时间、前方阻挡脱困后退、速度超时、`obstacle_avoidance` 的 Nav2 活跃目标和失败后脱困窗口和 `nav_controller_node` 的目标状态门控参数。

## 调试话题

检查 Nav2 到底盘的速度链路：

```bash
ros2 topic echo /cmd_vel_nav_raw --once
ros2 topic echo /cmd_vel_nav_smoothed --once
ros2 topic echo /cmd_vel_nav --once
```

启用超声波避障时继续检查：

```bash
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /obstacle_warning --once
ros2 topic echo /obstacle_avoidance/status --once
ros2 topic echo /scan --once
```

检查 `nav_controller_node` 是否因为 goal 终态或速度超时切断速度：

```bash
ros2 topic echo /nav_controller/status
ros2 topic echo /navigate_to_pose/_action/status --once
ros2 topic echo /navigate_through_poses/_action/status --once
```

手动底盘速度测试建议发布到 `/cmd_vel_nav`，这样启用超声波避障时仍会经过最终安全层：

```bash
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

## 验证命令

```bash
python3 -m py_compile src/robot/robot/*.py
python3 -c "import yaml; yaml.safe_load(open('src/robot/config/nav2_params.yaml')); yaml.safe_load(open('src/robot/config/terrain_params.yaml'))"
colcon build --packages-select robot
```

启动检查：

```bash
ros2 launch robot robot.launch.py use_pointcloud_map:=true enable_ultrasonic_avoidance:=false
```

确认节点列表里没有 `collision_monitor`、`route_server`、`docking_server`、`reverse_node`，并确认 `/cmd_vel_nav_raw -> /cmd_vel_nav_smoothed -> /cmd_vel_nav` 单向存在。

## 双目实机独立版

双目实机链路与现有 PLY/虚拟超声波链路完全分开。启动关系如下：

```text
robot.launch.py
  ├─ common_bringup.launch.py（URDF/TF、地图、底盘、Nav2、Foxglove）
  └─ PLY、虚拟地形、虚拟 IMU、虚拟超声波、/scan

stereo_robot.launch.py
  ├─ common_bringup.launch.py
  └─ stereo_camera.launch.py、双目点云过滤、/stereo/scan
```

`stereo_robot.launch.py` 不启动 PLY、虚拟地形、虚拟 IMU或虚拟超声波。实机 Nav2 通过
`nav2_stereo_overrides.yaml` 只订阅 `/nav/stereo_obstacle_points`；`/stereo/scan`
只用于 Foxglove 显示、诊断和将来的 2D 回退，不重复写入 costmap。

### RK3588 依赖与设备准备

ROCK 5B+ 应直接运行 Debian 12 和源码版 ROS 2 Jazzy，不要经 VMware 转接相机。先确认
相机实际输出模式：

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

默认 profile 请求横向拼接的 `1280x480 MJPEG @ 30 FPS`，设备名为
`/dev/stereo_camera`。应根据 USB VID、PID 和序列号编写 udev 规则创建该稳定符号链接；
不要把会随插拔变化的 `/dev/videoN` 写进节点代码。

源码版 Jazzy underlay 需要提供：

```text
usb_cam  image_pipeline(image_proc/stereo_image_proc)
camera_calibration  cv_bridge  image_transport
compressed_image_transport  pointcloud_to_laserscan
```

Debian 12 是 Tier 3 平台。依赖在板端完成实际构建验证后，使用 `vcs export --exact`
导出 `.repos` 并提交精确 SHA；不要提交未经板端构建的浮动分支或猜测 SHA。

### 标定与相机单独启动

第一次仅启动取流和左右拆分：

```bash
ros2 launch robot stereo_camera.launch.py calibration_mode:=true
```

遮挡左右镜头，确认 `/stereo/left/image_raw` 和 `/stereo/right/image_raw` 对应正确；
若相反，修改 `stereo_camera.yaml` 的 `left_first`。用 8x6 内角点、30 mm 方格的刚性
标定板运行 `camera_calibration`，把结果保存到：

```text
src/robot/config/cameras/<型号_序列号_分辨率>/left.yaml
src/robot/config/cameras/<型号_序列号_分辨率>/right.yaml
```

仓库的 `_template_640x480` 只有字段模板，不是有效标定。正常模式会拒绝零焦距或右投影
矩阵中没有 Tx 的模板，禁止用 URDF 中的标称 65 mm 基线计算深度。标定完成后重新构建，
并显式传入当前相机文件：

```bash
ros2 launch robot stereo_camera.launch.py \
  calibration_mode:=false \
  left_calibration_file:=/绝对路径/left.yaml \
  right_calibration_file:=/绝对路径/right.yaml
```

该入口使用 `image_proc` 校正左右图，使用标准 `stereo_image_proc` 发布视差和原始点云。
`stereo_depth_node` 发布米制 `32FC1` 深度以及 8 位预览；无效或超范围深度为 NaN。
左右原图和深度预览默认另发 compressed transport，适合跨网络预览。

### RK3588 完整实机启动

先把 `robot.xacro` 中 `base_link -> stereo_camera_link` 的初始 `xyz/rpy` 改为实测安装
位姿，再运行：

```bash
ros2 launch robot stereo_robot.launch.py \
  left_calibration_file:=/绝对路径/left.yaml \
  right_calibration_file:=/绝对路径/right.yaml
```

Mac 不运行相机或 ROS 计算节点，只连接：

```text
ws://<RK3588_IP>:8765
```

Foxglove 优先显示左右 compressed 图像、`/stereo/depth/image_visual/compressed`、
`/nav/stereo_obstacle_points` 和 `/stereo/scan`，不要跨网络持续显示完整
`/stereo/points2`。

固定公共话题契约：

| 话题 | 类型/用途 |
| --- | --- |
| `/stereo/left/image_raw`、`/stereo/right/image_raw` | 同一采集时间戳的左右原图 |
| `/stereo/left/camera_info`、`/stereo/right/camera_info` | 当前序列号与分辨率的标定 |
| `/stereo/disparity` | `stereo_msgs/DisparityImage` |
| `/stereo/depth/image` | 米制 `32FC1` 深度，非法值为 NaN |
| `/stereo/depth/image_visual` | Foxglove 8 位深度预览 |
| `/stereo/points2` | 标准立体处理生成的完整点云 |
| `/nav/stereo_obstacle_points` | `base_link` 下过滤和降采样后的 Nav2 点云 |
| `/stereo/scan` | 独立诊断 LaserScan，不覆盖仿真 `/scan` |
| `/stereo/pointcloud_filter/status` | FPS、延迟、点数、TF 错误和丢帧 JSON |

更换同类拼接 UVC 相机时只新增相机 profile、标定文件并更新安装 TF。双独立 UVC
相机只替换取流/同步适配层；自带深度或点云的相机跳过 splitter/立体匹配并适配上述公共
话题。Nav2、底盘和 `common_bringup.launch.py` 不随相机型号变化。

### 双目验收

- 极线误差小于 1 px，优先达到 RMS 0.5 px 以下。
- 在 0.5、1、2、3 m 测深；0.5～2 m 中值误差不超过 5%，3 m 不超过 10%。
- 深度和点云稳定输出至少 15 Hz，端到端延迟 P95 小于 200 ms。
- 连续运行 30 分钟无 USB 重置、内存持续增长和热降频，RK3588 平均总 CPU 不超过
  75%；超限时依次降低处理帧率、视差范围和分辨率。
- 0.05～0.80 m 高、0.25～4.0 m 范围内的障碍应进入过滤点云和 local costmap，移除
  后约 1 秒清除；`/stereo/scan` 的距离、角度和 frame 应与点云一致。
