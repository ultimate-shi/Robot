# Robot

机器人小车 ROS 2 仿真与导航工作区。当前主启动入口是：

```bash
ros2 launch robot robot.launch.py
```

本项目目标是按现实小车模型搭建 ROS 2 数字孪生，逐步还原车体结构、传感器、底盘运动控制、反馈状态、地图和 Nav2 路径规划能力。改底盘、轮子、传感器、地图和导航参数时，应优先考虑是否仍然和现实小车一致。

## 机器人头部模型

车体前方中线安装了独立的两自由度头部模型，结构定义位于
`src/robot/urdf/head.xacro`。下部 `head_yaw_joint` 绕 Z 轴控制 yaw，初始范围为
±90°；上部 `head_pitch_joint` 绕 X 轴控制 pitch，初始范围为 ±45°。两部分分别使用
`yaw_camera.obj` 和 `pitch_camera.obj`，对应 MTL 已统一设为蓝色。

双目摄像头实际安装在 `head_pitch_link` 内部，使用浅灰色 `stereo_camera.obj` 网格。
网格中的左右镜头中心与头部上半部分面罩孔重合，模型孔距为 61 mm；左右 optical frame
保留实机标定得到的 61.145213 mm 基线，并随头部 yaw、pitch 一起运动。

当前 `body -> head_yaw_link` 安装位姿是根据参考图片和 OBJ 边界设置的初始值。完成实车
装配后，应测量安装面、pitch 转轴和舵机机械限位，并同步校准 `robot.xacro` 中的安装位姿、
`head.xacro` 中的 `pitch_joint_xyz` 与关节上下限。

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

### RK3588 + Debian 12（推荐）

ROS 2 Jazzy 官方 deb 面向 Ubuntu 24.04；Debian 12 ARM64 原生安装需要源码构建整套
ROS/Nav2，耗时、占用空间大且不易复现。本开发板使用官方 Ubuntu 24.04 Jazzy ARM64
容器，容器采用 host 网络，因此 ROS 话题和 Foxglove Bridge 都直接使用开发板 IP。

首次安装 Docker（需要在开发板本机终端输入 sudo 密码）：

```bash
cd /home/radxa/Robot
sudo bash scripts/install_jazzy_docker.sh
```

安装完成后注销并重新登录，使 docker 用户组生效。然后构建 Jazzy 镜像并启动：

```bash
cd /home/radxa/Robot
bash scripts/build_jazzy_image.sh
bash scripts/run_robot_jazzy.sh
```

如果终端依靠本机代理联网，而 `docker pull` 报连接 Docker Hub 超时，需要同时给 Docker
daemon 配置代理。以下命令默认使用 `http://127.0.0.1:7897`：

```bash
sudo bash scripts/configure_docker_proxy.sh
```

代理端口不同时，将实际地址作为参数传入，例如：

```bash
sudo bash scripts/configure_docker_proxy.sh http://127.0.0.1:7890
```

镜像构建脚本会把当前终端的 `HTTP_PROXY`/`HTTPS_PROXY` 作为临时构建参数，并使用 host
网络访问宿主机回环地址上的代理；代理不会保留在最终机器人运行环境中。
Dockerfile 默认使用 USTC 的 Ubuntu Ports 镜像和清华 ROS 2 镜像，以改善国内网络下
ARM64 软件包下载速度；需要切回官方源时可通过同名 build argument 覆盖。
`plyfile` 通过清华 PyPI 镜像安装，并在 Dockerfile 中固定版本，确保 PLY 地图不是回退
测试点云。

启动脚本会挂载当前仓库、执行 `colcon build --packages-select robot --symlink-install`，
最后运行 `ros2 launch robot robot.launch.py`。launch 参数可直接追加，例如：

```bash
bash scripts/run_robot_jazzy.sh log_level:=info foxglove_port:=8765
```

检测到宿主机 `/dev/stereo_camera` 时，该脚本会自动映射为容器内 `/dev/video0`，启动日志
应显示“已映射双目相机”。Docker 不能给已经创建的容器动态补加设备；如果旧容器内没有
`/dev/video0`，必须退出旧容器后重新运行脚本。进入容器单独启动相机时执行：

```bash
ros2 launch robot stereo_camera.launch.py video_device:=/dev/video0
```

完整实机入口执行：

```bash
ros2 launch robot stereo_robot.launch.py video_device:=/dev/video0
```

如果只需要启动 Jazzy 容器，不构建项目、也不运行任何 launch，执行：

```bash
bash scripts/run_jazzy_container.sh
```

该脚本在后台创建同名的 `robot-jazzy` 容器并挂载当前仓库。进入和停止容器分别使用：

```bash
docker exec -it robot-jazzy bash
docker stop robot-jazzy
```

`run_jazzy_container.sh` 同样会在宿主机稳定设备存在时自动映射到容器 `/dev/video0`。

进入容器后会自动加载 ROS 2 Jazzy，以及已经存在时的工作区 `install/setup.bash`。如需使用
最新代码，可在容器内自行执行 `colcon build --packages-select robot --symlink-install`；脚本
本身不会隐式构建或启动节点。

使用仓库内相对路径启动障碍测试地图时，路径相对于挂载后的工作区根目录
`/workspace`（即宿主机的仓库根目录）：

```bash
bash scripts/run_robot_jazzy.sh \
  map_yaml_file:=src/robot/map/obstacle_test.yaml \
  ply_file:=src/robot/map/obstacle_test.ply \
  nav2_params_file:=src/robot/config/nav2_params.yaml
```

启动脚本会为容器内的交互式 Bash 配置 ROS 环境。机器人运行期间另开一个终端执行：

```bash
docker exec -it robot-jazzy bash
```

进入后可以直接使用 `ros2 node list`、`ros2 topic list` 等命令，不需要再次手动执行
`source /opt/ros/jazzy/setup.bash` 和 `source /workspace/install/setup.bash`。该自动加载配置
随容器创建，不修改 Debian 宿主机的 Bash 环境。

Mac 与开发板位于同一局域网时，在 Foxglove 中添加 Foxglove WebSocket 连接：

```text
ws://<RK3588开发板IP>:8765
```

可用 `hostname -I` 查看开发板 IP。若连接失败，检查两台设备网络互通、8765/TCP
未被防火墙拦截，并确认启动日志中存在 `foxglove_bridge`。

容器运行期间按 `Ctrl+C` 会停止 launch 并自动删除容器；源码、`build/`、`install/`
和 `log/` 均保留在宿主机工作区。重新拉取代码后通常直接再次运行启动脚本即可，只有
Dockerfile 中的系统依赖变化时才需要重建镜像。

### Ubuntu 24.04 原生 ROS 2 环境

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

ROCK 5B+ 使用上述 Debian 12 + Jazzy ARM64 容器，不要经 VMware 转接相机。实机双目
启动时需要在 `docker run` 中额外映射稳定后的相机设备（例如
`--device /dev/stereo_camera`）。先确认相机实际输出模式：

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

当前板端实测 profile 请求横向拼接的 `1280x480 MJPEG @ 20 FPS`，设备名为
`/dev/stereo_camera`。应根据 USB VID、PID 和序列号编写 udev 规则创建该稳定符号链接；
不要把会随插拔变化的 `/dev/videoN` 写进节点代码。

当前相机已确认 VID:PID 为 `1bcf:0b15`、序列号为 `01.00.00`，仓库提供对应规则和安装
脚本。首次准备设备时在 Debian 桌面终端执行：

```bash
cd /home/radxa/Robot
sudo bash scripts/install_stereo_camera_udev.sh
```

执行后重新插拔相机，并用 `ls -l /dev/stereo_camera` 确认稳定设备存在。规则仅把 index 0
的图像采集节点映射为稳定名称，不会误选 index 1 的 metadata 节点。

Jazzy 环境需要提供：

```text
usb_cam  image_pipeline(image_proc/stereo_image_proc)
camera_calibration  cv_bridge  image_transport
compressed_image_transport  pointcloud_to_laserscan
```

上述依赖已写入 `docker/Dockerfile.jazzy`。更新代码后先重建镜像：

```bash
cd /home/radxa/Robot
bash scripts/build_jazzy_image.sh
```

如以后改为 Debian 12 原生源码 underlay，依赖在板端完成实际构建验证后应使用
`vcs export --exact` 导出 `.repos` 并提交精确 SHA；不要提交未经板端构建的浮动分支
或猜测 SHA。

### 标定与相机单独启动

从 Debian 图形桌面打开终端（通常可按 `Ctrl+Alt+T`），确认 `echo $DISPLAY` 有输出，然后
进入工作区。第一次先启动左右图形预览：

```bash
cd /home/radxa/Robot
bash scripts/run_stereo_calibration.sh preview
```

通过 VNC 登录时 `DISPLAY` 通常类似 `:1.0`。脚本会从当前用户的 `XAUTHORITY` 或
`~/.Xauthority` 提取该 VNC 显示的临时 MIT-MAGIC-COOKIE 并传给容器，不会调用
`xhost` 放宽整个 X Server 的访问控制；退出时临时 cookie 会自动删除。

脚本会固定亮度为 0、曝光为 166、白平衡为 4600 K，把宿主机稳定设备映射为容器内固定的
`/dev/video0`，构建工作区并同时打开左右图像窗口。容器内使用标准 `/dev/videoN` 是因为
`usb_cam` 的设备发现只枚举该命名；映射来源始终是宿主机 `/dev/stereo_camera`，不会因
宿主机节点编号变化而选错设备。依次遮挡物理左右镜头，确认窗口和话题对应；若相反，修改
`stereo_camera.yaml` 的 `left_first`。如果画面过亮或过暗，可覆盖曝光与白平衡后重试：

```bash
STEREO_EXPOSURE=200 STEREO_WHITE_BALANCE=5000 \
  bash scripts/run_stereo_calibration.sh preview
```

也可以让相机先自动调整 3 秒，再读取结果并立即切回手动锁定。运行命令前先把棋盘格放在
正常标定距离、保持静止：

```bash
STEREO_AUTO_TUNE=1 bash scripts/run_stereo_calibration.sh preview
```

需要更长的收敛时间时可设置 `STEREO_AUTO_TUNE_SECONDS=5`，允许范围为 1～15 秒。自动
调节只发生在启动前，预览和标定期间仍保持手动值，不会因棋盘格移动而持续改变亮度。

曝光范围为 3～2047，白平衡范围为 2800～6500 K。确认取流、方向和左右顺序后关闭预览
窗口。用卡尺测量刚性 8×6 内角点标定板的实际内角点间距，再以米为单位启动正式标定。
例如实测 29.82 mm：

```bash
bash scripts/run_stereo_calibration.sh calibrate 0.02982
```

如果希望正式标定前也重新自动适应当前灯光，可执行：

```bash
STEREO_AUTO_TUNE=1 bash scripts/run_stereo_calibration.sh calibrate 0.02982
```

脚本会等待左右话题就绪并打开 `camera_calibration` GUI。采样覆盖充分后依次点击
`CALIBRATE` 和 `SAVE`，不点击 `COMMIT`；结果通常保存为
`/tmp/calibrationdata.tar.gz`。脚本已把容器 `/tmp` 映射到工作区，退出 GUI 后结果仍在：

```text
/home/radxa/Robot/calibration_output/calibrationdata.tar.gz
```

`calibration_output/` 已加入 Git 忽略清单，避免原始采样图和临时文件被误提交。随后按相机
型号、序列号和单目分辨率保存左右 YAML：

```text
src/robot/config/cameras/<型号_序列号_分辨率>/left.yaml
src/robot/config/cameras/<型号_序列号_分辨率>/right.yaml
```

当前 `USB Camera 01.00.00` 的正式标定已归档为：

```text
src/robot/config/cameras/usb_camera_01_00_00_640x480/left.yaml
src/robot/config/cameras/usb_camera_01_00_00_640x480/right.yaml
```

正常相机入口和完整实机入口已默认读取这组文件，因此当前相机不需要再手工传入路径。
离线复核的极线垂直误差 RMS 为 0.345 px；详细结果和仍需现场完成的深度、TF、障碍及
30 分钟稳定性项目见
[双目标定验收记录](docs/stereo_calibration_acceptance_2026-08-04.md)。

仓库的 `_template_640x480` 只有字段模板，不是有效标定。正常模式会拒绝零焦距或右投影
矩阵中没有 Tx 的模板，禁止用标称基线计算深度。标定完成后重新构建。当前已登记的相机
可直接启动：

```bash
ros2 launch robot stereo_camera.launch.py calibration_mode:=false
```

该独立入口默认同时启动监听 `0.0.0.0:8765` 的 Foxglove Bridge。相机处理启动后可直接从
另一台电脑连接 `ws://<RK3588_IP>:8765`；如端口被其他入口占用，可使用
`foxglove_enabled:=false` 关闭，或用 `foxglove_port:=<端口>` 修改端口。完整实机入口由
`common_bringup.launch.py` 统一启动 Bridge，因此包含相机入口时会自动关闭重复实例。

更换相机 profile 时仍可用 `left_calibration_file` 和 `right_calibration_file` 覆盖默认路径。

该入口使用 `image_proc` 校正左右图，使用标准 `stereo_image_proc` 发布视差和原始点云。
`stereo_depth_node` 发布米制 `32FC1` 深度以及 8 位预览；无效或超范围深度为 NaN。
左右原图和深度预览默认另发 compressed transport，适合跨网络预览。

splitter 的左右图及 CameraInfo 使用 `RELIABLE` QoS，以兼容 `image_proc` 的可靠订阅；相机
拼接图输入仍使用传感器常用的 `BEST_EFFORT`。若话题存在但校正图、视差和深度没有消息，
应先用 `ros2 topic info <话题> -v` 检查发布/订阅两端的 Reliability 是否兼容。
米制深度和8位预览同样使用 `RELIABLE`，保证 Foxglove 与压缩转发器能够稳定订阅。
Jazzy 的 `image_transport republish` 通过 `in_transport=raw`、`out_transport=compressed`
参数选择插件，并把插件实际输出 `out/compressed` 分别映射到对应图像的 `/compressed`
话题；三个转发器不会把图像重新发布回原始基话题。

### RK3588 完整实机启动

先把 `robot.xacro` 中 `base_link -> stereo_camera_link` 的初始 `xyz/rpy` 改为实测安装
位姿，再运行：

```bash
ros2 launch robot stereo_robot.launch.py
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

相机图像到 Foxglove 的数据链如下：

```text
/dev/stereo_camera
  -> usb_cam: 1280×480 RGB 拼接图 /stereo/image_raw
  -> stereo_splitter: 左右 640×480 原图和 CameraInfo
  -> image_proc: /stereo/left|right/image_rect
  -> stereo_image_proc: /stereo/disparity 和 /stereo/points2
  -> stereo_depth_node: 米制 32FC1 /stereo/depth/image
                        8 位预览 /stereo/depth/image_visual
  -> image_transport: /stereo/depth/image_visual/compressed
  -> foxglove_bridge: ws://<RK3588_IP>:8765
  -> Foxglove Image/3D 面板
```

`/stereo/depth/image` 的每个有效像素直接表示以米为单位的 Z 深度，适合尺度验收；Foxglove
Image 面板把 `Value min/max` 设为 `0.25/4.0` 后，把鼠标悬停在目标中心即可读取像素深度。
`/stereo/depth/image_visual` 只是“近亮远暗”的 mono8 网络预览，像素值不是米，不能用来
判定距离。需要三维查看时，在 3D 面板直接显示 `/stereo/points2`，或把米制深度图设为
`Depth map`、`Distance type=Z-axis`、`Depth scale=1.0` 并配对左目 CameraInfo。

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

### 完整标定操作

双目真正用于实机前还需要完成相机模式确认、udev 固定设备名、双目内参/外参、相机到
`base_link` 的安装外参以及深度与 Nav2 验收。逐条命令、标定板要求、标定后需要修改的
文件和重标定条件见：

[双目相机从接线到 Nav2 的完整操作指南](docs/stereo_calibration_guide.md)

双目相关 YAML 已为每个参数补充中文说明：

- `src/robot/config/stereo_camera.yaml`
- `src/robot/config/stereo_pointcloud.yaml`
- `src/robot/config/nav2_stereo_overrides.yaml`
- `src/robot/config/cameras/_template_640x480/left.yaml`
- `src/robot/config/cameras/_template_640x480/right.yaml`
- `src/robot/config/cameras/usb_camera_01_00_00_640x480/left.yaml`
- `src/robot/config/cameras/usb_camera_01_00_00_640x480/right.yaml`
