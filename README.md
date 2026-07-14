# Robot

机器人小车 ROS 2 仿真与导航工作区。当前主启动入口是：

```bash
ros2 launch robot robot.launch.py
```

本项目目标是按现实小车模型搭建 ROS 2 数字孪生，逐步还原车体结构、传感器、底盘运动控制、反馈状态、地图和 Nav2 路径规划能力。改底盘、轮子、传感器、地图和导航参数时，应优先考虑是否仍然和现实小车一致。

## 当前导航链路

Nav2 只负责全局/局部规划、速度生成和平滑以及 Nav2 自身恢复；自定义节点不再执行倒车恢复或原地旋转恢复。超声波只作为底盘前最后一层近距离安全过滤，不再接入 Nav2 collision monitor。

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

默认使用 `studyroom.yaml` 作为 2D 地图，同时启动 `studyroom.ply` 点云、点云障碍物过滤、虚拟超声波、地形分析、`range_to_scan` 和 `obstacle_avoidance`。`/nav/obstacle_points` 进入 Nav2 local costmap，`/ultrasonic/*` 只进入 `obstacle_avoidance`。

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
- `range_to_scan`：把 8 路超声波转换为稀疏 `/scan`，仅用于调试可视化或兼容 LaserScan 显示。

只在 `use_pointcloud_map:=true` 且 `enable_ultrasonic_avoidance:=true` 时启动：

- `virtual_ultrasonic`：基于 `/perception/points` 和 TF 发布 8 路 `/ultrasonic/*`。
- `obstacle_avoidance`：根据 `/ultrasonic/*` 和 `/terrain_status` 把 `/cmd_vel_nav` 过滤成 `/cmd_vel_safe`。

## 参数文件

`src/robot/config/nav2_params.yaml` 只配置当前实际启动的 Nav2 节点。它包含 planner、controller、local/global costmap、behavior、BT navigator、waypoint follower 和 velocity smoother 参数，不包含 `collision_monitor`、`route_server`、`docking_server`。

`src/robot/config/terrain_params.yaml` 只保留项目运行时需要外部调整的参数：底盘尺寸/模式、地形阈值、点云过滤、虚拟超声波、安全距离、速度超时和 `nav_controller_node` 的目标状态门控参数。

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
