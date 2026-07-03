# Robot

机器人小车 ROS 2 仿真与导航工作区。当前主启动入口是：

```bash
ros2 launch robot robot.launch.py
```

本项目目标是按现实小车模型搭建 ROS 2 数字孪生，逐步还原车体结构、传感器、底盘运动控制、反馈状态、地图和 Nav2 路径规划能力。改底盘、轮子、传感器、地图和导航参数时，应优先考虑是否仍然和现实小车一致。

## 当前启动模式

`robot.launch.py` 现在支持两种模式，由 `use_pointcloud_map` 一个参数控制。

### 默认 2D 调试模式

```bash
ros2 launch robot robot.launch.py
```

等价于：

```bash
ros2 launch robot robot.launch.py use_pointcloud_map:=false
```

默认模式用于先确认 Foxglove 目标点、Nav2 和底盘控制链路能让小车动起来。它会使用空白 2D 地图 `blank.yaml`，不启动 PLY 点云、点云过滤、虚拟超声波、地形分析、`range_to_scan` 和 `obstacle_avoidance`。

默认模式速度链路：

```text
Nav2 -> /cmd_vel -> chassis_controller_node -> /steering_controller/commands
                                     -> /wheel_controller/commands
```

在这个模式下不要用 `/cmd_vel_safe` 判断底盘是否收到速度，因为安全避障节点没有启动。

### 点云地图模式

```bash
ros2 launch robot robot.launch.py \
  use_pointcloud_map:=true \
  nav2_params_file:=/home/shijiahao/Downloads/ros2/robot_ws/install/robot/share/robot/config/nav2_params.yaml
```

如果刚修改源码还没有重新 `colcon build`，可以临时使用源码路径：

```bash
ros2 launch robot robot.launch.py \
  use_pointcloud_map:=true \
  nav2_params_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot/config/nav2_params.yaml
```

点云模式会使用 `studyroom.yaml` 作为 2D 地图，同时启动 `studyroom.ply` 点云、点云障碍物过滤、虚拟超声波、地形分析、`range_to_scan` 和 `obstacle_avoidance`。

点云模式速度链路：

```text
Nav2 -> /cmd_vel -> obstacle_avoidance -> /cmd_vel_safe -> chassis_controller_node
                                                     -> /steering_controller/commands
                                                     -> /wheel_controller/commands
```

在这个模式下如果小车不动，需要同时检查 `/cmd_vel` 和 `/cmd_vel_safe`。如果 `/cmd_vel` 有速度但 `/cmd_vel_safe` 是 0，说明安全层、超声波或地形状态正在拦截速度。

## 启动参数

`robot.launch.py` 当前只声明下面 5 个启动参数。这里的“启动参数”指 `ros2 launch robot robot.launch.py 参数名:=参数值` 这种参数；节点内部 YAML 参数在后面的参数文件章节说明。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `initial_x` | `0.0` | 设置静态 TF `map -> odom` 的 x 平移，单位 m。用于把小车初始位置放到地图中的指定 x 坐标。 |
| `initial_y` | `0.0` | 设置静态 TF `map -> odom` 的 y 平移，单位 m。用于把小车初始位置放到地图中的指定 y 坐标。 |
| `initial_yaw` | `0.0` | 设置静态 TF `map -> odom` 的 yaw，单位 rad。用于设置小车在地图中的初始朝向。 |
| `use_pointcloud_map` | `false` | `false` 使用空白 2D 地图并关闭点云/超声波/地形/避障链路；`true` 使用 `studyroom.yaml` 和 `studyroom.ply`，并启动点云、超声波、地形和安全避障链路。 |
| `nav2_params_file` | `install/robot/share/robot/config/nav2_2d_params.yaml` | 传给 `nav2_bringup/navigation_launch.py` 的 Nav2 参数文件。默认文件适合空白 2D 调试；点云模式应显式传入 `nav2_params.yaml`。 |

示例：把小车初始放到地图中的 `(0.0, -0.5)`，朝向 0 rad：

```bash
ros2 launch robot robot.launch.py initial_x:=0.0 initial_y:=-0.5 initial_yaw:=0.0
```

注意：`initial_x`、`initial_y`、`initial_yaw` 只影响 `map -> odom` 平面初始位姿。高度、roll、pitch 不在静态 TF 中写死；点云模式下由 `/terrain_status` 和 `/odom` 表达。

## 构建和环境

首次运行或修改代码后：

```bash
cd /home/shijiahao/Downloads/ros2/robot_ws
colcon build --packages-select robot
source install/setup.bash
```

每次新开终端都需要执行：

```bash
cd /home/shijiahao/Downloads/ros2/robot_ws
source install/setup.bash
```

然后启动：

```bash
ros2 launch robot robot.launch.py
```

Foxglove Bridge 会监听：

```text
ws://<虚拟机IP>:8765
```

## 当前节点链路

### 两种模式都会启动

- `robot_state_publisher`：发布 `/robot_description`，根据 URDF 发布机器人模型相关 TF。
- `map_server`：发布 `/map`。默认模式读取 `blank.yaml`，点云模式读取 `studyroom.yaml`。
- `static_transform_publisher`：发布 `map -> odom`，使用 `initial_x`、`initial_y`、`initial_yaw`。
- `ros2_control_node` 和 controller spawner：加载转向、轮速、腿部和 joint state 控制器。
- `chassis_feedback_node`：从 `/joint_states` 提取轮子转角和轮速，发布 `/wheel_states`。
- `chassis_controller_node`：接收速度，输出转向和轮速命令，发布 `/odom` 和 `odom -> base_link` TF。
- `virtual_imu`：根据 `/odom` 发布 `/imu/data`。
- Nav2：由 `nav2_bringup/navigation_launch.py` 启动，提供 `/navigate_to_pose`。
- `foxglove_bridge`：给 Mac 上的 Foxglove 连接。

### 只在 `use_pointcloud_map:=true` 时启动

- `publish_ply`：读取 `studyroom.ply`，发布 `/pointcloud` 和 `/perception/points`。
- `pointcloud_obstacle_filter`：过滤 `/perception/points`，发布 `/nav/obstacle_points` 给 Nav2 local costmap。
- `virtual_ultrasonic`：基于 `/perception/points` 和 TF 发布 8 路 `/ultrasonic/*`。
- `range_to_scan`：把 8 路超声波转换为稀疏 `/scan`。
- `terrain_analyzer`：基于点云和 `/odom` 发布 `/terrain_status`。
- `obstacle_avoidance`：根据超声波和地形状态把 `/cmd_vel` 过滤成 `/cmd_vel_safe`。

## 地图和 Nav2 参数

### 空白 2D 调试地图

默认模式使用：

```text
src/robot/map/blank.yaml
src/robot/map/blank.pgm
src/robot/config/nav2_2d_params.yaml
```

`blank.yaml` 参数含义：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `image` | `blank.pgm` | OccupancyGrid 使用的图像文件。 |
| `mode` | `trinary` | 地图像素按占用、自由、未知三值解释。 |
| `resolution` | `0.05` | 每个像素代表 0.05 m。 |
| `origin` | `[-5.0, -5.0, 0.0]` | 地图左下角在 `map` 坐标系中的位置。当前空白地图范围约为 x/y `-5m` 到 `5m`。 |
| `occupied_thresh` | `0.65` | 高于该阈值的像素认为是占用。 |
| `free_thresh` | `0.196` | 低于该阈值的像素认为是自由。 |

`nav2_2d_params.yaml` 的用途：只验证 `/map`、TF、`/odom`、Nav2 控制器和底盘链路。local costmap 不订阅点云，collision monitor 的 `/scan` source 禁用，避免没有传感器输入时把速度置零。

关键 Nav2 参数：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `controller_server.controller_frequency` | `10.0` | Nav2 控制器输出速度的频率，和当前底盘 10Hz 控制链路匹配。 |
| `controller_server.costmap_update_timeout` | `2.0` | 等待 local costmap 更新的超时时间，虚拟机中放宽后可减少首帧超时 abort。 |
| `progress_checker.required_movement_radius` | `0.15` | 在允许时间内至少移动 0.15 m，否则认为没有进展。 |
| `progress_checker.movement_time_allowance` | `20.0` | progress checker 等待移动的时间，单位 s。 |
| `general_goal_checker.xy_goal_tolerance` | `0.12` | 到目标点 xy 距离小于 0.12 m 可认为到达。 |
| `general_goal_checker.yaw_goal_tolerance` | `0.25` | 到目标朝向误差小于 0.25 rad 可认为到达。 |
| `FollowPath.desired_linear_vel` | `0.12` | Regulated Pure Pursuit 期望线速度，单位 m/s。 |
| `FollowPath.rotate_to_heading_angular_vel` | `0.5` | 原地调整朝向时的角速度，单位 rad/s。 |
| `velocity_smoother.max_velocity` | `[0.18, 0.0, 0.8]` | Nav2 输出速度上限：x、y、yaw。当前 y 为 0，默认导航不输出横移。 |
| `velocity_smoother.min_velocity` | `[-0.08, 0.0, -0.8]` | Nav2 输出速度下限，允许小幅倒车和双向旋转。 |
| `collision_monitor.PolygonStop.enabled` | `false` | 默认 2D 模式关闭碰撞监控多边形停止。 |
| `collision_monitor.scan.enabled` | `false` | 默认 2D 模式关闭 `/scan` 输入。 |

### 点云地图模式

点云模式使用：

```text
src/robot/map/studyroom.yaml
src/robot/map/studyroom.pgm
src/robot/map/studyroom.ply
src/robot/config/nav2_params.yaml
```

`studyroom.yaml` 参数含义：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `image` | `studyroom.pgm` | 2D OccupancyGrid 图像。 |
| `resolution` | `0.05` | 每个像素代表 0.05 m。 |
| `origin` | `[-1.890675, -2.242989, 0.0]` | studyroom 地图原点在 `map` 坐标系中的位置。 |
| `occupied_thresh` | `0.65` | 高于该阈值的像素认为是占用。 |
| `free_thresh` | `0.196` | 低于该阈值的像素认为是自由。 |

点云数据链路：

```text
studyroom.ply -> publish_ply -> /perception/points
/perception/points -> pointcloud_obstacle_filter -> /nav/obstacle_points
/nav/obstacle_points -> Nav2 local_costmap VoxelLayer
/perception/points -> virtual_ultrasonic -> /ultrasonic/* -> range_to_scan -> /scan
/perception/points + /odom -> terrain_analyzer -> /terrain_status
```

`nav2_params.yaml` 用于真实 2D map + PLY 点云局部避障。打开 `use_pointcloud_map:=true` 时应使用它，否则点云障碍物不会按预期进入 local costmap。

## 底盘控制参数

底盘参数在 `src/robot/config/terrain_params.yaml` 的 `chassis_controller.ros__parameters` 下。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `wheelbase` | `0.4` | 前后轮轴距，单位 m。影响四轮转向和阿克曼运动学。 |
| `track` | `0.2` | 左右轮距，单位 m。影响转向半径和轮心速度计算。 |
| `radius` | `0.05` | 轮子半径，单位 m。用于线速度和轮子角速度换算。 |
| `motion_mode` | `four_ws` | 底盘运动模式。只接受 `crab`、`four_ws`、`ackermann`。拼错会被拒绝并提示有效值。 |
| `steering_limit` | `1.57` | 转向关节限幅，单位 rad，对应 URDF 中约 ±90°。超过范围时用反向轮速等价实现。 |
| `ackermann_min_turning_speed` | `0.04` | Ackermann 模式收到纯 `angular.z` 时使用的小前进速度，避免 Nav2 要求转向时完全不动。 |
| `terrain_check_enabled` | `true` | 是否让底盘控制器使用 `/terrain_status` 中的地形高度、姿态、打滑和阻挡状态。默认模式下没有地形节点时不会收到地形更新。 |
| `grid_resolution` | `0.02` | 地形高度图网格分辨率，单位 m。 |
| `ground_tolerance` | `0.05` | 地面高度容差，单位 m。 |
| `ground_to_base_height` | `0.15` | 地面到 `base_link` 的高度偏移，单位 m。 |
| `max_grade_deg` | `35.0` | 最大可通行坡度，单位 deg。 |
| `step_threshold` | `0.03` | 台阶阻挡阈值，单位 m。 |
| `dropoff_threshold` | `0.05` | 坑洼/跌落阻挡阈值，单位 m。 |
| `look_ahead_distance` | `0.10` | 地形前向检测距离，单位 m。 |
| `look_ahead_samples` | `5` | 前向检测采样点数量。 |

切换底盘模式：

```bash
ros2 param set /chassis_controller motion_mode four_ws
ros2 param set /chassis_controller motion_mode crab
ros2 param set /chassis_controller motion_mode ackermann
```

Nav2 导航推荐使用 `four_ws`。`crab` 更适合手动横移测试；`ackermann` 更接近汽车式转向，但不能真正原地旋转。

## 其他参数文件

### `virtual_imu`

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `publish_rate` | `20.0` | IMU 发布频率，单位 Hz。 |
| `orientation_noise_std` | `0.0087` | 姿态噪声标准差，约 0.5°。 |
| `angular_vel_noise_std` | `0.005` | 角速度噪声标准差，单位 rad/s。 |
| `linear_accel_noise_std` | `0.02` | 线加速度噪声标准差，单位 m/s²。 |
| `gravity` | `9.81` | 重力加速度，单位 m/s²。 |

### `obstacle_avoidance`

只在 `use_pointcloud_map:=true` 时启动。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `front_stop_distance` | `0.15` | 前方障碍小于该距离时停止前进，单位 m。 |
| `front_warn_distance` | `0.40` | 前方障碍小于该距离时减速，单位 m。 |
| `side_stop_distance` | `0.10` | 侧向障碍小于该距离时限制对应方向转向，单位 m。 |
| `side_warn_distance` | `0.25` | 侧向障碍小于该距离时降低转向速度，单位 m。 |
| `terrain_traversability_min` | `0.3` | 最低地形可通行评分，低于该值会限制运动。 |
| `update_rate` | `20.0` | 避障循环频率，单位 Hz。 |
| `cmd_vel_timeout` | `0.3` | 超过该时间没有新速度命令就输出零速度，单位 s。 |

### `pointcloud_obstacle_filter`

只在 `use_pointcloud_map:=true` 时启动。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `input_topic` | `/perception/points` | 输入点云话题。 |
| `output_topic` | `/nav/obstacle_points` | 输出障碍点云话题。 |
| `target_frame` | `map` | 输出点云目标坐标系。 |
| `robot_base_frame` | `base_link` | 机器人底盘坐标系。 |
| `min_obstacle_height` | `0.05` | 障碍物最低高度阈值，低于该高度的点会被过滤，单位 m。 |
| `max_obstacle_height` | `0.80` | 障碍物最高高度阈值，高于该高度的点会被过滤，单位 m。 |
| `local_radius` | `4.0` | 只保留机器人附近该半径内的点，单位 m。 |
| `voxel_size` | `0.05` | 点云体素降采样尺寸，单位 m。 |
| `max_points` | `20000` | 输出点云最大点数。 |

### `terrain_analyzer`

只在 `use_pointcloud_map:=true` 时启动。参数含义与底盘地形参数一致，额外包括：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `input_topic` | `/perception/points` | 输入点云话题。 |
| `publish_rate` | `10.0` | `/terrain_status` 发布频率，单位 Hz。 |
| `terrain_voxel_size` | `0.03` | 地形点云降采样尺寸，单位 m。 |
| `grid_resolution` | `0.02` | 地形高度图网格分辨率，单位 m。 |
| `ground_tolerance` | `0.05` | 地面高度容差，单位 m。 |
| `max_grade_deg` | `35.0` | 最大可通行坡度，单位 deg。 |
| `step_threshold` | `0.03` | 台阶阻挡阈值，单位 m。 |
| `dropoff_threshold` | `0.05` | 坑洼/跌落阻挡阈值，单位 m。 |
| `look_ahead_distance` | `0.10` | 地形前向检测距离，单位 m。 |
| `look_ahead_samples` | `5` | 前向检测采样点数量。 |
| `ground_to_base_height` | `0.15` | 地面到 `base_link` 的高度偏移，单位 m。 |
| `wheelbase` | `0.4` | 前后轮轴距，单位 m。 |
| `track` | `0.2` | 左右轮距，单位 m。 |

### `virtual_ultrasonic`

只在 `use_pointcloud_map:=true` 时启动。

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `input_topic` | `/perception/points` | 输入点云话题。 |
| `max_range` | `4.0` | 超声波最大量程，单位 m。 |
| `min_range` | `0.02` | 超声波最小量程，单位 m。 |
| `fov_half_deg` | `15.0` | 单个超声波视场半角，单位 deg。 |
| `min_height` | `0.03` | 参与测距的最低点云高度，单位 m。 |
| `max_height` | `0.60` | 参与测距的最高点云高度，单位 m。 |
| `voxel_size` | `0.03` | 测距前点云降采样尺寸，单位 m。 |
| `publish_period` | `0.2` | 超声波发布周期，单位 s。 |

## Nav2 目标点测试

启动后检查 Nav2 lifecycle：

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 action list -t
```

预期 `/navigate_to_pose [nav2_msgs/action/NavigateToPose]` 存在，Nav2 相关 lifecycle 节点为 `active [3]`。

发送目标点：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

默认 2D 模式检查：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /wheel_controller/commands --once
ros2 topic echo /steering_controller/commands --once
ros2 run tf2_ros tf2_echo map base_link
```

点云模式检查：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /terrain_status --once
ros2 topic echo /pointcloud_obstacle_status --once
ros2 run tf2_ros tf2_echo map base_link
```

## 手动速度测试

默认 2D 模式下直接发布 `/cmd_vel`：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

原地转向：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}" --rate 10
```

横移只对 `crab` 模式有直接意义：

```bash
ros2 param set /chassis_controller motion_mode crab
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.05}, angular: {z: 0.0}}" --rate 10
```

观察输出：

```bash
ros2 topic echo /wheel_controller/commands
ros2 topic echo /steering_controller/commands
ros2 topic echo /odom
```

点云模式下还可以观察安全速度：

```bash
ros2 topic echo /cmd_vel_safe
ros2 topic echo /obstacle_warning
```

## Foxglove 常用话题

默认 2D 模式建议显示：

- `/tf`
- `/robot_description`
- `/map`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/cmd_vel`
- `/wheel_controller/commands`
- `/steering_controller/commands`
- `/odom`
- `/imu/data`

点云模式额外显示：

- `/pointcloud`
- `/perception/points`
- `/nav/obstacle_points`
- `/ultrasonic/front_fl`
- `/ultrasonic/front_fr`
- `/ultrasonic/side_fl`
- `/ultrasonic/side_fr`
- `/scan`
- `/cmd_vel_safe`
- `/terrain_status`

## 常见问题排查

### Foxglove 拖目标点后不动

先确认 Nav2 是否真的收到 goal：

```bash
ros2 action list -t
ros2 topic echo /cmd_vel --once
```

如果 `/cmd_vel` 没有速度，问题在 Nav2、TF、地图或 goal。继续检查：

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map base_link
```

如果 `/cmd_vel` 有速度但轮子命令没有变化，检查底盘节点和模式：

```bash
ros2 param get /chassis_controller motion_mode
ros2 topic echo /wheel_controller/commands --once
ros2 topic echo /steering_controller/commands --once
```

点云模式下如果 `/cmd_vel` 有速度但 `/cmd_vel_safe` 是 0，检查安全层：

```bash
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /terrain_status --once
ros2 topic echo /ultrasonic/front_fl --once
ros2 topic echo /ultrasonic/front_fr --once
```

### 切换模式后不动

`motion_mode` 只接受准确拼写：

```text
crab
four_ws
ackermann
```

错误拼写会被拒绝。推荐在取消当前 Nav2 goal 或停车后再切换模式。导航目标点建议使用 `four_ws`。

### 轮子转角超过 90 度

`chassis_controller_node` 会把所有模式的转角限制在 `steering_limit` 内。目标角超过 ±90° 时，会用反向轮速加限幅内舵角等价实现。若 Foxglove 中仍看到超过 90°，优先确认已经重新 build 并 source 了当前工作区：

```bash
colcon build --packages-select robot
source install/setup.bash
```

## 文件清单

当前主链路依赖这些文件：

- `src/robot/launch/robot.launch.py`
- `src/robot/setup.py`
- `src/robot/config/terrain_params.yaml`
- `src/robot/config/nav2_2d_params.yaml`
- `src/robot/config/nav2_params.yaml`
- `src/robot/config/controller_manager.yaml`
- `src/robot/config/controllers.yaml`
- `src/robot/map/blank.yaml`
- `src/robot/map/blank.pgm`
- `src/robot/map/studyroom.yaml`
- `src/robot/map/studyroom.pgm`
- `src/robot/map/studyroom.ply`
- `src/robot/urdf/robot.xacro`
- `src/robot/urdf/*.xacro`
- `src/robot/meshes/*`
- `src/robot/robot/chassis_controller_node.py`
- `src/robot/robot/chassis_feedback_node.py`
- `src/robot/robot/virtual_imu_node.py`
- `src/robot/robot/publish_ply.py`
- `src/robot/robot/pointcloud_obstacle_filter.py`
- `src/robot/robot/virtual_ultrasonic.py`
- `src/robot/robot/range_to_scan_node.py`
- `src/robot/robot/terrain_analyzer_node.py`
- `src/robot/robot/terrain_heightmap.py`
- `src/robot/robot/terrain_physics.py`
- `src/robot/robot/obstacle_avoidance_node.py`

判断是否可以删除文件时，先检查它是否被 `robot.launch.py`、`setup.py`、URDF include、mesh 引用或 Python import 使用。上述文件不建议删除。
