# Robot

机器人小车 ROS 2 仿真与导航工作区。当前主入口是：

```bash
ros2 launch robot robot.launch.py
```

本 README 以 `src/robot/launch/robot.launch.py` 为准，说明当前仍在使用的文件、功能链路和测试方法。后续删除文件时，应优先保留本文件列出的依赖。

## 项目目标

本项目目标是构建现实机器人小车的 ROS 2 数字孪生：

- 在 Foxglove 中显示机器人 URDF、地图、点云、TF、速度和传感器数据。
- 使用 iPhone 扫描房间得到的 PLY/PGM/YAML 地图资源进行仿真。
- 使用 Nav2 做全局路径规划和局部点云避障。
- 使用虚拟超声波作为近距离安全保护。
- 使用 3D 底盘控制器输出四轮转向和轮速，并发布 6DOF 里程计。
- 预留现实双目摄像头接入方式，使仿真和现实复用同一套 `/perception/points` 点云接口。

## 当前主启动链路

```text
robot.launch.py
  -> robot_state_publisher
  -> nav2_map_server
  -> publish_ply
  -> pointcloud_obstacle_filter
  -> virtual_ultrasonic
  -> terrain_analyzer
  -> range_to_scan
  -> ros2_control_node + controllers
  -> chassis_feedback_node
  -> obstacle_avoidance
  -> chassis_controller_node
  -> virtual_imu
  -> Nav2 navigation_launch.py
  -> foxglove_bridge
```

核心数据流：

```text
studyroom.ply
  -> /pointcloud
  -> /perception/points
  -> /nav/obstacle_points
  -> Nav2 local_costmap
```

```text
/cmd_vel
  -> obstacle_avoidance
  -> /cmd_vel_safe
  -> chassis_controller_node
  -> /steering_controller/commands + /wheel_controller/commands
```

```text
/joint_states
  -> chassis_feedback_node
  -> /wheel_states
  -> chassis_controller_node
  -> /odom + odom->base_link TF
```

## robot.launch 使用文件清单

### 启动与安装

- `src/robot/launch/robot.launch.py`
  - 当前主 launch 文件。
  - 负责组合地图、模型、点云、超声波、Nav2、底盘控制、IMU、Foxglove Bridge。
  - 删除风险：整个当前仿真入口失效。

- `src/robot/setup.py`
  - 注册 `ros2 run` / `ros2 launch` 能找到的 Python 节点入口。
  - 安装 launch、config、urdf、meshes、map 等 share 资源。
  - 删除风险：自定义节点和资源无法被 ROS 2 包系统找到。

### Python 节点

- `src/robot/robot/publish_ply.py`
  - 读取 `studyroom.ply`。
  - 发布 `/pointcloud` 给 Foxglove 显示。
  - 发布 `/perception/points` 作为仿真/现实统一点云输入。

- `src/robot/robot/pointcloud_obstacle_filter.py`
  - 订阅 `/perception/points`。
  - 按机器人附近范围、高度阈值和体素大小过滤点云。
  - 发布 `/nav/obstacle_points` 给 Nav2 local costmap。
  - 发布 `/pointcloud_obstacle_status` 便于调试过滤后的点数。

- `src/robot/robot/virtual_ultrasonic.py`
  - 订阅 `/perception/points`，结合 TF 计算 8 路虚拟超声波距离。
  - 发布 `/ultrasonic/front_*` 和 `/ultrasonic/side_*`。
  - 仿真和现实都可以复用，只要点云进入 `/perception/points`。

- `src/robot/robot/terrain_analyzer_node.py`
  - 订阅 `/perception/points` 和 `/odom`。
  - 使用地形高度图和物理约束计算坡度、台阶、坑洼、打滑。
  - 统一发布 `/terrain_status`，供底盘控制和避障使用。

- `src/robot/robot/range_to_scan_node.py`
  - 把 8 路 `/ultrasonic/*` 转成稀疏 `/scan`。
  - 当前主要用于 Foxglove/RViz 调试或兼容 LaserScan 显示。
  - Nav2 当前主要使用 `/nav/obstacle_points`，不是 `/scan`。

- `src/robot/robot/obstacle_avoidance_node.py`
  - 安全过滤层。
  - 输入 `/cmd_vel`，输出 `/cmd_vel_safe`。
  - 根据前后左右超声波、地形状态和命令超时过滤速度。
  - 不根据运动模式切换策略；只要 `linear.x/y` 或 `angular.z` 有速度分量，就执行对应方向避障。

- `src/robot/robot/chassis_controller_node.py`
  - 当前主底盘控制器。
  - 订阅 `/cmd_vel_safe` 和 `/wheel_states`。
  - 支持 `crab`、`four_ws`、`ackermann` 三种运动方式。
  - 发布轮速、转向角、`/odom`、TF、`/terrain_status`。

- `src/robot/robot/chassis_feedback_node.py`
  - 从 `/joint_states` 提取四轮转角和轮速。
  - 发布 `/wheel_states` 给 `chassis_controller_node.py` 计算里程计。

- `src/robot/robot/virtual_imu_node.py`
  - 根据 `/odom` 生成 `/imu/data`。
  - 用于仿真 IMU 数据和数字孪生传感器链路。

### Python 支撑模块

- `src/robot/robot/terrain_heightmap.py`
  - 被 `terrain_analyzer_node.py` import。
  - 把 `/perception/points` 解析出的点云数组构造成高度栅格。
  - 提供四轮高度、车体高度、roll/pitch 和前方地形查询。

- `src/robot/robot/terrain_physics.py`
  - 被 `terrain_analyzer_node.py` import。
  - 根据高度图判断坡度、台阶、坑洼、打滑和可通行性。
  - 输出结果最终进入 `/terrain_status`。

### 配置文件

- `src/robot/config/controller_manager.yaml`
  - ros2_control 控制管理器参数。
  - 设置控制管理器更新频率。

- `src/robot/config/controllers.yaml`
  - 定义 joint_state_broadcaster、四个大腿控制器、四个小腿控制器、转向控制器和轮速控制器。
  - joint 名称必须和 URDF/hardware 配置一致。

- `src/robot/config/terrain_params.yaml`
  - 给 `chassis_controller_node`、`virtual_imu`、`obstacle_avoidance`、`pointcloud_obstacle_filter`、`terrain_analyzer`、`virtual_ultrasonic` 使用。
  - 集中管理底盘尺寸、地形阈值、IMU 噪声、避障距离、点云过滤参数。

- `src/robot/config/nav2_params.yaml`
  - Nav2 参数。
  - `global_costmap` 使用 `/map`。
  - `local_costmap` 使用 VoxelLayer 订阅 `/nav/obstacle_points`。

### 地图资源

- `src/robot/map/studyroom.yaml`
  - Nav2 map_server 使用的地图描述文件。
  - 指向 `studyroom.pgm`。

- `src/robot/map/studyroom.pgm`
  - 2D 栅格地图图像。
  - 用于 `/map` 和 Nav2 全局规划。

- `src/robot/map/studyroom.ply`
  - 3D 点云地图。
  - `publish_ply.py` 用它发布 `/pointcloud`、`/perception/points`。
  - `publish_ply.py` 把它转换为 `/perception/points`，超声波和地形节点都订阅这个统一点云话题。
  - `terrain_analyzer_node.py` 通过 `/perception/points` 间接使用它计算高度和坡度。

### 机器人模型资源

- `src/robot/urdf/robot.xacro`
  - `robot.launch.py` 的 robot_description 来源。
  - 通过 xacro 展开成 URDF。

- `src/robot/urdf/*.xacro`
  - `robot.xacro` include 的车体、轮子、电机、腿、雷达、IMU、ros2_control 硬件描述。

- `src/robot/meshes/*`
  - URDF 中引用的 OBJ/MTL 可视化模型资源。
  - Foxglove 显示机器人模型时需要这些资源。

说明：之前已明确要求不修改 `src/robot/urdf/`，所以本次只在 README 中说明 URDF/mesh 用途，没有向 URDF 文件写注释。

## 当前功能点

### 1. Foxglove 前端显示

- 通过 `foxglove_bridge` 开放 `0.0.0.0:8765`。
- Mac 端 Foxglove 可连接虚拟机 ROS 2。
- 支持显示机器人模型、TF、点云、地图、速度、代价地图、传感器话题。

建议显示：

- `/robot_description`
- `/tf`
- `/map`
- `/pointcloud`
- `/perception/points`
- `/nav/obstacle_points`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/ultrasonic/*`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/odom`

### 2. 2D 地图与 Nav2 全局规划

- `nav2_map_server` 读取 `studyroom.yaml`。
- 发布 `/map`。
- Nav2 `global_costmap` 使用静态地图做全局规划。

限制：

- `/map` 是 2D OccupancyGrid，不包含高度信息。
- 斜坡、台阶、坑洼不应只依赖 `/map` 判断。

### 3. 3D 点云与 Nav2 局部避障

仿真：

```text
studyroom.ply -> publish_ply -> /perception/points
```

现实：

```text
双目摄像头 PointCloud2 -> /perception/points
```

统一处理：

```text
/perception/points -> pointcloud_obstacle_filter -> /nav/obstacle_points -> Nav2 VoxelLayer
```

这样仿真和现实可以复用同一套 Nav2 local costmap 配置。

### 4. 虚拟超声波近距离避障

- `virtual_ultrasonic.py` 根据 `/perception/points` 和 TF 发布 8 路超声波。
- `obstacle_avoidance_node.py` 用这些 Range 话题过滤 `/cmd_vel`。
- 支持前、后、左、右和朝墙转向限速。

避障输出：

```text
/cmd_vel -> obstacle_avoidance -> /cmd_vel_safe
```

### 5. 底盘三种运动模式

`chassis_controller_node.py` 支持：

- `crab`：四轮同向，支持 `linear.x` 和 `linear.y` 平移。
- `four_ws`：四轮转向，主要使用 `linear.x + angular.z`。
- `ackermann`：近似阿克曼转向，主要使用 `linear.x + angular.z`。

避障层不按模式切换；它始终按速度分量做前后左右安全过滤。

### 6. 地形高度和 6DOF 里程计

- `terrain_heightmap.py` 从 `/perception/points` 解析出的点数组建高度图。
- `terrain_physics.py` 判断坡度、台阶、坑洼和打滑。
- `terrain_analyzer_node.py` 发布 `/terrain_status`。
- `chassis_controller_node.py` 订阅 `/terrain_status` 并发布 `/odom` 和 odom->base_link TF。
- `/terrain_status` 同时被避障节点作为额外安全约束。

### 7. 虚拟 IMU

- `virtual_imu_node.py` 根据 `/odom` 生成 `/imu/data`。
- 可用于后续状态估计、前端显示或传感器融合实验。

### 8. ros2_control 控制链路

- `controller_manager.yaml` 配置控制管理器。
- `controllers.yaml` 配置关节状态广播器、转向控制器、轮速控制器和腿部位置控制器。
- `chassis_controller_node.py` 发布控制命令到：

```text
/steering_controller/commands
/wheel_controller/commands
```

## 启动

```bash
colcon build --packages-select robot
source install/setup.bash
ros2 launch robot robot.launch.py
```

## 手动速度测试

前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

左移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.05}, angular: {z: 0.0}}" --rate 10
```

右移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.05}, angular: {z: 0.0}}" --rate 10
```

观察：

```bash
ros2 topic echo /cmd_vel_safe
ros2 topic echo /obstacle_warning
ros2 topic echo /wheel_controller/commands
```

停止发布 `/cmd_vel` 后，`/cmd_vel_safe` 应在 `cmd_vel_timeout` 后变为零速度。

## 点云避障测试

```bash
ros2 topic hz /perception/points
ros2 topic hz /nav/obstacle_points
ros2 topic echo /pointcloud_obstacle_status
```

如果 `/nav/obstacle_points` 没有数据，Nav2 local costmap 就不会看到点云障碍物。

## 超声波避障测试

前方：

```bash
ros2 topic echo /ultrasonic/front_fl
ros2 topic echo /ultrasonic/front_fr
```

左侧：

```bash
ros2 topic echo /ultrasonic/side_fl
ros2 topic echo /ultrasonic/side_rl
```

右侧：

```bash
ros2 topic echo /ultrasonic/side_fr
ros2 topic echo /ultrasonic/side_rr
```

预期告警：

- `FRONT_APPROACH` / `FRONT_WALL`
- `REAR_WALL`
- `LEFT_SIDE_APPROACH` / `LEFT_SIDE_WALL`
- `RIGHT_SIDE_APPROACH` / `RIGHT_SIDE_WALL`
- `LEFT_WALL` / `RIGHT_WALL`

## 删除文件判断标准

可以优先考虑删除或归档的文件：

- 没有被 `robot.launch.py` 引用。
- 没有被 `setup.py` 中当前 launch 使用的 executable 引用。
- 没有被当前节点 import。
- 没有被 `robot.xacro` include 或 mesh 引用。
- 不是 `studyroom.yaml/pgm/ply` 这类当前地图资源。

不建议删除的文件：

- 本 README “robot.launch 使用文件清单”中列出的文件。
- URDF include 链路中的 xacro 文件。
- URDF 引用的 mesh OBJ/MTL 文件。
- 当前 Nav2、ros2_control、地形、点云、超声波和底盘控制配置。

## 设计原则

- Nav2 负责全局路径规划和点云局部避障。
- 超声波负责最后一层近距离安全保护。
- 仿真和现实统一通过 `/perception/points` 接入点云。
- `/map` 只做 2D 全局规划，不承担坡度/台阶判断。
- 控制参数、地图路径、点云过滤阈值尽量放在 YAML 或 launch 参数中，避免硬编码。
