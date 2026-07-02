# Robot

机器人小车 ROS 2 仿真与导航工作区。

## 当前导航架构

本项目使用统一的点云感知接口，让仿真和现实小车复用同一套 Nav2、避障和底盘控制链路。

仿真链路：

```text
iPhone 扫描得到的 studyroom.ply
  -> publish_ply
  -> /pointcloud
  -> /perception/points
  -> pointcloud_obstacle_filter
  -> /nav/obstacle_points
  -> Nav2 local_costmap
```

现实链路建议：

```text
双目摄像头 / 深度相机
  -> PointCloud2
  -> /perception/points
  -> pointcloud_obstacle_filter
  -> /nav/obstacle_points
  -> Nav2 local_costmap
```

这样现实环境中只需要把相机点云接到 `/perception/points`，后面的点云过滤、Nav2 局部避障、超声波安全层和底盘控制可以继续复用。

## 关键话题

- `/map`：2D OccupancyGrid，用于 Nav2 全局路径规划。
- `/pointcloud`：仿真点云显示话题，主要给 Foxglove/RViz 使用。
- `/perception/points`：统一点云输入话题，仿真来自 PLY，现实来自双目摄像头。
- `/nav/obstacle_points`：过滤后的障碍物点云，供 Nav2 local costmap 使用。
- `/ultrasonic/*`：虚拟超声波距离，用于近距离安全保护。
- `/scan`：由 8 路超声波转换出的稀疏 LaserScan，保留用于兼容和调试。
- `/cmd_vel`：Nav2、Foxglove 或手动测试输入速度。
- `/cmd_vel_safe`：避障过滤后的安全速度，底盘控制器实际消费该话题。

## Nav2 感知配置

`src/robot/config/nav2_params.yaml` 中 local costmap 使用 `VoxelLayer` 直接订阅 PointCloud2：

```text
/nav/obstacle_points -> nav2_costmap_2d::VoxelLayer
```

`/map` 仍然用于 global costmap 的静态地图层。由于 `/map` 是 2D 数据，它不适合直接表达斜坡、台阶和地面高度；这些信息应通过 `/perception/points` 的点云预处理和地形判断处理。

## 启动

```bash
colcon build --packages-select robot
source install/setup.bash
ros2 launch robot foxglove3d.launch.py
```

## 手动速度测试

发布原始速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

观察安全速度和底盘输出：

```bash
ros2 topic echo /cmd_vel_safe
ros2 topic echo /wheel_controller/commands
```

停止发布 `/cmd_vel` 后，`/cmd_vel_safe` 应在 `cmd_vel_timeout` 时间后变为零速度。

## 点云避障测试

检查仿真点云是否发布：

```bash
ros2 topic hz /perception/points
ros2 topic hz /nav/obstacle_points
```

检查过滤状态：

```bash
ros2 topic echo /pointcloud_obstacle_status
```

在 Foxglove 中建议显示：

- `/map`
- `/pointcloud`
- `/perception/points`
- `/nav/obstacle_points`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/tf`
- `/cmd_vel`
- `/cmd_vel_safe`

## 超声波避障测试

查看前方距离：

```bash
ros2 topic echo /ultrasonic/front_fl
ros2 topic echo /ultrasonic/front_fr
ros2 topic echo /obstacle_warning
```

如果 `/cmd_vel` 和 `/cmd_vel_safe` 内容完全相同，通常表示当前未触发超声波或地形避障。靠近墙体时应看到 `/obstacle_warning` 出现 `FRONT_APPROACH` 或 `FRONT_WALL`，并且 `/cmd_vel_safe.linear.x` 被降低或置零。

## 设计原则

- Nav2 负责全局路径规划和点云局部避障。
- 超声波只作为近距离安全层，不作为主导航感知来源。
- 仿真和现实统一通过 `/perception/points` 接入点云。
- 不把相机设备路径、控制参数或地图路径硬编码到节点逻辑中，优先使用参数和 launch 配置。
