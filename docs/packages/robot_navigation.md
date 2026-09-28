<!-- 作用：说明 robot_navigation 的定位、建图、任务规划和 Nav2 职责。 -->
<!-- 使用方法：修改里程计融合、TF、RTAB-Map、地图、任务目标或 Nav2 前先阅读本文件。 -->
# robot_navigation 功能包

## 职责与边界

`robot_navigation` 负责视觉里程计与协方差保护、多源 EKF、RTAB-Map、地图快照、Nav2、人工目标、物体停靠/跟随/探索的路径预演与确认，以及离线 PLY 地图发布。本包归 `navigation_owner` 管理。

任务规划只产生预览和经确认的导航目标，实际运动交给 Nav2 和控制安全链。

## 节点、launch 和配置

- 节点：`mapping_snapshot_manager`、`publish_ply`、`goal_manager`、`mission_planner`（兼容别名 `brain_mission`）、`odometry_covariance_guard_node`。
- `stereo_odometry.launch.py`：输出 `/visual_odom_raw`，不发布 TF。
- `state_estimation.launch.py`：融合 `/visual_odom`、`/sensors/imu/data`、`/wheel/odom`，输出 `/odom` 和 `odom -> base_link`。
- `rtabmap_mapping.launch.py`：输出 `/map`、`/mapping/cloud_map` 和 `map -> odom`。
- `nav2.launch.py`：启动完整 Nav2 栈；速度经 `/cmd_vel_nav_raw`、`/cmd_vel_nav_smoothed` 进入控制包。
- `stereo_mapping.launch.py` 组合真实双目建图；`robot.launch.py`、`stereo_robot.launch.py` 是转发到 `robot_main` 的兼容入口。
- 各链路参数位于 `config/` 的同名 YAML，地图快照是运行产物，不应提交。

## 关键接口

- `/mission/plan` Action；`/mission/confirm`、`/mission/cancel`、`/mission/stop` Service；发布任务预览、`/mission/state` 和 `/mission/navigation_goal`。
- 快照服务 `/mapping/create_preview_snapshot`、`/mapping/save_snapshot`。
- Nav2 代价地图消费 `/nav/obstacle_points`、`/scan`，真实双目模式还消费语义和双目障碍点。
- 静态地图模式由静态来源提供 `map -> odom`；在线 RTAB-Map 模式必须避免同一 TF 被重复发布。

## 验证

```bash
colcon build --symlink-install --packages-up-to robot_navigation
python3 -m pytest -q src/robot_navigation/test
source install/setup.bash
ros2 launch robot_navigation state_estimation.launch.py --show-args
ros2 launch robot_navigation nav2.launch.py --show-args
ros2 launch robot_navigation stereo_mapping.launch.py --show-args
git diff --check
```

当前节点代码直接使用 `numpy`，但安装依赖未显式声明；`goal_manager` 与 `mission_planner` 存在重名任务接口，不应在未核对编排时同时启动。
