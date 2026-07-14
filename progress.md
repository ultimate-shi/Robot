# 过程记录

## 2026-07-09

- 检查 `range_to_scan_node`、`obstacle_avoidance_node`、`nav2_params.yaml` 和 `terrain_params.yaml`，确认点云模式下 Nav2 local costmap 使用 `/nav/obstacle_points`，但 collision monitor 会订阅 `range_to_scan` 发布的 `/scan`。
- 为 `range_to_scan` 增加纯原地旋转检测：订阅 `/cmd_vel_smoothed`，当线速度接近 0 且角速度有效时继续发布 `/scan`，但 ranges 全部为 `inf`，避免超声波误触发 Nav2 collision monitor。
- 为 `obstacle_avoidance` 增加同样的纯旋转判断：只在纯旋转时跳过侧向超声波对 `angular.z` 的限速/置零，前进、倒车和横移防撞保持不变。
- 在 `terrain_params.yaml` 增加 `ignore_steering_ultrasonic_on_pure_rotation`、`rotation_linear_threshold`、`rotation_angular_threshold` 等参数，并同步更新 README 说明。
- 踩坑：本次 `apply_patch` 和部分读文件命令遇到沙箱 `bwrap: loopback: Failed RTM_NEWADDR`，改用受控提权写入完成工作区内文件修改。

## 2026-07-09 追加

- 根据实际避障需求修正自转超声波策略：Nav2 纯原地旋转时只屏蔽 `front_fl/front_fr/front_rl/front_rr` 这 4 个会随轮子转向的前/后超声波，保留 `side_fl/side_fr/side_rl/side_rr` 这 4 个侧向超声波。
- `obstacle_avoidance` 恢复侧向超声波对 `angular.z` 的限制，避免自转时侧面过近仍继续转。
- `range_to_scan` 不再发布全空 `/scan`，而是在纯旋转时跳过前/后超声波 bin，继续写入侧向超声波 bin。

## 2026-07-09 追加 2

- 在 `robot.launch.py` 增加启动参数 `enable_ultrasonic_avoidance`，默认 `true`。
- `enable_ultrasonic_avoidance:=false` 时不启动 `virtual_ultrasonic` 和 `obstacle_avoidance`，底盘控制器直接订阅 `/cmd_vel`，避免仍等待 `/cmd_vel_safe` 导致不动。
- 点云地图、点云障碍过滤和地形分析仍由 `use_pointcloud_map` 控制，不受该参数影响。
- `range_to_scan` 在点云模式下仍启动；关闭虚拟超声波后会发布空 `/scan`，避免 Nav2 collision monitor 缺少 scan 输入。

## 2026-07-09 追加 3

- 排查 Nav2 一直输出 `/cmd_vel_nav.angular.z`、`linear.x=0` 的原因：确认 `/cmd_vel_nav` 是 collision_monitor 输出，前级来自 controller_server 的 RegulatedPurePursuitController 和 velocity_smoother。
- 当前 `nav_controller_node` 对 `/cmd_vel_nav` 只做转发和卡住检测，不会把 Nav2 的线速度改成 0；若 `/cmd_vel_nav` 已经是纯角速度，应优先检查 RPP `rotate_to_heading` 分支、`odom -> base_link` yaw 反馈、collision monitor 状态和全局路径起点朝向。
- 重点参数包括 `use_rotate_to_heading`、`rotate_to_heading_min_angle`、`rotate_to_heading_angular_vel`、`yaw_goal_tolerance`、`velocity_smoother.feedback` 以及底盘 odom 中 `twist.angular.z` 和 yaw 积分是否与实际车体一致。

## 2026-07-14

- 按用户给定目标向 `/navigate_to_pose` 发送 goal，目标点为 map 下 `(0.2379, -2.0289)`，目标四元数 `z=-0.6836, w=0.7299`。
- goal 被 Nav2 接受，但最终状态为 `ABORTED`，返回 `error_code: 0`、`error_msg: ''`；运行轨迹显示小车确实从约 `(0.0, 0.0)` 前进到约 `(0.640, -0.12)`，距离目标仍约 `1.99m`，期间 `number_of_recoveries` 增加到 8。
- `/rosout` 明确显示 `RegulatedPurePursuitController detected collision ahead!`，随后 `Controller patience exceeded`、`[follow_path] ActionServer Aborting handle`、`Goal failed`。当时主要卡点是 Nav2 RPP/local costmap 碰撞预测，不是 odom yaw 方向，也不是超声波安全层。
- 用户关闭超声波避障和 RPP 碰撞检测后又发现目标结束附近仍持续直线运动；排查确认底盘节点自身有速度超时，若仍运动通常说明 `/cmd_vel` 仍被上游非零刷新。
- 在 `nav_controller_node` 增加 Nav2 action 目标活跃门控：订阅 `/navigate_to_pose/_action/status` 和 `/navigate_through_poses/_action/status`，只有 goal 处于 accepted/executing/canceling 且 status 未超时时才转发 `/cmd_vel_nav`；目标成功、失败、取消或 status 超时后清空遗留速度并发布零速。
- 在 `terrain_params.yaml` 增加 `require_active_nav_goal` 和 `nav_goal_status_timeout`，README 同步记录停止逻辑和参数含义。
- 发现默认 QoS 订阅 action status 会导致 `nav_controller_node` 约 2 秒后误判目标状态超时，表现为 `/cmd_vel_nav` 仍有速度但 `/cmd_vel` 被置零；改为 `qos_profile_action_status` 订阅，并将 `nav_goal_status_timeout` 放宽到 3 秒。
- 踩坑：ROS graph 查询偶发不稳定，部分 `ros2 node info` 或 `ros2 lifecycle get` 一度返回 `Node not found`，但进程和 action server 实际存在；抓取 topic 时仍需用 `timeout` 并在沙箱报 bwrap 错误时提权重试。

## 2026-07-14 追加：Nav 路径规划链路精简

- 按重设计方案精简导航速度链路：Nav2 controller/behavior 输出 `/cmd_vel_nav_raw`，`velocity_smoother` 输出 `/cmd_vel_nav_smoothed`，`nav_controller_node` 门控后发布 `/cmd_vel_nav`，启用超声波避障时再由 `obstacle_avoidance` 输出 `/cmd_vel_safe` 给底盘。
- `robot.launch.py` 不再 include `nav2_bringup/navigation_launch.py`，改为显式启动 `controller_server`、`planner_server`、`smoother_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` 和 lifecycle manager。
- 移除 `collision_monitor`、`route_server`、`docking_server` 和自定义倒车恢复链路；删除 `reverse_node.py` 并移除 `setup.py` 中的 console script。
- 重写 `nav_controller_node`：只订阅 Nav2 action status 和 `/cmd_vel_nav_smoothed`，目标成功、失败、取消、status 超时或速度超时时发布零速，并通过 `/nav_controller/status` 输出 JSON 状态。
- 精简 `nav2_params.yaml` 与 `terrain_params.yaml`，删除未启动节点参数、倒车恢复参数和 collision monitor 的 `/scan` 输入配置；`range_to_scan` 保留为调试可视化。
- README 已同步为新链路、启动参数、节点职责和调试话题说明。
- 踩坑：本环境中 `apply_patch` 和带重定向/写入的普通沙箱命令继续触发 `bwrap: loopback: Failed RTM_NEWADDR`，仓库内写入改用受控提权命令完成。

## 2026-07-14 追加 2：Nav 目标 2 秒后停发排查

- 复现用户反馈：发送导航目标后约 2 秒停止输出，底盘不动。排查确认问题出在 nav_controller_node 对 Nav2 action status 的严格活跃目标门控：ROS graph/action status 在当前环境偶发发现不稳定时，会被误判为 goal inactive 或 status timeout，从而提前发布零速。
- 修复策略：nav_controller_node 默认 require_active_nav_goal=false，只要求 /cmd_vel_nav_smoothed 新鲜即可转发到 /cmd_vel_nav；Nav2 action status 仍保留用于成功、失败、取消时记录日志和清零遗留速度。
- 同步更新 terrain_params.yaml 和 README，说明默认不再依赖 action status 做持续放行门控，避免 DDS/action status 抖动导致导航中途被自定义节点切断。
- 验证：python3 -m py_compile、YAML 解析和 colcon build --packages-select robot 均通过。
- 现场测试：手动启动修复版 nav_controller_node 后，发送目标 (x=1.2, y=-0.15)；小车从约 (0.736, -0.154) 移动到约 (1.104, -0.148)，Nav2 返回 SUCCEEDED，说明 /cmd_vel_nav_smoothed -> /cmd_vel_nav -> chassis_controller_node 链路恢复。
- 踩坑：当前已有 launch 中的旧 nav_controller_node 被手动杀掉后不会自动 respawn；临时前台启动节点可以验证问题，但正式使用应重新启动完整 robot.launch.py，让 launch 管理修复版节点。
