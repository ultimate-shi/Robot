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
## 2026-07-16

- 排查超声波避障链路：此前 `range_to_scan` 生成的 `/scan` 只用于调试可视化，没有进入 Nav2；`obstacle_avoidance` 只截断 `/cmd_vel_nav`，不会让 Nav2 重新规划或进入恢复行为。
- 在 `nav2_params.yaml` 中把 local costmap 插件扩展为 `voxel_layer + ultrasonic_scan_layer + inflation_layer`，让稀疏 `/scan` 通过 `nav2_costmap_2d::ObstacleLayer` 标记/清除近距离障碍；按当前测试需求，local/global `inflation_radius` 暂时保持 0.0，便于单独观察超声波效果。
- 重新启用 RPP `use_collision_detection`，并开启 `use_cost_regulated_linear_velocity_scaling`，让 Nav2 controller 能根据 local costmap 中的点云和超声波障碍提前降速、停止，并触发 Nav2 自身恢复/重规划流程。
- 调整超声波安全距离和频率：前向 stop/warn 改为 0.18/0.35m，侧向 stop/warn 改为 0.15/0.30m；虚拟超声波和 `/scan` 周期改为 0.03s；根据实测单路超声波约 5~6Hz，最终把 `range_timeout` 放宽到 0.35s。
- 根据实际结构确认超声波固定在小腿上，前向和外侧传感器的 URDF/TF 朝向原本正确；已恢复 `virtual_ultrasonic` 按传感器 link 朝向计算测距。
- `obstacle_avoidance_node` 为每路超声波增加接收时间戳，过期数据不参与避障；新增 `/obstacle_avoidance/status`，用于观察 front/rear/left/right 最小距离、过滤后的安全速度、告警以及具体触发的超声波。检测到障碍时后台日志会打印传感器名和距离。
- README 已同步说明 `/scan` 已接入 Nav2 local costmap，不再只是调试话题。
- 踩坑：当前环境中 `apply_patch` 和部分普通沙箱读写命令仍会触发 `bwrap: loopback: Failed RTM_NEWADDR`，本次仓库内修改改用受控提权命令完成。

- 现场按 obstacle_test 地图启动并发送目标 `(0.1413, -2.0306)`：Nav2 接受目标，小车移动到约 `(0.04, -0.32)` 后 RPP 报 `detected collision ahead`，随后 controller patience exceeded，恢复一次后因 costmap update timeout 失败，action 结果 `ABORTED`，`error_code=107`。
- 实测 `/scan` 约 33Hz，但单路 `/ultrasonic/front_fl` 只有约 6Hz，间隔约 0.16~0.17s；原 `range_timeout=0.15s` 会导致 `obstacle_avoidance` 把超声波读数误判为过期，状态中 front/rear/left/right 显示 null，无法打印具体障碍日志。
- 调整 `terrain_params.yaml`：`obstacle_avoidance.range_timeout` 和 `range_to_scan.range_timeout` 放宽到 0.35s；`range_to_scan.stamp_backdate` 改为 0.0，避免 `/scan` 被 local costmap 的 TF MessageFilter 判为时间戳过旧。
- 调整 `nav2_params.yaml`：`ultrasonic_scan_layer.expected_update_rate` 放宽到 0.25s，匹配虚拟超声波约 6Hz 的实测更新能力，减少误报 scan observation buffer stale。
- 最终验证：python3 -m py_compile、YAML 解析和 colcon build --packages-select robot 均通过。
- 沉淀本次导航测试流程为 Codex skill：`~/.codex/skills/ros2-nav2-navigation-test`，覆盖启动 Nav2、发送 NavigateToPose、检查话题/日志、区分 Nav2 控制器与安全过滤层影响、清理进程和记录测试结果；已通过 skill quick_validate。

## 2026-07-17

- 现象：小车遇到障碍物后能停下，但未到目标点时会一直停在障碍前；发送新目标点也无法让它脱离当前局部障碍。
- 判断：`enable_reverse_recovery:=false` 目前没有在 `robot.launch.py` 中声明或使用；真正影响卡住的是 Nav2/RPP 停速后缺少一个把车从障碍前拉开的动作，且安全过滤层此前只截断危险速度，不主动脱困。
- 修改 `obstacle_avoidance_node`：新增前方持续阻挡脱困逻辑。最近收到过前进命令、前方距离持续低于 `front_stop_distance`、后方距离大于 `rear_escape_clearance` 时，短暂发布 `escape_reverse_speed` 低速后退；后退期间状态话题 `escape_active=true`，告警包含 `ESCAPE_REVERSE_START` 或 `ESCAPE_REVERSE`。
- 新增可调参数：`escape_reverse_enabled`、`escape_trigger_time`、`escape_reverse_duration`、`escape_reverse_speed`、`escape_cooldown`、`rear_escape_clearance`、`recent_forward_timeout`，均写入 `terrain_params.yaml`。
- 踩坑：第一次实现时把脱困计时状态放在地形状态回调后会被高频 `/terrain_status` 重置，导致持续阻挡时间无法累计；已改为只在脱困逻辑自身维护状态。
- 验证：python3 -m py_compile、YAML 解析、colcon build --packages-select robot 均通过；节点级模拟确认前方 0.05m、后方 1.0m 且最近有前进命令时，会发布 `linear.x=-0.06`，`/obstacle_avoidance/status` 中 `escape_active=true`。`colcon test --packages-select robot` 仍失败，原因是仓库现有 flake8/pep257 批量风格问题（315 条），不是本次功能验证失败。

## 2026-07-20

- 按 obstacle_test 地图启动用户给定命令并发送目标 `(2.0225, 0.9154)`，Nav2 goal 被接受，小车从原点附近前进到约 `x=0.30m` 后停止，最终 action 结果为 `ABORTED`，`error_code=107`。
- 停住时 `/cmd_vel_nav` 和 `/cmd_vel_safe` 都已经是 0；`/obstacle_avoidance/status` 仍显示前方超声波约 `0.161m`、左侧约 `0.247m`，说明小车不是被安全层截断非零速度，而是 Nav2/RPP 已停止输出并进入失败流程。
- 发现上次脱困条件只依赖 `recent_forward_timeout` 内是否收到过前进速度；Nav2 先停速超过该窗口后，前方仍被挡也不会触发 `escape_reverse`，所以重新发送目标时仍可能卡在同一个障碍前。
- 修改 `obstacle_avoidance_node`：订阅 `/navigate_to_pose/_action/status` 和 `/navigate_through_poses/_action/status`，仅用于判断 Nav2 是否还有活跃目标；前方持续低于 `front_stop_distance` 时，只要最近有前进命令或 Nav2 目标仍活跃，且后方安全，就会触发短暂低速后退。
- 新增参数 `obstacle_avoidance.nav_goal_active_timeout`，并在 `/obstacle_avoidance/status` 输出 `nav_goal_active`，便于确认停速卡住时自动后退是否由 Nav2 活跃目标触发。
- 复测发现 Nav2 abort 发生得比超声波近距离读数更新更早：`nav_goal_active=true` 时前方还是 4.0m，前方变成约 0.11m 时 action 已经终态。继续调整 `nav_goal_active_timeout` 语义为“最后一次看到 Nav2 目标活跃后的脱困宽限窗口”，终态后短时间内仍允许安全层后退。
- 再次复测发现车已经停在近障碍前时，新目标可能在约 20ms 内直接 `ABORTED`，避障节点来不及看到 EXECUTING；因此把 `STATUS_ABORTED` 也纳入短时脱困候选，目标快速失败后的 `nav_goal_active_timeout` 窗口内仍可自动后退。
- 验证：`python3 -m py_compile src/robot/robot/obstacle_avoidance_node.py`、YAML 解析、`colcon build --packages-select robot` 均通过；节点级模拟确认仅收到 `STATUS_ABORTED`、前方 0.05m、后方 1.0m 且无前进速度时，会触发 `ESCAPE_REVERSE_START` 并输出 `linear.x=-0.06`。端到端复测确认当前卡住状态下新目标会极快 `ABORTED`，该现象已作为本次最终触发条件覆盖，但最终版本未再次完整跑到实车后退段。
- 踩坑：ROS CLI 在当前环境需要 `ROS_USE_SIM_TIME=0` 才能稳定发现 `/navigate_to_pose` 等 action；`apply_patch` 仍因 `bwrap: loopback: Failed RTM_NEWADDR` 失败，仓库内修改改用受控提权文本替换完成。
- 本次继续复测同一目标 `(2.0225, 0.9154)` 并增加 `/odom` 监测：小车停在约 `(0.324, 0.0)`，距离目标仍约 `1.93m`，因此不是到达目标后的正常停止。
- 监测同时显示 Nav2 action 状态为 `[6]` (`ABORTED`)，`/cmd_vel_nav=0`、`/cmd_vel_safe=0`；随后前方超声波才从 `4.0m` 逐步变为 `0.456m -> 0.217m -> 0.11m`，说明近距离超声波更新晚于 Nav2 abort。
- 停住后的 `/obstacle_avoidance/status`：`front_min=0.11m`、`rear_min=4.0m`、`left_min=0.146m`、`escape_active=false`、`nav_goal_active=false`，后方满足后退条件，但原 3 秒 Nav2 终态窗口已经过期。
- 修改 `obstacle_avoidance_node`：新增 `nav_goal_escape_timeout`，把“Nav2 失败后允许超声波触发后退”的窗口从 `nav_goal_active_timeout` 分离出来；`terrain_params.yaml` 中设置为 `30.0s`，并在状态话题增加 `nav_goal_escape_recent`。
- README 已同步 `nav_goal_escape_timeout` 的作用；本次编辑环境中 `apply_patch` 仍被 bwrap 限制，实际代码修改使用受控提权文本替换完成。
- 新版本构建后重新启动同一 launch 并发送同一目标：Nav2 仍因 `RegulatedPurePursuitController detected collision ahead`、`Controller patience exceeded` 和 costmap update timeout 返回 `ABORTED(error_code=107)`，但避障层已在 action abort 后继续保持 `nav_goal_escape_recent=true`。
- 端到端验证结果：前方超声波降到 `0.11m` 后，`/cmd_vel_safe.linear.x` 自动变为 `-0.06`、`escape_active=true`，odom 从约 `x=0.324` 后退到 `x=0.228`，随后前方距离恢复到约 `0.405m`，确认“碰到障碍物后完全不动”的问题已修复。
- 脱困后再次发送同一目标，Nav2 action 能重新接受 goal，但仍很快因当前局部路径碰撞预测失败；因此下一步重点应放在局部代价地图/RPP 规划参数、恢复行为或绕障路径可行性，而不是速度安全层卡死。
- 验证命令：`python3 -m py_compile src/robot/robot/obstacle_avoidance_node.py`、YAML 解析、`colcon build --packages-select robot` 均通过；测试结束后确认无 ROS 进程残留并清理 `__pycache__`。

- 继续按同一目标做完整到点验证，而不是只看是否触发后退：第一次复测中 action 仍为 `ABORTED(error_code=105)`，最终 odom 约 `(0.363, -0.336)`，离目标仍约 `2.08m`，确认问题未解决。
- 解析 `/nav/obstacle_points` 后发现障碍点集中在 `x=0.5..1.1, y=0.225..0.375`，而 `/plan` 中心线贴着障碍下边通过；此前为测试超声波把 local/global `inflation_radius` 设为 `0.0`，导致 planner 只保证中心点不进障碍格，没有给 `robot_radius=0.25m` 的车体边缘留空间。
- 修改 `nav2_params.yaml`：local/global costmap 的 `inflation_radius` 调整为 `0.35m`，大于当前车体内切半径，并保留全局 `global_obstacle_layer` 叠加 `/nav/obstacle_points`，让全局路径按车体半径外扩障碍。
- 重新构建并启动用户给定 launch 后验证：控制器全部 active，发送目标 `(2.0225, 0.9154)` 返回 `SUCCEEDED(error_code=0)`；监测 `/odom` 最终约 `(1.993, 0.883)`，距离目标 `0.044m`，小于 `xy_goal_tolerance=0.10m`。
- 到达后再次发送同一目标，Nav2 goal 被接受并立即 `SUCCEEDED`，未再出现第二个目标直接 `ABORTED`。
- 仍需注意：运行中 local costmap 偶发打印 `Sensor origin ... out of map bounds`，但本次不影响到点；后续若换更窄地图或更靠边起点，需要继续检查 rolling window 原点和点云 raytrace 配置。
- 本次验证命令包括 YAML 解析、`colcon build --packages-select robot`、完整 `ros2 launch`、两次 `/navigate_to_pose` goal，以及 `/odom` 距离监测；测试结束后已停止 launch 和监视进程。

- 继续排查目标 `(1.8351, 1.0269)` 半路停住：当前配置被改成 local/global `inflation_radius=0.20m`，启动时 Nav2 明确报 `inflation radius (0.200) is smaller than the computed inscribed radius (0.255)`。
- 复现结果：目标被接受，小车停在 odom 约 `(0.310, 0.711)`，离目标约 `1.557m`；停住时 `/cmd_vel_nav_raw`、`/cmd_vel_nav`、`/cmd_vel_safe` 都为 0，`/obstacle_avoidance/status` 无 warnings、`escape_active=false`，超声波侧边没有截停非零速度。
- launch 日志显示根因是 Nav2/RPP：持续 `RegulatedPurePursuitController detected collision ahead!`，随后 `Failed to make progress`，最后 `navigate_to_pose` 返回 `ABORTED`；因此这是代价地图/RPP 碰撞预测导致停速，不是超声波避障层主动停车。
- 将 `nav2_params.yaml` 的 local/global `inflation_radius` 恢复为 `0.35m`，重新构建并复测同一目标：没有再出现小于内切半径的错误，目标返回 `SUCCEEDED(error_code=0)`；最终 odom 约 `(1.784, 1.019)`，距离目标约 `0.051m`，小于 `xy_goal_tolerance=0.10m`。

## 2026-07-28

- 根据当前仓库状态做下一阶段规划：项目已经具备主 launch、URDF 模型、2D/PLY 地图、Nav2 精简导航链路、虚拟 IMU、虚拟超声波、点云障碍过滤、地形分析、底盘控制和 Foxglove 可视化基础。
- 当前优先级应从“单个目标能到达”转向“稳定复现、参数标定、场景覆盖和现实小车一致性校准”。
- 下一步建议先固定一套回归测试目标点和地图场景，记录每次导航的成功率、最终误差、恢复次数、是否触发超声波脱困、是否出现 costmap timeout 或 RPP collision ahead。
- 后续阶段建议依次推进：实车尺寸/TF/传感器姿态校准、Nav2 参数系统化调参、多场景地图验证、反馈状态和可视化面板完善、最后再对接真实硬件或硬件在环。
- 踩坑提醒：不要再把 inflation_radius 调小到小于车体内切半径；超声波频率低于 /scan 聚合频率时要保留足够 range_timeout；Nav2 action status 在当前环境偶发不稳定，底盘安全停速不能只依赖 action status 持续放行。

### 双目实机独立版实现

- 将原 `robot.launch.py` 拆为 `common_bringup.launch.py` 公共底盘/Nav2/Foxglove 层和
  仿真专属 PLY、虚拟地形、虚拟 IMU、虚拟超声波层；现有仿真参数与速度安全链保留。
- 新增 `stereo_camera.launch.py` 和 `stereo_robot.launch.py`。实机入口不启动 PLY 或
  虚拟传感器，Nav2 覆盖配置只消费 `/nav/stereo_obstacle_points`；独立
  `/stereo/scan` 不写入 costmap。
- 新增横向拼接图拆分、视差转米制深度、向量化 PointCloud2 解析/TF 变换/高度距离视野
  体素过滤三个节点；过滤状态包含 FPS、延迟、输入输出点数、TF 错误与丢帧。
- 新增相机、点云、Nav2 实机覆盖和标定模板配置；URDF 加入标称 65 mm 双目结构和待实测
  的安装位姿。模板不包含可用于测深的焦距或 Tx，正常使用必须传入实测标定。
- 静态验证通过：新增 Python/launch 可编译，YAML/XML 可解析，NumPy 结构数组解析带
  `point_step` 的 PointCloud2 测试通过，xacro 可生成 URDF，`colcon build
  --packages-select robot` 通过，三个 launch 的 `--show-args` 均能展开。
- 当前机器缺少 `usb_cam`、`image_proc`、`stereo_image_proc` 和
  `pointcloud_to_laserscan`，且没有真实双目/RK3588，因此本轮不能完成取流、标定、
  15 Hz/P95 延迟、30 分钟稳定性、CPU、深度误差和实机 Nav2 验收；板端依赖精确 SHA
  也必须等 Debian 12 实际构建后用 `vcs export --exact` 记录，不能伪造。
- 踩坑：本环境的补丁沙箱读取既有文件时持续出现
  `bwrap: loopback: Failed RTM_NEWADDR`；新增文件仍用补丁创建，既有文件仅在补丁
  重试失败后使用受控精确替换。项目原有 flake8/pep257 存量问题仍存在，本次针对新增
  文件的 flake8 也报告 import-order/docstring 风格项，功能构建不受影响，后续可统一
  做代码风格清理。
- 下一步：在 ROCK 5B+ 确认 V4L2 拼接模式并安装依赖，生成 udev 规则，实测相机安装
  `xyz/rpy`，完成 8x6/30 mm 双目标定后依次执行相机单测、深度精度测试、完整实机导航
  和 30 分钟性能稳定性验收。
- 公共栈运行回归：首次实际启动暴露 Jazzy `LifecycleNode` 必须显式传入
  `namespace`，已给 map server 补上 `namespace=''` 并重建。随后以
  `use_pointcloud_map:=false foxglove_enabled:=false` 启动成功，
  `controller_server`、`planner_server`、`bt_navigator` 均为 active，
  `/cmd_vel_nav_smoothed` 有 1 个发布者/1 个订阅者，`/odom` 有 1 个发布者，
  两个导航 action 均存在。
- 导航回归目标 `(0.2, -0.5)` 被接受；前期 planner 在当前 studyroom 静态地图上多次
  报 `Failed to create plan`，恢复后机器人移动到约 `(0.173, -0.333)`，25 秒测试
  窗口结束时距目标约 `0.171 m`，action 尚未终态，因此本次不能记为到点成功。测试
  结束后已中止 launch；该结果说明重构后的 action/速度/odom 链路实际工作，但不是完整
  的既有 PLY 导航到点回归。
- 最终补充验证：对照 Jazzy `stereo_image_proc` 官方组件接口，将 PointCloudNode 的
  `left/right/image_rect_color` 显式重映射到本链路的 `left/right/image_rect`；
  新增无硬件单测覆盖左右顺序与同时间戳、`Z=fT/d` 及非法深度 NaN、带 padding 的
  PointCloud2 向量化解析，`pytest` 结果为 3 passed。

## 2026-07-29

- 补充双目从接线到 Nav2 的完整实机操作指南，覆盖 V4L2 模式确认、udev 稳定设备名、
  左右遮挡检查、8×6 内角点/30 mm 棋盘格标定、标定 YAML 保存、真实基线检查、
  `base_link -> stereo_camera_link` 安装外参测量、深度精度和 costmap 验收。
- 明确区分两类外参：左右镜头之间的双目外参由 `camera_calibration` 写入 R/P；整个
  相机相对车体的安装外参写入 `robot.xacro` 的 `xyz/rpy`。只完成棋盘格标定仍不能
  直接用于 Nav2 障碍物高度和位置判断。
- 为 `stereo_camera.yaml`、`stereo_pointcloud.yaml`、
  `nav2_stereo_overrides.yaml` 和左右标定模板的每个参数补充中文用途、单位、约束及
  调整影响说明。
- 标定后应新增型号/序列号/单目分辨率 profile，不覆盖模板；右目真实基线按
  `-P_right[3]/P_right[0]` 检查并同步到 URDF 结构参数，禁止用标称 65 mm 覆盖标定。
- 当前没有连接真实双目和 RK3588，本次只能验证配置语法和构建；标定 RMS、极线误差、
  深度误差、15 Hz/P95 延迟和 30 分钟稳定性仍需在板端按指南记录。

## 2026-07-31

- 在 RK3588 开发板确认当前系统为 Debian 12 arm64，内存 8GB、swap 4GB，仓库
  `main` 与 `origin/main` 一致且工作区起初干净。
- 确认 ROS 2 Jazzy 官方 deb 面向 Ubuntu 24.04；为避免在仅剩约 11GB 空间的 Debian
  主机上源码编译整套 ROS/Nav2，新增基于官方 `ros:jazzy-ros-base-noble` 的 ARM64
  容器方案，安装 Nav2、ros2_control、控制器、xacro、Foxglove Bridge、NumPy 和
  SciPy 等主 launch 依赖。
- 新增 Docker 安装、镜像构建和机器人启动脚本；启动脚本使用 host 网络、挂载当前
  工作区、自动 colcon 构建并执行 `ros2 launch robot robot.launch.py`。
- 补全 `package.xml` 中 launch、Nav2、控制器、Foxglove、xacro 和 SciPy 运行依赖，
  README 增加 Debian 12/RK3588 安装、启动、Foxglove 连接及排障说明。
- 当前卡点：Codex 受控会话不是 root，`apt-get` 无权取得系统锁，且 sudo 被
  `no_new_privileges` 禁止；需要用户在开发板本机终端运行
  `sudo bash scripts/install_jazzy_docker.sh`，随后才能实际拉取镜像、构建并完成 launch
  验收。
- 踩坑：即使外部命令获得执行许可，也不会自动提升为 root；系统包安装必须由本机 sudo
  会话完成。后续镜像构建前应留意根分区仅剩约 11GB，脚本和 Dockerfile 已清理 apt
  缓存以减少占用。

## 2026-08-01

- Docker 20.10.24 已安装并运行，daemon 确认是 aarch64/overlay2；通过 ACL 让当前
  Codex 会话可访问 Docker socket。
- 首次构建 Jazzy 镜像在拉取 `ros:jazzy-ros-base-noble` 时连续两次超时。网络诊断
  确认终端可通过 `http://127.0.0.1:7897` 代理访问 Docker Registry 和鉴权接口，但
  Docker systemd 服务的 Environment 为空，因此 daemon 直连超时。
- 新增 `configure_docker_proxy.sh`，用于写入 Docker systemd HTTP/HTTPS 代理、重启
  daemon，并在 socket 重建后恢复当前用户 ACL；README 同步代理配置与自定义端口用法。
- 当前卡点：需要本机 sudo 执行 Docker daemon 代理配置。完成后继续拉取镜像、构建
  工作区并验证 `robot.launch.py`、Nav2 和 Foxglove 端口。
- 踩坑：Docker CLI 所在终端设置 `HTTP_PROXY` 不等于 Docker daemon 使用代理；镜像
  pull 阶段必须配置 daemon 的 systemd 环境。
- Docker daemon 代理生效后已成功拉取官方 Jazzy ARM64 基础镜像。首次依赖解析发现
  `navigation2`、`nav2_bringup` 和 `ros2_controllers` 元包会带入 Gazebo、RViz、Qt、
  OpenCV 和全套控制器，共 968 个新包、约 2.8GB；已主动停止中间构建，改为按
  `robot.launch.py` 和参数文件列出实际 Nav2 服务、RPP/NavFn 插件以及位置/速度控制器，
  避免在 64GB 板载存储上安装无关仿真依赖。
- 第二次构建确认精简后为 425 个新包、约 1.36GB，但 apt 构建容器仍未使用宿主机代理。
  已给 Docker 构建增加 host 网络并传入当前终端 HTTP/HTTPS 代理作为临时 build args，
  避免 `127.0.0.1` 在默认 bridge 网络中指向构建容器自身；代理不保留在最终运行环境。
- 代理对 Ubuntu HTTP 软件源仍只有约 83KB/s；验证 USTC Ubuntu Ports 与清华 ROS 2
  镜像可用后，将其设为 Dockerfile 默认软件源，并保留 `UBUNTU_MIRROR`、
  `ROS2_MIRROR` build args 供其他网络环境覆盖。
- 首次切源构建发现当前官方基础镜像使用 deb822 的 `ros2.sources` 符号链接，而不是旧版
  `ros2-latest.list`；已读取镜像内真实配置并修正 Dockerfile。
- 国内镜像把 apt 索引下载从 7 分 31 秒缩短到 19 秒；清华 ROS 2 镜像不提供源码索引，
  而基础镜像默认 `Types: deb deb-src`，因此进一步关闭容器内不需要的 ROS `deb-src`。
- 第一轮完整运行已成功构建 robot 包并启动全部主进程；地图与 Nav2 生命周期管理器均
  报告 managed nodes active，11 个 ros2_control 控制器全部激活，`/map`、`/odom`、
  `/pointcloud` 有发布者，Foxglove Bridge 在 `0.0.0.0:8765` 监听。
- 运行中发现 `publish_ply` 因缺少 `plyfile` 使用 2 点回退云，不能作为最终点云验收；
  已补充 `python3-plyfile`。同时补充 `ros2controlcli`，便于用 `ros2 control` 诊断控制器。
- 首轮停止时 `range_to_scan` 显示 exit code -2，日志为 Ctrl+C 引发的 KeyboardInterrupt，
  属于测试主动停止而非运行期崩溃；其余节点正常完成生命周期清理。
- Ubuntu 24.04 无 `python3-plyfile` apt 包；改为安装 `python3-pip` 后从清华 PyPI 镜像
  固定安装 `plyfile==1.1.3`，并在 setup.py 声明兼容范围，避免依赖缺失或版本漂移。
- 最终 Jazzy ARM64 镜像构建成功，并在开发板上实际执行
  `ros2 launch robot robot.launch.py`。`controller_server` 与 `bt_navigator` 均为
  `active [3]`，11 个 ros2_control 控制器全部为 active，`/map`、`/odom`、
  `/pointcloud` 均有发布者。
- `publish_ply` 已用 plyfile 正常读取 `studyroom.ply` 的 494,114 个点，不再进入 2 点
  回退模式；Foxglove Bridge 在 8765 端口监听，主机侧 TCP 连接成功。当前容器保持运行，
  Mac 可连接 `ws://192.168.0.115:8765`。
- 运行时仅见 ros2_control 无法启用 FIFO 实时调度的容器权限警告，以及启动初期控制器
  顺序加载锁重试、局部代价地图旧时间戳丢帧；重试后控制器全部激活，未阻塞当前功能。
- 使用方式补充：Jazzy 位于 Ubuntu 24.04 ARM64 容器中，Debian 宿主机不会直接出现
  `ros2` 命令；宿主机应运行 `bash scripts/run_robot_jazzy.sh`，或用
  `docker exec -it robot-jazzy bash` 进入已经运行的容器后再执行 ROS 2 CLI。当前
  `robot-jazzy` 容器已持续运行 11 小时。
- 当前部署结构复核：宿主机为 Debian 12 arm64，Docker 镜像 `robot-jazzy:local` 为
  Linux/arm64、约 2.27GB；容器采用 host network、host IPC，并把
  `/home/radxa/Robot` 挂载到 `/workspace`。检查时 launch 及地图、Nav2、ros2_control、
  Foxglove、PLY/地形/虚拟传感器等 24 个业务进程仍在运行。根分区 57GB，已用 46GB，
  剩余 8.7GB；内存 7.8GiB，可用约 4.9GiB。
- 记录容器方案取舍：Jazzy 官方 ARM64 二进制目标是 Ubuntu 24.04；当前 Debian 12
  宿主机为 Python 3.11/glibc 2.36，而容器为 Python 3.12/glibc 2.39，不能安全地把
  Ubuntu Noble ROS deb 直接混装到 Debian。Debian 原生可选源码编译，但依赖解析、
  编译时间、磁盘占用和后续升级维护成本更高；当前优先使用容器获得官方 ABI 环境，
  后续接真实 CAN/串口/相机时再按设备访问和实时性需求评估原生部署。
- 补充部署文件职责说明：`docker/Dockerfile.jazzy` 定义 Ubuntu 24.04/Jazzy 运行环境和
  精简依赖；`install_jazzy_docker.sh` 只负责宿主机 Docker 安装；
  `configure_docker_proxy.sh` 配置 Docker daemon 代理；`build_jazzy_image.sh` 根据
  Dockerfile 生成 `robot-jazzy:local`；`run_robot_jazzy.sh` 挂载工作区、增量构建 robot
  包并启动主 launch。四个脚本按安装、网络、构建、运行四阶段分工。
- 明确日常使用方式：Docker 安装脚本通常只在新系统执行一次；daemon 代理脚本仅在
  初次配置或代理地址变化时执行；镜像构建脚本在新板首次部署或 Dockerfile/系统依赖
  变化时执行。机器人启动后，在第二个终端通过 `docker exec -it robot-jazzy bash`
  进入容器，并加载 `/opt/ros/jazzy/setup.bash` 与 `/workspace/install/setup.bash` 后使用
  ROS 2 CLI。
- 改进容器终端体验：`run_robot_jazzy.sh` 在创建容器时把 Jazzy 和当前工作区的环境
  加载命令写入容器 root 用户的 `.bashrc`。后续执行
  `docker exec -it robot-jazzy bash` 会自动加载两个 setup 文件，可以直接使用 `ros2`；
  README 已同步说明。现有运行中容器需要重启后才应用该行为。
- 给 `scripts/` 下四个部署脚本增加顶部中文注释，明确首次安装、代理变更、镜像依赖
  变更和日常启动各自的执行时机及重复执行条件。README 新增以工作区根目录为基准的
  `obstacle_test.yaml`、`obstacle_test.ply` 和 `nav2_params.yaml` 相对路径启动示例。

## 2026-08-04

- 按 `docs/stereo_calibration_guide.md` 在 RK3588 板端完成双目标定前只读检查。确认宿主机
  为 ROCK 5B+、Debian 12、aarch64；USB 双目相机由 `uvcvideo` 识别，VID:PID 为
  `1bcf:0b15`，序列号为 `01.00.00`，视频采集节点为 `/dev/video1`（index 0），
  `/dev/video2` 是 metadata 节点，`/dev/video0` 是板载 HDMI RX。
- 相机支持 `1280x480 MJPG` 横向拼接模式，但该模式最高仅为 20 fps；仓库当前
  `stereo_camera.yaml` 配置为 30 fps，与硬件能力不符。使用目标模式连续采集 100 帧，
  实测约 19.99～20.00 fps，无序号跳变或取流错误。
- 抓取并检查一帧 1280×480 JPEG，确认图像为左右横向拼接、单目各 640×480，方向一致，
  未见明显镜像、上下翻转、撕裂或解码异常；物理左右镜头与话题的对应关系仍需用逐个
  遮挡测试确认。
- 系统已生成稳定 by-id 链接
  `/dev/v4l/by-id/usb-USB_Camera_USB_Camera_01.00.00-video-index0`，但仓库配置要求的
  `/dev/stereo_camera` 尚未创建。当前自动曝光与自动白平衡开启，标定前还需固定曝光、
  白平衡、分辨率和相机结构。
- Jazzy 容器镜像存在，但实测缺少 `usb_cam`、`image_proc`、`stereo_image_proc`、
  `camera_calibration`、`compressed_image_transport` 和 `pointcloud_to_laserscan`；
  `cv_bridge`、`image_transport` 和 `colcon` 已存在。当前启动脚本也未映射相机设备，
  终端无 GUI 显示环境，因此尚不能执行 ROS 左右遮挡检查和标定 GUI。
- 下一步：把相机依赖加入 Jazzy 镜像并重建；创建 `/dev/stereo_camera` udev 规则；将
  `1280x480` profile 帧率改为 20 fps；给标定容器映射相机和图形界面；随后启动
  `calibration_mode:=true` 完成话题帧率、尺寸、左右遮挡和图像质量检查，再使用实测格子
  尺寸执行 8×6 双目标定。
- 踩坑：Debian 宿主机本来就不直接安装 ROS 2，ROS 依赖必须在 Jazzy 容器中检查；
  UVC 设备会同时暴露采集与 metadata 节点，不能按 `/dev/videoN` 数字顺序猜测取流节点。
- 完成标定软件整改：Jazzy 镜像加入 `usb_cam`、`camera_calibration`、`image_proc`、
  `stereo_image_proc`、`image_view`、压缩图像传输、点云转激光和 `v4l-utils` 等依赖，
  新镜像已成功构建，逐包检查全部通过；`robot` 工作区在新镜像中构建通过。
- 将 `stereo_camera.yaml` 的实测 profile 改为 `1280x480 MJPEG @ 20 FPS`，并配置
  `usb_cam` 不覆盖板端固定的相机控制。新增 `configure_stereo_camera.sh`，默认固定亮度 0、
  手动曝光 166 和白平衡 4600 K；ROS 运行时回读确认这些值没有被覆盖。
- 新增当前相机专用 udev 规则和安装脚本，按 `1bcf:0b15`、序列号 `01.00.00`、index 0
  创建 `/dev/stereo_camera`。当前 Codex 会话没有 sudo 密码，规则文件已完成但系统安装
  仍需用户在 Debian 桌面终端执行一次 `sudo bash scripts/install_stereo_camera_udev.sh`。
- 新增 `run_stereo_calibration.sh` 图形启动脚本：`preview` 模式同时打开左右图像窗口用于
  遮挡检查，`calibrate <实测格子米数>` 模式启动标定 GUI；脚本自动映射 X11、相机和
  持久化 `calibration_output/`。README 和标定指南已同步桌面操作步骤。
- 容器实测 `usb_cam` 成功使用 `/dev/video0` 启动 `1280x480 MJPEG @ 20 FPS`，左右拆分
  话题均为 640×480。宿主机稳定名作为 Docker 映射来源，容器内固定为标准
  `/dev/video0`，避免 `usb_cam` 不识别非 `/dev/videoN` 设备名的问题。
- 踩坑：ROS 2 setup 脚本不能在 `set -u` 下加载，标定容器内部已改为不启用 nounset；
  `ros2 topic hz` 在板端首次订阅会受 DDS 建链与成批到达影响产生虚高/虚低，取流模式以
  V4L2、usb_cam 日志和消息时间戳综合确认。测试临时容器均已停止。
- 当前磁盘状态：根分区剩余约 6.6 GB，新 `robot-jazzy:local` 镜像约 2.35 GB；标定原始
  文件目录已加入 Git 忽略，后续仍应及时归档或清理无用采样和旧镜像。
- 短时验证强制停止 ROS 时曾在工作区生成 181 MB `core` 转储，确认属于本轮容器测试后
  已删除；标定启动脚本增加 `--ulimit core=0`，避免后续异常退出再次占用板端空间。
- VNC 实际运行预览时，原 X11 回退逻辑调用 `xhost +SI:localuser:root`，当前 VNC X Server
  不支持该授权族并返回 `BadValue`，随后 Qt 因容器未授权而无法连接 `DISPLAY=:1.0`。
  已移除 `xhost` 方案，改为从当前 `XAUTHORITY` 或 `~/.Xauthority` 提取对应显示的
  MIT-MAGIC-COOKIE，转换为 FamilyWild 临时授权文件后只读映射到容器，退出时自动删除。
- 使用当前 VNC `DISPLAY=:1.0` 实测修正版预览，两个 `image_view` 均成功连接 X Server，
  不再出现 `Client is not authorized` 或 Qt xcb 初始化失败；VNC 报告的 XKeyboard/DRI3
  警告只表示无键盘扩展和硬件渲染，软件显示不受影响。测试结束后容器与临时 cookie 已
  清理；同时把容器内清理信号改为 TERM，并增加超时后的 KILL 兜底，避免 GUI 异常退出
  时相机 launch 长时间残留。
- 为曝光和白平衡增加可选的一次性自动调节：设置 `STEREO_AUTO_TUNE=1` 后，脚本先在
  `1280x480 MJPG @ 20 FPS` 下开启自动曝光/白平衡并丢弃采集 3 秒，再读取收敛值并切回
  手动锁定；可用 `STEREO_AUTO_TUNE_SECONDS` 在 1～15 秒内调整，标定过程中不会持续漂移。
- 明确标定板尺寸判据：`--square` 是相邻内角点距离。9×7 格子产生 8×6 内角点；只要
  中间角点间距均为 30 mm，沿 9 格/7 格方向的首尾内角点跨度分别为 210/150 mm，最外侧
  两个边缘格为 27 mm 不影响角点模型，仍可使用 0.030 m；若内角点间距不均匀则重新打印。
- 补充现场采样操作顺序：在左右画面都完整识别 8×6 内角点的前提下，按中距正视、四周
  平移、近远尺度、俯仰/偏航/滚转倾斜逐组覆盖 GUI 的 X、Y、Size、Skew；每个姿态静止
  1～2 秒，采集 40～80 组差异明显的清晰双目样本。CALIBRATE 可用后停止移动并计算，
  检查校正后同名角点水平对齐，再点击 SAVE，不使用 COMMIT。
- 检查用户保存的 `calibrationdata.tar.gz`：归档可安全读取，包含 101 组 640×480 左右图、
  左右 YAML 和 `ost.txt`。矩阵字段完整有效，右目 `P[3]=-28.48706`，计算基线为
  61.145 mm；与模型原标称 65 mm 相差约 5.9%，还需用卡尺核对实际光心距离。
- 对原始样本独立复核极线：101 组中 94 组可重新检测完整 8×6 内角点，校正后垂直误差
  平均绝对值 0.254 px、RMS 0.345 px、P95 0.709 px；所有可检组的逐组 RMS 均低于
  1 px。样本覆盖画面四周、远近及多种倾角，离线几何验收通过。
- 把正式左右参数归档到
  `src/robot/config/cameras/usb_camera_01_00_00_640x480/`，并把相机单测及完整实机入口的
  默认文件从占位模板切换到该 profile。URDF 几何基线同步为 0.061145213 m，安装
  `xyz/rpy` 仍待相机装车后实测。
- 新增 `docs/stereo_calibration_acceptance_2026-08-04.md`，区分已通过的文件/极线检查与
  尚需现场完成的 0.5～3 m 深度尺度、TF、障碍、性能和 30 分钟稳定性验收，避免把离线
  标定合格误写成整车验收完成。
- Jazzy 容器重新构建 `robot` 包成功；正式 profile 已进入安装空间，xacro 展开通过。
  使用保存样本以 20 Hz 合成回放，左右 CameraInfo 的矩阵和 optical frame 正确，左右
  校正图、视差、深度和点云均有输出，证明配置读取和处理链可运行。
- 当前复查环境未暴露 `/dev/video*` 或 `/dev/stereo_camera`，因此不能在本轮把离线回放
  消息数当作真实 USB 帧率/延迟成绩；在线性能、深度尺度和 30 分钟稳定性仍列为现场项。
- `test_stereo_processing.py` 的 3 个双目功能测试全部通过；全包测试的 flake8/pep257
  仍因仓库既有的 306 条跨文件风格问题失败，属于历史 lint 债务，没有把它误报为本次
  标定功能失败，也未在本次任务中扩大范围批量改写旧节点。
