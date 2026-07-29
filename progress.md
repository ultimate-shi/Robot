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
