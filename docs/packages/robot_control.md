<!-- 作用：说明 robot_control 的底盘控制、里程计和安全链职责。 -->
<!-- 使用方法：修改运动学、控制器、速度门控、避障、轮动作、头部或车身平衡前先阅读本文件。 -->
# robot_control 功能包

## 职责与边界

`robot_control` 把任务和 Nav2 的运动意图转换为安全的四轮转向与轮速命令，负责控制器编排、轮式里程计、速度门控、超声波/地形最终过滤、命令超时保护、定距运动、建图头部归中和主动腿平衡。

本包与 `robot_hardware` 一起归 `control_hardware_owner` 管理。任何新速度来源都必须经过现有安全链，不能直接旁路到底盘控制器。

## 节点、launch 和配置

- 节点：`chassis_controller_node`、`chassis_feedback_node`、`nav_controller_node`、`obstacle_avoidance`、`four_wheel_odometry_node`、`wheel_command_guard_node`、`wheel_motion_server_node`、`head_mapping_lock_node`、`body_balance_node`。
- `controllers.launch.py`：启动 ros2_control 和关节控制器。
- `chassis_control.launch.py`：反馈、命令保护和底盘运动学。
- `safety.launch.py`：组合导航速度门控与最终避障。
- 其余 launch 分别负责轮里程计、轮定距 Action、头部建图锁和车身平衡；参数放在同名 YAML 中。

## 关键链路

```text
/cmd_vel_nav_smoothed -> /cmd_vel_nav -> /cmd_vel_safe
-> chassis_controller -> /wheel_controller/commands_raw
-> wheel_command_guard -> /wheel_controller/commands -> ros2_control
```

- `/joint_states` 经 `four_wheel_odometry` 生成 `/wheel/odom` 和 `/wheel/odometry_status`，供 EKF 使用，不发布 TF。
- `chassis_controller` 仅在兼容模式显式启用时发布 `/odom` 和 `odom -> base_link`；完整双目链由 EKF 独占该输出。
- `/execute_wheel_motion` 执行先转向后定距；实机 T 服务缺失时拒绝回退到轮速驱动，取消/超时通过 `/hardware/stop_encoder_motion` 停车。`/body_balance/set` 和 `/body_balance/status` 控制并报告主动腿平衡。
- `chassis_controller` 收到新转向目标后等待 `/joint_states` 角度到位，反馈缺失或过期时维持四轮零速。
- `wheel_command_guard` 可按每轮正反向的实测曲线反查轮速命令；默认禁用，数据缺失或不单调时拒绝启用，零速与命令超时仍直接输出零。
  当前实机有三轮低速零响应和左前启动过峰值电流，曲线报告不可用于补偿；软件电流门槛无法替代驱动器的硬件限流。
- 最终安全层消费 8 路 `/ultrasonic/*` 和 `/perception/terrain_state`。

## 验证

```bash
colcon build --symlink-install --packages-up-to robot_control
python3 -m pytest -q src/robot_control/test
source install/setup.bash
ros2 launch robot_control safety.launch.py --show-args
ros2 launch robot_control controllers.launch.py --show-args
git diff --check
```

当前代码还直接使用 `numpy`、`rcl_interfaces`，包清单未显式声明；`nav2_collision_monitor` 已声明但本包 launch 未直接启动，后续整理依赖时应核对。
