<!-- 作用：说明 robot_control 的底盘控制、里程计和安全链职责。 -->
<!-- 使用方法：修改运动学、控制器、速度门控、避障、轮动作、头部或车身平衡前先阅读本文件。 -->
# robot_control 功能包

## 职责与边界

`robot_control` 把任务和 Nav2 的运动意图转换为四轮转向与轮速命令，负责控制器编排、轮式里程计、速度门控、超声波/地形最终过滤、命令超时保护、定距运动、建图头部归中和主动腿平衡。整车入口由 `robot_web` 控制台仲裁导航与人工速度，并提供软件停止锁。

本包与 `robot_hardware` 共同组成控制和实机执行链。导航速度经门控和超声波安全层；当前网页手动速度由 `robot_web` 仲裁后直接进入 `/cmd_vel_safe`，仍经过本包的底盘运动学、轮速保护及硬件故障保护。新增速度来源时须明确接入整车仲裁，并按其使用场景验证安全链。

## 节点、launch 和配置

- 节点：`controller_bootstrap`、`chassis_controller_node`、`chassis_feedback_node`、`nav_controller_node`、`obstacle_avoidance`、`four_wheel_odometry_node`、`wheel_command_guard_node`、`wheel_motion_server_node`、`head_mapping_lock_node`、`body_balance_node`。
- `controllers.launch.py`：启动 ros2_control 和关节控制器。
- `controller_bootstrap`：每个管理服务调用限时，丢失应答后按真实生命周期状态继续加载、配置或激活；失败时启动流程退出，避免控制器缺席被静默忽略。
- `chassis_control.launch.py`：反馈、命令保护和底盘运动学。
- `safety.launch.py`：组合导航速度门控与最终避障。
- 其余 launch 分别负责轮里程计、轮定距 Action、头部建图锁和车身平衡；参数放在同名 YAML 中。

## 关键链路

```text
/cmd_vel_nav_smoothed -> /cmd_vel_nav -> robot_web -> /cmd_vel_console
-> 超声波安全层 -> /cmd_vel_nav_safe -> robot_web -> /cmd_vel_safe
-> chassis_controller -> /wheel_controller/commands_raw
-> wheel_command_guard -> /wheel_controller/commands -> ros2_control
```

网页手动速度由 `robot_web` 直接送入 `/cmd_vel_safe`，不经过超声波安全层。

- `/joint_states` 经 `four_wheel_odometry` 生成 `/wheel/odom` 和 `/wheel/odometry_status`，供 EKF 使用，不发布 TF。
- `chassis_controller` 仅在兼容模式显式启用时发布 `/odom` 和 `odom -> base_link`；完整双目链由 EKF 独占该输出。
- `/execute_wheel_motion` 执行先转向后定距；实机 T 服务缺失时拒绝回退到轮速驱动，取消/超时通过 `/hardware/stop_encoder_motion` 停车。`/body_balance/set` 和 `/body_balance/status` 控制并报告主动腿平衡。
- `chassis_controller` 收到新转向目标后等待连续两帧 `/joint_states` 进入允许误差，再依据实时转角误差逐步降低轮速；明显偏离、反馈缺失或过期时四轮停车，零速期间保持上一转向角。
- 实机 `controllers.yaml` 以 6 Hz 更新串口硬件与关节反馈，避免 DMC1/DMC0/SE2 事务超过 7 Hz 周期；转向反馈超过 0.4 s 仍会停车。SE2 的高频 PWM 刷新由 `robot_hardware` 独立完成。
- 最终安全层若否决复合速度中的任一运动分量，则整条命令停车，避免弧线或蟹行变成错误方向的运动。
- `wheel_command_guard` 只做超时停车与 ROS 关节命令限幅，电机方向、轴号和曲线均由 `robot_hardware` 管理。
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
