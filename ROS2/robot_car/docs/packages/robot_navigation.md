<!-- 作用：说明 robot_navigation 的定位、建图、任务规划和 Nav2 职责。 -->
<!-- 使用方法：修改里程计融合、TF、RTAB-Map、地图、任务目标或 Nav2 前先阅读本文件。 -->
# robot_navigation 功能包

## 职责与边界

`robot_navigation` 负责视觉里程计与协方差保护、多源 EKF、RTAB-Map、地图快照、Nav2、人工目标、物体停靠/跟随/探索的路径预演与确认，以及离线 PLY 地图发布。

实机已有地图默认联合定位：RTAB-Map 图像匹配与 AMCL 点云轮廓匹配经 `global_localization` 保守融合，冲突优先通过检查的视觉结果；统一发布 `map -> odom` 与兼容导航许可。点云在感知包选择 stereo/astra，使用独立 `/localization/scan`。`visual` 保持原模式，`amcl` 可只用二维地图。参数为 `amcl_localization.yaml`、`global_localization.yaml`，完整说明见 [联合定位](../joint_localization.md)。

任务规划只产生预览和经确认的导航目标，实际运动交给 Nav2 和控制安全链。

## 节点、launch 和配置

整车 Astra 模式使用 `rgbd_odometry.launch.py`，建图和重定位以 `camera_source:=astra` 订阅 `/camera/rgbd_image`，保留原里程计输出及联合定位接口；独立旧入口默认 stereo 保持兼容。

- 节点：`mapping_snapshot_manager`、`publish_ply`、`goal_manager`、`mission_planner`（兼容别名 `brain_mission`）、`odometry_covariance_guard_node`、`magnetic_map_initializer`、`localization_guard`、`global_localization`、`map_server_recovery`。
- `stereo_odometry.launch.py`：输出 `/visual_odom_raw`，不发布 TF。
- `state_estimation.launch.py`：实机三维 EKF 融合视觉三维位姿、IMU roll/pitch 与三轴角速度、轮侧平面速度，输出 `/odom` 和 `odom -> base_link`；视觉新轴无效协方差不得被当作高可信测量。没有独立三维传感源的虚拟模式通过 `two_d_mode_override:=true` 保持二维约束。
- `rtabmap_mapping.launch.py`：输出 `/map`、`/mapping/cloud_map` 和 `map -> odom`。
- `map_localization.launch.py`：按 visual/amcl/fused 编排定位，视觉数据库只操作临时副本；初始估计不能直接授予自主导航许可。
- `nav2.launch.py`：启动完整 Nav2 栈；实机已有地图通过 `publish_map_to_odom:=false` 让所选定位后端独占地图变换，独立调试入口仍保留静态/地磁初始化；速度经 `/cmd_vel_nav_raw`、`/cmd_vel_nav_smoothed` 进入控制包。
- `map_server_recovery.launch.py`：监督静态地图实际生命周期，通过原管理器按未配置/未激活状态执行 STARTUP/RESUME；请求超时后先重读状态，连续失败降低重试频率，状态发布 `/mapping/map_server_recovery/status`。Nav2 静态地图入口默认启用，人工维护暂停前可动态设置 enabled=false。
- `stereo_mapping.launch.py` 组合真实双目建图；`robot.launch.py`、`stereo_robot.launch.py` 是转发到 `robot_main` 的兼容入口。
- 各链路参数位于 `config/` 的同名 YAML，地图快照是运行产物，不应提交。

## 关键接口

- `/mission/plan` Action；`/mission/confirm`、`/mission/cancel`、`/mission/stop` Service；发布任务预览、`/mission/state` 和 `/mission/navigation_goal`。
- 快照服务 `/mapping/create_preview_snapshot`、`/mapping/save_snapshot`；长期保存暂停建图、读取最终栅格、请求 RTAB-Map 备份并成套输出 `map.{yaml,pgm,ply,json,db}`，含成套校验摘要。
- `/mapping/recover_snapshot`、`mapping_recovery.launch.py` 与 `mapping_recover`：建图退出后只操作数据库副本，选择最新有效原库/`.back`，离线导出并原子保存。异常退出自动触发；快照服务在所有在线服务消失后自动回退。持久活跃库默认 `/workspace/mapping/rtabmap.db`，重启不清库。静止相似帧合并、200 节点工作内存目标及按需加载降低重复积累和备份重载峰值；该目标不是进程 RSS 硬限额。操作、三维缺失标记和验证边界见 [地图异常恢复](../mapping_recovery.md)。
- `/localization/initialpose` 接收人工位置/朝向估计并转发标准 `/initialpose`；`/localization/retry` 重新等待匹配。`/localization/status` 发布状态，`/localization/ready` 发布需要持续刷新的自主运动许可。
- Nav2 代价地图消费 `/nav/obstacle_points`、`/scan`，真实双目模式还消费语义和双目障碍点。
- 已有地图 fused/amcl 由 global_localization 唯一发布 `map -> odom`；在线建图和 visual 由 RTAB-Map 发布，仿真保留静态来源。
- 数据库加载与视觉输入准备阶段显示“视觉地图加载中”，默认 120 秒仍未处理新鲜双目帧时提示检查输入；实际处理就绪后才发送排队的初始估计，并开始匹配计时。明确知道摆放位置时可设置 `initial_pose_enabled:=true` 及 `initial_x/initial_y/initial_yaw`（弧度），否则由网页设置。配准使用历史原图重提取、均匀特征和双向交叉验证，保持至少 20 个内点和三次连续确认。
- 60 秒未定位成功显示失败并保留手动恢复；已确认定位可按时间、里程、转角预算短时跟踪，持续失效撤销许可。导航中定位丢失取消任务，恢复后需重新规划确认。联合模式阈值在 `global_localization.yaml`，原视觉模式在 `map_localization.yaml`。

## 验证

```bash
colcon build --symlink-install --packages-up-to robot_navigation
python3 -m pytest -q src/robot_navigation/test
source install/setup.bash
ros2 launch robot_navigation state_estimation.launch.py --show-args
ros2 launch robot_navigation nav2.launch.py --show-args
ros2 launch robot_navigation map_localization.launch.py --show-args
ros2 launch robot_navigation stereo_mapping.launch.py --show-args
git diff --check
```

当前节点代码直接使用 `numpy`，但安装依赖未显式声明；`goal_manager` 与 `mission_planner` 存在重名任务接口，不应在未核对编排时同时启动。

合成双目集成验证（不接实车，必须使用独立 DDS 域）可运行：

```bash
ROS_DOMAIN_ID=183 RUN_RTABMAP_INTEGRATION=1 python3 -m pytest -q src/robot_navigation/test/test_rtabmap_roundtrip.py
```
