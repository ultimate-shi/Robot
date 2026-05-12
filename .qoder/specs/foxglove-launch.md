# 3D 地形仿真系统实现计划

## 目标

在现有 `robot` 包中新增文件，实现：小车在PLY点云地形上运动时自动计算6DOF位姿，返回真实的超声波和IMU数据，支持坡度打滑/台阶阻挡/悬崖检测/墙壁避障。Mac通过Foxglove可视化。

## 约束

- **不修改现有节点代码**（chassis_controller_node.py等保持不变）
- **可以修改** setup.py、package.xml 等全局配置
- **所有新功能通过新建文件实现**
- IMU频率 20Hz
- 新建 `foxglove3d.launch.py` 启动

## 文件变更清单

### 新建文件 (7个)
```
src/robot/
├── robot/
│   ├── terrain_heightmap.py        # 库: PLY→高度网格+查询API
│   ├── terrain_physics.py          # 库: 物理约束引擎
│   ├── chassis_controller_3d.py    # 节点: 6DOF底盘控制器(替代原版)
│   ├── virtual_imu_node.py         # 节点: 虚拟IMU 20Hz
│   └── obstacle_avoidance_node.py  # 节点: 避障控制器
├── config/
│   └── terrain_params.yaml         # 地形+IMU+避障参数
└── launch/
    └── foxglove3d.launch.py        # 新启动文件
```

### 修改文件 (1个)
```
src/robot/setup.py  → 新增3个entry_points
```

## 数据流

```
teleop_joy_node (原有，launch中remap输出到/cmd_vel_raw)
    │ /cmd_vel_raw
    ▼
obstacle_avoidance_node (新建, 20Hz)
    │ 订阅: /cmd_vel_raw, /ultrasonic/*(8路), /terrain_status
    │ 逻辑: 墙壁→停止/减速, 台阶→停, 悬崖→停, 陡坡→减速
    │ 发布: /cmd_vel(安全速度), /obstacle_warning
    ▼
chassis_controller_3d (新建, 10Hz, 替代原chassis_controller)
    │ 订阅: /cmd_vel, /wheel_states
    │ 逻辑: 原有运动学 + 地形高度查询 + 物理约束 + 6DOF积分
    │ 发布: /odom(6DOF), /tf(6DOF), /terrain_status, /steering/*, /wheel/*
    ▼
virtual_imu_node (新建, 20Hz)
    │ 订阅: /odom
    │ 逻辑: 姿态+角速度微分+重力旋转+GY25T噪声
    │ 发布: /imu/data
    ▼
foxglove_bridge → Mac Foxglove Studio
```

## 模块设计

### 1. terrain_heightmap.py (纯Python库，被节点import)

**职责**: PLY→2D高度网格，O(1)查询

**构建**:
1. plyfile读PLY → 体素降采样(3cm)
2. z直方图自动检测地面高度
3. 2cm分辨率构建XY高度网格
4. 空洞最近邻插值填充
5. 有限差分计算法向量

**API**:
```python
class TerrainHeightmap:
    __init__(ply_path, resolution=0.02, ground_tolerance=0.05)
    query_height(x, y) -> float
    query_normal(x, y) -> ndarray[3]
    query_4wheels(cx, cy, yaw, wheelbase, track) -> WheelTerrainInfo
    query_lookahead(x, y, heading, distance=0.10) -> LookaheadResult
```

`WheelTerrainInfo`: body_z, roll, pitch
- roll = atan2((z_FL+z_RL)-(z_FR+z_RR), 2*track)
- pitch = atan2((z_RL+z_RR)-(z_FL+z_FR), 2*wheelbase)
- body_z = mean(4轮z) + ground_to_base_height

`LookaheadResult`: height_diff, is_step, is_dropoff

### 2. terrain_physics.py (纯Python库)

**职责**: 物理约束判定

```python
class TerrainPhysics:
    __init__(max_grade_deg=35, step_threshold=0.03, dropoff_threshold=0.05)
    evaluate(heightmap, x, y, heading, vx) -> TerrainConstraint
```

`TerrainConstraint`: is_blocked, block_reason, slip_factor, traversability, body_z, roll, pitch

| 约束 | 条件 | 结果 |
|------|------|------|
| 陡坡阻挡 | slope > 35° | is_blocked, slip_factor=0 |
| 坡度打滑 | slope > 21° | slip_factor线性衰减(1→0) |
| 台阶阻挡 | 前方+高度差>3cm | is_blocked |
| 悬崖阻挡 | 前方-高度差>5cm | is_blocked |

### 3. chassis_controller_3d.py (ROS2节点)

**完全复刻** `chassis_controller_node.py` 全部逻辑（crab/four_ws/ackermann、最小转角优化、低通滤波、里程计积分），**叠加**:

1. 启动时加载PLY构建TerrainHeightmap
2. wheel_state_callback中，2D积分得(x,y,yaw)后:
   - 查地形 → 获取z, roll, pitch
   - 查约束 → 若blocked则回退位置并停止
   - 若slipping → 有效速度×slip_factor
3. 发布6DOF odom（position.z + 完整四元数含roll/pitch）
4. 发布6DOF TF（translation.z + 完整四元数）
5. 发布/terrain_status (JSON字符串)

**接口与原chassis_controller完全相同**，在launch中替代使用。

### 4. virtual_imu_node.py (ROS2节点, 20Hz)

订阅/odom → 发布/imu/data (sensor_msgs/Imu)

- orientation: odom四元数 + 噪声(σ=0.0087rad≈0.5°)
- angular_velocity: 姿态数值微分 + 噪声(σ=0.005rad/s)
- linear_acceleration: 速度微分 + R^T*[0,0,9.81] + 噪声(σ=0.02m/s²)
- frame_id = "imu_link"
- 协方差矩阵对角线填充

### 5. obstacle_avoidance_node.py (ROS2节点, 20Hz)

**输入**: /cmd_vel_raw, /ultrasonic/*(8路Range), /terrain_status(JSON)
**输出**: /cmd_vel(安全速度), /obstacle_warning(String)

**判断逻辑**（只在linear.x>0即前进时检查前方）:

| 检测源 | 条件 | 动作 |
|--------|------|------|
| 前方超声波(front_fl,front_fr) < 15cm | 墙壁 | linear.x=0 |
| 前方超声波 < 40cm | 接近 | linear.x按比例减速 |
| 侧方超声波(side_fl,side_rl) < 10cm | 左墙 | 禁止左转 |
| 侧方超声波(side_fr,side_rr) < 10cm | 右墙 | 禁止右转 |
| terrain_status.step_blocked | 台阶 | linear.x=0 |
| terrain_status.dropoff_blocked | 悬崖 | linear.x=0 |
| terrain_status.slip_factor<1 | 打滑 | linear.x*=slip_factor |

### 6. foxglove3d.launch.py

基于foxglove.launch.py，修改点:
- **替换** chassis_controller → chassis_controller_3d（同包，改executable名）
- **新增** virtual_imu 节点
- **新增** obstacle_avoidance 节点
- **teleop_joy** 添加 remappings=[('/cmd_vel', '/cmd_vel_raw')]
- 参数从 terrain_params.yaml 加载
- 所有新节点使用相同VENV环境变量

### 7. terrain_params.yaml

```yaml
chassis_controller_3d:
  ros__parameters:
    wheelbase: 0.4
    track: 0.2
    radius: 0.05
    motion_mode: "crab"
    terrain_check_enabled: true
    grid_resolution: 0.02
    ground_tolerance: 0.05
    ground_to_base_height: 0.15
    max_grade_deg: 35.0
    step_threshold: 0.03
    dropoff_threshold: 0.05
    look_ahead_distance: 0.10

virtual_imu:
  ros__parameters:
    publish_rate: 20.0
    orientation_noise_std: 0.0087
    angular_vel_noise_std: 0.005
    linear_accel_noise_std: 0.02
    gravity: 9.81

obstacle_avoidance:
  ros__parameters:
    front_stop_distance: 0.15
    front_warn_distance: 0.40
    side_stop_distance: 0.10
    side_warn_distance: 0.25
    terrain_traversability_min: 0.3
    update_rate: 20.0
```

### 8. setup.py 修改

新增entry_points:
```python
'chassis_controller_3d = robot.chassis_controller_3d:main',
'virtual_imu = robot.virtual_imu_node:main',
'obstacle_avoidance = robot.obstacle_avoidance_node:main',
```

## 输入/输出/效果

### 系统输入
| 输入 | 来源 | 说明 |
|------|------|------|
| PLY文件 | iPhone扫描 | 环境3D点云(自动从robot包map/目录加载) |
| 速度指令 | 手柄 或 Foxglove Teleop面板 | 发到/cmd_vel_raw |
| 轮子反馈 | ros2_control mock | /wheel_states |

### 系统输出
| 输出 | 话题 | 频率 | 说明 |
|------|------|------|------|
| 6DOF里程计 | /odom | 10Hz | x,y,z + roll,pitch,yaw |
| 6DOF坐标变换 | /tf (odom→base_link) | 10Hz | 完整空间位姿 |
| IMU数据 | /imu/data | 20Hz | 姿态+角速度+线加速度(含重力) |
| 超声波 | /ultrasonic/* | 5Hz×8路 | 基于3D位置的障碍距离 |
| 地形状态 | /terrain_status | 10Hz | JSON: 坡度/台阶/悬崖/可通行性 |
| 避障警告 | /obstacle_warning | 20Hz | 障碍类型和距离 |
| 安全速度 | /cmd_vel | 20Hz | 经避障过滤 |
| 环境点云 | /pointcloud | 2Hz | Foxglove 3D显示 |

### 用户可观察效果
1. **Foxglove 3D面板**: 机器人模型在点云环境中3D运动，上坡时倾斜
2. **遇墙**: 超声波<15cm → 自动停止（/obstacle_warning: "FRONT_WALL:0.12m"）
3. **遇台阶**: 前方高度差>3cm → 停止（"STEP_BLOCKED:0.04m"）
4. **遇悬崖**: 前方高度降>5cm → 停止（"DROPOFF_BLOCKED:0.06m"）
5. **上坡**: Z升高+pitch倾斜+IMU重力分量变化+太陡则减速
6. **IMU**: 静止平地[0,0,9.81]，斜面时重力分解到body，运动时有加速度
7. **换环境**: 替换map/目录下PLY+重启 → 自动适配

## Foxglove配置

Mac浏览器 → Foxglove Studio → 连接 `ws://虚拟机IP:8765`

### 面板配置:
1. **3D Panel**: 勾选 /pointcloud, /tf → 看到机器人+环境
2. **Teleop Panel**: topic设为 `/cmd_vel_raw` → 拖动控制小车
3. **Plot Panel (IMU)**: `/imu/data.linear_acceleration.z` 等
4. **Plot Panel (高度)**: `/odom.pose.pose.position.z`
5. **Raw Messages**: `/terrain_status`, `/obstacle_warning`
6. **超声波**: 在3D面板中Range自动显示为锥形

## 验证步骤

```bash
# 1. 编译
cd ~/ros2_ws && colcon build --packages-select robot && source install/setup.bash

# 2. 启动
ros2 launch robot foxglove3d.launch.py

# 3. 检查话题
ros2 topic list | grep -E "odom|imu|terrain|obstacle|cmd_vel"
ros2 topic hz /imu/data          # 应≈20Hz
ros2 topic echo /terrain_status  # 应看到JSON

# 4. 验证6DOF
ros2 run tf2_ros tf2_echo odom base_link  # 应有z/roll/pitch

# 5. 验证避障
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
ros2 topic echo /obstacle_warning  # 接近障碍物时出现警告
ros2 topic echo /cmd_vel --field linear.x  # 接近时被减速/停止

# 6. Foxglove
# Mac打开Foxglove → ws://VM_IP:8765 → 添加3D+Teleop+Plot面板
```

## 实现顺序

1. terrain_heightmap.py
2. terrain_physics.py
3. chassis_controller_3d.py
4. virtual_imu_node.py
5. obstacle_avoidance_node.py
6. terrain_params.yaml
7. foxglove3d.launch.py
8. 修改setup.py
9. 编译+验证

## 关键参考文件(只读复刻)
- `robot/chassis_controller_node.py` → chassis_controller_3d必须复刻全部运动学
- `robot/virtual_ultrasonic.py` → PLY加载模式参考
- `launch/foxglove.launch.py` → 新launch文件蓝本
