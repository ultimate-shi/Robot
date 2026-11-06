<!-- 作用：说明双目视觉与轮廓联合定位和 Astra 点云预留接口；使用方法：部署前阅读配置、启动命令和验收边界。 -->
# 视觉与 AMCL 联合定位

## 数据流与使用范围

实机加载已有地图时，`robot_main` 默认 `localization_backend:=fused`：

```text
相机图像/深度 → RGB-D/双目里程计 → 局部 EKF（另融合轮速、IMU）→ /odom、odom → base_link
相机图像/深度 → RTAB-Map 视觉地图定位 ──────────────────┐
可配置点云 → 定位专用过滤/投影 → /localization/scan → AMCL ─┴→ 全局融合 → map → odom
```

当前实机已全部切换 Astra Pro：RGB-D 提供视觉里程计和视觉地图定位，配准深度生成点云。`camera_source:=stereo` 可恢复旧双目链；部署和实测安装 TF 见 [Astra Pro 接入](astra_pro.md)。下面描述的融合、质量与断流规则对两种来源相同。

全局定位不直接改写 `/odom`，也不向局部 EKF 回灌地图位姿；`map → odom` 修正地图中的累计偏差，`odom → base_link` 保持连续。全局融合是唯一实际发送地图 TF 的节点；RTAB-Map 的 `publish_tf` 和 AMCL 的 `tf_broadcast` 均关闭。在线建图和虚拟模式保持原有编排。

`localization_backend` 可选 `fused`（联合）、`visual`（原视觉定位）、`amcl`（纯轮廓定位）。独立 `map_localization.launch.py` 为兼容既有调用默认 `visual`，整车入口默认 `fused`。联合/视觉模式要求配套数据库，纯 AMCL 可只提供二维地图。地图服务仍由整车 Nav2 入口提供；单独启动定位节点时应另行提供 `/map`、相机和 odom/TF。

## 启动与来源选择

当前 Astra 联合定位：

```bash
ros2 launch robot_main robot.launch.py mode:=real \
  map_yaml_file:=/workspace/maps/room/map.yaml \
  localization_backend:=fused camera_source:=astra
```

地图数据库默认使用 YAML 同名 `.db`，也可通过 `database_path` 指定。首次定位可在网页设置位置/朝向，或显式传入 `initial_pose_enabled:=true initial_x:=... initial_y:=... initial_yaw:=...`，人工估计本身不会放行导航。

覆盖外部 Astra 点云接口（需提供对应的真实安装 TF）：

```bash
ros2 launch robot_main robot.launch.py mode:=real \
  map_yaml_file:=/workspace/maps/room/map.yaml \
  localization_backend:=fused localization_cloud_source:=astra \
  localization_cloud_topic:=/camera/points \
  localization_sensor_frame:=astra_color_optical_frame
```

`localization_cloud_topic` 和 `localization_sensor_frame` 可覆盖任何驱动的实际名称。来源默认值：

| 来源 | PointCloud2 | 真实测距光心坐标 |
| --- | --- | --- |
| stereo | `/stereo/points2` | `stereo_left_camera_frame` |
| astra | `/camera/points` | `astra_color_optical_frame` |

整车 `camera_source:=astra` 已编排真实 Astra Pro 驱动、RGB-D 配准及点云处理；`localization_cloud_source` 仅选择 AMCL 点云来源。驱动内部光心 TF 与车身安装标定分开维护，安装坐标必须实测。两路相机不能同名发布光学坐标或相同点云话题。整车 Astra 模式的避障点云也切到 `/camera/points`，保留 `/nav/stereo_obstacle_points` 兼容输出；超声波 `/scan` 不变。

## 扫描与质量保护

- 感知包 `localization_scan.yaml` 配置独立定位投影：默认 4Hz、0.25～4m、底盘高度 0.05～0.80m、0.5°角分辨率、最大 120°前向区域；实际没有深度的区域仍为 NaN。不要把配置视场误当成相机实际视场。
- 点云按原始采集时间查询精确 TF，超过 1.5 秒或缺失历史 TF 时拒绝，不回退最新 TF。保留时间戳，适配已有 EKF 历史回放。
- 测距原点位于真实相机光心。节点首次有效处理时建立水平虚拟扫描坐标，后续监测光心移动；超过 5mm 时停用该帧，必须检查归中、安装或腿部状态后重启定位。头部在已有地图模式保持归中。
- 底盘倾斜超过 10°时不输出二维定位扫描；二维导航不覆盖大坡度和多楼层。定位扫描缺失不会伪造自由空间。
- 导航包 `amcl_localization.yaml` 使用横移运动模型，500～1500 粒子；全局节点定期请求静止更新，停车时也能继续定位确认。
- `global_localization.yaml` 要求每路独立连续三次稳定修正，检查帧、时间、有限位姿、四元数、完整平面协方差及运动连续性。视觉定位器保留至少 20 个几何内点及只在真实匹配成功时发布位姿的配置。
- AMCL 另要求至少 30 条有效扫描、30°覆盖，至少 55% 端点距地图障碍不超过 0.20m；未知区域与越界点不能成为吻合证据。

两路先按定位测量时刻减去历史 odom，转到同一地图修正空间，避免把迟到位姿与当前位姿直接平均。传播完整平面协方差；一致时以等权协方差交集保守融合，避免将同相机的相关误差当作两份独立证据。每次从原始来源估计融合，不重复积分缓存测量的置信度。

## 冲突、初始化和断流

两路修正相差超过 0.5m 或 20°时，优先使用**已经通过质量检查和连续确认**的视觉结果，并报告轮廓冲突。视觉过期或质量不合格时，AMCL 可独立接管；新收到的一帧视觉结果不能立即覆盖 AMCL。

已确认来源出现明显跳变时先撤销导航许可，重新连续确认；地图修正大幅变化后默认维持 1 秒暂停窗口，确保网页控制循环取消旧导航任务，之后需重新规划确认。人工初始估计和重试也撤销旧许可。视觉首次确认后可向 AMCL 私有初始入口播种，不回送视觉定位器、不把播种计作匹配；地图服务较晚启动或首次播种未生效时默认每 5 秒重试，收到合格 AMCL 位姿后停止。

两路均无有效匹配时只允许既有的有界 odom 跟踪，默认最多 120 秒/5m/6.28rad，先到即失效；定位输入整体中断超过 20 秒、odom 过期/协方差超限或 odom TF 失效时撤销许可。恢复输入心跳不能恢复旧确认，仍需新的有效匹配。保持标准 `/initialpose`、网页 `/localization/initialpose` 与 `/localization/retry` 入口。

| 接口 | 用途 |
| --- | --- |
| `/localization/scan` | 定位专用 LaserScan，不能用来替换超声波 `/scan` |
| `/localization/scan_status` | 点云来源、帧时间、有效射线和拒绝原因 |
| `/rtabmap/localization_pose`、`/amcl_pose` | 两路原始地图定位 |
| `/localization/fused_pose` | 新测量触发的地图位姿，保留测量时间；协方差经位姿合成保守传播，不假设与局部 odom 独立 |
| `/localization/status`、`/localization/ready` | 兼容的状态和自主导航许可；增加来源、冲突、吻合度与逐路拒绝原因 |
| `/localization/amcl_initialpose` | AMCL 私有播种入口，由全局节点转发，不直接授予许可 |

现有 9100 端口 `/robot/` 显示来源、轮廓吻合度和拒绝原因，不增加网页服务端口。

## 测试与现场验收

纯算法测试覆盖时刻对齐、转向/横移的坐标关系、协方差雅可比、相关测量不过度增信、角度跨 ±π、来源独立确认、冲突视觉优先、过期接管及未知地图拒绝。启动测试覆盖纯 AMCL 缺库可启动、联合模式要求数据库、来源参数传递和无效参数拒绝。

独立通信域的真实 AMCL 集成使用合成房间点云、0.8 秒延迟和可控视觉位姿，分别验证 stereo/astra 来源接口，并让 Astra 用例的地图服务延迟 4 秒启动：正常联合、视觉大跳变后重新确认、冲突优先、低质量视觉拒绝与 AMCL 接管、全输入断流撤销许可、恢复重新确认和人工估计不直接放行，并检查实际地图 TF 消息只来自统一融合节点。另执行真实 RTAB-Map 建图/数据库保存/重载定位回归。此联合用例的 Astra 选项使用合成标准点云；真实相机另按 Astra 接入文档验证。联合用例的视觉位姿为受控输入，不宣称已完成实车跨传感器定位验收。

```bash
source /workspace/install/setup.bash
cd /workspace
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 RUN_JOINT_LOCALIZATION_INTEGRATION=1 \
  python3 -m pytest -q -s src/robot_navigation/test/test_joint_localization_integration.py
```

现场仍需检查：CPU/USB 负载、真实延迟、地面剔除、窄视野/相似走廊、玻璃反光、点云稀疏、地图与当前障碍变化、头部归中反馈和 Astra 安装标定。iPhone 地图可先保留 `.db`，用 RTAB-Map 导出同坐标二维地图与点云；手机视角与机器人不同，跨设备视觉定位需样本验证，本次不提供手机自动导入或上传服务。

全局纠偏不会消除局部轮侧滑移，也不能在全部平移观测消失时推断真实停车。两个来源都来自双目时仍可能共同失效；视觉优先也不能消除重复纹理误定位，质量门限只是保护而不是绝对正确性证明。
