<!-- 使用方法：从本文件选择运行模式，在 /home/radxa/Robot 统一构建和启动。 -->
# Robot

本仓库是机器人小车的 ROS 2 Jazzy 工作区，用于维护实机与虚拟机器人的统一模型、双目视觉、
SLAM、Nav2 路径规划、局部避障和底盘控制。

## 项目结构

```text
Robot/
├── AGENTS.md                      # 仓库开发约定和验证要求
├── docker/                         # ROS 2 Jazzy ARM64 容器镜像
├── docs/                           # 双目标定和验收文档
├── scripts/                        # 环境安装、镜像构建、相机配置和启动脚本
├── src/
│   ├── robot_interfaces/           # 感知、任务与控制的 msg/srv/action
│   ├── robot_brain/                # Qwen、网页、多用户租约、工具调度和 ROS Bridge
│   ├── robot_perception/           # 双目、YOLO、深度、地形和虚拟传感器
│   ├── robot_navigation/           # RTAB-Map、Nav2、停靠点、跟随和探索预演
│   ├── robot_control/              # 四轮转向、里程计、速度门控和最终安全层
│   ├── robot_description/          # URDF、ros2_control 描述和 mesh
│   └── robot_stereo_components/    # 高带宽双目处理 C++ 节点
├── progress.md                     # 按日期记录开发过程、卡点和踩坑
└── README.md
```

## 多 Package 架构

工作区按领域拆分，但仍使用同一个 `src/`、`build/` 和 `install/`。所有 Package 统一
通过根目录 `colcon build --symlink-install` 构建。旧 `robot` 兼容包已经退役，启动和单独
运行节点时应直接使用对应的领域 Package。

依赖方向固定为：`robot_brain` 只依赖 `robot_interfaces` 并通过 ROS Bridge 调用感知和
导航；`robot_navigation` 通过类型化消息消费感知结果；`robot_control` 不依赖 Qwen、相机
或导航实现。Qwen 只能建议 `goto_object`、`follow_person`、`explore`，不能发布 Topic、
生成坐标或取得控制权。

关键类型化接口：

- `/perception/semantic_detections`：`robot_interfaces/msg/SemanticDetectionArray`。
- `/perception/detect_objects`、`/perception/set_detection_mode`：按需识别与持续识别模式。
- `/mission/plan`：只生成目标和路径的 `PlanMission` Action。
- `/mission/confirm`：确认已缓存预览，只发布 `/mission/navigation_goal`。
- `/mission/state`：类型化任务状态；旧 `/mission/status` JSON 暂时兼容。
- `/perception/terrain_state`：感知到控制的类型化地形约束。

## Python 功能分层

Python 节点按职责分别位于 `robot_perception`、`robot_navigation`、`robot_control` 和
`robot_brain`，不再提供旧 `robot` 包的运行入口。

### sensing：传感器接入层

- `stereo_splitter.py`：Python 双目拼接图拆分实现，作为 C++ 实现的回退方案。
- `stereo_pair_throttle.py`：选择同时间戳左右图像对，并限制立体匹配频率。
- `virtual_imu.py`：根据虚拟底盘里程计生成模拟 IMU。
- `virtual_ultrasonic.py`：根据环境点云和传感器 TF 生成 8 路模拟超声波。

正常情况下，双目拆分优先使用
`src/robot_stereo_components/src/stereo_splitter_node.cpp`。需要调试 Python 实现时，在相机
launch 后追加：

```bash
splitter_backend:=python
```

### perception：感知处理层

- `stereo_depth.py`：将视差转换为米制深度和 8 位预览图。
- `stereo_pointcloud_filter.py`：过滤真实双目点云，输出 Nav2 使用的局部障碍点。
- `pointcloud_obstacle_filter.py`：过滤离线 PLY 环境点云。
- `snapshot_local_observer.py`：从保存的全局点云裁剪虚拟机器人当前可见的局部障碍。
- `terrain_heightmap.py`：从点云生成地形高度网格。
- `terrain_physics.py`：计算坡度、台阶、坑洼和可通行性。
- `terrain_analyzer.py`：订阅点云并发布统一的地形状态。

### localization：定位层

该目录用于放置定位实现。目前双目视觉里程计直接使用
`rtabmap_odom/stereo_odometry`，由 `stereo_mapping.launch.py` 组合启动。后续接入
IMU、轮式里程计或 robot_localization 时，应在这一层增加融合节点，不需要改动感知和任务层。

### mapping：建图和地图层

- `snapshot_manager.py`：缓存 RTAB-Map 的二维地图和三维点云，并按服务请求保存快照。
- `ply_publisher.py`：读取离线 PLY 文件，发布 Foxglove 显示和仿真感知使用的点云。

地图快照服务：

```bash
# 创建临时导航预演快照
ros2 service call /mapping/create_preview_snapshot std_srvs/srv/Trigger '{}'

# 创建长期保存快照
ros2 service call /mapping/save_snapshot std_srvs/srv/Trigger '{}'
```

临时快照位于 `/tmp/robot_preview/current.{yaml,pgm,ply,json}`，长期快照默认位于
`/workspace/maps/map_YYYYMMDD_HHMMSS/`。长期目录名和 `map.json` 的 `created_at`
默认按 `rtabmap_stereo_mapping.yaml` 中的 `snapshot_timezone: Asia/Shanghai` 生成，
不受容器本地时区影响。

### mission：任务层

- `goal_manager.py`：接收 Foxglove 发布的 `/goal_pose`，校验后发送给 Nav2
  `NavigateToPose` action。

相关接口：

- `/goal_pose`：输入 `geometry_msgs/msg/PoseStamped`，frame 必须是 `map`。
- `/mission/status`：任务状态。
- `/mission/cancel`：取消当前任务的 `std_srvs/srv/Trigger` 服务。

后续的人体跟随、物体目标和自主探索，应先在任务层生成导航目标，再交给 Nav2，不直接控制底盘。

### safety：安全层

- `range_to_scan.py`：把 8 路超声波 Range 合成为 LaserScan。
- `obstacle_avoidance.py`：底盘前最后一级安全过滤；无有效量程、量程超时或距离过近时
  限速或停车。

导航预演的速度链：

```text
Nav2
  -> /cmd_vel_nav
  -> 双目 Collision Monitor
  -> /cmd_vel_stereo_safe
  -> 超声波最终安全层
  -> /cmd_vel_safe
  -> 虚拟底盘
```

### control：控制层

- `nav_controller.py`：连接 Nav2 速度平滑器和底盘安全链。
- `chassis_controller.py`：将安全速度转换为轮速和转向控制，并发布虚拟里程计。
- `chassis_feedback.py`：将 `/joint_states` 整理为底盘反馈。

真实运动控制接入后，可以替换本层底盘实现，同时保持上游 `/cmd_vel_safe` 接口不变。

### diagnostics：诊断层

- `stereo_pipeline_benchmark.py`：统计双目链路各阶段频率、消息年龄和处理延迟。

## 关键启动文件

启动文件按领域位于各 Package 的 `launch/` 目录。
每个对外 launch 参数都带有中文说明，可在启动前查看参数、用途和默认值：

```bash
ros2 launch <包名> <launch文件> --show-args
# 示例
ros2 launch robot_perception stereo_camera.launch.py --show-args
```

### 默认虚拟机器人

```bash
ros2 launch robot_navigation robot.launch.py
```

加载仓库内的二维地图、PLY 点云、Nav2、虚拟传感器和虚拟底盘，适合调试现有数字孪生链路。

### 单独启动真实双目相机

```bash
ros2 launch robot_perception stereo_camera.launch.py video_device:=/dev/video0
```

主要输出左右原图、校正图、视差、深度和点云。常用参数：

- `calibration_mode:=true`：只保留标定需要的左右原图。
- Foxglove Bridge 默认随启动文件运行，使用 `foxglove_port` 修改监听端口。
- `splitter_backend:=cpp|python`：选择双目拆分实现。
- `navigation_processing_enabled:=false`：关闭视差、深度和导航点云处理。

### 真实双目建图

```bash
ros2 launch robot_navigation stereo_mapping.launch.py video_device:=/dev/video0
```

若启动时报 `XML or text declaration not at start of entity`，请确认已重新构建
`robot_description`，且 `robot.xacro` 的 `<?xml ...?>` 声明位于文件第一行。

该入口启动真实相机、双目视觉里程计、RTAB-Map、地图点云、机器人模型、快照管理和
Foxglove，不启动虚拟底盘和 Nav2。

当前没有 IMU 和轮式里程计时仍可依靠双目视觉里程计建图。扩大地图时应保持相机安装在
机器人上并缓慢移动整车；相机完全静止只能建立当前视野附近的局部地图。

建图入口默认在 `usb_cam` 占用设备前，通过相机真实的 V4L2 控制名开启自动曝光和自动
白平衡，并在终端回读控制值。若现场需要固定曝光和色温，可先运行
`scripts/stereo/configure_stereo_camera.sh`，再以
`apply_auto_camera_controls:=false` 启动建图。双目标定必须保持自动控制关闭，避免采样中
亮度和颜色漂移。

所有正常双目入口的 `apply_auto_camera_controls` 默认值均为 `true`；标定脚本会显式传入
`false`，且 `calibration_mode:=true` 时相机入口也会自动跳过控制写入。左右校正图的
`/compressed` 由 `rectify_node` 的 image_transport 插件按订阅需求发布，不再额外启动左右
压缩转发器，避免同一话题出现两个发布者和重复帧；显式压缩转发仅保留给深度预览。

`/visual_odom` 只表示双目视觉估计的机器人位姿，并通过 `odom -> base_link` TF 让
Foxglove/RViz 中的 `robot_description` 随估计轨迹移动；它不会发布速度命令，也不会直接
驱动真实底盘。真实小车只有在底盘控制链收到 `/cmd_vel` 后才会运动。

Foxglove 常用话题：

- `/map`：二维占据栅格。
- `/mapping/cloud_map`：三维地图点云。
- `/visual_odom`：视觉里程计。
- `/tf`、`/robot_description`：机器人模型和轨迹。
- `/mapping/snapshot_status`：快照状态。

### 已保存地图导航预演

先在建图模式调用临时快照服务，停止建图 launch，然后启动：

```bash
ros2 launch robot_navigation navigation_preview.launch.py
```

该入口不读取位置固定的真实相机，而是加载
`/tmp/robot_preview/current.yaml` 和 `current.ply`，让虚拟机器人在已保存环境中规划、
运动和避障。也可以指定其他地图：

```bash
ros2 launch robot_navigation navigation_preview.launch.py \
  map_yaml_file:=/workspace/src/robot_navigation/map/studyroom.yaml \
  ply_file:=/workspace/src/robot_navigation/map/studyroom.ply
```

### 完整在线双目机器人

```bash
ros2 launch robot_navigation stereo_robot.launch.py
```

用于相机和机器人运动链已经处于同一坐标关系时的在线模式。真实底盘、IMU 和超声波尚未接入
ROS 前，不应把固定在现实环境中的相机点云与正在移动的虚拟机器人混合使用。
容器启动脚本将宿主机 `/dev/stereo_camera` 映射为容器内 `/dev/video0`，因此该入口默认使用
`/dev/video0`；只有自定义容器设备映射时才需要传入 `video_device:=...`。
该入口默认以 `log_level:=warn` 启动各 ROS 节点，需要调试时可显式传入
`log_level:=info`。launch 框架输出的进程启动与退出提示不受该参数控制。

### 机器人本地大脑与路径预演

`stereo_brain.launch.py` 在真实双目在线建图基础上增加语义识别、Nav2 规划服务、前沿
探索、人员跟随/前往物体预演、多用户控制权和局域网 HTTP 网页。第一版在代码和参数中
双重固定 `motion_enabled=false`，只发布 `/mission/preview_goal` 和
`/mission/preview_path`，不会向底盘发送速度。

宿主机推理网关监听回环地址，ROS 容器通过 host network 访问：

```bash
# 首次准备约 3.6 GiB 的纯文本 Qwen 与匹配 Runtime（支持断点续传）。
./scripts/inference/download_rk3588_models.sh qwen

# 常驻加载 YOLO RKNN 和 Qwen2.5-3B-Instruct，并启动 9100 网关。
./scripts/inference/start_yolo_gateway.sh
```

`ROBOT_DETECTOR_PLUGIN` 指向一个 Python 函数，函数签名为
`detect(jpeg_bytes, min_confidence)`，返回字典列表；每项至少包含
`class_name`、`label_zh`、`confidence` 和像素坐标 `bbox=[x1,y1,x2,y2]`。YOLO 结果会被
压缩为最多 20 项 `id`、中英文类别、置信度、距离和九宫格位置，再作为 JSON 交给纯文本 Qwen；图片
不会发送给语言模型。系统提示词明确将 `scene` 数组定义为 Qwen “看到的内容”，
要求回答以该内容为依据。`model/qwen2.5-3b-instruct/` 保存 RK3588 W8A8 模型和私有
RKLLM 1.2.1 Runtime，不覆盖系统库。

`start_yolo_gateway.sh` 先在 9101 端口启动常驻文本模型，健康检查成功后再启动 9100 调度
网关。模型只加载一次，默认使用 4096 上下文、最多生成 96 token、关闭思考模式并使用
确定性采样。也可用 `ROBOT_LLM_ENDPOINT` 和 `ROBOT_LLM_FALLBACK_ENDPOINT` 指向其他
OpenAI Chat Completions 兼容服务；旧 `ROBOT_VLM_*` 变量暂作兼容别名。Qwen 推理期间
YOLO 会因共用 NPU 串行锁暂停，回答完成后自动恢复实时识别。
新链路稳定后，如确认不再需要回退到 VL，可先用 `du -sh model/qwen2.5-vl`
核对目录，再手动执行 `rm -r -- model/qwen2.5-vl`；当前可释放约 4.9 GiB
（约 5.2 GB），启动和下载脚本不会自动删除它。

网页按标签页保留最近 2 轮对话。Qwen 提示词要求模型仅从 `goto_object`、
`follow_person` 和 `null` 中判断用户意图；非空动作仍按严格 Schema 输出
`name` 和 `arguments`，模型返回坐标、速度或其他动作会被严格拒绝。提案随后经过
独立 `CommandPolicy`，按“否定优先、疑问句不执行、明确命令、检测标签匹配”的顺序验证
用户原话，支持“前往杯子”“去杯子的地方/旁边”“跟着我”和“开始自主探索”等表达。
Schema 正确不代表动作已获授权，策略层会拦截“杯子在哪里”“你能跟随人吗”和“不要去
杯子那里”等误触发。动作通过后仍只生成确认面板，并结合最新检测候选和 Nav2 路径预演；
当前 `motion_enabled=false`，确认后也只发布导航目标接口，不驱动底盘。
普通问答若返回了有效 `answer`，但 Qwen 错把 `action` 写成 `"terminate"` 等字符串，
系统会保留文字回答并丢弃非法动作。代码围栏、JSON 前后解释、Python 风格单引号/`None`
以及完整 action-only 对象会在本地尝试一次确定性修复，不会再次调用模型；所有候选仍必须
通过严格白名单 Schema，多个对象或含坐标、速度、额外字段的动作一律拒绝。
Qwen 若编造 `scene` 中不存在的 `goto_object` 目标，本地策略层会强制丢弃动作，
并把回答替换为“当前画面未检测到该目标”，不会生成确认面板或机器人任务。
“人在哪里”“杯子在什么位置”等空间问题会进一步使用提问前最新 YOLO 检测框和右目图像
尺寸，把目标中心换算成左/中/右与上/中/下九宫格方位；因此即使 Qwen 只回答“在画面中”，
网页也会给出类似“人位于画面左侧中部”的落地结果。Qwen 推理期间视觉暂停不会影响本轮
回答，因为本轮使用的是提交问题时保存的版本化 `SceneSnapshot S0`。

动作链与问答链不复用同一时刻的数据：`S0` 只用于 Qwen、位置问答和审计；Qwen 返回动作
提案并释放 NPU 后，Web 层强制等待一份新 YOLO 结果 `S1`，再由 `TargetResolver` 解析人员
或物体候选并生成路径预演；用户确认非探索任务时还会获取 `S2` 重新绑定目标和规划。跟踪
ID 若变化，只有最新画面仍唯一匹配时才允许安全重绑；多目标、目标消失或默认 5 秒刷新超时
都会拒绝确认。超时时间由 `brain.yaml` 的 `action_scene_refresh_timeout` 配置。

9100 网关在 Qwen 占用共享 NPU 时为 YOLO 返回结构化 `NPU_BUSY_LLM`。语义感知将其记录为
`paused`，丢弃中间帧但保留上一份真实检测，不再发布伪造空数组；只有 YOLO 实际成功执行并
返回零目标时才发布 `valid_empty`。Qwen 结束后只识别最新帧，不积压推理期间的旧图。
`contracts.py` 定义版本化场景和策略结果，`command_policy.py` 负责用户命令授权，
`scene_coordinator.py` 负责场景冻结与等待，`target_resolver.py` 只在最新结构化检测中匹配
目标；Qwen 输出始终只是提案，不能直接调用 ROS 或机器人执行器。

Qwen 每轮网页对话会在 `/workspace/qwen_logs/` 保存同名的一对文件。ROS Bridge 用最多
30 帧、3 秒的有界缓存按 ROS 时间戳精确配对 YOLO 检测与右目压缩图；`.jpg` 在该配对帧上
绘制目标框、ID、英文类别、置信度和距离，零检测时显示 `YOLO: 0 objects`。标注图只用于
审计，不发送给 Qwen；若时间戳配对失败则记录告警且不保存错位图片。`.txt` 记录用户原文、
完整 system 提示词、user 内容、两个时间戳、图片 SHA-256、绘制告警、网关原始返回、解析路径、
首 token/完整生成耗时、最终白名单答案和动作。日志还分别记录队列等待、
状态快照与历史准备、提示词和请求构造、网关 HTTP/Qwen 推理、响应解析与位置落地、任务
解析与路径预演、端到端总时长，单位均为毫秒。动作轮还记录策略原因码、提案来源，以及回答
使用的 S0 和动作重新验证使用的 S1 快照摘要。目录可通过 `brain.yaml` 的
`qwen_log_directory` 修改，属于运行数据并已被 Git 忽略。默认 `warn` 日志级别不会在终端
重复打印 INFO 明细；需要同时看终端时可传入 `stereo_brain.launch.py log_level:=info`。

任务预演阶段发布 `geometry_msgs/msg/PoseStamped` 类型的 `/mission/preview_goal` 和
`nav_msgs/msg/Path` 类型的 `/mission/preview_path`。点击“确认并取得控制权”后，确认服务
把同一个 `map` 坐标系目标放在 `ConfirmMission` 响应的 `navigation_goal` 字段，并发布到
`geometry_msgs/msg/PoseStamped` 类型的 `/mission/navigation_goal`。当前没有把该话题转发
给 Nav2 `NavigateToPose`，因此它是未来导航执行器的输入接口，不代表机器人已经开始移动。
可在确认前运行 `ros2 topic echo /mission/navigation_goal` 查看下一次发布的目标。
确认时会使用当前地图和目标再次调用 Nav2 验证路径，并重建 ROS 端预演缓存；
因此旧预演失败或被停止清理后不会误报“任务预览不存在或已过期”。如果当前
仍没有可达路径，网页返回具体的规划错误且不取得控制权，不再产生 HTTP 500。任务目标
若贴近当前 OccupancyGrid 外缘，会按 `mission.yaml` 的 `goal_boundary_margin`
收进地图安全边界后再交给 Nav2，避免 `Goal Coordinates ... was outside bounds`。

本地大脑入口的任务规划节点会记住入口使用的 `detection_mode`。任务确认、取消、立即停止、
释放控制权或人员丢失后都会恢复该模式；默认是 `continuous`。因此除 Qwen 实际占用 NPU 的
问答窗口外，任务预演和任务结束后 YOLO 都持续更新，不会再被切回 `on_demand`。

重新构建包含 FastAPI/Uvicorn 的 Jazzy 镜像后，在容器中启动：

```bash
ros2 launch robot_brain stereo_brain.launch.py video_device:=/dev/video0
```

该入口默认将 RTAB-Map、Nav2、Foxglove、感知节点、ROS Bridge 和 Uvicorn 网页服务统一为
`log_level:=warn`；需要查看里程计质量、地图更新周期或 HTTP 请求时可临时传入
`log_level:=info`。ROS launch 自身的进程启动和退出 `[INFO]` 不受节点日志级别控制。
完整大脑负载下双目校正输出默认限制为 5 Hz，图像与 CameraInfo 使用相同的单帧可靠队列；
RTAB-Map 等待对应时间戳 TF 的窗口为 1 秒，用于避免处理积压时出现同步和向未来外推告警。
若仍持续出现大量 stereo correspondences rejected 或 odometry lost，应检查左右镜头遮挡、
亮度差、极线标定和场景纹理，不能通过降低最小内点数掩盖错误位姿。

手机或电脑连接同一局域网后访问 `http://<RK3588-IP>:8080`。网页不使用 HTTPS、登录、
配对码、令牌、来源检查或登录限流；这只适用于可信局域网。仅做路由器端口映射会让任何
公网访问者都能控制或停止机器人，公网使用前必须另行在 VPN、路由器或反向代理层增加
认证和加密。

前端的标签页 ID 和请求 ID 使用兼容纯 HTTP 的随机生成回退，不依赖安全上下文中的
`crypto.randomUUID()`；相机 JPEG 同时兼容 `createImageBitmap` 和普通 `Image` 解码。
HTML、JavaScript 和样式响应禁用缓存，更新后普通刷新即可取得新前端资源。

若 Safari 能访问而 Chrome 显示“无法访问此网站”，先在地址栏完整输入
`http://<RK3588-IP>:8080/`，不要省略 `http://`，并用
`http://<RK3588-IP>:8080/api/health` 区分网页问题和网络问题。Chrome 开启“始终使用安全
连接”、配置独立代理或自动升级 HTTPS 时，需要为该局域网 HTTP 地址关闭对应设置；服务端
只提供 HTTP，不监听 443 端口。
页面不再展示“系统健康”面板；后台仍保留健康检查，断线轮询也继续使用
`/api/health`。

`stereo_brain.launch.py` 默认把 YOLO 切换为 `continuous`，按
`semantic_perception.yaml` 的 `max_inference_rate`（默认最高 5 Hz）持续识别最新右目校正
画面；网页会实时更新物体中文名称、置信度、距离和检测框。“立即刷新识别”仍可强制等待
一份新结果；若 5 秒内没有新结果，会明确说明当前展示的是上一次结果。单独启动
`stereo_perception.launch.py` 时仍默认 `on_demand`，需要持续识别可传入
`detection_mode:=continuous`；本地大脑入口也可以用 `detection_mode:=on_demand` 临时关闭
持续识别。

网页、YOLO、Qwen视觉输入和验收采样统一使用右目校正图
`/stereo/right/image_rect`。深度节点根据 `x_right=x_left-disparity` 生成右目对齐深度
`/stereo/depth/right/image`；语义目标使用右目 CameraInfo 与该深度计算距离和地图坐标，
不会把左目深度直接套到右目检测框。

WebSocket运行依赖 `websockets`，Jazzy镜像已固定安装对应版本；缺少该库时Uvicorn会把
`/ws/state` 升级请求返回404。前端同时保留每2秒一次的HTTP健康状态降级轮询，连接徽标会
显示“HTTP轮询模式”，因此临时缺少WebSocket也不会阻断识别明细和健康状态显示。

多用户规则：

- 所有人可同时查看共享地图、画面、识别结果和任务状态；聊天回答只返回发起请求的标签页。
- Qwen 通过长度为 8 的单队列串行执行，队列满时返回系统繁忙；导航和立即停止不进入队列。
- 第一个确认运动类预演的标签页取得控制权；其他用户仍可创建自己的预览，但不能确认、
  取消或覆盖活动任务。
- 任意用户都能立即停止。控制者断线后保留 10 秒重连窗口，超时自动停止并释放控制权。
- 浏览器 `sessionStorage` 中的 `client_id` 仅用于正常用户之间的仲裁，不是安全凭证。

主要接口为 `/api/chat`、`/api/missions/preview`、
`/api/missions/{id}/confirm`、`/api/missions/cancel`、`/api/control/release`、
`/api/stop`、`/api/detections`、`/api/health` 和 `/ws/state`。参数集中在
`src/robot_brain/config/brain.yaml`。

### 现场物品和视觉问答验收

网页“保存验收样本”会把右目校正图、右目对齐米制深度、CameraInfo、机器人 TF、自动检测结果和
人工问答模板写入 `/workspace/acceptance_dataset/`。该目录是运行数据，已被 Git 忽略。

人工验收步骤：

1. 每类物品采集至少 50 张，重要类别建议 100 张；覆盖远近、方向、遮挡、光照和无目标
   负样本，避免连续保存大量相同画面。
2. 将 `samples/*/right_rect.jpg` 导入 CVAT 或 Label Studio，导入类别清单，修正自动框和
   类别后导出 YOLO 格式；不确定的目标标为 `unknown`，至少由第二个人抽查 20%。
3. 编辑 `vqa_template.jsonl`，至少准备 30 个场景、每场景 3～5 题，填写标准答案、可接受
   同义答案和是否能从画面回答；开放题人工标为正确、部分正确、错误或幻觉。
4. 如果板端不能安装标注工具，把整个 `acceptance_dataset` 复制到另一台电脑完成上述
   标注和复核，再把 YOLO 标注与已评分 JSONL 放回原目录。

没有人工框真值时只能展示检测样例，不能报告精确率或召回率；没有人工标准答案和开放题
复核时只能展示问答样例，不能宣称视觉问答准确率。

兼容包中的 `common_bringup.launch.py` 现在只组合新 Package，不再拥有模型、Nav2或底盘实现。

## 关键配置文件

配置已随职责拆到 `src/robot_{brain,control,perception,navigation}/config/`；兼容包中的旧配置
只供旧节点回退。

| 文件 | 用途 |
| --- | --- |
| `stereo_camera.yaml` | UVC 相机格式、分辨率、帧率和拆分参数 |
| `cameras/*/left.yaml`、`right.yaml` | 按相机型号保存的左右目标定参数 |
| `stereo_pointcloud.yaml` | 视差、深度范围和双目点云过滤参数 |
| `rtabmap_stereo_mapping.yaml` | 双目视觉里程计、RTAB-Map 和地图点云参数 |
| `navigation_preview.yaml` | 静态点云局部观察和目标管理参数 |
| `stereo_collision_monitor.yaml` | 双目局部减速、停车区域和传感器超时 |
| `nav2_params.yaml` | Nav2 控制器、规划器、代价地图和行为树参数 |
| `nav2_stereo_overrides.yaml` | 在线双目模式的 Nav2 覆盖参数 |
| `controllers.yaml` | 轮子、转向和关节控制器参数 |
| `controller_manager.yaml` | ros2_control 管理器和硬件插件参数 |
| `robot_control/config/control.yaml` | 底盘、速度门控和最终避障阈值 |
| `robot_perception/config/terrain_perception.yaml` | 地形与虚拟传感器参数 |
| `robot_perception/config/semantic_perception.yaml` | YOLO、深度融合和验收采样参数 |
| `robot_brain/config/brain.yaml` | HTTP、Qwen队列和控制租约参数 |

设备路径、网络端口和现场参数应通过 YAML 或 launch 参数修改，不要写死在 Python 节点中。

## 模型、地图和世界

- `src/robot_description/urdf/robot.xacro`：机器人模型总入口。
- `src/robot_description/urdf/head.xacro`：两自由度头部和双目相机安装结构。
- `src/robot_description/urdf/hardware.xacro`：ros2_control 硬件接口。
- `src/robot_description/meshes/`：车体、轮子、头部和传感器网格。
- `src/robot_navigation/map/studyroom.*`：默认二维地图和三维 PLY 点云。
- `src/robot_navigation/map/obstacle_test.*`：避障测试地图。
- `src/robot_navigation/map/blank.*`：基础运动链调试使用的空白地图。

二维地图的 `.yaml` 和 `.pgm` 必须配套；需要三维局部观察或虚拟超声波时，还要提供同一
坐标系下的 `.ply`。

## 构建与容器

RK3588 的 Debian 12 主机通过 Ubuntu 24.04 ARM64 容器运行 ROS 2 Jazzy。

首次安装 Docker：

```bash
sudo bash scripts/docker/install_jazzy_docker.sh
```

构建包含 Nav2、RTAB-Map 和双目依赖的镜像：

```bash
bash scripts/docker/build_jazzy_image.sh
```

RTAB-Map 会安装 PCL、VTK 等较大依赖，构建前建议至少预留约 8 GB。

日常构建并启动默认虚拟机器人：

```bash
bash scripts/docker/run_robot_jazzy.sh
```

只启动容器，不自动构建和运行 launch：

```bash
bash scripts/docker/run_jazzy_container.sh
docker exec -it robot-jazzy bash
```

进入容器后手动构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

双目标定入口：

```bash
bash scripts/stereo/run_stereo_calibration.sh preview
bash scripts/stereo/run_stereo_calibration.sh calibrate 0.030
```

相机配置和标定的详细步骤见 `docs/stereo_calibration_guide.md`。

## 测试

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q \
  src/robot_brain/test/test_action_schema.py \
  src/robot_brain/test/test_brain_core.py \
  src/robot_navigation/test/test_mapping_snapshot.py \
  src/robot_perception/test/test_stereo_processing.py
```

新增节点后，需要同时：

1. 放入正确的领域 Package，不向兼容 `robot` 增加实现。
2. 在所属 Package 的 `setup.py` 注册 `console_scripts`。
3. 将可调参数放入所属 Package 的 `config/` 或 launch 参数。
4. 更新 README 和当天的 `progress.md`。

更完整但保持精简的开发约定见 `AGENTS.md`。
