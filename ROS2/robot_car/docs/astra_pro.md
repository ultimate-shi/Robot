<!-- 作用：说明实机全部切换 Astra Pro RGB-D 的部署、接口和验收；用法：先配置驱动并核对头部安装，再启动整车。 -->
# Astra Pro 全 RGB-D 链路

## 当前架构

实机默认 `camera_source:=astra`。Astra Pro 替换原头部双目，彩色图像、深度、视觉里程计、视觉地图定位、建图、轮廓定位、避障与语义均使用 Astra 数据。轮速、IMU、EKF、速度安全链、Nav2 和 9100 端口网页接口保持兼容。

```text
Astra 原生 RGB/深度及工厂光学 TF
  → image_proc 彩色/深度校正
  → depth_image_proc 按内参及光心外参软件配准（不重打采集时间）
  → 有界近似同步 RGBDImage → C++ 校验、8Hz 限速
      ├→ /camera/rgbd_image → rgbd_odometry → /visual_odom_raw → 原保护/EKF → /odom
      ├→ /camera/rgbd_image → RTAB-Map RGB-D 建图/视觉重定位
      ├→ /camera/depth/image_rect + 内参 → C++ 点云 → 原避障/AMCL 投影
      └→ 校正彩色、米制深度、预览 → 语义/网页
```

全局联合定位继续由统一节点发布 `map → odom`，冲突优先通过质量检查的视觉结果；局部 EKF 仍唯一发布 `odom → base_link`。视觉不再使用左右图像或旧双目基线。

## 驱动与容器

本机 USB 确认为深度设备 `2bc5:0403 ASTRA Pro`、彩色设备 `2bc5:0501 Astra Pro HD Camera`。使用官方 [ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera)，不能把 Astra Pro 与新版 Astra Pro Plus 驱动混用。

驱动锁定提交 `f7e71d9ce806e788cb48d8580aac2c778fba4214`。`scripts/docker/install_astra_driver.sh` 将其安装到 `/opt/robot/astra_ws`，仅修正 Jazzy 头文件/参数回调兼容及缺失标准头文件，保持测距与标定算法不变。厂商源码不复制进业务包。Dockerfile 集成安装，入口脚本自动加载驱动环境；容器脚本挂载 `/dev/bus/usb` 并开放 USB 设备 cgroup。

摄像头映射可以同时保留：

| 来源 | 宿主机 → 容器 | 访问方式 |
| --- | --- | --- |
| 原双目 | `/dev/stereo_camera` → `/dev/video0` | 保留原 V4L2 入口，创建容器时需已连接且别名存在 |
| Astra 深度和彩色 | `/dev/bus/usb` → `/dev/bus/usb` | 新增 USB 总线挂载及 `c 189:* rmw`，驱动用 OpenNI/libuvc |

Astra 不占用双目的 `/dev/video0`。设备映射同时存在不代表自动启动两个相机驱动；实际视觉来源仍由 `camera_source` 选择。脚本遇到已运行的容器会检查 Astra USB 挂载/权限及双目映射并报告缺项；设备映射缺失需在接好设备后重建容器，单纯重启无法追加。当前双目设备别名不存在时不会映射 `/dev/video0`，也不会把 Astra 冒充双目设备。

正式镜像部署：

```bash
bash scripts/docker/build_jazzy_image.sh
bash scripts/docker/run_jazzy_container.sh
```

若使用已有旧镜像，可在容器内执行 `bash /workspace/scripts/docker/install_astra_driver.sh`，然后加载 `/opt/robot/astra_ws/install/setup.bash`。运行中的旧容器无法追加 USB 挂载，应退出后按标准脚本重建，不需要触碰地图目录。

## 头部安装位置

按用户确认，Astra 替换原头部双目，中心距离和高度沿用旧 xacro。`robot_perception/config/astra_mount.yaml` 已默认启用 `head_pitch_link → astra_link`：

```yaml
xyz: [0.0, -0.0165, 0.0468]             # 米，旧双目两光心中点
rpy: [0.0, 0.0, -1.5707963267948966]    # 弧度，yaw=-90°
```

位置来自 `stereo_camera.xacro` 的 `lens_y=-0.0165`、`lens_z=0.0468`，不是相机外壳的零原点。旧头部局部 +X 向左、-Y 向前、+Z 向上；Astra 驱动的机身 +X 向前、+Y 向左、+Z 向上，因此须绕 Z 旋转 -90°。父坐标系使用 `head_pitch_link`，相机随头部 yaw/pitch 运动，不固定到车身。

这里将驱动的 `astra_link`（与深度参考点重合）作为用户所指的相机中心，彩色光心仍由工厂内部 TF 定位，不把彩色和深度光心强行重合，也不沿用旧双目基线。这是用户确认的模型安装值，后续实测发现偏差可在同一 YAML 校正；不代表已完成整车导航精度验收。若外部已发布相同安装关系，设 `enabled: false`，防止重复发布；也可通过 `astra_mount_config_file` 指定自己的文件。

原双目外观和光学坐标在 Astra 模式不加载，控制器和 robot_state_publisher 使用相同模型选择。模拟模式保持旧虚拟模型，避免固定实机图像与移动虚拟机器人混用。

## 启动与配置

```bash
# 在线 RGB-D 建图，仍使用原导航与速度安全链
ros2 launch robot_main robot.launch.py mode:=real

# 已有地图联合定位
ros2 launch robot_main robot.launch.py mode:=real \
  map_yaml_file:=/workspace/maps/room/map.yaml

# 临时回退原双目完整链路
ros2 launch robot_main robot.launch.py mode:=real camera_source:=stereo
```

`localization_cloud_source:=auto` 默认跟随 `camera_source`；Astra 的统一点云为 `/camera/points`，扫描光心为 `astra_color_optical_frame`，因为点云由配准到彩色图的深度生成。外部原生深度点云可显式覆盖 `localization_cloud_topic` 与 `localization_sensor_frame`，两者必须匹配，不能只改名称假装已对齐。

`astra_config_file` 控制原生相机与 C++ 配准/同步/限速，`astra_start_driver:=false` 用于外部驱动或数据回放，仍需发布约定的原生图像、CameraInfo 和内部光学 TF。相机内部配准无需底盘运动；RGB 与深度属于同一设备但不假定硬同步，允许时间差默认最多 50ms。

| 接口 | 当前内容 |
| --- | --- |
| `/astra/color/image_raw`、`/astra/depth/image_raw` | 原生彩色与毫米深度，保留不同光心 |
| `/camera/rgbd_image` | 已校正、配准且限速的 RGBDImage；原包深度保持 16UC1 毫米，RTAB-Map 原生支持 |
| `/camera/rgb/image_rect`、`/camera/rgb/camera_info` | 统一彩色校正图和对应内参 |
| `/camera/depth/image_rect` | 配准到彩色光心的米制 32FC1 深度；无效距离为 NaN，兼容原语义链 |
| `/camera/depth/image_visual` | 单通道 8 位深度预览，供网页 |
| `/camera/points` | 配准深度生成的 XYZ 点云，默认图像降采样 2 倍 |
| `/visual_odom_raw`、`/visual_odom`、`/odom` | 保留原上游里程计接口，估计器已切为 RGB-D |
| `/localization/scan`、`/localization/status` | 保留联合定位和导航许可接口 |

网页在 Astra 模式显示“彩色/深度”，隐藏左右目切换；旧 left/right 图像请求作为同一彩色源的兼容入口，不代表两台相机。旧双目专用标定文件不用于 Astra。RGB/深度内参与工厂外参仍需对齐实物边缘验证；可通过驱动的 `color_info_url`、`ir_info_url` 加载重新标定结果。

## 验证结果与复现

- Jazzy ARM64 上官方驱动经兼容修正构建并识别真实 Astra Pro。
- 实机仅相机链路采样 25 秒：使用新镜像收到 91 个 RGB-D 包、101 个点云，点云每帧 76800 点，135 个有效相机自身视觉里程计样本；末次 RGB/深度时间差约 11ms。统一深度输出已验证为米制 32FC1。这是探针接收到的样本数，不能视作全部负载条件下的性能保证。
- 相机自身里程计测试将 frame_id 设为 `astra_link`，不提供猜测的相机到车身变换，结果不表示整车安装已标定。
- 合成 RGB-D 验证真实 RGB-D 里程计、RTAB-Map 地图保存和重载匹配；C++ 输入测试覆盖旧帧、RGB/深度时间差、帧和尺寸不一致、缺标定、非法深度编码、限速、毫米到米转换和无效值。

仅相机诊断（不启动底盘，必须使用独立通信域）：

```bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 \
  python3 scripts/diagnostics/check_astra_rgbd.py --start-camera --test-odometry
```

若相机已由其他进程打开，应先明确使用哪个实例，不要同时启动两个驱动。脚本输出报告和同名日志，结束时清理自己启动的相机与里程计进程。

现场剩余项：模型安装值的现场精度复核、RGB/深度标定精度、遮挡/反光、弱纹理及整车并发负载验证。结构光近距离盲区不能由 NaN 当作可通行，8 路超声波和原安全链继续保留。旧双目地图可尝试重定位，但传感器和视角变化可能影响匹配，失败时应使用 Astra 重新建图。
