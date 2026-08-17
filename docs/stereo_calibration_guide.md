<!-- 使用方法：按本文在 RK3588 上完成双目标定、启动和验收。 -->
# 双目相机从接线到 Nav2 的完整操作指南

本文适用于当前项目的“单个 UVC 设备输出横向拼接左右图”相机。默认总图为
1280×480，拆分后左右图各为 640×480。

## 一、真正使用前必须完成什么

必须依次完成以下四件事：

1. 固定取流模式和左右顺序：确认相机能稳定输出目标分辨率、格式和帧率。
2. 双目内参与双目外参标定：得到左右相机的 `D/K/R/P`，特别是右目 `P[3]` 中的真实基线。
3. 相机安装外参标定：得到 `base_link -> stereo_camera_link` 的实际 `xyz/rpy`。
4. 深度和障碍物验收：检查深度误差、点云高度、TF、帧率、延迟和 Nav2 costmap。

其中第 2 项描述左右镜头之间的几何关系，第 3 项描述整个相机模组和车体之间的几何
关系。这两类外参不能互相替代。

## 二、板端依赖和相机模式

在 ROCK 5B+ 的 Debian 12 原生系统中连接相机，不要经过 VMware USB 转接。Jazzy
underlay 至少需要：

```text
usb_cam
image_pipeline（image_proc、stereo_image_proc）
camera_calibration
cv_bridge
image_transport
compressed_image_transport
pointcloud_to_laserscan
nav2_collision_monitor
v4l-utils
```

查看设备和真实支持的模式：

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
udevadm info --query=property --name=/dev/video0
```

必须在输出中找到计划使用的拼接模式。当前板端实测为 `1280x480 MJPG 20 fps`。如果只有
`2560x720`，则修改 `stereo_camera.yaml`：

```yaml
image_width: 2560
image_height: 720
```

此时单目实际分辨率为 1280×720，必须重新标定，不能继续使用 640×480 标定文件。

检查 `usb_cam` 支持的像素格式：

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"
```

## 三、创建稳定的 /dev/stereo_camera

先读取 VID、PID、序列号和 video index：

```bash
udevadm info -a -n /dev/video0 | less
v4l2-ctl -D -d /dev/video0
```

按实际值创建 `/etc/udev/rules.d/99-stereo-camera.rules`。示例中的值必须替换：

```udev
SUBSYSTEM=="video4linux", ATTR{index}=="0", \
ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", \
ATTRS{serial}=="实际序列号", SYMLINK+="stereo_camera"
```

加载规则并重新插拔相机：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/stereo_camera
```

如果相机没有序列号，应至少组合 VID、PID 和稳定 USB 端口路径；不要只按
`/dev/video0` 创建规则。多功能 UVC 设备常暴露多个 video 节点，`ATTR{index}=="0"`
用于避免符号链接随机指向 metadata 节点。

## 四、第一次取流和左右确认

当前 RK3588 Debian 12 使用 Jazzy 容器。首次安装相机稳定设备规则并重建带标定依赖的
镜像：

```bash
cd /home/radxa/Robot
sudo bash scripts/stereo/install_stereo_camera_udev.sh
# 重新插拔相机并确认 /dev/stereo_camera 存在
bash scripts/docker/build_jazzy_image.sh
```

从 Debian 图形桌面打开终端，只启动原图拆分和左右预览，暂不启动视差和点云：

```bash
bash scripts/stereo/run_stereo_calibration.sh preview
```

脚本会固定亮度、曝光和白平衡，并同时弹出左右图像窗口。需要用 ROS CLI 检查时，保持
预览容器运行并另开桌面终端：

```bash
docker exec -it robot-jazzy-calibration bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
ros2 topic echo /stereo/left/image_raw --once --field width
ros2 topic echo /stereo/left/image_raw --once --field height
```

依次完全遮住物理左镜头和右镜头：

- 遮住物理左镜头时，必须只有 `/stereo/left/image_raw` 变黑。
- 遮住物理右镜头时，必须只有 `/stereo/right/image_raw` 变黑。
- 如果相反，将 `stereo_camera.yaml` 中 `left_first` 改为 `false`。

同时确认左右图没有上下翻转、镜像、撕裂或一帧错位。拼接相机的左右图来自同一条 UVC
消息，因此当前 splitter 会给两幅图使用完全相同的时间戳。

## 五、制作和测量标定板

当前建议使用：

- 棋盘格内角点：8×6。
- 实际格子数量：9×7。
- 单格边长：30 mm，即命令中的 `--square 0.030`。
- 材料：刚性平板，不能拿一张会弯曲的纸直接标定。
- 测量：用卡尺测量相邻内角点间距，不要默认打印比例正好是 100%。

如果实测格子为 29.82 mm，命令必须写 `--square 0.02982`。格子尺寸误差会直接按比例
进入基线和深度尺度。

`--square` 表示相邻内角点之间的距离，不是标定板最外边缘到第一个内角点的距离。9×7
格子对应 8×6 内角点：沿 9 格方向，首尾内角点跨 7 个间距，30 mm 标准板应为 210 mm；
沿 7 格方向，首尾内角点跨 5 个间距，应为 150 mm。如果只有最外侧两个边缘格为
27 mm，而所有相邻内角点间距和上述总跨度仍为 30 mm、210 mm、150 mm，则外侧格不参与
角点几何约束，仍使用 `--square 0.030`。如果内角点间距本身不均匀，则必须重新打印。

A4 短边只有 210 mm，不适合在该方向无页边距打印 7 个 30 mm 格子。重新制作时优先使用
A3，或把 9×7 格子的统一边长缩小到 25 mm，并按打印后的真实内角点间距填写命令。

标定前固定好镜头焦距、曝光模式、分辨率和左右相对位置。标定后不允许旋转镜头、改变
焦距、拆开相机支架或切换分辨率；发生这些变化必须重新标定。

## 六、执行双目标定

当前 RK3588 推荐关闭预览窗口后，直接从 Debian 图形桌面终端传入卡尺实测的格子边长。
例如实际边长为 29.82 mm：

```bash
cd /home/radxa/Robot
bash scripts/stereo/run_stereo_calibration.sh calibrate 0.02982
```

脚本会重新启动 `calibration_mode:=true` 并打开标定 GUI，效果等同于在带图形界面的
Jazzy 环境手工执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/shijiahao/Downloads/ros2/robot_ws/install/setup.bash

ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.030 \
  --camera_name stereo_model_serial_640x480 \
  --no-service-check \
  --queue-size 5 \
  --ros-args \
  --remap left:=/stereo/left/image_raw \
  --remap right:=/stereo/right/image_raw \
  --remap left_camera:=/stereo/left \
  --remap right_camera:=/stereo/right
```

当前拼接图左右时间戳完全相同，因此不要先加 `--approximate`。只有确认时间戳不同的双
独立 UVC 相机才考虑例如 `--approximate 0.02`。

采样时让标定板同时完整出现在左右图中，并覆盖：

- 左、右、上、下四个区域。
- 近、中、远不同距离，近距离时标定板应占据较大画面。
- 向左、向右、向上、向下倾斜。
- 正对镜头和明显斜视姿态。
- 至少 40～80 组清晰、静止、不过曝的有效双目图。

不要连续采集几乎相同的姿态，不要在标定板移动或模糊时采样，也不要让角点贴到画面
边缘之外。GUI 中 X、Y、Size、Skew 覆盖充分并点亮 `CALIBRATE` 后：

1. 点击 `CALIBRATE`，等待计算完成。
2. 查看校正图，直线不应明显弯曲。
3. 左右校正图中的同一角点应位于同一水平扫描线上。
4. 点击 `SAVE` 保存标定包。当前 splitter 没有 `set_camera_info` 服务，不使用
   `COMMIT`，这也是命令使用 `--no-service-check` 的原因。

通常输出位于：

```text
/tmp/calibrationdata.tar.gz
```

先检查再解压：

```bash
tar -tzf /tmp/calibrationdata.tar.gz
mkdir -p src/robot_perception/config/cameras/model_serial_640x480
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/stereo_calibration
```

从解压内容中找到左右相机 YAML，分别保存为：

```text
src/robot_perception/config/cameras/model_serial_640x480/left.yaml
src/robot_perception/config/cameras/model_serial_640x480/right.yaml
```

如果输出文件名不是 `left.yaml`、`right.yaml`，按文件中的 `camera_name` 和左右投影矩阵
确认后再重命名，不能凭文件排列顺序猜测。

## 七、标定文件必须检查什么

左右文件都必须满足：

- `image_width`、`image_height` 等于单目运行分辨率。
- `camera_matrix.data` 中 `fx`、`fy` 大于 0，`cx/cy` 位于图像范围内。
- `distortion_coefficients.data` 不是模板占位值。
- `rectification_matrix.data` 来自本次双目标定。
- 左目 `projection_matrix.data[3]` 通常为 0。
- 右目 `projection_matrix.data[3]` 必须为非零负值。

从右目投影矩阵计算标定基线：

```text
baseline_m = -P_right[3] / P_right[0]
```

结果应接近用卡尺测得的两个光心距离。标称 65 mm 只用于合理性检查，不能覆盖标定结果。
若计算出负基线、零基线或与实测相差很大，应先检查左右顺序、格子尺寸和 YAML 是否配对，
不要靠修改 `P[3]` 人工“修好”深度。

## 八、标定后具体修改哪些地方

### 1. 保存左右标定 YAML

新增而不是覆盖模板：

```text
src/robot_perception/config/cameras/<型号_序列号_单目分辨率>/left.yaml
src/robot_perception/config/cameras/<型号_序列号_单目分辨率>/right.yaml
```

矩阵数值全部来自 `camera_calibration`。不要手工把标称焦距或 65 mm 基线写进去。

### 2. 修改相机取流 profile

检查 `src/robot_perception/config/stereo_camera.yaml`：

- `video_device`：应为 udev 稳定路径。
- `framerate/pixel_format/image_width/image_height`：必须与实际取流和标定模式一致。
- `left_first`：必须与遮挡测试一致。
- SGBM 参数先保留默认值，完成几何标定后再根据性能和点云质量调整。

### 3. 修改相机安装 TF

编辑 `src/robot_description/urdf/robot.xacro` 中：

```xml
<xacro:stereo_camera parent_link="base_link" xyz="0.20 0 0.30"
                     rpy="0 0 0" baseline="0.065"/>
```

本项目定义：

- `stereo_camera_link` 位于左右镜头光心中点。
- `base_link` 使用 x 前、y 左、z 上。
- `xyz` 是相机中点在 `base_link` 下的位置，单位 m。
- `rpy` 是相机模组坐标系相对 `base_link` 的 roll、pitch、yaw，单位 rad。
- `baseline` 改为 `-P_right[3]/P_right[0]` 计算出的真实基线，单位 m。

初次可在水平地面上用卡尺、直角尺和电子水平仪测量。更精确时用位置已知的
AprilTag/棋盘格求相机到车体的外参。无论使用哪种方法，都要通过点云验证：

- 平地在 `base_link` 下应形成稳定水平面。
- 竖直墙面不应明显倾斜。
- 已知高度的障碍物在点云中高度应正确。
- 车辆静止时点云不应随时间大幅漂移。

修改 xacro 后重新构建并加载：

```bash
colcon build --packages-select robot_description robot_perception
source install/setup.bash
```

### 4. 启动时传入当前标定文件

相机单测：

```bash
ros2 launch robot_perception stereo_camera.launch.py \
  calibration_mode:=false \
  left_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot_perception/config/cameras/model_serial_640x480/left.yaml \
  right_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot_perception/config/cameras/model_serial_640x480/right.yaml
```

完整实机：

```bash
ros2 launch robot_navigation stereo_robot.launch.py \
  left_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot_perception/config/cameras/model_serial_640x480/left.yaml \
  right_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot_perception/config/cameras/model_serial_640x480/right.yaml
```

只有再次 `colcon build` 后，profile 才会被安装到
`install/robot_perception/share/robot_perception/config/cameras`。
直接传源码目录绝对路径可以立即测试，但正式部署建议构建后使用安装空间中的文件。

本机 `USB Camera 01.00.00` 的 640×480 单目 profile 已保存为：

```text
src/robot_perception/config/cameras/usb_camera_01_00_00_640x480/left.yaml
src/robot_perception/config/cameras/usb_camera_01_00_00_640x480/right.yaml
```

两个实机启动入口已默认读取这组文件。2026-08-04 的离线验收数据见
[`stereo_calibration_acceptance_2026-08-04.md`](stereo_calibration_acceptance_2026-08-04.md)。

## 九、标定后分层验收

### 1. 图像与 CameraInfo

```bash
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
ros2 topic echo /stereo/left/camera_info --once
ros2 topic echo /stereo/right/camera_info --once
```

确认左右帧率稳定、时间戳成对、K/R/P 非零且 frame_id 正确。

### 2. 极线校正和视差

在 Foxglove 同时显示：

```text
/stereo/left/image_rect
/stereo/right/image_rect
/stereo/disparity
```

同一物体特征点的校正后 y 坐标差应小于 1 px，优先达到 RMS 0.5 px 以下。若垂直误差
明显，不要先调整 SGBM；应重新检查标定板、左右顺序和双目标定。

### 3. 深度尺度

把大块、平整、纹理丰富且正对相机的目标分别放在 0.5、1、2、3 m。距离应从左目光心
沿前向测量，而不是从车头保险杠测量。观察：

```bash
ros2 topic hz /stereo/depth/image
ros2 topic hz /stereo/points2
```

频率诊断时优先读取不含大图的状态话题，避免诊断订阅本身改变被测结果：

```bash
ros2 topic echo /stereo/splitter/status --once
ros2 topic echo /stereo/pair_throttle/status --once
ros2 topic echo /stereo/depth/status --once
ros2 topic echo /stereo/pointcloud_filter/status --once
```

验收目标：

- 0.5～2 m 中值误差不超过 5%。
- 3 m 中值误差不超过 10%。
- 左右640×480校正识别图均不低于 9 Hz。
- 深度、完整点云、导航点云和 Scan 均不低于 3.5 Hz，目标为 4 Hz。
- 导航链 P95 帧间隔不超过 400 ms，连续无输出时间不超过 1 s。
- 从采集时间戳到深度、导航点云或 Scan 的端到端延迟 P95 小于 600 ms。

如果所有距离都按相同比例偏大或偏小，优先检查棋盘格实际尺寸和标定基线；如果边缘误差
大而中心正常，优先检查畸变和极线校正；如果低纹理区域破碎，应最后再调整 SGBM。

### 4. TF 和障碍点云

```bash
ros2 run tf2_ros tf2_echo base_link stereo_left_optical_frame
ros2 topic hz /nav/stereo_obstacle_points
ros2 topic echo /stereo/pointcloud_filter/status --once
ros2 topic echo /stereo/scan --once
```

在相机前方放置 0.05～0.80 m 高、0.25～4.0 m 范围内的障碍物：

- `/nav/stereo_obstacle_points` 中应出现障碍。
- local costmap 应标记障碍。
- 障碍出现后 0.75 秒内应标记，移除后 1.5 秒内应清除。
- `/stereo/scan` 的角度、距离和 `base_link` frame 应与过滤点云一致。
- 障碍进入 0.40 m 区域时 `/cmd_vel_safe` 不得超过输入速度的 50%。
- 障碍进入 0.25 m 区域时 `/cmd_vel_safe` 必须为零。
- 停止发布 `/stereo/scan` 后 0.75 秒内必须输出零速。

### 5. 完整稳定性

先关闭 Foxglove，连续运行底盘、Nav2和双目 30 分钟，记录：

- USB reset、丢帧和 TF error。
- RSS 内存是否持续增长。
- CPU 温度、是否热降频。
- RK3588 平均总 CPU 是否低于 75%。

CPU 超限时依次调整：

1. 检查是否误订阅 raw 深度或完整稠密点云，并关闭 Foxglove 后复测。
2. 确认最新帧调度为识别 10 Hz、导航 4 Hz，所有高带宽队列为深度 1。
3. 优化节点实现或使用 RK3588 硬件加速；不得降低 640×480 图像质量或缩小
   `disparity_range=128` 来掩盖超限。

不要通过增大 `max_depth`、伪造无效深度或跳过 TF 错误来掩盖性能问题。

关闭 Foxglove 后可用仓库的一次性工具测量60秒。工具按阶段串行订阅单个话题或一对相邻
话题，避免同时反序列化多路大消息；仍只应在验收时运行，同时间戳阶段差值用于定位瓶颈：

```bash
ros2 run robot_perception stereo_pipeline_benchmark --ros-args \
  -p duration:=60.0 \
  -p output_file:=/tmp/stereo_pipeline_result.json
```

需要人工监测时再启动 Bridge：

```bash
ros2 launch robot_navigation stereo_robot.launch.py
```

Foxglove 只显示左右校正压缩图、深度预览、过滤点云和 `/stereo/scan`，不要持续订阅
`/stereo/depth/image` 或 `/stereo/points2`。

## 十、哪些变化必须重新标定

必须重新做双目标定：

- 更换任一镜头或整个相机。
- 改变左右镜头相对位置、焦距或对焦状态。
- 更改单目分辨率或图像裁剪方式。
- 相机支架发生形变或拆装后无法保证左右关系不变。

只需重新测安装外参：

- 整个双目模组内部没有变化，但在车上的位置或角度改变。

通常不需要重新标定：

- 只调整 SGBM、点云体素、Nav2 距离阈值。
- 只改变 Foxglove 显示或压缩传输设置。
