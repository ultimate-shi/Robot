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
v4l-utils
```

查看设备和真实支持的模式：

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
udevadm info --query=property --name=/dev/video0
```

必须在输出中找到计划使用的拼接模式，例如 `1280x480 MJPG 30 fps`。如果只有
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

构建并加载工作区：

```bash
cd /home/shijiahao/Downloads/ros2/robot_ws
colcon build --packages-select robot
source install/setup.bash
```

只启动原图拆分，暂不启动视差和点云：

```bash
ros2 launch robot stereo_camera.launch.py calibration_mode:=true
```

另开终端检查：

```bash
source install/setup.bash
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
- 测量：用卡尺测量打印后的实际格子，不要默认打印比例正好是 100%。

如果实测格子为 29.82 mm，命令必须写 `--square 0.02982`。格子尺寸误差会直接按比例
进入基线和深度尺度。

标定前固定好镜头焦距、曝光模式、分辨率和左右相对位置。标定后不允许旋转镜头、改变
焦距、拆开相机支架或切换分辨率；发生这些变化必须重新标定。

## 六、执行双目标定

保持 `calibration_mode:=true` 正在运行，另开带图形界面的 Linux 终端执行：

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
mkdir -p src/robot/config/cameras/model_serial_640x480
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/stereo_calibration
```

从解压内容中找到左右相机 YAML，分别保存为：

```text
src/robot/config/cameras/model_serial_640x480/left.yaml
src/robot/config/cameras/model_serial_640x480/right.yaml
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
src/robot/config/cameras/<型号_序列号_单目分辨率>/left.yaml
src/robot/config/cameras/<型号_序列号_单目分辨率>/right.yaml
```

矩阵数值全部来自 `camera_calibration`。不要手工把标称焦距或 65 mm 基线写进去。

### 2. 修改相机取流 profile

检查 `src/robot/config/stereo_camera.yaml`：

- `video_device`：应为 udev 稳定路径。
- `framerate/pixel_format/image_width/image_height`：必须与实际取流和标定模式一致。
- `left_first`：必须与遮挡测试一致。
- SGBM 参数先保留默认值，完成几何标定后再根据性能和点云质量调整。

### 3. 修改相机安装 TF

编辑 `src/robot/urdf/robot.xacro` 中：

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
colcon build --packages-select robot
source install/setup.bash
```

### 4. 启动时传入当前标定文件

相机单测：

```bash
ros2 launch robot stereo_camera.launch.py \
  calibration_mode:=false \
  left_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot/config/cameras/model_serial_640x480/left.yaml \
  right_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot/config/cameras/model_serial_640x480/right.yaml
```

完整实机：

```bash
ros2 launch robot stereo_robot.launch.py \
  left_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot/config/cameras/model_serial_640x480/left.yaml \
  right_calibration_file:=/home/shijiahao/Downloads/ros2/robot_ws/src/robot/config/cameras/model_serial_640x480/right.yaml
```

只有再次 `colcon build` 后，profile 才会被安装到 `install/robot/share/robot/config/cameras`。
直接传源码目录绝对路径可以立即测试，但正式部署建议构建后使用安装空间中的文件。

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

验收目标：

- 0.5～2 m 中值误差不超过 5%。
- 3 m 中值误差不超过 10%。
- 深度和点云稳定输出至少 15 Hz。
- 端到端延迟 P95 小于 200 ms。

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
- 移除后约 1 秒内应清除。
- `/stereo/scan` 的角度、距离和 `base_link` frame 应与过滤点云一致。

### 5. 完整稳定性

连续运行底盘、Nav2、双目和 Foxglove Bridge 30 分钟，记录：

- USB reset、丢帧和 TF error。
- RSS 内存是否持续增长。
- CPU 温度、是否热降频。
- RK3588 平均总 CPU 是否低于 75%。

CPU 超限时依次调整：

1. 增大 `frame_skip` 或降低 `max_input_rate`。
2. 减小 `disparity_range`，但仍须为 16 的倍数。
3. 降低相机分辨率并对新分辨率重新标定。

不要通过增大 `max_depth`、伪造无效深度或跳过 TF 错误来掩盖性能问题。

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
