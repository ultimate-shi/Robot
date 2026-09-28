<!-- 作用：说明 robot_perception 的双目、IMU、点云、语义、地形和虚拟传感职责。 -->
<!-- 使用方法：修改传感器输入、时间戳、QoS、TF、视觉推理或感知输出前先阅读本文件。 -->
# robot_perception 功能包

## 职责与边界

`robot_perception` 统一实机和仿真感知，包含 UVC 横拼双目采集、拆分/校正/节流/视差/深度/点云过滤、语义检测与分割、地形分析、真实 GY95T IMU、虚拟 IMU、8 路虚拟超声波、Range 转 Scan、快照局部观察和验收诊断。

本包与 `robot_stereo_components` 一起归 `perception_owner` 管理。固定真实相机数据不得与运动虚拟机器人混用。

## 入口和启动

- 14 个 Python 入口覆盖双目拆分、配对节流、深度、点云过滤、地形、语义、IMU、虚拟传感器和诊断。
- `stereo_camera.launch.py` 组合 usb_cam、拆分、左右校正、节流、视差、深度和点云；`splitter_backend` 默认使用 C++，Python 是回退实现。
- `stereo_perception.launch.py` 组合语义和点云过滤；其余 launch 按同名 YAML 分别启动 IMU、地形、虚拟传感器和验收采样。
- 相机标定位于 `config/cameras/<profile>/left.yaml` 与 `right.yaml`。

## 关键接口

- 双目主链：`/stereo/image_raw` 到左右 raw/CameraInfo、左右 rect、导航节流图、`/stereo/disparity`、深度、`/stereo/points2`，再输出 `/nav/stereo_obstacle_points` 和 `/stereo/scan`。
- 语义链消费右目、右目对齐深度和 CameraInfo，经 TF 定位到 `map`，输出类型化检测、覆盖图、语义障碍/清除点；提供 `/perception/detect_objects` 与 `/perception/set_detection_mode`。
- GY95T 输出 raw IMU，经 Madgwick 生成 `/sensors/imu/data`，该链不发布 TF。
- 虚拟超声查询 `map -> sonar_*`，输出 8 路 `/ultrasonic/*`；`range_to_scan` 汇总为 `/scan`。
- 点云、图像和 CameraInfo 必须保留匹配时间戳与 frame；传感器话题需核对 QoS。

## 验证

```bash
colcon build --symlink-install --packages-select robot_perception robot_stereo_components
python3 -m pytest -q src/robot_perception/test
source install/setup.bash
ros2 launch robot_perception stereo_camera.launch.py --show-args
ros2 launch robot_perception stereo_perception.launch.py --show-args
git diff --check
```

`/perception/points` 的来源随运行模式变化，真实设备、标定、推理网关与完整 TF 必须在运行态确认。
