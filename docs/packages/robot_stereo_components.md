<!-- 作用：说明 robot_stereo_components 的高带宽 C++ 双目拆分节点。 -->
<!-- 使用方法：修改横拼图拆分、标定加载、QoS 或高带宽图像路径前先阅读本文件。 -->
# robot_stereo_components 功能包

## 职责与归属

`robot_stereo_components` 提供 `stereo_splitter_cpp`，将 UVC 横向拼接图拆成左右图像并发布匹配的 CameraInfo。包内没有独立 launch/config，由 `robot_perception/stereo_camera.launch.py` 传入话题、左右顺序、标定 URL 和限频参数。

本包与 `robot_perception` 一起归 `perception_owner` 管理。Python splitter 作为调试回退，默认链路使用本 C++ 节点。

## 接口和行为

- 输入默认 `/stereo/image_raw`。
- 输出默认 `/stereo/left/image_raw`、`/stereo/right/image_raw` 及两路 `camera_info`。
- 使用 `camera_info_manager` 加载左右标定；正式模式缺少有效标定时拒绝继续。
- 支持左右顺序和识别频率配置，标定模式不执行识别限频。
- 输出使用 Reliable、深度 1 QoS；修改时需与 image_proc 和下游订阅者联合核对。

## 验证

```bash
colcon build --symlink-install --packages-select robot_stereo_components
source install/setup.bash
ros2 run robot_stereo_components stereo_splitter_cpp --ros-args -p calibration_mode:=true
git diff --check
```
