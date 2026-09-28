<!-- 作用：说明 robot_main 的整车统一启动职责和 real/virtual 模式边界。 -->
<!-- 使用方法：修改整车启动顺序、模式开关或跨包参数传递前先阅读本文件。 -->
# robot_main 功能包

## 职责与边界

`robot_main` 是实机和数字孪生的唯一整车编排入口，自身不实现业务节点。入口为：

```bash
ros2 launch robot_main robot.launch.py mode:=real
ros2 launch robot_main robot.launch.py mode:=virtual map_yaml_file:=/path/to/map.yaml
```

本包归 `platform_integration_owner` 管理。修改组合关系时需要对应领域 owner 复核参数和接口。

## 模式组成

- `real`：使用 `RobotSerialSystem`，组合模型、硬件控制器、底盘与轮动作、轮里程计、安全链、状态估计和 Nav2；可选 IMU、双目、点云过滤、超声波转 Scan、语义感知和车身平衡。没有已有地图时可启动头部建图锁、RTAB-Map 和快照服务。
- `virtual`：使用 `mock_components/GenericSystem`，要求显式提供地图 YAML，并组合 PLY 环境与虚拟传感器。
- `config/robot.yaml`：保存默认 PLY、日志级别、错峰延迟、Foxglove 端口和功能开关。

实机和虚拟实现应向上提供一致的话题、服务、Action 和 TF 语义。固定真实相机数据不得与运动中的虚拟机器人混用。

## 验证

```bash
colcon build --symlink-install --packages-up-to robot_main
source install/setup.bash
ros2 launch robot_main robot.launch.py --show-args
git diff --check
```

当前 launch 直接导入 `yaml`，但 `package.xml` 未声明 `python3-yaml`；后续依赖整理时应补充并复测。
