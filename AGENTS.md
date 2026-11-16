<!-- 使用方法：本文件约定适用于 Robot 仓库；ROS 2 工作区位于 ROS2/robot_car/。 -->
# Repository Guidelines

## 项目定位

仓库的 ROS 2 Jazzy 机器人小车工作区位于 `ROS2/robot_car/`，目标是让实机与仿真共用模型、传感器接口、运动控制和导航链路，逐步形成数字孪生；STM32 控制板工程位于仓库根目录 `STM32/`。修改底盘、轮子、头部、传感器、TF、控制参数或地图时，应优先保证与实车结构和坐标关系一致。

当前已具备默认虚拟机器人、双目视觉处理、IMU、RTAB-Map 建图、地图快照、Nav2 导航预演、分级避障，以及 DMC0/DMC1/SE2 实机底盘和 8 路超声波接入。固定在环境中的真实相机数据不得与运动中的虚拟机器人混用。

机械臂独立工作区位于 `ROS2/robot_arm/`，初始功能包为 `robot_arm`；与小车分别构建，过程记录分别放在各自工作区。

## 主要目录

- `ROS2/robot_car/src/robot_{brain,perception,navigation,control}/`：按职责拆分的 Python 节点、launch、配置和测试。
- `ROS2/robot_car/src/robot_hardware/`、`ROS2/robot_car/src/robot_main/`：实机 ros2_control 插件与整车统一编排入口。
- `ROS2/robot_car/src/robot_interfaces/`、`ROS2/robot_car/src/robot_description/`：跨包 ROS 接口与静态机器人模型资源。
- `ROS2/robot_car/src/robot_stereo_components/`：高带宽双目处理 C++ 节点。
- `ROS2/robot_car/scripts/`、`ROS2/robot_car/docker/`、`ROS2/robot_car/docs/`：运行脚本、Jazzy ARM64 容器和专项文档。
- `ROS2/robot_car/README.md`：ROS 2 使用方法；`ROS2/robot_car/progress.md`：按日期记录开发过程。
- `STM32/`：STM32F103 和 STM32F401 控制板工程。

## 开发约定

- 代码注释和修改说明使用中文；保持现有 ROS 话题、服务、Action、TF 和可执行入口兼容，确需变更时同步说明影响。
- 新 Python 节点放入对应领域 Package，并在所属 `setup.py` 的 `console_scripts` 注册；高带宽图像处理优先评估放入 C++ 包。
- 创建或修改文件以后在文件的头部写上该文件的作用和使用方法。
- launch文件应在每个参数写上描述。
- 设备路径、网络端口、阈值和控制参数放入所属 Package 的 `config/*.yaml` 或 launch 参数，不在节点中硬编码，参数文件名称跟launch文件保持相同。
- 实机与仿真实现尽量保持上游接口一致；任务层生成导航目标，安全层过滤速度，避免绕过 Nav2 或安全链直接控制底盘。
- 不提交 `build/`、`install/`、`log/`、标定临时文件或运行生成的地图快照。
- 所有网页服务都走9100端口，通过不同路径区分不同服务

## 构建与验证

```bash
cd ROS2/robot_car
colcon build --symlink-install
source install/setup.bash
```

按修改范围运行相关测试；至少检查受影响的包可构建、launch 可解析、Python 测试通过，并执行 `git diff --check`。

## 文档记录

ROS相关代码有修改，更新对应ROS目录下的`progress.md`文件。
