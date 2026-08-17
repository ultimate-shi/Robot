<!-- 使用方法：所有在 /home/radxa/Robot 内工作的开发者和代理必须遵守本文件。 -->
# Repository Guidelines

## 项目定位

本仓库是 ROS 2 Jazzy 机器人小车工作区，目标是让实机与仿真共用模型、传感器接口、运动控制和导航链路，逐步形成数字孪生。修改底盘、轮子、头部、传感器、TF、控制参数或地图时，应优先保证与实车结构和坐标关系一致。

当前已具备默认虚拟机器人、双目视觉处理、RTAB-Map 建图、地图快照、Nav2 导航预演和分级避障；真实底盘、IMU、超声波仍待接入。固定在环境中的真实相机数据不得与运动中的虚拟机器人混用。

## 主要目录

- `src/robot_{brain,perception,navigation,control}/`：按职责拆分的 Python 节点、launch、配置和测试。
- `src/robot_interfaces/`、`src/robot_description/`：跨包 ROS 接口与静态机器人模型资源。
- `src/robot_stereo_components/`：高带宽双目处理 C++ 节点。
- `scripts/`、`docker/`、`docs/`：运行脚本、Jazzy ARM64 容器和专项文档。
- `README.md`：当前使用方法；`progress.md`：按日期记录开发过程。

## 开发约定

- 代码注释和修改说明使用中文；保持现有 ROS 话题、服务、Action、TF 和可执行入口兼容，确需变更时同步说明影响。
- 新 Python 节点放入对应领域 Package，并在所属 `setup.py` 的 `console_scripts` 注册；高带宽图像处理优先评估放入 C++ 包。
- 创建或修改文件以后在文件的头部写上该文件的使用方法。
- launch文件应在每个参数写上描述。
- 设备路径、网络端口、阈值和控制参数放入所属 Package 的 `config/*.yaml` 或 launch 参数，不在节点中硬编码。
- 实机与仿真实现尽量保持上游接口一致；任务层生成导航目标，安全层过滤速度，避免绕过 Nav2 或安全链直接控制底盘。
- 不提交 `build/`、`install/`、`log/`、标定临时文件或运行生成的地图快照。

## 构建与验证

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_navigation robot.launch.py
```

按修改范围运行相关测试；至少检查受影响的包可构建、launch 可解析、Python 测试通过，并执行 `git diff --check`。双目和地图核心测试见 README 的“测试”章节。

## 文档记录

每次修改都要同步更新 `README.md` 中受影响的现状或用法，并只编辑 `progress.md` 当天的记录，简要写明：本次修改、验证结果、当前卡点和踩坑；没有卡点或踩坑时明确写“无”。
