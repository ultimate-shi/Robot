# Repository Guidelines

## 项目目标

本项目基于现实中的机器人小车模型进行 ROS 2 仿真搭建。当前已有小车的现实模型，仓库目标是逐步构建对应的仿真环境，最终实现数字孪生效果：在仿真中尽可能还原现实小车的结构、传感器、运动控制、反馈状态和路径规划能力。

开发时应优先保持仿真模型与现实小车一致。涉及底盘、轮子、传感器、控制参数、地图和运动逻辑的改动，都应考虑是否会影响现实小车与仿真小车之间的对应关系。

## 代码目录结构

- `src/robot/robot/`：ROS 2 Python 节点代码，例如底盘控制、手柄遥控、虚拟传感器、避障和地形相关节点。
- `src/robot/launch/`：启动文件，用于组合运行仿真、RViz、Foxglove、底盘和地图等功能。
- `src/robot/config/`：ROS 参数和控制器配置，例如手柄、控制器和地形参数。
- `src/robot/urdf/`：机器人结构描述文件，主要使用 xacro 组织车体、轮子、雷达、IMU 等部件。
- `src/robot/meshes/`：机器人模型网格资源，包括 OBJ 和 MTL 文件。
- `src/robot/map/`：地图相关资源，例如 PGM、YAML 和 PLY 文件。
- `src/robot/world/`：仿真世界文件，例如 SDF 场景。
- `src/robot/test/`：测试与 lint 检查文件。
- `src/robot/setup.py`：Python 包安装配置和 `ros2 run` 可执行入口声明。

常用命令：

- `colcon build --packages-select robot`：构建 `robot` 包。
- `source install/setup.bash`：加载工作区环境。
- `ros2 launch robot simulation.launch.py`：启动仿真入口。

## 代码规范
写代码同时用中文写注释

完成代码修改后同时加上说明，并且要修改README.md文件

新增 ROS 节点时，应在 `src/robot/robot/` 中实现，并在 `src/robot/setup.py` 的 `console_scripts` 中声明入口。配置项优先放入 `src/robot/config/*.yaml` 或 launch 参数中，不要把设备路径、控制参数、网络地址等机器相关内容硬编码到节点代码里。

## 文档记录
每个对话结束后把做了什么，当前卡在哪里，踩过的坑有哪些记录下来，形成项目的过程文档，记录到progress.md中，只需要修改当天的内容，再把本次修改的内容做一个简单的总结方便查看修改了什么
