<!-- 使用方法：从仓库根目录进入 ROS2/ 阅读并运行 ROS 2 工作区；STM32 工程位于 STM32/。 -->
# Robot

机器人项目仓库，按控制平台分为 ROS 2 工作区和 STM32 控制板工程：

```text
Robot/
├── ROS2/   # ROS 2 Jazzy 工作区，src、scripts、docs 等目录直接放在这里
└── STM32/  # 两套 STM32 控制板工程
    ├── Sevo_Echo_F103_V2/
    └── motor2_stm32f401rct6_mini/
```

ROS 2 工作区的功能介绍、构建和运行方法见 [ROS2/README.md](ROS2/README.md)。从仓库根目录构建时：

```bash
cd ROS2
colcon build --symlink-install
source install/setup.bash
```

实机硬件插件在 `/hardware/serial_tx` 发布三块控制板每次实际串口写入及事务结果；
默认 MCAP 日志采集后可在 9100 端口的 `/log/` 页面按该话题查询。

机器人控制台位于同一 9100 端口的 `/robot/`，提供手动驾驶、地图导航、相机与目标、
探索与跟随、状态和 `/robot/console` 实时终端输出。`/log/` 继续作为独立的历史日志查询页。
网页入口与控制台节点、页面、配置均位于 `ROS2/src/robot_web/`。
网页手动速度无需连接心跳和网页限速，直接送到底盘运动控制器；导航速度仍经避障层。
首页标题下方显示底盘故障和反馈中断提示；未经验证的轮速指令会被拒绝并停车。
右后轮 S7 速度闭环仍未通过实测；方向可在轮位标定中配置，
默认和诊断配置均继续禁用 S7，完成架空实测验收后才能启用。
显式按下软件停止后，须在首页解除才能再次运动。
实机接地运动仍需先完成仿真和架空验收。启动与操作步骤见
[ROS2/README.md](ROS2/README.md)。

STM32 工程可分别用 STM32CubeMX/CubeIDE 或自带的 CMake 配置打开。每个工程的 `build/`
输出不纳入版本控制。
SE2（`Sevo_Echo_F103_V2`）固件启动时六路舵机 PWM 均输出 1500 µs；某路收到非中位命令后，
若 100 ms 内没有新命令，该路自动回到 1500 µs 并持续输出。显式发送 0 仍可关闭该路输出。
