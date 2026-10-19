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

STM32 工程可分别用 STM32CubeMX/CubeIDE 或自带的 CMake 配置打开。每个工程的 `build/`
输出不纳入版本控制。
SE2（`Sevo_Echo_F103_V2`）固件启动时六路舵机 PWM 均输出 1500 µs；某路收到非中位命令后，
若 100 ms 内没有新命令，该路自动回到 1500 µs 并持续输出。显式发送 0 仍可关闭该路输出。
