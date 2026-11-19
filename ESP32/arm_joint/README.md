<!-- 作用：说明ESP32-C3关节固件构建、烧录和验证；使用方法：先读API，分别选择master/slave环境。 -->
# ESP32-C3机械臂关节固件

源码对应机械臂文本API 0.2：位置命令R在板端执行PID，U直接输出舵机脉宽，两个模式均逐轴100ms超时。Mac或RK3588通过原生USB连接master；master同时控制本地关节1并通过UART控制从板关节2。

API、标定、反馈格式和实测边界见[机械臂硬件API](../../ROS2/robot_arm/docs/hardware_api.md)。板级引脚、超时、初始参数集中在`include/arm_config.h`；角色由PlatformIO环境指定。电流/温度当前仅采集ADC原始值，电流物理量返回NA，未启用未经校准的电流/温度保护。

两板通信超时时，可分别通过各自USB发送只读`$bus`，查看本板UART收发计数；`$busraw`查看最近收到的原始字节。诊断步骤见[固件测试指南](../../ROS2/robot_arm/docs/firmware_test_guide.md)。这些命令需要包含对应诊断的新固件，旧版返回CMD。

## 构建和烧录

固定PlatformIO Core 6.1.18、espressif32 6.10.0、ESP-IDF 5.4.0；需要Python、网络及首次下载的交叉编译工具。

```bash
cd ESP32/arm_joint
python3 -m venv .venv
.venv/bin/pip install 'platformio==6.1.18'
.venv/bin/pio run -e master -e slave
.venv/bin/pio run -e master -t upload --upload-port /dev/cu.usbmodemXXXX
```

给第二块板烧录时使用`-e slave`。不要把slave镜像写入水平master板。关节设置使用独立arm_cfg分区（0x110000），首次烧录不自动擦除原通用NVS；新板未标定不输出脉冲。已经保存中立点的板上电持续输出该中立脉宽，等待新指令，不恢复旧目标。

## VS Code烧录与首次下载

安装PlatformIO IDE扩展，直接打开本目录，等待项目配置结束，再选择`PROJECT TASKS → slave → General → Upload`烧录第二块板；master板选择对应的master任务。工程默认环境是master，第二块板不要直接用未明确选择环境的通用上传按钮。

扩展通常使用`~/.platformio/`存储工具包，和命令行指定的`PLATFORMIO_CORE_DIR`可以不同；因此终端曾成功构建，并不代表扩展已经有同一套工具。若长时间停在Downloading 0%，在“输出 → PlatformIO Project Configuration”确认卡住的包名，避免同时启动多次配置或上传。补齐工具后用`Cmd+Shift+P → Developer: Reload Window`重载窗口，再选择明确的slave任务；无需改固件或反复按BOOT处理下载问题。

## 核心测试与只读检查

逐步操作见[通信、零位/中立与固件测试](../../ROS2/robot_arm/docs/firmware_test_guide.md)。标定推荐串口工具加`--interactive`，按回车整行发送并保持RAM配置；先确认J设置立即输出的中立值风险。

```bash
ESP32/arm_joint/scripts/test.sh
python3 ESP32/arm_joint/scripts/api_cli.py --port /dev/cu.usbmodemXXXX
```

本次master已在2026-11-16烧录并通过身份/传感器只读检查；运动标定与从板联动尚未验收。原Flash备份位于`backups/master_before_2026-11-16.bin`，不纳入版本管理。

测试脚本依赖主机clang，直接编译实际`arm_core.c`并启用AddressSanitizer/UndefinedBehaviorSanitizer，不使用模拟副本。串口脚本依赖pyserial，默认只查身份、状态、ADC原始值和配置，不发运动/标定命令。运动的20Hz刷新须显式指定`--command`、`--repeat`，执行结束会发X。

默认每关节速度参考10°/s、加速度参考30°/s²、Kp=2、Ki=Kd=0；脉宽/转速映射20µs/(°/s)是未实测的初始值，不能当成MG995真实速度曲线或已经整定的PID。实际使用前需逐轴低速标定和检查限位。固件不会自动假定当前安装位置就是零位。

`src/main.c`为ESP-IDF硬件/USB/总线适配，`src/arm_core.c`为纯C协议与控制，`test/test_core.c`验证边界。构建产物在`.pio/`，不纳入版本管理。
