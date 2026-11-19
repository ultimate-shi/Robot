<!-- 作用：说明ESP32-C3的0.5编码器/角度/弧度反馈及固件构建；使用方法：先读API，分别选择master/slave并配套升级工具。 -->
# ESP32-C3机械臂关节固件

源码对应机械臂文本API 0.5：R1/R2状态为“编号、原始编码器、角度°、弧度、电流、PWM、状态、故障、模式”，不再返回速度及剩余有效期/反馈龄期。原始编码器范围0～4095，不可用为-1；J查询与设置已移除，主从及串口工具须配套升级。角度/PID直接使用实际浮点数值。位置命令R发送一次后在板端执行PID，H逐轴选择到位停转或保持，U直接输出舵机脉宽且100ms到期。Mac或RK3588通过原生USB连接master；master同时控制本地关节1并通过UART控制从板关节2。

U不要求零位、方向或中立标定；编码器有效和真实故障检查仍保留。master关节1不依赖从板在线，slave收不到有效总线数据时仍受100ms看门狗约束。两轴默认中立均为1500µs，上电、停止和PWM到期输出该值；旧配置未设置中立时也回退到1500µs，已设置的自定义中立保留。R角度控制仍要求完成标定。仅当标定完整时启用依赖方向映射的限位制动，已设置零位的绝对越限检查仍保留。100ms限制的是本次U脉宽输出时间，不代表断电或对未知中立的物理停转保证。

本机械臂是两关节控制板固件测试台；每轴独立闭环，上位机负责是否同步到位。ESP32-C3继续使用单精度float（软件浮点），不依赖硬件FPU。

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

给第二块板烧录时使用`-e slave`。不要把slave镜像写入水平master板。关节设置使用独立arm_cfg分区（0x110000），首次烧录不自动擦除原通用NVS；新板上电输出默认中立1500µs。已经保存自定义中立的板上电输出该中立脉宽，等待新指令，不恢复旧目标。

## VS Code烧录与首次下载

安装PlatformIO IDE扩展，直接打开本目录，等待项目配置结束，再选择`PROJECT TASKS → slave → General → Upload`烧录第二块板；master板选择对应的master任务。工程默认环境是master，第二块板不要直接用未明确选择环境的通用上传按钮。

扩展通常使用`~/.platformio/`存储工具包，和命令行指定的`PLATFORMIO_CORE_DIR`可以不同；因此终端曾成功构建，并不代表扩展已经有同一套工具。若长时间停在Downloading 0%，在“输出 → PlatformIO Project Configuration”确认卡住的包名，避免同时启动多次配置或上传。补齐工具后用`Cmd+Shift+P → Developer: Reload Window`重载窗口，再选择明确的slave任务；无需改固件或反复按BOOT处理下载问题。

## 核心测试与只读检查

逐步操作见[通信、零位与固件测试](../../ROS2/robot_arm/docs/firmware_test_guide.md)。推荐串口工具加`--interactive`，按回车整行发送；R1查询当前raw、角度和弧度，不发运动命令。

```bash
ESP32/arm_joint/scripts/test.sh
python3 -m unittest discover -s ESP32/arm_joint/test -p 'test_*.py'
python3 ESP32/arm_joint/scripts/api_cli.py --port /dev/cu.usbmodemXXXX
```

历史0.2/0.3已做部分身份和传感器检查，过程见机械臂progress.md；运动标定与从板联动尚未验收。本次0.5尚未烧录或实机运动验收，主从板和工具必须配套升级；不兼容旧版状态行。旧schema=2配置可读取，H默认0，save后写入schema=3，移除J不改变NVS布局及已有内部控制配置。原Flash备份位于backups目录，不纳入版本管理。

测试脚本依赖主机clang，直接编译实际`arm_core.c`并启用AddressSanitizer/UndefinedBehaviorSanitizer，不使用模拟副本。串口脚本依赖pyserial，默认只查身份、状态、ADC原始值和配置，不发运动/标定命令。PWM的20Hz刷新须显式指定`--command`、`--repeat`，结束发送对应轴X。角度命令默认一次发送并等待到位（保持模式按连续角度观测估算速度）（`--wait-timeout`默认60秒）；保持模式到位后正常退出不发送X，超时/故障/中断则发送X。交互模式便于运动中覆盖命令，退出时停止。

默认每关节速度参考10°/s、加速度参考30°/s²、Kp=2、Ki=Kd=0；脉宽/转速映射20µs/(°/s)是未实测的初始值，不能当成MG995真实速度曲线或已经整定的PID。实际使用前需逐轴低速标定和检查限位。固件不会自动假定当前安装位置就是零位。

`src/main.c`为ESP-IDF硬件/USB/总线适配，`src/arm_core.c`为纯C协议与控制，`test/test_core.c`验证边界。构建产物在`.pio/`，不纳入版本管理。

示例（完成标定后）：`P1 1.25 0.01 0`；STOP时`H1 1`、`$save 1`开启并保存保持。批处理单次角度：

```bash
python3 ESP32/arm_joint/scripts/api_cli.py --port /dev/cu.usbmodemXXXX --command 'R1 2.5'
```

H=0到位后停止，H=1到位后继续位置纠偏。从板有效通信100ms看门狗与PWM期限相互独立；从板离线不再停止master本地轴，已在线从板返回真实故障时仍保留整机故障联锁。仅主轴指令可独立执行，包含关节2的运动命令仍检查从板在线。上位机USB断开不等于停止角度控制。到位判据为误差≤0.5°、滤波速度≤1°/s连续200ms；速度滤波时间常数100ms，仍需实机整定。
