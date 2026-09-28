<!-- 作用：说明本仓库多 Agent 的职责划分、协作协议和使用方法。 -->
<!-- 使用方法：开始复杂任务前由协调 Agent 阅读；用户可在 Codex 中直接点名下列 Agent 并要求并行协作。 -->
# 多 Agent 执行与协作

本仓库使用一个协调 Agent 和五个专业 Agent。专业 Agent 是按稳定职责划分的工作角色，一个 Agent 可以维护多个紧密相关的 ROS 2 包；任务中的临时并行线程由协调 Agent 创建，不要求每个包常驻一个独立进程。

| Agent | 管理范围 | 典型任务 |
| --- | --- | --- |
| `system_coordinator` | 全仓库协调 | 拆解跨包问题、指定文件所有者、汇总结论、整体验证 |
| `brain_owner` | `robot_brain` | Qwen、网页、多用户租约、工具调度、ROS Bridge |
| `perception_owner` | `robot_perception`、`robot_stereo_components` | 双目、IMU、点云、语义、地形、虚拟传感器 |
| `navigation_owner` | `robot_navigation` | 状态估计、RTAB-Map、Nav2、地图快照、任务预演 |
| `control_hardware_owner` | `robot_control`、`robot_hardware` | ros2_control、串口、轮腿控制、里程计、速度安全链 |
| `platform_integration_owner` | `robot_interfaces`、`robot_description`、`robot_main` | 公共接口、URDF/TF、整车 real/virtual 编排 |

## 协作协议

1. `system_coordinator` 先根据症状列出可能经过的包、话题、服务、Action、TF 和 launch 边界。
2. 各 owner 并行做只读排查，返回证据、疑点、影响面和建议验证，不立即争抢共享文件。
3. 协调 Agent 汇总根因后建立“一个文件一个写入者”的清单。多个 Agent 可以同时改不同文件；公共接口、统一 launch 和共享配置只交给一个 Agent 收口。
4. 修改公共接口时，生产者和所有消费者必须在同一任务内检查；修改 TF 时同时检查 URDF 发布者、里程计发布者、定位/建图消费者；修改速度链时从 Nav2 输出一直检查到硬件命令。
5. 各 owner 完成包内测试，协调 Agent 最后执行依赖包构建、launch 解析和 `git diff --check`，并把结果写入当天 `progress.md`。

## 常见并行组合

- “真实超声波有数据但不停车”：`control_hardware_owner` 检查驱动、QoS 和安全门控；`perception_owner` 检查 Range/Scan 转换；`navigation_owner` 检查代价地图消费；协调 Agent 验证完整速度链。
- “建图漂移或 TF 跳变”：`perception_owner` 检查相机/IMU 时间戳；`navigation_owner` 检查里程计融合和 RTAB-Map；`platform_integration_owner` 检查 URDF 外参与 frame 归属。
- “网页目标可见但任务不执行”：`brain_owner` 检查租约与工具调用；`navigation_owner` 检查任务 Action 和 Nav2；`platform_integration_owner` 复核类型化接口。
- “实机正常、虚拟模式失败”：各领域 owner 检查自身实现，`platform_integration_owner` 对比 `robot_main` 的 real/virtual 参数和上游接口。

## 调用示例

在 Codex 中可以直接说：

```text
使用 system_coordinator 排查建图时机器人原地旋转的问题。
让 perception_owner、navigation_owner 和 platform_integration_owner 并行只读排查，
等全部结论回来后再分配修改，最后统一构建和验证。
```

项目配置位于 `.codex/config.toml` 和 `.codex/agents/*.toml`。新会话会加载这些项目级 Agent；当前 Codex 版本也支持用户直接要求创建临时子 Agent。并行线程共享同一工作区，所以并发读取很适合排查，并发写入必须遵守文件所有权。

## 包文档

- [robot_brain](packages/robot_brain.md)
- [robot_control](packages/robot_control.md)
- [robot_description](packages/robot_description.md)
- [robot_hardware](packages/robot_hardware.md)
- [robot_interfaces](packages/robot_interfaces.md)
- [robot_main](packages/robot_main.md)
- [robot_navigation](packages/robot_navigation.md)
- [robot_perception](packages/robot_perception.md)
- [robot_stereo_components](packages/robot_stereo_components.md)
