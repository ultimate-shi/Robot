<!-- 作用：列出 robot_interfaces 提供的跨包类型化契约和修改规则。 -->
<!-- 使用方法：新增或修改 msg、srv、action 前阅读；修改后检查全部生产者和消费者。 -->
# robot_interfaces 功能包

## 职责与归属

`robot_interfaces` 是感知、任务规划、控制和硬件之间的稳定类型契约，不包含运行节点、launch 或参数文件。本包归 `platform_integration_owner` 管理。

## 接口清单

- 消息：`SemanticDetection`、`SemanticDetectionArray`、`TerrainState`、`MissionState`、`ChassisState`、`BodyBalanceStatus`、`WheelOdometryStatus`。
- 服务：`DetectObjects`、`SetDetectionMode`、`CaptureSample`、`ConfirmMission`、`SetBodyBalance`、`ExecuteEncoderMotion`。
- Action：`PlanMission`、`ExecuteWheelMotion`。

接口变更必须同步检查 `robot_brain`、`robot_perception`、`robot_navigation`、`robot_control` 和 `robot_hardware` 的生产者、消费者与测试。优先保持字段兼容；确需破坏性变更时在 README 和当天进度中写明迁移影响。

## 验证

```bash
colcon build --symlink-install --packages-select robot_interfaces
source install/setup.bash
ros2 interface list | rg '^robot_interfaces/'
ros2 interface show robot_interfaces/msg/ChassisState
ros2 interface show robot_interfaces/action/PlanMission
git diff --check
```
