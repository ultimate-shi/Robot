<!-- 作用：列出当前 ROS 2 工作区的全部功能包文档；使用方法：从本页进入对应包说明，修改包职责时同步更新。 -->
# 功能包文档索引

当前 `ROS2/src/` 有 11 个 ROS 2 功能包：

| 功能包 | 主要职责 |
| --- | --- |
| [robot_brain](robot_brain.md) | Qwen 决策、动作白名单和任务工具 |
| [robot_control](robot_control.md) | 四轮转向、运动控制、速度门控与最终安全层 |
| [robot_description](robot_description.md) | URDF、mesh 与 ros2_control 模型描述 |
| [robot_hardware](robot_hardware.md) | DMC0、DMC1、SE2 实机串口硬件接口 |
| [robot_interfaces](robot_interfaces.md) | 跨包 msg、srv 和 action 接口 |
| [robot_logging](robot_logging.md) | MCAP 录制、索引与历史日志查询 |
| [robot_main](robot_main.md) | 实机与虚拟模式的整车编排入口 |
| [robot_navigation](robot_navigation.md) | 建图、定位、Nav2 导航与探索工具 |
| [robot_perception](robot_perception.md) | 双目视觉、目标检测、深度与传感器处理 |
| [robot_stereo_components](robot_stereo_components.md) | 高带宽双目处理 C++ 组件 |
| [robot_web](robot_web.md) | 9100 网页入口、人工控制台与速度仲裁 |

新增或删除功能包时，同时更新本页和 `ROS2/README.md` 的项目结构。
