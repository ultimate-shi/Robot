<!-- 作用：说明 robot_description 的模型、TF 和 ros2_control 描述职责。 -->
<!-- 使用方法：修改实车尺寸、关节、传感器外参、网格或硬件接口前先阅读本文件。 -->
# robot_description 功能包

## 职责与边界

`robot_description` 保存实机与虚拟模式共用的 URDF/Xacro、碰撞和惯性、网格、传感器外参及 ros2_control 接口。模型必须以实车结构和坐标关系为准。

本包归 `platform_integration_owner` 管理；轮腿和控制接口变化需与 `control_hardware_owner` 联调，相机、IMU、超声波外参变化需与 `perception_owner` 联调。

## 资源和入口

- `urdf/robot.xacro` 组合车身、四腿、四轮、四转向、双轴头部、双目、IMU、8 路超声波和硬件描述。
- `urdf/hardware.xacro` 声明腿部位置、转向位置、车轮速度以及头部位置接口。
- `meshes/` 保存 OBJ/MTL 外观资源。
- `description.launch.py` 展开 Xacro 并启动 `robot_state_publisher`。
- `joint_states.launch.py` 为未接反馈的关节提供默认状态；`foxglove.launch.py` 启动可视化桥。

`robot_state_publisher` 根据 URDF 和 `/joint_states` 发布模型 TF；动态 `odom -> base_link`、`map -> odom` 不属于本包。

## 验证

```bash
colcon build --symlink-install --packages-select robot_description
source install/setup.bash
xacro install/robot_description/share/robot_description/urdf/robot.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
ros2 launch robot_description description.launch.py --show-args
git diff --check
```
