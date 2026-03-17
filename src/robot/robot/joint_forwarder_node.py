#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# 核心映射：GUI中的关节名 → 控制器话题（必须和你的yaml完全匹配！）
JOINT_CONTROLLER_MAP = {
    # 大腿关节
    "lap_body_joint_rf": "/robot/gazebo/lap_rf_position_controller/commands",
    "lap_body_joint_lf": "/robot/gazebo/lap_lf_position_controller/commands",
    "lap_body_joint_rr": "/robot/gazebo/lap_rr_position_controller/commands",
    "lap_body_joint_lr": "/robot/gazebo/lap_lr_position_controller/commands",
    # 小腿关节
    "shin_lap_rf_joint_rf": "/robot/gazebo/shin_rf_position_controller/commands",
    "shin_lap_lf_joint_lf": "/robot/gazebo/shin_lf_position_controller/commands",
    "shin_lap_rr_joint_rr": "/robot/gazebo/shin_rr_position_controller/commands",
    "shin_lap_lr_joint_lr": "/robot/gazebo/shin_lr_position_controller/commands",
    # 转向关节
    "motor_shin_rf_joint_rf": "/robot/gazebo/motor_rf_position_controller/commands",
    "motor_shin_lf_joint_lf": "/robot/gazebo/motor_lf_position_controller/commands",
    "motor_shin_rr_joint_rr": "/robot/gazebo/motor_rr_position_controller/commands",
    "motor_shin_lr_joint_lr": "/robot/gazebo/motor_lr_position_controller/commands",
    # 轮子关节（速度控制）
    "wheel_motor_rf_joint_rf": "/robot/gazebo/wheel_rf_velocity_controller/commands",
    "wheel_motor_lf_joint_lf": "/robot/gazebo/wheel_lf_velocity_controller/commands",
    "wheel_motor_rr_joint_rr": "/robot/gazebo/wheel_rr_velocity_controller/commands",
    "wheel_motor_lr_joint_lr": "/robot/gazebo/wheel_lr_velocity_controller/commands",
}

class JointForwarder(Node):
    def __init__(self):
        super().__init__('joint_forwarder')
        # 订阅GUI的/joint_states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        # 为每个控制器创建发布者（缓存起来避免重复创建）
        self.publishers = {}
        for joint_name, topic in JOINT_CONTROLLER_MAP.items():
            self.publishers[joint_name] = self.create_publisher(Float64, topic, 10)
        
        self.get_logger().info("独立关节转发节点已启动！")

    def joint_state_callback(self, msg):
        # 遍历GUI发布的每个关节，分发到对应控制器
        for idx, joint_name in enumerate(msg.name):
            if joint_name in self.publishers:
                # 构造单个关节的指令（Float64类型，而非MultiArray）
                cmd = Float64()
                # 位置控制器用position，速度控制器用velocity（按需切换）
                if "wheel" in joint_name:  # 轮子是速度控制
                    cmd.data = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
                else:  # 腿/转向是位置控制
                    cmd.data = msg.position[idx] if len(msg.position) > idx else 0.0
                # 发布指令到对应控制器
                self.publishers[joint_name].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JointForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()