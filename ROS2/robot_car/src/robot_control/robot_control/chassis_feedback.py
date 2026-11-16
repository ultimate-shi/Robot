"""
使用方法：由 robot_control/control.launch.py 启动底盘反馈节点。
本节点由 robot.launch.py 以 executable='chassis_feedback_node' 启动。
它把 ros2_control 发布的 /joint_states 转换成底盘使用的 /wheel_states。

输入：
- /joint_states：joint_state_broadcaster 发布，包含所有关节的位置和速度。

输出：
- /wheel_states：Float64MultiArray，前 4 个值是四轮转向角，后 4 个值是四轮轮速。

为什么不能删除：
chassis_controller_node 依赖 /wheel_states 计算 /odom 和 odom->base_link TF。
如果删除，机器人位姿、虚拟传感器 TF、Nav2 定位都会受到影响。
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.executors import ExternalShutdownException

# 底盘运动反馈节点
class ChassisFeedback(Node):

    def __init__(self):
        super().__init__('chassis_feedback')

        # 初始化默认数据（防止启动无数据报错）
        self.steer_data = [0.0, 0.0, 0.0, 0.0]  # 4个转向角
        self.wheel_speed_data = [0.0, 0.0, 0.0, 0.0]  # 4个轮速

        # ======================
        # 订阅话题
        # ======================
        # 订阅ros2_control发布的joint_states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        # ======================
        # 发布轮子状态（给底盘控制器计算里程计）
        # ======================
        self.feedback_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_states',
            10
        )

        # ======================
        # 关节名称（必须和URDF/ros2_control完全一致）
        # ======================
        self.steer_joints = [
            'front_left_steer_joint',
            'front_right_steer_joint',
            'rear_left_steer_joint',
            'rear_right_steer_joint'
        ]
        self.wheel_joints = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

        # 10Hz固定频率发布
        self.create_timer(0.1, self.publish_feedback)

        self.get_logger().info("Chassis Feedback 启动完成")

    # 解析joint_states数据
    def joint_callback(self, msg: JointState):
        joint_index = {name: idx for idx, name in enumerate(msg.name)}

        try:
            self.steer_data = [msg.position[joint_index[j]] for j in self.steer_joints]
            self.wheel_speed_data = [msg.velocity[joint_index[j]] for j in self.wheel_joints]
        except KeyError as e:
            self.get_logger().warn(f"未找到关节: {e}，请检查URDF配置")
            return

    # 固定频率发布反馈
    def publish_feedback(self):
        # 拼接数据
        feedback_msg = Float64MultiArray(data=self.steer_data + self.wheel_speed_data)
        self.feedback_pub.publish(feedback_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ChassisFeedback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # 正常退出，不打印traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
