import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState


class ChassisFeedback(Node):

    def __init__(self):
        super().__init__('chassis_feedback')

        # 运动模式缓存
        self.current_mode = "unknown"

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
        # 订阅底盘运动模式
        self.mode_sub = self.create_subscription(
            String,
            '/chassis_mode',
            self.mode_callback,
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

        self.get_logger().info("✅ Chassis Feedback 启动完成 (带单位日志输出)")

    # 模式订阅回调
    def mode_callback(self, msg: String):
        self.current_mode = msg.data

    # 解析joint_states数据
    def joint_callback(self, msg: JointState):
        joint_index = {name: idx for idx, name in enumerate(msg.name)}

        try:
            self.steer_data = [msg.position[joint_index[j]] for j in self.steer_joints]
            self.wheel_speed_data = [msg.velocity[joint_index[j]] for j in self.wheel_joints]
        except KeyError as e:
            self.get_logger().warn(f"未找到关节: {e}，请检查URDF配置")
            return

    # 固定频率发布 + 打印带单位的日志
    def publish_feedback(self):
        # 拼接数据
        feedback_msg = Float64MultiArray(data=self.steer_data + self.wheel_speed_data)
        self.feedback_pub.publish(feedback_msg)

        # ======================
        # 🔥 核心：带单位的清晰日志输出
        # ======================
        steer_names = ["左前", "右前", "左后", "右后"]
        wheel_names = ["左前", "右前", "左后", "右后"]
        
        # 打印转向角度（弧度 + 角度 双单位）
        steer_log = "转向角度: "
        for i, angle in enumerate(self.steer_data):
            deg = math.degrees(angle)  # 弧度转角度
            steer_log += f"{steer_names[i]}: {angle:.2f}rad / {deg:.1f}° | "
        
        # 打印轮速（rad/s 单位）
        speed_log = "轮子转速: "
        for i, speed in enumerate(self.wheel_speed_data):
            speed_log += f"{wheel_names[i]}: {speed:.2f}rad/s | "

        # 输出日志
        self.get_logger().info(f"当前状态: {self.current_mode} | {steer_log} | {speed_log}")
        # self.get_logger().info(f"当前底盘模式: {self.current_mode}\n")


def main(args=None):
    rclpy.init(args=args)
    node = ChassisFeedback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()