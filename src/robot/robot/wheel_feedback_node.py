import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelHardwareFeedback(Node):
    def __init__(self):
        super().__init__('wheel_hardware_feedback')
        
        # 订阅控制器发出的轮子指令
        self.steer_cmd = None
        self.speed_cmd = None
        
        self.steer_sub = self.create_subscription(
            Float64MultiArray, '/steering_commands', self.steer_callback, 10)
        self.speed_sub = self.create_subscription(
            Float64MultiArray, '/wheel_speed_commands', self.speed_callback, 10)
        
        # 发布轮子真实反馈（给控制器）
        self.feedback_pub = self.create_publisher(
            Float64MultiArray, '/wheel_states', 10)
        
        # 20Hz 发布反馈
        self.timer = self.create_timer(0.05, self.publish_feedback)
        
        self.get_logger().info("Wheel Feedback Node Started")

    def steer_callback(self, msg):
        self.steer_cmd = msg.data

    def speed_callback(self, msg):
        self.speed_cmd = msg.data

    def publish_feedback(self):
        # ======================
        # 【模拟硬件：指令=反馈】
        # 真实硬件：替换为 编码器读取的转角+转速
        # ======================
        if self.steer_cmd is not None and self.speed_cmd is not None:
            # 反馈格式：[4个转向角, 4个轮速]
            feedback_data = list(self.steer_cmd) + list(self.speed_cmd)
            
            # ==============================================
            # 真实硬件修改这里：
            # 1. 读取4个转向电机编码器 → 转为弧度(rad)
            # 2. 读取4个驱动电机编码器 → 转为rad/s
            # 3. 替换 feedback_data 为真实硬件数据
            # ==============================================
            
            msg = Float64MultiArray(data=feedback_data)
            self.get_logger().info(f"Wheel Feedback: {feedback_data}")
            self.feedback_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    feedback_node = WheelHardwareFeedback()
    rclpy.spin(feedback_node)
    feedback_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()