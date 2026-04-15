import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelHardwareFeedback(Node):
    def __init__(self):
        super().__init__('wheel_hardware_feedback')
        
        self.steer_cmd = [0.0, 0.0, 0.0, 0.0]
        self.speed_cmd = [0.0, 0.0, 0.0, 0.0]
        
        self.steer_sub = self.create_subscription(
            Float64MultiArray, '/steering_controller/commands', self.steer_callback, 10)
        self.speed_sub = self.create_subscription(
            Float64MultiArray, '/wheel_controller/commands', self.speed_callback, 10)
        
        self.feedback_pub = self.create_publisher(
            Float64MultiArray, '/wheel_states', 10)
        
        self.timer = self.create_timer(0.05, self.publish_feedback)
        
        self.get_logger().info("✅ Wheel Feedback Node Started (Fixed)")

    def steer_callback(self, msg):
        self.steer_cmd = msg.data

    def speed_callback(self, msg):
        self.speed_cmd = msg.data

    def publish_feedback(self):
        # ======================
        # 🔥 修复：永远发送数据，不需要判断 None
        # ======================
        feedback_data = list(self.steer_cmd) + list(self.speed_cmd)
        
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