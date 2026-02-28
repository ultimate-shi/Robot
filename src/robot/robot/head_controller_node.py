import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class HeadControllerNode(Node):
    """Controller node for head rotation."""

    def __init__(self):
        super().__init__("head_controller_node")

        # Joint name
        self.joint_name = "body_head_joint"

        # Publishers / subscribers
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.head_subscriber = self.create_subscription(
            Float64, "/head_controller", self.head_callback, 10
        )

        # Internal state
        self.head_angle = 0.0

        # Set publish rate (10Hz)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info(
            f"Head Controller Node started. Subscribing to /head_controller"
        )

    def head_callback(self, msg: Float64):
        # Limit head angle to -90° to 90°
        self.head_angle = max(-1.5708, min(1.5708, msg.data))

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.head_angle]
        msg.velocity = [0.0]
        self.joint_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

