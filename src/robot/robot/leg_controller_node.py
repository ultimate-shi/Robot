import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class LegJointPublisher(Node):
    """Controller node that can drive any leg by supplying parameters."""

    def __init__(self):
        super().__init__("leg_joint_publisher")

        # Declare parameters (can be overridden per leg)
        self.declare_parameter("leg_id", "lf")
        self.leg_id = self.get_parameter("leg_id").value.lower()

        self.thigh_topic = self.declare_parameter("thigh_topic", self._topic("thigh_position")).value
        self.shin_topic = self.declare_parameter("shin_topic", self._topic("shin_position")).value
        self.steering_topic = self.declare_parameter(
            "steering_topic", self._topic("steering_controller")
        ).value
        self.wheel_topic = self.declare_parameter("wheel_topic", self._topic("wheel_controller")).value
        self.state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.publish_period = self.declare_parameter("publish_period", 0.1).value

        # Joint names must match URDF for the selected leg
        self.joint_names = [
            f"body_thigh_{self.leg_id}_joint",
            f"thigh_shin_{self.leg_id}_joint",
            f"shin_motor_{self.leg_id}_joint",
            f"shin_wheel_{self.leg_id}_joint",
        ]

        # Publishers / subscribers
        self.joint_publisher = self.create_publisher(JointState, self.state_topic, 10)
        self.create_subscription(Float64, self.thigh_topic, self.thigh_callback, 10)
        self.create_subscription(Float64, self.shin_topic, self.shin_callback, 10)
        self.create_subscription(Float64, self.steering_topic, self.steering_callback, 10)
        self.create_subscription(Float64, self.wheel_topic, self.wheel_callback, 10)

        # Internal state
        self.thigh_angle = 0.0
        self.shin_angle = 0.0
        self.steering_angle = 0.0
        self.wheel_velocity = 0.0
        self.wheel_angle = 0.0
        self.last_publish_time = None

        self.timer = self.create_timer(self.publish_period, self.publish_joint_states)

        self.get_logger().info(
            f"[{self.leg_id.upper()}] Leg Joint Publisher.started "
            f"thigh={self.thigh_topic}, shin={self.shin_topic}, "
            f"steering={self.steering_topic}, wheel={self.wheel_topic}"
        )

    def thigh_callback(self, msg: Float64):
        self.thigh_angle = max(-1.57, min(1.57, msg.data))

    def shin_callback(self, msg: Float64):
        self.shin_angle = max(-1.57, min(1.57, msg.data))

    def steering_callback(self, msg: Float64):
        self.steering_angle = max(-math.pi / 2.0, min(math.pi / 2.0, msg.data))

    def wheel_callback(self, msg: Float64):
        self.wheel_velocity = msg.data

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        now = self.get_clock().now()
        if self.last_publish_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = now

        self.wheel_angle += self.wheel_velocity * dt
        self.wheel_angle = math.atan2(math.sin(self.wheel_angle), math.cos(self.wheel_angle))

        desired_shin_angle = self.shin_angle - self.thigh_angle
        shin_angle = max(-1.57, min(desired_shin_angle, 1.57))

        msg.position = [
            self.thigh_angle,
            shin_angle,
            self.steering_angle,
            self.wheel_angle,
        ]

        msg.velocity = [
            0.0,
            0.0,
            0.0,
            self.wheel_velocity,
        ]
        self.joint_publisher.publish(msg)

    def _topic(self, base: str) -> str:
        """Build default topic `/base_legid`."""
        return f"/{base}_{self.leg_id}"


def main(args=None):
    rclpy.init(args=args)
    node = LegJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

