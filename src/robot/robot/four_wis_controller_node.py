import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class FourWISController(Node):

    def __init__(self):
        super().__init__('four_wis_controller')

        # 参数
        self.L = 0.4
        self.W = 0.2
        self.R = 0.05

        # pub
        self.pub_steer = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        self.pub_wheel = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )

        # sub
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

    def callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        pos = [
            ( self.L/2,  self.W/2),
            ( self.L/2, -self.W/2),
            (-self.L/2,  self.W/2),
            (-self.L/2, -self.W/2)
        ]

        steer = []
        wheel = []

        for (x, y) in pos:
            vx_i = vx - wz * y
            vy_i = wz * x

            angle = math.atan2(vy_i, vx_i)
            speed = math.sqrt(vx_i**2 + vy_i**2)

            # ⭐优化：最小转向
            if abs(angle) > math.pi/2:
                angle -= math.pi
                speed *= -1

            steer.append(angle)
            wheel.append(speed / self.R)

        msg_steer = Float64MultiArray()
        msg_steer.data = steer

        msg_wheel = Float64MultiArray()
        msg_wheel.data = wheel

        self.pub_steer.publish(msg_steer)
        self.pub_wheel.publish(msg_wheel)


def main():
    rclpy.init()
    node = FourWISController()
    rclpy.spin(node)
    rclpy.shutdown()