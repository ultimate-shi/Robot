import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class FourWISController(Node):

    def __init__(self):
        super().__init__('four_wis_controller')

        # ======================
        # 底盘参数
        # ======================
        self.wheel_base = 0.4
        self.wheel_track = 0.2
        self.wheel_radius = 0.05

        self.Lx = self.wheel_base / 2.0
        self.Ly = self.wheel_track / 2.0

        # ======================
        # 通信接口
        # ======================
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/wheel_states', self.wheel_state_callback, 10)

        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.speed_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ======================
        # 状态变量
        # ======================
        self.last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 滤波
        self.vx_filtered = 0.0
        self.vy_filtered = 0.0
        self.w_filtered = 0.0
        self.filter_alpha = 0.2

        # 🔥 🔥 🔥 关键修复：增加指令超时判断
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timeout = 0.2  # 200ms 没收到指令就停止
        self.has_received_cmd = False

        self.get_logger().info("FourWIS Controller Started")

    # =========================================================
    # 逆运动学：cmd_vel → 轮子控制
    # =========================================================
    def cmd_vel_callback(self, msg):
        # 标记收到指令
        self.has_received_cmd = True
        self.last_cmd_vel_time = self.get_clock().now()

        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        wheels = [
            ( self.Lx,  self.Ly),   # FL
            ( self.Lx, -self.Ly),   # FR
            (-self.Lx,  self.Ly),   # RL
            (-self.Lx, -self.Ly)    # RR
        ]

        steering_angles = []
        wheel_speeds = []

        for lx, ly in wheels:
            vx_i = vx - w * ly
            vy_i = vy + w * lx
            wheel_velocity = math.hypot(vx_i, vy_i)

            if wheel_velocity < 1e-3:
                angle = 0.0
                wheel_rad = 0.0
            else:
                angle = math.atan2(vy_i, vx_i)
                wheel_rad = wheel_velocity / self.wheel_radius

            max_angle = math.pi / 2
            angle = max(min(angle, max_angle), -max_angle)

            steering_angles.append(angle)
            wheel_speeds.append(wheel_rad)

        self.steer_pub.publish(Float64MultiArray(data=steering_angles))
        self.speed_pub.publish(Float64MultiArray(data=wheel_speeds))

    # =========================================================
    # 正运动学：轮子反馈 → odom
    # =========================================================
    def wheel_state_callback(self, msg):
        current_time = self.get_clock().now()

        # 🔥 🔥 🔥 核心修复：指令超时 → 强制停止所有输出
        if (current_time - self.last_cmd_vel_time).nanoseconds * 1e-9 > self.cmd_vel_timeout:
            self.send_stop()
            return

        # 没有收到过任何 cmd_vel → 直接停止
        if not self.has_received_cmd:
            self.send_stop()
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0 or dt > 0.05:
            return

        fl_steer, fr_steer, rl_steer, rr_steer = msg.data[0:4]
        fl_speed, fr_speed, rl_speed, rr_speed = msg.data[4:8]

        fl_v = fl_speed * self.wheel_radius
        fr_v = fr_speed * self.wheel_radius
        rl_v = rl_speed * self.wheel_radius
        rr_v = rr_speed * self.wheel_radius

        wheel_data = [
            (fl_steer, fl_v, self.Lx, self.Ly),
            (fr_steer, fr_v, self.Lx, -self.Ly),
            (rl_steer, rl_v, -self.Lx, self.Ly),
            (rr_steer, rr_v, -self.Lx, -self.Ly)
        ]

        A = []
        b = []
        for theta, v, lx, ly in wheel_data:
            A.append([1, 0, -ly])
            b.append(v * math.cos(theta))
            A.append([0, 1, lx])
            b.append(v * math.sin(theta))

        try:
            vx, vy, w = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)[0]
        except:
            vx, vy, w = 0.0, 0.0, 0.0

        alpha = self.filter_alpha
        self.vx_filtered = alpha * vx + (1 - alpha) * self.vx_filtered
        self.vy_filtered = alpha * vy + (1 - alpha) * self.vy_filtered
        self.w_filtered  = alpha * w  + (1 - alpha) * self.w_filtered

        vx = self.vx_filtered
        vy = self.vy_filtered
        w  = self.w_filtered

        self.yaw += w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        self.x += dx
        self.y += dy

        self.publish_odom(current_time, vx, vy, w)

    # =========================================================
    # 🔥 新增：发送停止指令（关键修复）
    # =========================================================
    def send_stop(self):
        stop_steer = [0.0, 0.0, 0.0, 0.0]
        stop_speed = [0.0, 0.0, 0.0, 0.0]
        self.steer_pub.publish(Float64MultiArray(data=stop_steer))
        self.speed_pub.publish(Float64MultiArray(data=stop_speed))

    # =========================================================
    # 发布 odom + TF
    # =========================================================
    def publish_odom(self, time, vx, vy, w):
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)

        odom = Odometry()
        odom.header.stamp = time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = time.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FourWISController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()