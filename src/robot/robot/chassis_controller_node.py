import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import SetParametersResult

# 底盘控制节点 输入：cmd_vel 线速度和角速度
# 输出：steering_controller/commands, wheel_controller/commands
class FourWISController(Node):

    def __init__(self):
        # 节点名 和 launch 完全一致
        super().__init__('chassis_controller')

        # 读取 launch 传入的参数
        self.declare_parameter("wheelbase", 0.4)
        self.declare_parameter("track", 0.2)
        self.declare_parameter("radius", 0.05)
        self.declare_parameter("motion_mode", "crab")

        self.wheel_base = self.get_parameter("wheelbase").value
        self.wheel_track = self.get_parameter("track").value
        self.wheel_radius = self.get_parameter("radius").value
        self.motion_mode = self.get_parameter("motion_mode").value

        self.Lx = self.wheel_base / 2.0
        self.Ly = self.wheel_track / 2.0

        # 动态参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 10Hz 控制频率
        self.control_rate = 0.1
        self.latest_cmd_vel = Twist()

        # ROS 通信接口
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/wheel_states', self.wheel_state_callback, 10)

        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.speed_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.mode_pub = self.create_publisher(String, '/chassis_mode', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 10Hz 主控循环
        self.control_timer = self.create_timer(self.control_rate, self.control_loop)

        # 状态变量
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx_filtered = 0.0
        self.vy_filtered = 0.0
        self.w_filtered = 0.0
        self.filter_alpha = 0.2

        self.prev_angles = [0.0, 0.0, 0.0, 0.0]

        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timeout = 0.5
        self.has_received_cmd = False

        # 初始化发布模式
        self.mode_pub.publish(String(data=self.motion_mode))
        self.get_logger().info(f"✅ 底盘控制器启动 | 模式：{self.motion_mode} | 10Hz控制")

    # 参数回调
    def parameter_callback(self, params):
        for param in params:
            if param.name == "motion_mode":
                self.motion_mode = param.value
                self.get_logger().info(f"🔄 切换模式 → {self.motion_mode}")
                self.mode_pub.publish(String(data=self.motion_mode))
        return SetParametersResult(successful=True)

    # 缓存cmd_vel指令
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.has_received_cmd = True
        self.last_cmd_vel_time = self.get_clock().now()

    # 10Hz 主控循环
    def control_loop(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_vel_time).nanoseconds * 1e-9 > self.cmd_vel_timeout:
            self.send_stop()
            return
        if not self.has_received_cmd:
            self.send_stop()
            return

        vx = self.latest_cmd_vel.linear.x
        vy = self.latest_cmd_vel.linear.y
        w = self.latest_cmd_vel.angular.z

        # 模式解算
        if self.motion_mode == "crab":
            angles, speeds = self.mode_crab(vx, vy)
        elif self.motion_mode == "four_ws":
            angles, speeds = self.mode_four_ws(vx, w)
        elif self.motion_mode == "ackermann":
            angles, speeds = self.mode_ackermann(vx, w)
        else:
            angles, speeds = self.stop_command()

        # 🔥 最小转角优化
        opt_angles = []
        opt_speeds = []

        for i in range(4):
            a, s = self.optimize_steering(
                angles[i],
                speeds[i],
                self.prev_angles[i]
            )
            opt_angles.append(a)
            opt_speeds.append(s)

        self.prev_angles = opt_angles

        # 日志
        steer_str = f"转向={[f'{a:.2f}rad/{math.degrees(a):.0f}°' for a in opt_angles]}"
        speed_str = f"轮速={[f'{s:.2f}rad/s' for s in opt_speeds]}"
        self.get_logger().info(f"📤 下发指令 | {steer_str} | {speed_str}")

        # 发布
        self.steer_pub.publish(Float64MultiArray(data=opt_angles))
        self.speed_pub.publish(Float64MultiArray(data=opt_speeds))

    # 转向角限位
    def limit_steering_angle(self, angles):
        limited = []
        max_angle = math.pi / 2
        for ang in angles:
            ang = max(min(ang, max_angle), -max_angle)
            limited.append(ang)
        return limited

    # 蟹行模式（单位100%正确）
    def mode_crab(self, vx, vy):

        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            return self.stop_command()

        angle = math.atan2(vy, vx)
        speed = math.hypot(vx, vy) / self.wheel_radius

        return [angle]*4, [speed]*4

    # 四轮转向模式
    def mode_four_ws(self, vx, w):
        if abs(vx) < 1e-3 and abs(w) < 1e-3:
            return self.stop_command()

        if abs(w) < 1e-5:
            return [0.0]*4, [vx/self.wheel_radius]*4

        # 1. 计算旋转半径（ICR在Y轴上的位置）
        # vx=0.5, w=0.5 -> R=1.0 (在中心点左侧1米)
        R = vx / w

        # 2. 计算四个轮子的转向角 (标准右手系)
        # 前轮：向左转时计算出正值
        delta_fl = math.atan2(self.wheel_base/2, R - self.wheel_track/2)
        delta_fr = math.atan2(self.wheel_base/2, R + self.wheel_track/2)
        
        # 后轮：四轮转向模式下，后轮偏转方向与前轮相反
        delta_rl = -math.atan2(self.wheel_base/2, R - self.wheel_track/2)
        delta_rr = -math.atan2(self.wheel_base/2, R + self.wheel_track/2)

        # 3. 计算四个轮子的线速度
        # 速度 = 旋转角速度 * 到旋转中心的距离
        v_fl = abs(w) * math.hypot(R - self.wheel_track/2, self.wheel_base/2)
        v_fr = abs(w) * math.hypot(R + self.wheel_track/2, self.wheel_base/2)
        v_rl = abs(w) * math.hypot(R - self.wheel_track/2, self.wheel_base/2)
        v_rr = abs(w) * math.hypot(R + self.wheel_track/2, self.wheel_base/2)

        # 4. 速度方向补偿
        # 旋转线速度始终为正，需要根据前进方向 vx 确定电机转动正反
        sign = 1.0 if vx >= 0 else -1.0
        
        angles = [delta_fl, delta_fr, delta_rl, delta_rr]
        speeds = [
            (v_fl * sign) / self.wheel_radius,
            (v_fr * sign) / self.wheel_radius,
            (v_rl * sign) / self.wheel_radius,
            (v_rr * sign) / self.wheel_radius
        ]

        return angles, speeds



    # 阿克曼模式
    def mode_ackermann(self, vx, w):

        if abs(vx) < 1e-3 and abs(w) < 1e-3:
            return self.stop_command()

        if abs(w) < 1e-3:
            return [0,0,0,0], [vx/self.wheel_radius]*4

        R_icr = vx / w

        # 转向角（内外轮差）
        delta_l = math.atan(self.wheel_base / (R_icr - self.wheel_track/2))
        delta_r = math.atan(self.wheel_base / (R_icr + self.wheel_track/2))

        # 速度
        v_fl = w * math.hypot(R_icr - self.wheel_track/2, self.wheel_base)
        v_fr = w * math.hypot(R_icr + self.wheel_track/2, self.wheel_base)
        v_rl = w * (R_icr - self.wheel_track/2)
        v_rr = w * (R_icr + self.wheel_track/2)

        angles = [delta_l, delta_r, 0.0, 0.0]
        speeds = [
            v_fl/self.wheel_radius,
            v_fr/self.wheel_radius,
            v_rl/self.wheel_radius,
            v_rr/self.wheel_radius
        ]

        return angles, speeds
    
    def angle_diff(self, a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))


    def optimize_steering(self, target_angle, target_speed, prev_angle):

        # 候选1
        a1 = math.atan2(math.sin(target_angle), math.cos(target_angle))
        s1 = target_speed

        # 候选2（翻转）
        a2 = math.atan2(math.sin(target_angle + math.pi), math.cos(target_angle + math.pi))
        s2 = -target_speed

        # 🔥 优先让角度接近“运动方向”
        if abs(a1) <= math.pi/2:
            return a1, s1
        else:
            return a2, s2

    # 停止指令
    def stop_command(self):
        return [0.0]*4, [0.0]*4

    def send_stop(self):
        ang, spd = self.stop_command()
        # self.get_logger().warn("⚠️ 超时触发 STOP")
        self.steer_pub.publish(Float64MultiArray(data=ang))
        self.speed_pub.publish(Float64MultiArray(data=spd))



    # 里程计解算（计算完全正确）
    def wheel_state_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        if dt <= 0.0:
            return

        fl_s, fr_s, rl_s, rr_s = msg.data[0:4]
        fl_v, fr_v, rl_v, rr_v = np.array(msg.data[4:8]) * self.wheel_radius

        # 底盘速度分解（正确）
        vx = np.mean([fl_v*math.cos(fl_s), fr_v*math.cos(fr_s), rl_v*math.cos(rl_s), rr_v*math.cos(rr_s)])
        vy = np.mean([fl_v*math.sin(fl_s), fr_v*math.sin(fr_s), rl_v*math.sin(rl_s), rr_v*math.sin(rr_s)])
        
        # 角速度计算（四轮全向车运动学，正确）
        w = ((-fl_v*math.cos(fl_s)*self.Ly + fl_v*math.sin(fl_s)*self.Lx) +
             (-fr_v*math.cos(fr_s)*(-self.Ly) + fr_v*math.sin(fr_s)*self.Lx) +
             (-rl_v*math.cos(rl_s)*self.Ly + rl_v*math.sin(rl_s)*(-self.Lx)) +
             (-rr_v*math.cos(rr_s)*(-self.Ly) + rr_v*math.sin(rr_s)*(-self.Lx))) / (4.0*(self.Lx**2 + self.Ly**2))

        # 低通滤波
        self.vx_filtered = self.filter_alpha * vx + (1-self.filter_alpha)*self.vx_filtered
        self.vy_filtered = self.filter_alpha * vy + (1-self.filter_alpha)*self.vy_filtered
        self.w_filtered = self.filter_alpha * w + (1-self.filter_alpha)*self.w_filtered

        # 位姿积分（正确）
        self.yaw += self.w_filtered * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        dx = (self.vx_filtered * math.cos(self.yaw) - self.vy_filtered * math.sin(self.yaw)) * dt
        dy = (self.vx_filtered * math.sin(self.yaw) + self.vy_filtered * math.cos(self.yaw)) * dt
        self.x += dx
        self.y += dy

        self.publish_odom(current_time)

    # 发布odom+tf
    def publish_odom(self, time):
        qz = math.sin(self.yaw/2)
        qw = math.cos(self.yaw/2)
        odom = Odometry()
        odom.header.stamp = time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx_filtered
        odom.twist.twist.linear.y = self.vy_filtered
        odom.twist.twist.angular.z = self.w_filtered
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