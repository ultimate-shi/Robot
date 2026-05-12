import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException


class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node')

        # ===== 参数声明 =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('axes.left_stick_y', 1),
                ('axes.right_stick_x', 3),
                ('axes.lt', 2),
                ('axes.rt', 5),

                ('buttons.lb', 4),
                ('buttons.rb', 5),
                ('buttons.estop', 3),

                ('scale.max_linear_x', 0.6),
                ('scale.max_angular_z', 1.2),
                ('scale.max_spin_angular', 1.0),

                ('deadzone', 0.05),
                ('min_linear_threshold', 0.05),
                ('smoothing_alpha', 0.2),

                ('modes.list', ['crab', 'four_ws', 'ackermann']),

                ('chassis.node_name', '/chassis_controller'),
                ('chassis.param_name', 'motion_mode'),
                ('chassis.mode_topic', '/chassis_mode'),
            ]
        )

        # ===== 读取参数 =====
        self.axes_map = {
            'ly': self.get_parameter('axes.left_stick_y').value,
            'rx': self.get_parameter('axes.right_stick_x').value,
            'lt': self.get_parameter('axes.lt').value,
            'rt': self.get_parameter('axes.rt').value,
        }

        self.btn_map = {
            'lb': self.get_parameter('buttons.lb').value,
            'rb': self.get_parameter('buttons.rb').value,
            'estop': self.get_parameter('buttons.estop').value,
        }

        self.max_linear_x = self.get_parameter('scale.max_linear_x').value
        self.max_angular_z = self.get_parameter('scale.max_angular_z').value
        self.max_spin_angular = self.get_parameter('scale.max_spin_angular').value

        self.deadzone = self.get_parameter('deadzone').value
        self.min_linear_threshold = self.get_parameter('min_linear_threshold').value
        self.alpha = self.get_parameter('smoothing_alpha').value

        self.mode_list = self.get_parameter('modes.list').value

        self.chassis_node = self.get_parameter('chassis.node_name').value
        self.param_name = self.get_parameter('chassis.param_name').value
        self.mode_topic = self.get_parameter('chassis.mode_topic').value

        # ===== 状态 =====
        self.current_mode_index = 0
        self.current_mode = self.mode_list[0]

        self.prev_buttons = []
        self.last_v = 0.0
        self.last_w = 0.0

        # ===== 通信 =====
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(String, self.mode_topic, self.mode_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.param_client = self.create_client(SetParameters, f'{self.chassis_node}/set_parameters')

        self.timer = self.create_timer(0.1, self.update)  # 10Hz

        self.latest_joy = None

    # ========================
    # 工具函数
    # ========================
    def apply_deadzone(self, x):
        return 0.0 if abs(x) < self.deadzone else x

    def smooth(self, new, old):
        return self.alpha * new + (1 - self.alpha) * old

    def get_axis(self, joy, name):
        idx = self.axes_map[name]
        if idx < len(joy.axes):
            return joy.axes[idx]
        return 0.0

    def get_button(self, joy, name):
        idx = self.btn_map[name]
        if idx < len(joy.buttons):
            return joy.buttons[idx]
        return 0

    # ========================
    # 回调
    # ========================
    def joy_callback(self, msg):
        self.latest_joy = msg

    def mode_callback(self, msg):
        if msg.data in self.mode_list:
            self.current_mode = msg.data
            self.current_mode_index = self.mode_list.index(msg.data)

    # ========================
    # 模式切换
    # ========================
    def switch_mode(self, direction):
        self.current_mode_index = (self.current_mode_index + direction) % len(self.mode_list)
        new_mode = self.mode_list[self.current_mode_index]

        if not self.param_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Param service not available')
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name=self.param_name,
                value=new_mode
            ).to_parameter_msg()
        ]

        self.param_client.call_async(req)
        self.get_logger().info(f'Switch mode → {new_mode}')

        # 切换时清零
        self.publish_cmd(0.0, 0.0)

    # ========================
    # 主循环
    # ========================
    def update(self):

        if self.latest_joy is None:
            return

        joy = self.latest_joy

        # === 按键 ===
        lb = self.get_button(joy, 'lb')
        rb = self.get_button(joy, 'rb')
        estop = self.get_button(joy, 'estop')

        # === 边沿检测 ===
        if not self.prev_buttons:
            self.prev_buttons = joy.buttons

        if lb == 1 and self.prev_buttons[self.btn_map['lb']] == 0:
            self.switch_mode(-1)

        if rb == 1 and self.prev_buttons[self.btn_map['rb']] == 0:
            self.switch_mode(1)

        self.prev_buttons = joy.buttons.copy()

        # === 轴 ===
        ly = self.apply_deadzone(self.get_axis(joy, 'ly'))
        rx = self.apply_deadzone(self.get_axis(joy, 'rx'))

        lt = self.apply_deadzone(self.get_axis(joy, 'lt'))
        rt = self.apply_deadzone(self.get_axis(joy, 'rt'))

        # 处理扳机（很多手柄是 [-1,1]）
        lt = (1 - lt) / 2 if lt < 0 else lt
        rt = (1 - rt) / 2 if rt < 0 else rt

        # === 线速度 ===
        v = ly * self.max_linear_x

        # === 判断静止 ===
        is_stationary = abs(v) < self.min_linear_threshold

        # === 角速度 ===
        if is_stationary:
            spin_input = rt - lt
            w = spin_input * self.max_spin_angular
        else:
            w = rx * self.max_angular_z

        # === 急停 ===
        if estop:
            v = 0.0
            w = 0.0

        # === 平滑 ===
        v = self.smooth(v, self.last_v)
        w = self.smooth(w, self.last_w)

        self.last_v = v
        self.last_w = w

        # === 发布 ===
        self.publish_cmd(v, w)

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # 正常退出，不打印traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()