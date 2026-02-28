import rclpy
import sys
from rclpy.node import Node
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSlider, QLabel, QComboBox
from python_qt_binding.QtCore import Qt
from std_msgs.msg import Float64

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        
        # Initialize ROS 2 node
        rclpy.init()
        self.node = Node('robot_control_gui')
        
        # Supported legs: left-front, right-front, left-rear, right-rear
        self.legs = [
            ("LF", "lf"),
            ("RF", "rf"),
            ("LR", "lr"),
            ("RR", "rr"),
        ]
        self.current_leg = "lf"

        # Create publishers for every leg/joint combination
        self.thigh_publishers = {}
        self.shin_publishers = {}
        self.steering_publishers = {}
        self.wheel_publishers = {}
        for _, leg_id in self.legs:
            self.thigh_publishers[leg_id] = self.node.create_publisher(
                Float64, f"/thigh_position_{leg_id}", 10
            )
            self.shin_publishers[leg_id] = self.node.create_publisher(
                Float64, f"/shin_position_{leg_id}", 10
            )
            self.steering_publishers[leg_id] = self.node.create_publisher(
                Float64, f"/steering_controller_{leg_id}", 10
            )
            self.wheel_publishers[leg_id] = self.node.create_publisher(
                Float64, f"/wheel_controller_{leg_id}", 10
            )
        
        # Create head controller publisher
        self.head_publisher = self.node.create_publisher(
            Float64, "/head_controller", 10
        )
        
        # Setup UI
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()

        # Leg selector
        self.leg_selector = QComboBox()
        for label, _ in self.legs:
            self.leg_selector.addItem(label)
        self.leg_selector.currentIndexChanged.connect(self.leg_changed)
        layout.addWidget(QLabel("Select Leg"))
        layout.addWidget(self.leg_selector)
        
        #Create slider for thigh angle
        self.slider_thigh = QSlider(Qt.Horizontal)
        self.slider_thigh.setMinimum(-157)
        self.slider_thigh.setMaximum(157)
        self.slider_thigh.setValue(0)
        self.slider_thigh.valueChanged.connect(self.slider_thigh_changed)
        #Create label to show current value
        self.label_thigh = QLabel("Thigh Angle: 0.00 rad")
        self.thigh_title = QLabel("Thigh Joint Control (LF)")
        layout.addWidget(self.thigh_title)
        layout.addWidget(self.slider_thigh)
        layout.addWidget(self.label_thigh)

        # Create slider for shin angle
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(-157)  # -1.57 radians * 100
        self.slider.setMaximum(157)   # 1.57 radians * 100
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.slider_changed)
        
        # Create label to show current value
        self.label = QLabel("Shin Angle: 0.00 rad")
        
        self.shin_title = QLabel("Shin Joint Control (LF)")
        layout.addWidget(self.shin_title)
        layout.addWidget(self.slider)
        layout.addWidget(self.label)

        # Create slider for steering angle
        self.slider_steering = QSlider(Qt.Horizontal)
        self.slider_steering.setMinimum(-90)  # degrees
        self.slider_steering.setMaximum(90)
        self.slider_steering.setValue(0)
        self.slider_steering.valueChanged.connect(self.slider_steering_changed)
        self.label_steering = QLabel("Steering Angle: 0.00 rad")
        self.steering_title = QLabel("Steering Joint Control (LF)")
        layout.addWidget(self.steering_title)
        layout.addWidget(self.slider_steering)
        layout.addWidget(self.label_steering)

        # Create slider for wheel velocity
        self.slider_wheel = QSlider(Qt.Horizontal)
        self.slider_wheel.setMinimum(-2000)  # -20 rad/s * 100
        self.slider_wheel.setMaximum(2000)   # 20 rad/s * 100
        self.slider_wheel.setValue(0)
        self.slider_wheel.valueChanged.connect(self.slider_wheel_changed)
        self.label_wheel = QLabel("Wheel Velocity: 0.00 rad/s")
        self.wheel_title = QLabel("Wheel Velocity Control (LF)")
        layout.addWidget(self.wheel_title)
        layout.addWidget(self.slider_wheel)
        layout.addWidget(self.label_wheel)

        # Create slider for head rotation
        self.slider_head = QSlider(Qt.Horizontal)
        self.slider_head.setMinimum(-90)  # -90 degrees
        self.slider_head.setMaximum(90)   # 90 degrees
        self.slider_head.setValue(0)
        self.slider_head.valueChanged.connect(self.slider_head_changed)
        self.label_head = QLabel("Head Angle: 0.00 rad")
        self.head_title = QLabel("Head Rotation Control")
        layout.addWidget(self.head_title)
        layout.addWidget(self.slider_head)
        layout.addWidget(self.label_head)

        self.setLayout(layout)
        self.setWindowTitle('Four Legs Joint Control')
        
    def slider_changed(self, value):
        # Convert to radians
        angle_rad = value / 100.0
        
        # Update label
        self.label.setText(f"Shin Angle: {angle_rad:.2f} rad")
        
        self.publish_value(self.shin_publishers, angle_rad)
        
        # Spin ROS 2 nodex
        rclpy.spin_once(self.node, timeout_sec=0)

    def slider_thigh_changed(self, value):
        # Convert to radians
        angle_rad = value / 100.0
        
        # Update label
        self.label_thigh.setText(f"Thigh Angle: {angle_rad:.2f} rad")
        
        self.publish_value(self.thigh_publishers, angle_rad)
        # Spin ROS 2 node
        rclpy.spin_once(self.node, timeout_sec=0)

    def slider_steering_changed(self, value):
        # Convert to radians (-90 to 90 degrees)
        angle_rad = (value / 90.0) * (3.141592653589793 / 2)

        self.label_steering.setText(f"Steering Angle: {angle_rad:.2f} rad")

        self.publish_value(self.steering_publishers, angle_rad)
        rclpy.spin_once(self.node, timeout_sec=0)

    def slider_wheel_changed(self, value):
        # Convert to rad/s
        velocity = value / 100.0

        self.label_wheel.setText(f"Wheel Velocity: {velocity:.2f} rad/s")

        self.publish_value(self.wheel_publishers, velocity)
        rclpy.spin_once(self.node, timeout_sec=0)

    def slider_head_changed(self, value):
        # Convert to radians (-90 to 90 degrees)
        angle_rad = (value / 90.0) * (3.141592653589793 / 2)

        self.label_head.setText(f"Head Angle: {angle_rad:.2f} rad")

        msg = Float64()
        msg.data = angle_rad
        self.head_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0)

    def leg_changed(self, index):
        # Update current leg ID and section titles
        _, leg_id = self.legs[index]
        self.current_leg = leg_id
        suffix = f"({self.legs[index][0]})"
        self.thigh_title.setText(f"Thigh Joint Control {suffix}")
        self.shin_title.setText(f"Shin Joint Control {suffix}")
        self.steering_title.setText(f"Steering Joint Control {suffix}")
        self.wheel_title.setText(f"Wheel Velocity Control {suffix}")

    def publish_value(self, publisher_dict, value):
        msg = Float64()
        msg.data = value
        publisher_dict[self.current_leg].publish(msg)

def main():
    from python_qt_binding.QtWidgets import QApplication
    app = QApplication(sys.argv)
    gui = RobotControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()