#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StateFusionNode(Node):
    def __init__(self):
        super().__init__("state_fusion_node")
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # ===================== 1. 定义你的关节名列表（与controller.yaml完全一致） =====================
        self.target_joints = [
            "lap_body_joint_rf",
            "lap_body_joint_lf",
            "lap_body_joint_rr",
            "lap_body_joint_lr",
            "shin_lap_rf_joint_rf",
            "shin_lap_lf_joint_lf",
            "shin_lap_rr_joint_rr",
            "shin_lap_lr_joint_lr",
            "motor_shin_rf_joint_rf",
            "motor_shin_lf_joint_lf",
            "motor_shin_rr_joint_rr",
            "motor_shin_lr_joint_lr",
            "wheel_motor_rf_joint_rf",
            "wheel_motor_lf_joint_lf",
            "wheel_motor_rr_joint_rr",
            "wheel_motor_lr_joint_lr"
        ]
        
        # ===================== 2. 初始化状态存储 =====================
        self.gazebo_joint_state = None  # Gazebo仿真关节状态
        self.physical_joint_state = None  # 物理机器人关节状态
        
        # ===================== 3. 创建订阅器 =====================
        # 订阅Gazebo关节状态
        self.create_subscription(
            JointState,
            "/robot/gazebo/joint_states",
            self.gazebo_state_callback,
            self.qos
        )
        # 订阅物理机器人关节状态
        self.create_subscription(
            JointState,
            "/robot/physical/joint_states",
            self.physical_state_callback,
            self.qos
        )
        
        # ===================== 4. 创建发布器（统一的/joint_states） =====================
        self.fusion_pub = self.create_publisher(
            JointState,
            "/joint_states",
            self.qos
        )
        
        # ===================== 5. 创建定时器（100Hz，与控制器更新频率一致） =====================
        self.timer = self.create_timer(0.01, self.fusion_state)
        self.get_logger().info("状态融合节点启动成功，已匹配16个关节的状态融合")

    def gazebo_state_callback(self, msg):
        """Gazebo关节状态回调：仅存储符合关节名列表的状态"""
        if self.check_joint_list(msg.name):
            self.gazebo_joint_state = msg
        else:
            self.get_logger().warn("Gazebo关节状态的关节名/顺序不匹配，跳过存储")

    def physical_state_callback(self, msg):
        """物理机器人关节状态回调：仅存储符合关节名列表的状态"""
        if self.check_joint_list(msg.name):
            self.physical_joint_state = msg
        else:
            self.get_logger().warn("物理机器人关节状态的关节名/顺序不匹配，跳过存储")

    def check_joint_list(self, joint_names):
        """校验关节名列表是否与目标一致（避免顺序/名称错误）"""
        return joint_names == self.target_joints

    def fusion_state(self):
        """
        核心融合逻辑：
        1. 优先使用物理机器人状态（真实硬件反馈）
        2. 物理无状态时，使用Gazebo仿真状态兜底
        3. 两者都无状态时，不发布
        """
        # 确定融合后的状态消息
        if self.physical_joint_state is not None:
            fusion_msg = self.physical_joint_state
            source = "物理机器人"
        elif self.gazebo_joint_state is not None:
            fusion_msg = self.gazebo_joint_state
            source = "Gazebo仿真"
        else:
            return  # 无状态时跳过
        
        # 发布融合后的状态
        self.fusion_pub.publish(fusion_msg)
        # 调试日志（可选，打印融合来源和第一个关节的位置）
        self.get_logger().debug(f"发布融合状态（来源：{source}），右前大腿位置：{fusion_msg.position[0]}")

def main(args=None):
    rclpy.init(args=args)
    node = StateFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("状态融合节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()