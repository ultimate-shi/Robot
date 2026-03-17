#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CmdForwarderNode(Node):
    def __init__(self):
        super().__init__("cmd_forwarder_node")
        # QoS配置：可靠传输，匹配控制器的通信需求
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publishers = {}  # 存储物理机器人的指令发布器
        
        # ===================== 1. 定义所有控制器名称（与你的controller.yaml完全一致） =====================
        # 大腿位置控制器列表
        lap_controllers = [
            "lap_rf_position_controller",
            "lap_lf_position_controller",
            "lap_rr_position_controller",
            "lap_lr_position_controller"
        ]
        # 小腿位置控制器列表
        shin_controllers = [
            "shin_rf_position_controller",
            "shin_lf_position_controller",
            "shin_rr_position_controller",
            "shin_lr_position_controller"
        ]
        # 转向位置控制器列表
        motor_controllers = [
            "motor_rf_position_controller",
            "motor_lf_position_controller",
            "motor_rr_position_controller",
            "motor_lr_position_controller"
        ]
        # 轮子速度控制器列表
        wheel_controllers = [
            "wheel_rf_velocity_controller",
            "wheel_lf_velocity_controller",
            "wheel_rr_velocity_controller",
            "wheel_lr_velocity_controller"
        ]
        # 合并所有控制器
        all_controllers = lap_controllers + shin_controllers + motor_controllers + wheel_controllers

        # ===================== 2. 构建话题映射并创建订阅/发布器 =====================
        for controller_name in all_controllers:
            # Gazebo仿真控制器指令话题（主指令源）
            gazebo_topic = f"/robot/gazebo/{controller_name}/command"
            # 物理机器人控制器指令话题（转发目标）
            physical_topic = f"/robot/physical/{controller_name}/command"
            
            # 创建物理机器人指令发布器
            self.publishers[physical_topic] = self.create_publisher(
                Float64, physical_topic, self.qos
            )
            
            # 订阅Gazebo指令，绑定转发回调（通过lambda传递目标话题）
            self.create_subscription(
                Float64,
                gazebo_topic,
                lambda msg, pt=physical_topic: self.forward_cmd(msg, pt),
                self.qos
            )
        
        # 打印启动日志，确认绑定的控制器数量
        self.get_logger().info(f"指令转发节点启动成功，已绑定{len(all_controllers)}个控制器指令话题")

    def forward_cmd(self, msg, physical_topic):
        """
        核心转发逻辑：将Gazebo的指令原样转发给物理机器人
        :param msg: Gazebo控制器的指令消息（Float64）
        :param physical_topic: 物理机器人控制器的指令话题
        """
        self.publishers[physical_topic].publish(msg)
        # 调试日志（可选，发布时打印指令值）
        # self.get_logger().debug(f"转发指令到 {physical_topic}：{msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdForwarderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("指令转发节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()