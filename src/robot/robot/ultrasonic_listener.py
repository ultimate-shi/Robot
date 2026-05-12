import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.executors import ExternalShutdownException

# 8个传感器配置
SENSORS = [
    {"link": "radar-front_fl", "topic": "/ultrasonic/front_fl"},
    {"link": "radar-front_fr", "topic": "/ultrasonic/front_fr"},
    {"link": "radar-front_rl", "topic": "/ultrasonic/front_rl"},
    {"link": "radar-front_rr", "topic": "/ultrasonic/front_rr"},
    {"link": "radar-side_fl", "topic": "/ultrasonic/side_fl"},
    {"link": "radar-side_fr", "topic": "/ultrasonic/side_fr"},
    {"link": "radar-side_rl", "topic": "/ultrasonic/side_rl"},
    {"link": "radar-side_rr", "topic": "/ultrasonic/side_rr"},
]

class UltrasonicListenerNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_listener_node')
        self.get_logger().info('✅ 超声波监听节点已启动（单行打印模式）')
        
        # 缓存所有传感器的最新距离
        self.distances = {sensor['link']: 4.0 for sensor in SENSORS}
        
        # 创建订阅
        self.subscribers = []
        for sensor in SENSORS:
            topic = sensor['topic']
            link = sensor['link']
            sub = self.create_subscription(
                Range,
                topic,
                lambda msg, link=link: self.update_distance(msg, link),
                10
            )
            self.subscribers.append(sub)
        
        # 10Hz 统一打印（一行输出所有数据）
        self.create_timer(0.1, self.print_all_distances)

    def update_distance(self, msg: Range, sensor_name: str):
        """仅更新数据，不打印"""
        self.distances[sensor_name] = msg.range

    def print_all_distances(self):
        """🔥 一行打印所有8路超声波数据"""
        log_str = "📡 超声波数据："
        for name, dist in self.distances.items():
            log_str += f"[{name}]: {dist:.2f}m  "
        # self.get_logger().info(log_str.strip())

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicListenerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # 正常退出，不打印traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()