#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
try:
    from plyfile import PlyData
except:
    print("❌ 未安装plyfile，请运行：pip install plyfile")

class PLYPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        
        self.pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
        self.timer = self.create_timer(0.5, self.publish_cloud)
        
        # 🔥 强制绝对路径（launch模式下100%找到文件）
        self.ply_file = "/home/shijiahao/Downloads/studyroom.ply"
        self.points = None
        
        # 🔥 加异常捕获：加载失败直接打印错误
        try:
            self.load_ply()
            self.get_logger().info(f"✅ 成功加载点云：{len(self.points)} 个点")
        except Exception as e:
            self.get_logger().error(f"❌ 加载PLY失败：{str(e)}")
            # 加载失败时，发布测试点（和simple_pc一样，保证能显示）
            self.points = np.array([[0,0,0], [0.1,0.1,0.1]], dtype=np.float32)

    def load_ply(self):
        plydata = PlyData.read(self.ply_file)
        x = plydata['vertex']['x']
        y = plydata['vertex']['y']
        z = plydata['vertex']['z']
        self.points = np.column_stack((x, y, z)).astype(np.float32)

    def publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  # 必须和地图一致

        # 字段格式和simple_pc完全一致
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = PointCloud2()
        cloud.header = header
        cloud.width = len(self.points)
        cloud.height = 1
        cloud.fields = fields
        cloud.point_step = 12
        cloud.row_step = cloud.width * 12
        cloud.data = self.points.tobytes()
        cloud.is_dense = True

        self.pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = PLYPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()