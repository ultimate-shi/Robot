from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    MAP_YAML = "/home/shijiahao/Downloads/studyroom.yaml"

    return LaunchDescription([
        # 1. 2D地图服务器（删除错误参数，纯生命周期节点）
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[
                {'yaml_filename': MAP_YAML},
                {'use_sim_time': False}
            ]
        ),

        # 2. 生命周期管理器：自动配置+激活地图（核心）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

        # 3. 静态坐标变换 map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 4. 3D点云发布节点（你的真实房间点云）
        Node(
            package='robot',
            executable='publish_ply',
            name='ply_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # 5. 启动RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])