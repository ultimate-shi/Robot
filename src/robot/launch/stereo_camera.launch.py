"""真实横向拼接双目相机的独立取流、校正、深度和点云启动入口."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    camera_params = os.path.join(
        pkg_share, 'config', 'stereo_camera.yaml')
    calibration_dir = os.path.join(
        pkg_share, 'config', 'cameras',
        'usb_camera_01_00_00_640x480')
    calibration_mode = LaunchConfiguration('calibration_mode')
    camera_config = LaunchConfiguration('camera_config')
    video_device = LaunchConfiguration('video_device')
    apply_auto_camera_controls = LaunchConfiguration(
        'apply_auto_camera_controls')
    # 独立启动时默认开启；组合入口可用内部配置避免同一端口重复启动。
    start_foxglove_bridge = LaunchConfiguration(
        '_camera_start_foxglove_bridge', default='true')
    foxglove_port = LaunchConfiguration('foxglove_port')
    log_level = LaunchConfiguration('log_level')
    splitter_backend = LaunchConfiguration('splitter_backend')
    ros_args = ['--ros-args', '--log-level', log_level]
    processing_condition = UnlessCondition(calibration_mode)
    navigation_processing_condition = IfCondition(PythonExpression([
        '"', LaunchConfiguration('navigation_processing_enabled'),
        '" == "true" and "', calibration_mode, '" != "true"',
    ]))
    compressed_condition = IfCondition(PythonExpression([
        '"', LaunchConfiguration('publish_compressed'), '" == "true" and "',
        calibration_mode, '" != "true"',
    ]))
    cpp_splitter_condition = IfCondition(PythonExpression([
        '"', splitter_backend, '" == "cpp"',
    ]))
    python_splitter_condition = IfCondition(PythonExpression([
        '"', splitter_backend, '" == "python"',
    ]))

    declarations = [
        DeclareLaunchArgument('calibration_mode', default_value='false'),
        DeclareLaunchArgument('publish_compressed', default_value='true'),
        DeclareLaunchArgument(
            'navigation_processing_enabled', default_value='true'),
        DeclareLaunchArgument('splitter_backend', default_value='cpp'),
        DeclareLaunchArgument('camera_config', default_value=camera_params),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='false',
            description=(
                '启动取流前直接写入 UVC 自动曝光和自动白平衡控制；'
                '标定模式应保持 false')),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(calibration_dir, 'left.yaml')),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(calibration_dir, 'right.yaml')),
        DeclareLaunchArgument('log_level', default_value='warn'),
    ]

    def camera_node(condition=None):
        """创建取流节点；自动控制开启时由控制命令完成事件启动."""
        return Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[camera_config, {'video_device': video_device}],
            remappings=[
                ('image_raw', '/stereo/image_raw'),
                ('camera_info', '/stereo/combined/camera_info'),
            ],
            condition=condition,
            arguments=ros_args,
            output='screen',
        )

    # 当前 UVC 驱动控制名与 usb_cam 通用参数名不一致，ROS 参数为 true
    # 不能保证硬件生效。建图入口显式写入实际 V4L2 控制并回读确认，
    # 命令退出后才启动 usb_cam，避免设备占用导致设置失败。
    camera_controls = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '-d', video_device,
            '--set-ctrl=brightness=0,white_balance_automatic=1,auto_exposure=3',
            '--get-ctrl=brightness,white_balance_automatic,'
            'white_balance_temperature,auto_exposure,exposure_time_absolute',
        ],
        condition=IfCondition(apply_auto_camera_controls),
        output='screen',
    )
    camera_after_controls = RegisterEventHandler(
        OnProcessExit(
            target_action=camera_controls,
            on_exit=[camera_node()],
        ),
        condition=IfCondition(apply_auto_camera_controls),
    )
    camera_without_controls = camera_node(
        condition=UnlessCondition(apply_auto_camera_controls))
    splitter = Node(
        package='robot',
        executable='stereo_splitter_node',
        name='stereo_splitter',
        parameters=[
            camera_config,
            {
                'calibration_mode': calibration_mode,
                'left_calibration_file': LaunchConfiguration(
                    'left_calibration_file'),
                'right_calibration_file': LaunchConfiguration(
                    'right_calibration_file'),
            },
        ],
        condition=python_splitter_condition,
        arguments=ros_args,
        output='screen',
    )
    cpp_splitter = Node(
        package='robot_stereo_components',
        executable='stereo_splitter_cpp',
        name='stereo_splitter',
        parameters=[
            camera_config,
            {
                'calibration_mode': calibration_mode,
                'left_calibration_file': LaunchConfiguration(
                    'left_calibration_file'),
                'right_calibration_file': LaunchConfiguration(
                    'right_calibration_file'),
            },
        ],
        condition=cpp_splitter_condition,
        arguments=ros_args,
        output='screen',
    )
    left_rectify = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        namespace='stereo/left',
        remappings=[('image', 'image_raw')],
        condition=processing_condition,
        arguments=ros_args,
        output='screen',
    )
    right_rectify = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        namespace='stereo/right',
        remappings=[('image', 'image_raw')],
        condition=processing_condition,
        arguments=ros_args,
        output='screen',
    )
    pair_throttle = Node(
        package='robot',
        executable='stereo_pair_throttle_node',
        name='stereo_pair_throttle',
        parameters=[camera_config, {'use_sim_time': False}],
        condition=navigation_processing_condition,
        arguments=ros_args,
        output='screen',
    )
    disparity = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        parameters=[camera_config],
        remappings=[
            ('left/image_rect', '/stereo/navigation/left/image_rect'),
            ('left/camera_info', '/stereo/navigation/left/camera_info'),
            ('right/image_rect', '/stereo/navigation/right/image_rect'),
            ('right/camera_info', '/stereo/navigation/right/camera_info'),
            ('disparity', '/stereo/disparity'),
        ],
        condition=navigation_processing_condition,
        arguments=ros_args,
        output='screen',
    )
    point_cloud = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        remappings=[
            ('left/image_rect_color', '/stereo/navigation/left/image_rect'),
            ('left/camera_info', '/stereo/navigation/left/camera_info'),
            ('right/camera_info', '/stereo/navigation/right/camera_info'),
            ('disparity', '/stereo/disparity'),
            ('points2', '/stereo/points2'),
        ],
        condition=navigation_processing_condition,
        arguments=ros_args,
        output='screen',
    )
    depth = Node(
        package='robot',
        executable='stereo_depth_node',
        name='stereo_depth',
        parameters=[camera_config],
        condition=navigation_processing_condition,
        arguments=ros_args,
        output='screen',
    )
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='stereo_foxglove_bridge',
        parameters=[{
            'port': ParameterValue(foxglove_port, value_type=int),
            'address': '0.0.0.0',
            'asset_uri_allowlist': ['package://robot/.*'],
            'allow_file_transfer': True,
            'send_buffer_limit': 10000000,
            'max_packet_messages': 100,
            'client_timeout_ms': 300000,
            'keep_alive_interval_ms': 5000,
        }],
        condition=IfCondition(start_foxglove_bridge),
        arguments=ros_args,
        output='screen',
    )

    def compressed_republisher(name, topic):
        return Node(
            package='image_transport',
            executable='republish',
            name=name,
            # Jazzy 使用参数选择传输插件；旧式位置参数会让 out_transport 为空，
            # 从而在同名基话题上重新发布 raw 图并形成自回环。
            parameters=[{
                'in_transport': 'raw',
                'out_transport': 'compressed',
            }],
            arguments=ros_args,
            # compressed 插件直接发布 out/compressed，必须重映射插件话题本身。
            remappings=[
                ('in', topic),
                ('out/compressed', topic + '/compressed'),
            ],
            condition=compressed_condition,
            output='screen',
        )

    return LaunchDescription(declarations + [
        camera_after_controls,
        camera_controls,
        camera_without_controls,
        splitter,
        cpp_splitter,
        left_rectify,
        right_rectify,
        pair_throttle,
        disparity,
        point_cloud,
        depth,
        foxglove,
        compressed_republisher(
            'left_compressed_republisher', '/stereo/left/image_rect'),
        compressed_republisher(
            'right_compressed_republisher', '/stereo/right/image_rect'),
        compressed_republisher(
            'depth_compressed_republisher',
            '/stereo/depth/image_visual'),
    ])
