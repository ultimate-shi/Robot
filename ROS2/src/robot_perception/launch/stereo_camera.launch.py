"""使用方法：ros2 launch robot_perception stereo_camera.launch.py 启动真实双目处理链。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_perception')
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
    start_foxglove_bridge = LaunchConfiguration('start_foxglove_bridge')
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
    auto_controls_condition = IfCondition(PythonExpression([
        '"', apply_auto_camera_controls, '" == "true" and "',
        calibration_mode, '" != "true"',
    ]))
    camera_without_auto_condition = IfCondition(PythonExpression([
        '"', apply_auto_camera_controls, '" != "true" or "',
        calibration_mode, '" == "true"',
    ]))
    cpp_splitter_condition = IfCondition(PythonExpression([
        '"', splitter_backend, '" == "cpp"',
    ]))
    python_splitter_condition = IfCondition(PythonExpression([
        '"', splitter_backend, '" == "python"',
    ]))

    def staggered(index, actions):
        """在当前作用域解析间隔，再创建不会依赖子作用域的定时器。"""
        def create_timer(context):
            interval = float(LaunchConfiguration(
                'node_start_interval').perform(context))
            return [TimerAction(period=interval * index, actions=actions)]

        return OpaqueFunction(function=create_timer)

    declarations = [
        DeclareLaunchArgument(
            'calibration_mode', default_value='false',
            description='是否进入双目标定模式并跳过校正、深度等处理'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='true',
            description='是否显式发布深度预览压缩话题；左右校正图由 image_transport 按需提供'),
        DeclareLaunchArgument(
            'navigation_processing_enabled', default_value='true',
            description='是否启动导航所需的节流、视差、深度和点云处理'),
        DeclareLaunchArgument(
            'splitter_backend', default_value='cpp',
            description='双目拼接图拆分后端，可选 cpp 或 python'),
        DeclareLaunchArgument(
            'camera_config', default_value=camera_params,
            description='双目相机与图像处理参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='usb_cam 打开的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description=(
                '启动取流前直接写入 UVC 自动曝光和自动白平衡控制；'
                '默认开启，标定模式会自动跳过')),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='Foxglove Bridge 监听的 WebSocket 端口'),
        DeclareLaunchArgument(
            'start_foxglove_bridge', default_value='true',
            description='是否为独立相机入口启动 Foxglove Bridge'),
        DeclareLaunchArgument(
            'node_start_interval', default_value='0.0',
            description='双目处理链相邻启动阶段的错峰间隔，单位为秒'),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(calibration_dir, 'left.yaml'),
            description='左目相机标定 YAML 文件路径'),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(calibration_dir, 'right.yaml'),
            description='右目相机标定 YAML 文件路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='相机处理链各 ROS 节点的日志级别'),
    ]

    def camera_node(
            condition=None, config=camera_config, device=video_device,
            node_ros_args=ros_args):
        """创建取流节点；自动控制开启时由控制命令完成事件启动."""
        return Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[config, {'video_device': device}],
            remappings=[
                ('image_raw', '/stereo/image_raw'),
                ('camera_info', '/stereo/combined/camera_info'),
            ],
            condition=condition,
            arguments=node_ros_args,
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
        condition=auto_controls_condition,
        output='screen',
    )
    def register_camera_after_controls(context):
        """提前解析退出回调参数，避免被组合入口引用后丢失子 launch 作用域。"""
        resolved_config = camera_config.perform(context)
        resolved_device = video_device.perform(context)
        resolved_log_level = log_level.perform(context)
        return [RegisterEventHandler(
            OnProcessExit(
                target_action=camera_controls,
                on_exit=[camera_node(
                    config=resolved_config,
                    device=resolved_device,
                    node_ros_args=[
                        '--ros-args', '--log-level', resolved_log_level],
                )],
            ),
            condition=auto_controls_condition,
        )]
    camera_without_controls = camera_node(
        condition=camera_without_auto_condition)
    splitter = Node(
        package='robot_perception',
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
        package='robot_perception',
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
        package='robot_perception',
        executable='stereo_depth_node',
        name='stereo_depth',
        parameters=[camera_config],
        condition=navigation_processing_condition,
        arguments=ros_args,
        output='screen',
    )
    foxglove = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_description'),
                'launch', 'foxglove.launch.py')),
            launch_arguments={
                'port': foxglove_port,
                'log_level': log_level,
            }.items(),
            condition=IfCondition(start_foxglove_bridge),
        ),
    ])

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
        foxglove,
        staggered(0, [
            OpaqueFunction(function=register_camera_after_controls),
            camera_controls,
            camera_without_controls,
        ]),
        staggered(1, [splitter, cpp_splitter]),
        staggered(2, [left_rectify]),
        staggered(3, [right_rectify]),
        staggered(4, [pair_throttle]),
        staggered(5, [disparity]),
        staggered(6, [point_cloud]),
        staggered(7, [depth]),
        staggered(8, [
            # rectify_node 自带 image_transport 压缩插件；不再额外转发左右图。
            compressed_republisher(
                'depth_compressed_republisher',
                '/stereo/depth/image_visual'),
        ]),
    ])
