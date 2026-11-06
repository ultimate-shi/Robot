"""作用：验证 Astra 替换双目后的整车输入和旧模式兼容；用法：Jazzy 环境 pytest 本文件，不执行硬件 launch。"""
import importlib.util
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from test_map_localization_launch import main_launch, nested_includes


@pytest.mark.parametrize('source', ['astra', 'stereo'])
def test_main_selects_complete_camera_chain(source, monkeypatch):
    description = main_launch()
    context = LaunchContext()
    context.launch_configurations.update(mode='real', camera_source=source)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    actions = next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
    includes = list(nested_includes(actions))
    monkeypatch.setattr(PythonLaunchDescriptionSource, '_get_launch_description', lambda self, location: LaunchDescription())
    for item in includes:
        item.launch_description_source.get_launch_description(context)
    by_name = {Path(item.launch_description_source.location).name: dict(item.launch_arguments) for item in includes}
    if source == 'astra':
        assert 'astra_camera.launch.py' in by_name and 'rgbd_odometry.launch.py' in by_name
        assert 'stereo_camera.launch.py' not in by_name and 'stereo_odometry.launch.py' not in by_name
        assert by_name['stereo_pointcloud_filter.launch.py']['input_topic'] == '/camera/points'
        assert by_name['semantic_detection.launch.py']['depth_topic'] == '/camera/depth/image_rect'
        assert by_name['description.launch.py']['camera_model'] == 'none'
        assert by_name['controllers.launch.py']['camera_model'] == 'none'
    else:
        assert 'stereo_camera.launch.py' in by_name and 'stereo_odometry.launch.py' in by_name
        assert 'astra_camera.launch.py' not in by_name
        assert by_name['description.launch.py']['camera_model'] == 'stereo'
    assert by_name['rtabmap_mapping.launch.py']['camera_source'] == source
    assert by_name['web_console.launch.py']['camera_source'] == source


def test_mount_validation_and_external_tf_override():
    root = Path(__file__).parents[2]/'robot_perception'
    spec = importlib.util.spec_from_file_location('astra_mount_test', root/'launch/astra_mount.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = yaml.safe_load((root/'config/astra_mount.yaml').read_text())
    config['mount']['enabled'] = False
    assert module.mount_arguments(config) is None
    config['mount']['enabled'] = True
    config['mount'].update(xyz=[], rpy=[])
    with pytest.raises(ValueError):
        module.mount_arguments(config)
    config['mount'].update(xyz=[.01, -.02, .03], rpy=[0., 0., -1.57])
    args = module.mount_arguments(config)
    assert args[args.index('--x')+1] == '0.01'
    assert args[args.index('--child-frame-id')+1] == 'astra_link'


def test_mount_matches_original_head_midpoint_and_axes():
    # 展开旧模型，从两光心的真实装配位置求中心，防止只复制相机外壳的零原点。
    root = Path(__file__).parents[2]
    model = ET.fromstring(subprocess.check_output([
        'xacro', str(root/'robot_description/urdf/robot.xacro'), 'camera_model:=stereo'], text=True))
    def origin(name):
        return model.find(f"joint[@name='{name}']/origin")
    assert origin('stereo_camera_joint').attrib['xyz'] == '0 0 0'
    assert origin('stereo_camera_joint').attrib['rpy'] == '0 0 0'
    left, right = [tuple(map(float, origin(name).attrib['xyz'].split())) for name in
                   ('stereo_left_camera_joint', 'stereo_right_camera_joint')]
    config = yaml.safe_load((root/'robot_perception/config/astra_mount.yaml').read_text())['mount']
    assert config['enabled'] is True
    assert config['parent_frame'] == 'head_pitch_link'
    assert config['xyz'] == pytest.approx([(a+b)/2 for a, b in zip(left, right)])
    roll, pitch, yaw = config['rpy']
    assert [roll, pitch, yaw] == pytest.approx(list(map(float,
        origin('stereo_left_camera_joint').attrib['rpy'].split())))
    # Astra 机身 +X/光学 +Z 指向头部 -Y，机身 +Y 指向头部 +X。
    assert [math.cos(yaw), math.sin(yaw), 0.] == pytest.approx([0., -1., 0.], abs=1e-12)
    assert [-math.sin(yaw), math.cos(yaw), 0.] == pytest.approx([1., 0., 0.], abs=1e-12)
