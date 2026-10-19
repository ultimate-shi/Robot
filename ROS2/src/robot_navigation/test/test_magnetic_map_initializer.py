"""使用方法：pytest 运行本文件，离线验证磁 ENU、IMU 安装角和 EKF 航向的组合。"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace


def load_math_functions():
    """无 ROS 环境也能测试纯四元数运算。"""
    source = Path(__file__).parents[1] / 'robot_navigation' / 'magnetic_map_initializer.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    selected = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name in ('yaw_of', 'multiply', 'quaternion_of',
                                  'map_to_odom_yaw', 'map_to_odom_translation')]
    namespace = {'math': math}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'),
         namespace)
    return namespace


def quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_magnetic_map_yaw_applies_imu_mounting_and_odom_rotation():
    functions = load_math_functions()
    yaw = functions['map_to_odom_yaw'](
        0.3, quaternion(0.5), quaternion(-math.pi / 2), quaternion(-0.2))
    assert math.isclose(yaw, 0.3 + 0.5 - math.pi / 2 + 0.2, abs_tol=1e-9)


def test_invalid_orientation_is_rejected():
    functions = load_math_functions()
    assert functions['quaternion_of'](SimpleNamespace(
        x=0.0, y=0.0, z=0.0, w=0.0)) is None
    assert functions['quaternion_of'](SimpleNamespace(
        x=0.0, y=0.0, z=0.0, w=float('nan'))) is None


def test_magnetic_map_translation_preserves_requested_base_position():
    functions = load_math_functions()
    yaw = math.pi / 2
    x, y = functions['map_to_odom_translation'](4.0, 5.0, yaw, 2.0, 3.0)
    assert math.isclose(x - 3.0, 4.0, abs_tol=1e-9)
    assert math.isclose(y + 2.0, 5.0, abs_tol=1e-9)


def test_magnetic_mode_has_deferred_nav2_start_and_manual_default():
    launch = (Path(__file__).parents[1] / 'launch' / 'nav2.launch.py').read_text(
        encoding='utf-8')
    assert "'initial_yaw_source', default_value='manual'" in launch
    assert "'magnetic_east_yaw_in_map', default_value=''" in launch
    assert 'target_action=initializer, on_stdout=on_initializer_output' in launch
    assert 'return deferred_navigation' in launch
    assert "source == 'magnetic' and static_map != 'true'" in launch
