"""使用方法：pytest 运行本文件，检查静止建图与动态清空所需的 RTAB-Map 参数。"""

from pathlib import Path

import yaml


def test_mapping_accepts_observations_while_robot_is_stationary():
    """位姿没有变化时也必须保留低频新观测并刷新全局栅格。"""
    path = Path(__file__).parents[1] / 'config' / 'rtabmap_mapping.yaml'
    parameters = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'rtabmap']['ros__parameters']

    assert parameters['RGBD/LinearUpdate'] == '0.0'
    assert parameters['RGBD/AngularUpdate'] == '0.0'
    assert parameters['Mem/RehearsalSimilarity'] == '1.0'
    assert parameters['map_always_update'] is True
    assert 0.0 < float(parameters['Rtabmap/DetectionRate']) <= 1.0


def test_localization_improves_correspondences_without_bypassing_confidence():
    path = Path(__file__).parents[1] / 'config' / 'map_localization.yaml'
    values = yaml.safe_load(path.read_text(encoding='utf-8'))
    parameters = values['rtabmap']['ros__parameters']
    assert parameters['RGBD/LoopClosureReextractFeatures'] == 'true'
    assert parameters['Vis/FeatureType'] == '8'
    assert parameters['Vis/CorNNType'] == '5'
    assert parameters['Vis/MinInliers'] == '20'
    assert 'Kp/DetectorStrategy' not in parameters
    guard = values['localization_guard']['ros__parameters']
    assert guard['initial_pose_enabled'] is False
    assert guard['required_matches'] == 3
