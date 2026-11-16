"""使用方法：pytest 运行本文件，验证场景快照带版本且不受源字典后续修改影响。"""

from robot_brain.contracts import SceneSnapshot
from robot_brain.scene_coordinator import SceneCoordinator


def test_scene_snapshot_is_versioned_and_copied():
    source = {
        'state': 'valid', 'stamp_ns': 123,
        'image': {'width': 640, 'height': 480},
        'model': 'yolo',
        'detections': [{'id': 'person-1', 'class_name': 'person'}],
    }

    snapshot = SceneSnapshot.freeze(source, image_stamp_ns=123)
    source['detections'][0]['id'] = 'changed'

    assert snapshot.schema_version == 'scene.v1'
    assert snapshot.snapshot_id == 'scene-123'
    assert snapshot.detection_list()[0]['id'] == 'person-1'
    assert snapshot.audit_dict()['detection_count'] == 1


def test_scene_coordinator_does_not_replace_scene_when_only_woken():
    coordinator = SceneCoordinator()
    coordinator.update_detection({
        'state': 'valid', 'stamp_ns': 100,
        'detections': [{'id': 'person-1'}],
    })

    coordinator.wake()

    assert coordinator.snapshot().detection_stamp_ns == 100
    assert coordinator.snapshot().detection_list()[0]['id'] == 'person-1'


def test_scene_snapshot_limits_every_consumer_to_twenty_objects():
    snapshot = SceneSnapshot.freeze({
        'stamp_ns': 1,
        'detections': [{'id': str(index)} for index in range(25)],
    })

    assert len(snapshot.detections) == 20
    assert snapshot.detection_list()[-1]['id'] == '19'
