"""使用方法：大脑内部用本模块在感知、Qwen、策略和任务层之间传递版本化数据。"""

from copy import deepcopy
from dataclasses import dataclass

from robot_brain.action_schema import ModelAction


SCENE_SCHEMA_VERSION = 'scene.v1'


@dataclass(frozen=True)
class SceneSnapshot:
    """一次不可替换的 YOLO 场景；动作链必须显式创建更新后的新快照。"""

    schema_version: str
    snapshot_id: str
    state: str
    detection_stamp_ns: int | None
    image_stamp_ns: int | None
    image_size: dict
    model: str
    detections: tuple

    @classmethod
    def freeze(cls, detection_state, image_stamp_ns=None):
        """从共享 ROS 状态复制场景，避免聊天排队期间被新检测替换。"""
        source = deepcopy(detection_state or {})
        stamp = source.get('stamp_ns')
        # 同一快照最多二十项，确保 Qwen、策略、审计图和动作候选口径一致。
        values = tuple(deepcopy(source.get('detections', [])[:20]))
        default_state = ('waiting' if stamp is None else
                         ('valid' if values else 'valid_empty'))
        state = str(source.get('state', default_state))
        snapshot_id = f'scene-{stamp}' if stamp is not None else 'scene-none'
        return cls(
            schema_version=SCENE_SCHEMA_VERSION,
            snapshot_id=snapshot_id,
            state=state,
            detection_stamp_ns=stamp,
            image_stamp_ns=image_stamp_ns,
            image_size=deepcopy(source.get('image', {})),
            model=str(source.get('model', '')),
            detections=values,
        )

    def detection_list(self):
        """返回可交给现有 ROS 和网页适配层的独立字典列表。"""
        return deepcopy(list(self.detections))

    def audit_dict(self):
        """返回不含图片字节的审计摘要。"""
        return {
            'schema_version': self.schema_version,
            'snapshot_id': self.snapshot_id,
            'state': self.state,
            'detection_stamp_ns': self.detection_stamp_ns,
            'image_stamp_ns': self.image_stamp_ns,
            'image_size': deepcopy(self.image_size),
            'model': self.model,
            'detection_count': len(self.detections),
        }


@dataclass(frozen=True)
class PolicyResult:
    """确定性策略对不可信模型动作提案作出的授权结果。"""

    answer: str
    action: ModelAction | None
    reason_code: str
    source: str


@dataclass(frozen=True)
class TargetResolution:
    """在最新视觉场景中匹配动作目标后的结果。"""

    success: bool
    message: str
    candidates: tuple
    selected_target_id: str = ''

    def candidate_list(self):
        return deepcopy(list(self.candidates))
