"""使用方法：动作获授权后用本模块在最新 YOLO 快照中重新解析候选目标。"""

from copy import deepcopy

from robot_brain.contracts import TargetResolution


class TargetResolver:
    """只按白名单动作和最新结构化检测匹配，不接受模型生成坐标。"""

    @staticmethod
    def resolve(action, detections, preferred_target_id=''):
        values = deepcopy(list(detections or []))
        if action.name == 'explore':
            return TargetResolution(True, '探索任务不需要视觉目标', ())
        if action.name == 'follow_person':
            values = [item for item in values if str(
                item.get('class_name', '')).lower() == 'person']
            missing = '最新画面未检测到人员，未生成跟随任务。'
        elif action.name == 'goto_object':
            label = str(action.arguments.get('label', '')).strip().lower()
            values = [item for item in values if label in {
                str(item.get('class_name', '')).strip().lower(),
                str(item.get('label_zh', '')).strip().lower(),
            }]
            missing = (
                f'最新画面未检测到{action.arguments.get("label", "目标物体")}，'
                '未生成前往任务。')
        else:
            return TargetResolution(False, '动作不在目标解析白名单中', ())

        if not values:
            return TargetResolution(False, missing, ())
        ids = {str(item.get('id', '')) for item in values}
        selected = preferred_target_id if preferred_target_id in ids else (
            str(values[0].get('id', '')) if len(values) == 1 else '')
        return TargetResolution(
            True, f'最新画面匹配到 {len(values)} 个候选目标',
            tuple(values), selected)
