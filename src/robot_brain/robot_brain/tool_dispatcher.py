"""使用方法：MissionManager 调用 dispatch，把已校验的模型动作转换为固定任务参数。"""

from robot_brain.action_schema import ModelAction


def dispatch(action: ModelAction):
    """生成不含 ROS 坐标和控制权的任务预览请求。"""
    if action.name == 'goto_object':
        return {'task': 'goto_object', 'label': action.arguments['label'].strip()}
    if action.name == 'follow_person':
        return {'task': 'follow_person', 'label': ''}
    if action.name == 'explore':
        return {'task': 'explore', 'label': ''}
    raise ValueError('动作不在工具白名单中')
