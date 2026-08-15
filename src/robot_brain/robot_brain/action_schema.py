"""使用方法：QwenClient 对每轮模型输出调用 parse_model_response，校验失败时绝不执行工具。"""

from dataclasses import dataclass
import json


ALLOWED_ACTIONS = {'goto_object', 'follow_person', 'explore'}


@dataclass(frozen=True)
class ModelAction:
    """通过严格白名单校验的模型动作。"""

    name: str
    arguments: dict


@dataclass(frozen=True)
class ModelResponse:
    """模型文字答复和可选动作。"""

    answer: str
    action: ModelAction | None = None


def parse_model_response(raw):
    """只接受 answer/action 两个顶层字段以及固定动作参数。"""
    payload = json.loads(raw) if isinstance(raw, str) else raw
    if not isinstance(payload, dict) or set(payload) != {'answer', 'action'}:
        raise ValueError('模型输出必须且只能包含 answer 和 action')
    if not isinstance(payload['answer'], str):
        raise ValueError('answer 必须是字符串')
    action = payload['action']
    if action is None:
        return ModelResponse(answer=payload['answer'])
    if not isinstance(action, dict) or set(action) != {'name', 'arguments'}:
        raise ValueError('action 必须且只能包含 name 和 arguments')
    name = action['name']
    arguments = action['arguments']
    if name not in ALLOWED_ACTIONS or not isinstance(arguments, dict):
        raise ValueError('动作不在机器人白名单中')
    expected = {'label'} if name == 'goto_object' else set()
    if set(arguments) != expected:
        raise ValueError(f'{name} 参数必须是 {sorted(expected)}')
    if name == 'goto_object' and (
            not isinstance(arguments['label'], str)
            or not arguments['label'].strip()):
        raise ValueError('goto_object.label 不能为空')
    return ModelResponse(
        answer=payload['answer'], action=ModelAction(name, arguments))
