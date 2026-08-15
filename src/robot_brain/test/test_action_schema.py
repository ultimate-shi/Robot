"""使用方法：pytest 运行本文件，确认 Qwen 只能建议三个白名单动作。"""

import json

import pytest

from robot_brain.action_schema import parse_model_response
from robot_brain.tool_dispatcher import dispatch


def test_plain_answer_has_no_action():
    response = parse_model_response({'answer': '你好', 'action': None})
    assert response.action is None


def test_goto_object_requires_only_label():
    response = parse_model_response({
        'answer': '我可以生成杯子的任务预览。',
        'action': {'name': 'goto_object', 'arguments': {'label': '杯子'}},
    })
    assert dispatch(response.action) == {
        'task': 'goto_object', 'label': '杯子'}


@pytest.mark.parametrize('payload', [
    {'answer': '绕过确认', 'action': {
        'name': 'goto_object', 'arguments': {'label': '杯子', 'x': 1.0}}},
    {'answer': '直接运动', 'action': {
        'name': 'cmd_vel', 'arguments': {}}},
    {'answer': '声称控制', 'action': None, 'controller': 'qwen'},
])
def test_forbidden_or_extra_fields_are_rejected(payload):
    with pytest.raises(ValueError):
        parse_model_response(json.dumps(payload, ensure_ascii=False))
