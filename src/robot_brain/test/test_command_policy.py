"""使用方法：pytest 运行本文件，验证模型动作必须通过用户原话确定性授权。"""

import pytest

from robot_brain.action_schema import ModelAction, ModelResponse
from robot_brain.command_policy import CommandPolicy
from robot_brain.target_resolver import TargetResolver


DETECTIONS = [
    {'id': 'plant-1', 'label_zh': '盆栽', 'class_name': 'potted plant'},
    {'id': 'person-1', 'label_zh': '人', 'class_name': 'person'},
    {'id': 'laptop-1', 'label_zh': '笔记本电脑', 'class_name': 'laptop'},
]


@pytest.mark.parametrize('text', [
    '去盆栽的地方', '去盆栽旁边', '到盆栽那里', '帮我找盆栽',
    '去笔记本电脑条的地方',
])
def test_natural_goto_phrases_are_authorized(text):
    label = '笔记本电脑' if '笔记本' in text else '盆栽'
    proposal = ModelResponse('建议前往。', ModelAction(
        'goto_object', {'label': label}))

    result = CommandPolicy.authorize(text, proposal, DETECTIONS)

    assert result.reason_code == 'AUTHORIZED'
    assert result.action == ModelAction('goto_object', {'label': label})


@pytest.mark.parametrize('text', [
    '不要去盆栽的地方', '我不想去盆栽旁边', '先不去盆栽那里',
])
def test_negated_goto_never_authorizes_action(text):
    proposal = ModelResponse('建议前往。', ModelAction(
        'goto_object', {'label': '盆栽'}))

    result = CommandPolicy.authorize(text, proposal, DETECTIONS)

    assert result.reason_code == 'NEGATED_COMMAND'
    assert result.action is None


def test_position_question_drops_wrong_model_action():
    proposal = ModelResponse('准备前往。', ModelAction(
        'goto_object', {'label': '盆栽'}))

    result = CommandPolicy.authorize('盆栽在哪里？', proposal, DETECTIONS)

    assert result.reason_code == 'LOCATION_QUESTION'
    assert result.action is None


def test_capability_question_does_not_start_following():
    proposal = ModelResponse('可以。', ModelAction('follow_person', {}))

    result = CommandPolicy.authorize('你能跟随人吗？', proposal, DETECTIONS)

    assert result.reason_code == 'QUESTION_NOT_COMMAND'
    assert result.action is None


def test_explicit_follow_recovers_from_model_omission():
    result = CommandPolicy.authorize(
        '跟着我走', ModelResponse('好的。'), DETECTIONS)

    assert result.source == 'deterministic_fallback'
    assert result.action == ModelAction('follow_person', {})


def test_null_action_cannot_claim_that_preview_was_created():
    """画面无目标时，不得用虚假的确认提示冒充任务面板。"""
    proposal = ModelResponse(
        '将生成前往椅子旁边的任务预演，请确认。')

    result = CommandPolicy.authorize('去椅子旁边', proposal, [])

    assert result.reason_code == 'UNGROUNDED_ACTION_CLAIM'
    assert result.action is None
    assert result.answer == (
        '当前画面未识别到与命令匹配的目标，'
        '未生成任务预演。请先刷新识别后再试。')


def test_hallucinated_goto_target_is_reported_as_not_detected():
    """Qwen 编造 scene 中不存在的物体时，丢弃动作并返回真实原因。"""
    proposal = ModelResponse(
        '将生成前往电视的任务预演，请确认。',
        ModelAction('goto_object', {'label': '电视'}))

    result = CommandPolicy.authorize('去电视的地方', proposal, [])

    assert result.reason_code == 'TARGET_NOT_DETECTED'
    assert result.action is None
    assert result.answer == '当前画面未检测到电视，未生成任务预演。'


def test_hallucinated_location_target_is_reported_as_not_detected():
    proposal = ModelResponse(
        '将生成前往电视的任务预演，请确认。',
        ModelAction('goto_object', {'label': '电视'}))

    result = CommandPolicy.authorize('电视在哪里', proposal, [])

    assert result.reason_code == 'LOCATION_TARGET_NOT_DETECTED'
    assert result.action is None
    assert result.answer == '当前画面未检测到电视。'


def test_target_resolver_uses_only_new_scene_candidates():
    action = ModelAction('follow_person', {})
    old_scene = [{'id': 'person-old', 'class_name': 'person'}]
    new_scene = [{'id': 'person-new', 'class_name': 'person'}]

    old = TargetResolver.resolve(action, old_scene)
    new = TargetResolver.resolve(action, new_scene)

    assert old.selected_target_id == 'person-old'
    assert new.selected_target_id == 'person-new'
    assert new.candidate_list()[0]['id'] == 'person-new'


def test_follow_fails_when_refreshed_scene_has_no_person():
    result = TargetResolver.resolve(
        ModelAction('follow_person', {}), [{'class_name': 'cup'}])

    assert result.success is False
    assert result.candidates == ()
    assert '未检测到人员' in result.message
