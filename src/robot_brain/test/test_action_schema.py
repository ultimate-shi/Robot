"""使用方法：pytest 运行本文件，确认 Qwen 只能建议三个白名单动作。"""

import json

import cv2
import numpy as np
import pytest

from robot_brain.action_schema import parse_model_response
from robot_brain.qwen_client import QwenClient, SYSTEM_INSTRUCTION
from robot_brain.tool_dispatcher import dispatch


def test_plain_answer_has_no_action():
    response = parse_model_response({'answer': '你好', 'action': None})
    assert response.action is None


def test_qwen_prompt_limits_model_intents_to_requested_actions():
    """提示词只让模型选择跟随、前往物体或无动作。"""
    assert '已经完成物体识别的图片' in SYSTEM_INSTRUCTION
    assert 'scene 字段表示你看到的物体' in SYSTEM_INSTRUCTION
    assert 'scene 为空数组 [] 时' in SYSTEM_INSTRUCTION
    assert '此时不得编造物体、位置或机器人动作' in SYSTEM_INSTRUCTION
    assert 'answer 中直接放给用户的回答' in SYSTEM_INSTRUCTION
    assert '"name":"goto_object"' in SYSTEM_INSTRUCTION
    assert '"name":"follow_person"' in SYSTEM_INSTRUCTION
    assert 'action 为 null 时' in SYSTEM_INSTRUCTION
    assert '禁止要求用户确认' in SYSTEM_INSTRUCTION
    assert '"name":"explore"' not in SYSTEM_INSTRUCTION


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


def test_qwen_prompt_contains_history_and_writes_audit_files(tmp_path):
    client = QwenClient(
        'http://127.0.0.1:9100', log_directory=tmp_path)
    captured = {}

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return json.dumps({'answer': json.dumps({
                'answer': '可以生成杯子的路径预演。',
                'action': {'name': 'goto_object',
                           'arguments': {'label': '杯子'}},
            }, ensure_ascii=False)}).encode()

    def fake_urlopen(request, timeout):
        captured['body'] = json.loads(request.data.decode())
        captured['timeout'] = timeout
        return Response()

    original = __import__('robot_brain.qwen_client', fromlist=['urlopen']).urlopen
    module = __import__('robot_brain.qwen_client', fromlist=['urlopen'])
    module.urlopen = fake_urlopen
    source = np.zeros((120, 160, 3), dtype=np.uint8)
    success, encoded = cv2.imencode('.jpg', source)
    assert success
    try:
        result = client.chat(
            'request', '去那里', image=encoded.tobytes(), detections=[{
                'id': 'cup-1', 'label_zh': '杯子', 'class_name': 'cup',
                'confidence': 0.91, 'distance': 1.25,
                'bbox': [-5, 10, 80, 100],
            }], history=[{'role': 'user', 'text': '我说的是杯子'}],
            image_size={'width': 160, 'height': 120},
            image_stamp_ns=123, detection_stamp_ns=123)
    finally:
        module.urlopen = original

    assert result.action.name == 'goto_object'
    prompt = json.loads(captured['body']['text'])
    assert prompt['history_for_reference_only'][0]['text'] == '我说的是杯子'
    assert prompt['scene'][0] == {
        'id': 'cup-1', 'label': '杯子', 'class_name': 'cup',
        'confidence': 0.91, 'distance_m': 1.25, 'position': '左侧中部',
    }
    assert 'system' in captured['body']
    assert 'image_base64' not in captured['body']
    assert captured['timeout'] == 180.0
    client.log_timings('request', {
        '队列等待': 12.3,
        '任务解析与路径预演': 45.6,
        '端到端总时长': 78.9,
    })
    client.log_parsed_result('request', result)
    text_files = list(tmp_path.glob('*.txt'))
    image_files = list(tmp_path.glob('*.jpg'))
    assert len(text_files) == 1
    assert len(image_files) == 1
    annotated = cv2.imread(str(image_files[0]))
    assert annotated is not None
    assert np.any(annotated != 0)
    log_text = text_files[0].read_text(encoding='utf-8')
    assert '用户输入：去那里' in log_text
    assert '给 Qwen 的 system 提示词' in log_text
    assert SYSTEM_INSTRUCTION in log_text
    assert '给 Qwen 的 user 内容' in log_text
    assert json.dumps(prompt, ensure_ascii=False, separators=(',', ':')) \
        in log_text
    assert '给 Qwen 的结构化场景' not in log_text
    assert 'Qwen 原始返回' in log_text
    assert '提示词与请求准备' in log_text
    assert '网关HTTP与Qwen推理' in log_text
    assert '响应解析与位置落地' in log_text
    assert '端到端总时长' in log_text
    assert '最终解析结果' in log_text
    assert 'YOLO 标注审计图，未发送给 Qwen' in log_text
    assert '解析路径' in log_text


@pytest.mark.parametrize(('text', 'expected'), [
    ('开始自主探索', ('explore', {})),
    ('请跟着我', ('follow_person', {})),
    ('前往这个杯子', ('goto_object', {'label': '杯子'})),
])
def test_explicit_commands_have_safe_fallback(text, expected):
    detections = [{'label_zh': '杯子', 'class_name': 'cup'}]
    action = QwenClient._explicit_action(text, detections)
    assert (action.name, action.arguments) == expected


@pytest.mark.parametrize('text', [
    '不要开始自主探索', '停止跟随人员', '取消前往杯子', '你好',
])
def test_non_commands_do_not_get_fallback_action(text):
    assert QwenClient._explicit_action(text, []) is None


def test_goto_fallback_requires_current_visual_evidence():
    assert QwenClient._explicit_action('前往杯子', []) is None
    model = parse_model_response({
        'answer': '将前往杯子。',
        'action': {'name': 'goto_object', 'arguments': {'label': '杯子'}},
    })
    safe = QwenClient._validate_scene_action(model, [])
    assert safe.action is None
    assert '未检测到杯子' in safe.answer


def test_model_action_cannot_turn_position_question_into_navigation():
    model = parse_model_response({
        'answer': '将生成前往杯子的任务预演。',
        'action': {'name': 'goto_object', 'arguments': {'label': '杯子'}},
    })
    detections = [{
        'label_zh': '杯子', 'class_name': 'cup',
        'bbox': [400, 120, 520, 360],
    }]
    safe = QwenClient._gate_action_by_request(
        model, '杯子在哪里？', detections)
    grounded = QwenClient._ground_spatial_answer(
        safe, '杯子在哪里？', detections,
        {'width': 640, 'height': 480})
    assert grounded.action is None
    assert grounded.answer == '杯子位于画面右侧中部。'


def test_action_only_model_output_is_repaired_through_strict_schema():
    repaired = QwenClient._repair_action_only(
        '{"name":"explore","arguments":{}}')
    assert repaired.action.name == 'explore'
    assert QwenClient._repair_action_only(
        '{"name":"cmd_vel","arguments":{}}') is None


def test_invalid_action_keeps_answer_but_never_executes_it():
    response = QwenClient._salvage_plain_answer(json.dumps({
        'answer': '这个人位于图像左侧。',
        'action': 'terminate',
    }, ensure_ascii=False))
    assert response.answer == '这个人位于图像左侧。'
    assert response.action is None

    fenced = QwenClient._salvage_plain_answer(
        '```json\n{"answer":"仍保留","action":"terminate"}\n```')
    assert fenced.answer == '仍保留'
    assert fenced.action is None


@pytest.mark.parametrize(('raw', 'mode', 'action'), [
    ('```json\n{"answer":"你好","action":null}\n```',
     'json_extract', None),
    ('说明：{"answer":"开始预演","action":{"name":"explore",'
     '"arguments":{}}} 完毕', 'json_extract', 'explore'),
    ("{'answer':'你好','action':None}", 'json_extract', None),
    ('{"name":"follow_person","arguments":{}}',
     'action_wrap', 'follow_person'),
])
def test_common_small_model_formats_are_repaired(raw, mode, action):
    result, result_mode = QwenClient._parse_with_repair(raw)
    assert result_mode == mode
    assert (None if result.action is None else result.action.name) == action


@pytest.mark.parametrize('raw', [
    '{"answer":"a","action":null}{"answer":"b","action":null}',
    '{"answer":"越权","action":{"name":"cmd_vel","arguments":{}}}',
    '{"answer":"坐标","action":{"name":"goto_object",'
    '"arguments":{"label":"杯子","x":1}}}',
])
def test_ambiguous_or_unsafe_repairs_are_rejected(raw):
    result, mode = QwenClient._parse_with_repair(raw)
    assert result is None
    assert mode == 'rejected'


def test_zero_detection_image_has_audit_banner():
    source = np.zeros((80, 160, 3), dtype=np.uint8)
    success, encoded = cv2.imencode('.jpg', source)
    assert success
    annotated, errors = QwenClient._annotate_yolo_image(
        encoded.tobytes(), [])
    decoded = cv2.imdecode(
        np.frombuffer(annotated, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded is not None
    assert errors == []
    assert np.any(decoded[:36, :235] != 0)


def test_compact_scene_is_limited_to_twenty_items():
    detections = [{
        'id': f'cup-{index}', 'label_zh': '杯子', 'class_name': 'cup',
        'confidence': 0.9, 'distance': 1.0, 'bbox': [0, 0, 10, 10],
    } for index in range(25)]
    scene = QwenClient._compact_scene(
        detections, {'width': 100, 'height': 100})
    assert len(scene) == 20
    assert scene[-1]['id'] == 'cup-19'


def test_person_position_is_grounded_by_current_detection_box():
    model = parse_model_response({
        'answer': '在画面中。',
        'action': None,
    })
    result = QwenClient._ground_spatial_answer(
        model, '人在哪里？', [{
            'label_zh': '人', 'class_name': 'person',
            'bbox': [20, 100, 220, 420],
        }], {'width': 640, 'height': 480})
    assert result.answer == '人位于画面左侧中部。'
    assert result.action is None
