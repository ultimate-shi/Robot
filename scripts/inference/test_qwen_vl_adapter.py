"""使用方法：pytest 运行本文件，验证推理网关串行调度、纯文本请求和旧 VL 输出解析。"""

import asyncio
import base64
import json
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from qwen_vl_adapter import LocalQwenRunner  # noqa: E402
from brain_inference_server import (  # noqa: E402
    ChatRequest, DetectRequest, InferenceGateway)
from fastapi import HTTPException  # noqa: E402


def test_extract_single_line_answer():
    output = 'User: Answer: {"answer":"你好","action":null}\nI rkllm: stats\nUser: '
    assert LocalQwenRunner.extract_answer(output) == (
        '{"answer":"你好","action":null}')


def test_extract_multiline_fenced_answer():
    output = (
        '日志\nUser: Answer: ```json\n'
        '{"answer":"开始预演","action":{"name":"explore","arguments":{}}}\n'
        '```\nI rkllm: stats\nUser: ')
    assert LocalQwenRunner.extract_answer(output) == (
        '{"answer":"开始预演","action":{"name":"explore","arguments":{}}}')


def test_yolo_returns_busy_immediately_while_qwen_holds_npu():
    async def scenario():
        gateway = InferenceGateway()
        gateway.detector = lambda *_: []
        await gateway.lock.acquire()
        try:
            request = DetectRequest(image_base64=base64.b64encode(
                b'jpeg').decode('ascii'))
            try:
                await gateway.detect(request)
            except HTTPException as exc:
                assert exc.status_code == 503
                assert exc.detail['code'] == 'NPU_BUSY_LLM'
                assert exc.detail['retryable'] is True
            else:
                raise AssertionError('Qwen 占用 NPU 时 YOLO 不应进入等待')
        finally:
            gateway.lock.release()

    asyncio.run(scenario())


def test_text_gateway_never_forwards_compatible_image_field():
    gateway = InferenceGateway()
    captured = {}

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return json.dumps({'choices': [{'message': {
                'content': '{"answer":"你好","action":null}',
            }}]}).encode()

    def fake_urlopen(request, timeout):
        captured['body'] = json.loads(request.data.decode())
        captured['timeout'] = timeout
        return Response()

    module = __import__('brain_inference_server', fromlist=['urlopen'])
    original = module.urlopen
    module.urlopen = fake_urlopen
    try:
        request = ChatRequest(
            system='只输出 JSON', text='{"current_request":"你好"}',
            image_base64=base64.b64encode(b'never-send').decode())
        result = gateway._call_openai_compatible(
            'http://127.0.0.1:9101', 'qwen', request)
    finally:
        module.urlopen = original

    assert result == '{"answer":"你好","action":null}'
    assert captured['body']['messages'] == [
        {'role': 'system', 'content': '只输出 JSON'},
        {'role': 'user', 'content': '{"current_request":"你好"}'},
    ]
    assert 'image' not in json.dumps(captured['body'])
    assert captured['body']['max_tokens'] == 96
