"""使用方法：Web聊天工作线程调用 QwenClient.chat，经 localhost 网关获取严格 JSON 回复。"""

import base64
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from robot_brain.action_schema import ModelResponse, parse_model_response


SYSTEM_INSTRUCTION = """你是机器人意图理解器。只输出一个JSON对象，且只能包含answer和action。
action为null，或为以下之一：
{"name":"goto_object","arguments":{"label":"物体名称"}}
{"name":"follow_person","arguments":{}}
{"name":"explore","arguments":{}}
不得输出坐标、cmd_vel、ROS话题、控制权声明或已执行成功的表述。"""


class QwenClient:
    """访问宿主机推理网关，不包含任何 ROS 能力。"""

    def __init__(self, base_url, timeout=30.0):
        self.base_url = str(base_url).rstrip('/')
        self.timeout = float(timeout)

    def chat(self, request_id, text, image=None, detections=None):
        body = {
            'request_id': request_id,
            'text': SYSTEM_INSTRUCTION + '\n用户输入：' + str(text),
            'detections': detections or [],
        }
        if image:
            body['image_base64'] = base64.b64encode(image).decode('ascii')
        request = Request(
            self.base_url + '/v1/chat', data=json.dumps(body).encode('utf-8'),
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urlopen(request, timeout=self.timeout) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, TimeoutError) as exc:
            raise RuntimeError(f'Qwen服务不可用: {exc}') from exc
        raw = payload.get('answer', payload.get('content', ''))
        try:
            return parse_model_response(raw)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            text_answer = str(raw).strip() or '模型返回格式无效，未执行任何机器人动作。'
            return ModelResponse(answer=f'{text_answer}\n[动作未执行：{exc}]')
