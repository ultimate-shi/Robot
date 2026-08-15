#!/usr/bin/env python3
"""用法：由 scripts/inference/start_yolo_gateway.sh 启动宿主机推理网关."""

import asyncio
import base64
import importlib
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn


class DetectRequest(BaseModel):
    """检测请求只传一张已经压缩的左目图."""

    image_base64: str
    min_confidence: float = 0.35


class ChatRequest(BaseModel):
    """视觉问答请求；图像可省略以执行纯文字问答."""

    request_id: str = ''
    text: str
    image_base64: str | None = None
    detections: list = []


class InferenceGateway:
    """用同一把异步锁避免多个模型同时抢占 RK3588 NPU 和内存."""

    def __init__(self):
        self.lock = asyncio.Lock()
        self.detector_name = os.environ.get('ROBOT_DETECTOR_PLUGIN', '')
        self.detector = self._load_detector(self.detector_name)
        self.vlm_endpoint = os.environ.get('ROBOT_VLM_ENDPOINT', '').rstrip('/')
        self.vlm_fallback_endpoint = os.environ.get(
            'ROBOT_VLM_FALLBACK_ENDPOINT', '').rstrip('/')
        self.vlm_model = os.environ.get(
            'ROBOT_VLM_MODEL', 'qwen2.5-vl-3b-w8a8')
        self.vlm_fallback_model = os.environ.get(
            'ROBOT_VLM_FALLBACK_MODEL', 'internvl3-1b-w8a8')

    @staticmethod
    def _load_detector(specification):
        if not specification:
            return None
        try:
            module_name, function_name = specification.split(':', 1)
            function = getattr(importlib.import_module(module_name), function_name)
        except (ImportError, AttributeError, ValueError) as exc:
            raise RuntimeError(f'无法加载检测插件 {specification}: {exc}') from exc
        return function

    async def detect(self, request):
        if self.detector is None:
            raise HTTPException(
                503, '未配置 ROBOT_DETECTOR_PLUGIN，不能伪造物体识别结果')
        try:
            image = base64.b64decode(request.image_base64, validate=True)
        except ValueError as exc:
            raise HTTPException(400, 'image_base64 无效') from exc
        async with self.lock:
            try:
                detections = await asyncio.to_thread(
                    self.detector, image, request.min_confidence)
            except Exception as exc:
                raise HTTPException(500, f'RKNN 检测失败: {exc}') from exc
        return {
            'model': self.detector_name,
            'detections': detections,
        }

    async def chat(self, request):
        if not self.vlm_endpoint and not self.vlm_fallback_endpoint:
            raise HTTPException(
                503, '未配置 ROBOT_VLM_ENDPOINT，不能伪造大模型回答')
        async with self.lock:
            try:
                answer = await asyncio.to_thread(
                    self._call_openai_compatible,
                    self.vlm_endpoint, self.vlm_model, request)
                model = self.vlm_model
            except Exception as primary_error:
                if not self.vlm_fallback_endpoint:
                    raise HTTPException(
                        502, f'主视觉模型调用失败: {primary_error}') from primary_error
                try:
                    answer = await asyncio.to_thread(
                        self._call_openai_compatible,
                        self.vlm_fallback_endpoint,
                        self.vlm_fallback_model, request)
                    model = self.vlm_fallback_model
                except Exception as fallback_error:
                    raise HTTPException(
                        502, f'主模型和回退模型均失败: {fallback_error}') from fallback_error
        return {'answer': answer, 'model': model}

    @staticmethod
    def _call_openai_compatible(endpoint, model, request):
        content = [{'type': 'text', 'text': request.text}]
        if request.image_base64:
            content.insert(0, {
                'type': 'image_url',
                'image_url': {
                    'url': 'data:image/jpeg;base64,' + request.image_base64,
                },
            })
        system = (
            '你是机器人本地视觉助手。只描述当前输入中有证据的内容；'
            '看不清或画面外的信息必须明确说无法判断。你不能直接控制机器人。')
        body = json.dumps({
            'model': model,
            'messages': [
                {'role': 'system', 'content': system},
                {'role': 'user', 'content': content},
            ],
            'max_tokens': 256,
            'temperature': 0.2,
        }).encode('utf-8')
        target = endpoint
        if not target.endswith('/chat/completions'):
            target += '/v1/chat/completions'
        http_request = Request(
            target, data=body,
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urlopen(http_request, timeout=30.0) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, TimeoutError) as exc:
            raise RuntimeError(str(exc)) from exc
        try:
            content = payload['choices'][0]['message']['content']
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError('RKLLM 返回格式不兼容 OpenAI Chat Completions') from exc
        if isinstance(content, list):
            content = ''.join(str(item.get('text', '')) for item in content)
        return str(content).strip()


gateway = InferenceGateway()
app = FastAPI(title='Robot RK3588 Inference Gateway', docs_url=None)


@app.get('/health')
async def health():
    """明确报告模型是否配置，不把进程存活误报为模型可用."""
    return {
        'state': 'ok' if gateway.detector and gateway.vlm_endpoint else 'degraded',
        'detector_configured': gateway.detector is not None,
        'vlm_configured': bool(gateway.vlm_endpoint),
        'vlm_fallback_configured': bool(gateway.vlm_fallback_endpoint),
        'serialization': 'single_queue',
    }


@app.post('/v1/detect')
async def detect(request: DetectRequest):
    return await gateway.detect(request)


@app.post('/v1/chat')
async def chat(request: ChatRequest):
    return await gateway.chat(request)


if __name__ == '__main__':
    uvicorn.run(
        app,
        host=os.environ.get('ROBOT_INFERENCE_HOST', '127.0.0.1'),
        port=int(os.environ.get('ROBOT_INFERENCE_PORT', '9100')),
    )
