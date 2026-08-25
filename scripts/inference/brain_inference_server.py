#!/usr/bin/env python3
"""使用方法：由 start_yolo_gateway.sh 启动。

本程序串行调度 YOLO 与常驻纯文本 Qwen。
"""

import asyncio
import base64
import importlib
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
import uvicorn


class DetectRequest(BaseModel):
    """检测请求只传一张已经压缩的右目图。"""

    image_base64: str
    min_confidence: float = 0.35


class ChatRequest(BaseModel):
    """文本决策请求；image_base64 仅为旧客户端兼容字段，不会发送给 LLM。"""

    request_id: str = ''
    system: str = ''
    text: str
    image_base64: str | None = None
    detections: list = Field(default_factory=list)


class InferenceGateway:
    """用同一把异步锁避免 YOLO 与 Qwen 同时使用 RK3588 NPU。"""

    def __init__(self):
        self.lock = asyncio.Lock()
        self.detector_name = os.environ.get('ROBOT_DETECTOR_PLUGIN', '')
        self.detector = self._load_detector(self.detector_name)
        self.llm_endpoint = os.environ.get(
            'ROBOT_LLM_ENDPOINT', os.environ.get(
                'ROBOT_VLM_ENDPOINT', '')).rstrip('/')
        self.llm_fallback_endpoint = os.environ.get(
            'ROBOT_LLM_FALLBACK_ENDPOINT', os.environ.get(
                'ROBOT_VLM_FALLBACK_ENDPOINT', '')).rstrip('/')
        self.llm_model = os.environ.get(
            'ROBOT_LLM_MODEL', os.environ.get(
                'ROBOT_VLM_MODEL', 'qwen2.5-3b-instruct-w8a8-rk3588'))
        self.llm_fallback_model = os.environ.get(
            'ROBOT_LLM_FALLBACK_MODEL', os.environ.get(
                'ROBOT_VLM_FALLBACK_MODEL', 'qwen2.5-3b-instruct'))
        self.max_tokens = int(os.environ.get('ROBOT_LLM_MAX_TOKENS', '96'))
        self.timeout = float(os.environ.get('ROBOT_LLM_TIMEOUT', '180'))
        self.last_llm_metrics = {}

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
            raise HTTPException(503, '未配置检测插件，不能伪造识别结果')
        if self.lock.locked():
            raise HTTPException(503, {
                'code': 'NPU_BUSY_LLM',
                'message': 'NPU 正在执行 Qwen 问答，实时检测已暂停',
                'retryable': True,
            })
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
        return {'model': self.detector_name, 'detections': detections}

    async def chat(self, request):
        if not self.llm_endpoint and not self.llm_fallback_endpoint:
            raise HTTPException(503, '未配置 ROBOT_LLM_ENDPOINT')
        async with self.lock:
            failures = []
            for endpoint, model, label in (
                    (self.llm_endpoint, self.llm_model, '主模型'),
                    (self.llm_fallback_endpoint, self.llm_fallback_model,
                     '回退模型')):
                if not endpoint:
                    continue
                try:
                    answer = await asyncio.to_thread(
                        self._call_openai_compatible, endpoint, model, request)
                    return {
                        'answer': answer, 'model': model,
                        'metrics': self.last_llm_metrics,
                    }
                except Exception as exc:
                    failures.append(f'{label}: {exc}')
            raise HTTPException(502, '；'.join(failures))

    def _call_openai_compatible(self, endpoint, model, request):
        """只发送 system 和纯文本 user 内容，兼容字段中的图片始终忽略。"""
        self.last_llm_metrics = {}
        body = json.dumps({
            'model': model,
            'messages': [
                {'role': 'system', 'content': request.system},
                {'role': 'user', 'content': request.text},
            ],
            'stream': False,
            'max_tokens': self.max_tokens,
            'temperature': 0.1,
            'top_p': 0.1,
            'top_k': 1,
            'enable_thinking': False,
        }).encode('utf-8')
        target = endpoint
        if not target.endswith('/chat/completions'):
            target += '/v1/chat/completions'
        http_request = Request(
            target, data=body, headers={'Content-Type': 'application/json'},
            method='POST')
        try:
            with urlopen(http_request, timeout=self.timeout) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, TimeoutError) as exc:
            raise RuntimeError(str(exc)) from exc
        try:
            content = payload['choices'][0]['message']['content']
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError('LLM 返回格式不兼容 Chat Completions') from exc
        if isinstance(content, list):
            content = ''.join(str(item.get('text', '')) for item in content)
        metrics = payload.get('robot_metrics', {})
        self.last_llm_metrics = metrics if isinstance(metrics, dict) else {}
        return str(content).strip()

    @staticmethod
    def endpoint_health(endpoint):
        if not endpoint:
            return {'state': 'unconfigured'}
        base = endpoint.split('/v1/', 1)[0].rstrip('/')
        failures = []
        for path in ('/health', '/v1/models'):
            try:
                with urlopen(Request(base + path, method='GET'),
                             timeout=2.0) as response:
                    payload = json.loads(response.read().decode('utf-8'))
                    if path == '/health':
                        return payload
                    return {'state': 'ok', 'source': 'v1/models'}
            except Exception as exc:
                failures.append(f'{path}: {exc}')
        return {'state': 'error', 'message': '；'.join(failures)}


gateway = InferenceGateway()
app = FastAPI(title='Robot RK3588 Inference Gateway', docs_url=None)


@app.get('/health')
async def health():
    llm_health = await asyncio.to_thread(
        gateway.endpoint_health, gateway.llm_endpoint)
    fallback_health = await asyncio.to_thread(
        gateway.endpoint_health, gateway.llm_fallback_endpoint)
    llm_ok = (llm_health.get('state') == 'ok'
              or fallback_health.get('state') == 'ok')
    return {
        'state': 'ok' if gateway.detector and llm_ok else 'degraded',
        'detector_configured': gateway.detector is not None,
        'llm_configured': bool(gateway.llm_endpoint),
        'llm_ready': llm_ok,
        'llm_process_resident': bool(llm_health.get('resident')),
        'llm_runtime': llm_health,
        'llm_fallback_runtime': fallback_health,
        'llm_fallback_configured': bool(gateway.llm_fallback_endpoint),
        'local_llm_model': gateway.llm_model,
        # 兼容现有网页健康检查字段，后续版本再移除。
        'vlm_configured': bool(gateway.llm_endpoint),
        'local_qwen_model': gateway.llm_model,
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
        app, host=os.environ.get('ROBOT_INFERENCE_HOST', '127.0.0.1'),
        port=int(os.environ.get('ROBOT_INFERENCE_PORT', '9100')),
    )
