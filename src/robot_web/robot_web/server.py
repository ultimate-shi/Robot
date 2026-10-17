"""作用：提供唯一的 9100 网页入口；使用方法：运行 robot_web console_script。"""

import asyncio
from http.client import HTTPConnection, HTTPException
import json
from pathlib import Path
import socket

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, Response
import uvicorn

from robot_logging.web import create_app as create_log_app
from robot_logging.storage import load_config as load_log_config
from robot_web.config import load_config


class UnixHTTPConnection(HTTPConnection):
    """通过仅本机可访问的 Unix Socket 请求推理服务。"""

    def __init__(self, socket_path, timeout):
        super().__init__('localhost', timeout=timeout)
        self.socket_path = socket_path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(str(self.socket_path))


def call_inference(socket_path, method, path, body=b'', timeout=3,
                   content_type='application/json'):
    """保留上游 HTTP 状态与 JSON 正文，并确保每次请求关闭连接。"""
    connection = UnixHTTPConnection(socket_path, timeout)
    try:
        connection.request(method, path, body=body,
                           headers={'Content-Type': content_type})
        response = connection.getresponse()
        return response.status, response.read(), response.getheader(
            'Content-Type', 'application/json')
    finally:
        connection.close()


def create_app(config_file=None):
    config = load_config(config_file)
    app = FastAPI(title='机器人网页入口', docs_url=None, redoc_url=None)

    app.mount('/log', create_log_app(config['log_config']))
    log_root = load_log_config(config['log_config'])['root']

    async def forward(path, request, timeout):
        try:
            status, body, content_type = await asyncio.to_thread(
                call_inference, config['inference_socket'], request.method,
                path, await request.body(), timeout,
                request.headers.get('content-type', 'application/json'))
        except (OSError, TimeoutError, HTTPException) as exc:
            return JSONResponse(status_code=503, content={
                'detail': '推理服务不可用', 'reason': type(exc).__name__})
        return Response(content=body, status_code=status,
                        headers={'content-type': content_type})

    @app.get('/health')
    async def health():
        try:
            status, body, _ = await asyncio.to_thread(
                call_inference, config['inference_socket'], 'GET', '/health',
                b'', config['health_timeout_seconds'])
            inference = json.loads(body) if status == 200 else {
                'state': 'unavailable', 'status_code': status}
        except (OSError, TimeoutError, HTTPException, ValueError):
            inference = {'state': 'unavailable'}
        index = Path(log_root) / 'index' / 'logs.sqlite3'
        return {'state': 'ok', 'web': {'state': 'ok'},
                'logs': {'state': 'ok' if index.is_file() else 'unavailable'},
                'inference': inference}

    @app.post('/v1/detect')
    async def detect(request: Request):
        return await forward('/v1/detect', request,
                             config['request_timeout_seconds'])

    @app.post('/v1/segment')
    async def segment(request: Request):
        return await forward('/v1/segment', request,
                             config['request_timeout_seconds'])

    @app.post('/v1/chat')
    async def chat(request: Request):
        return await forward('/v1/chat', request,
                             config['chat_timeout_seconds'])

    return app


def main():
    config = load_config()
    uvicorn.run(create_app(), host=config['host'], port=config['port'])


if __name__ == '__main__':
    main()
