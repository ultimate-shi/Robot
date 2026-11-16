"""作用：验证 9100 入口与推理 Socket 解耦；使用方法：pytest -q src/robot_web/test/test_web_gateway.py。"""

from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, HTTPServer
from http.client import HTTPException
import json
from pathlib import Path
import socket
import sqlite3
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import uvicorn
import yaml

from robot_web.server import create_app


class UnixHTTPServer(HTTPServer):
    address_family = socket.AF_UNIX


class InferenceHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.reply(200, {'state': 'ok', 'llm_ready': True})

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        if self.path == '/v1/chat':
            self.reply(502, {'detail': '上游错误', 'text': body['text']})
        else:
            self.reply(200, {'path': self.path, 'image_base64': body['image_base64']})

    def reply(self, status, value):
        body = json.dumps(value).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


@contextmanager
def web_server(config):
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen()
    port = listener.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(create_app(config), log_level='error'))
    thread = threading.Thread(target=server.run, kwargs={'sockets': [listener]})
    thread.start()
    try:
        for _ in range(100):
            if server.started:
                break
            time.sleep(0.01)
        assert server.started
        yield f'http://127.0.0.1:{port}'
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        listener.close()


def get_json(url):
    with urlopen(url, timeout=2) as response:
        return response.status, json.load(response)


def test_log_and_health_survive_inference_stop(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    log_root = tmp_path / 'logs'
    (log_root / 'index').mkdir(parents=True)
    log_config = tmp_path / 'logging.yaml'
    log_config.write_text(yaml.safe_dump({'logging': {'root': str(log_root)}}))
    web_config = tmp_path / 'web.yaml'
    web_config.write_text(yaml.safe_dump({'web': {
        'host': '127.0.0.1', 'port': 9100,
        'inference_socket_name': 'inference.sock',
        'log_config': str(log_config), 'request_timeout_seconds': 2,
        'chat_timeout_seconds': 2, 'health_timeout_seconds': 1}}))

    with web_server(web_config) as base:
        with urlopen(base + '/log/', timeout=2) as response:
            assert response.status == 200
            assert '机器人日志' in response.read().decode()
        status, health = get_json(base + '/health')
        assert status == 200
        assert health['web']['state'] == 'ok'
        assert health['logs']['state'] == 'unavailable'
        assert health['inference']['state'] == 'unavailable'
        request = Request(base + '/v1/detect', data=b'{}',
                          headers={'Content-Type': 'application/json'})
        try:
            urlopen(request, timeout=2)
        except HTTPError as exc:
            assert exc.code == 503
        else:
            raise AssertionError('推理离线时必须返回 503')

        upstream = UnixHTTPServer(str(tmp_path / 'inference.sock'), InferenceHandler)
        upstream_thread = threading.Thread(target=upstream.serve_forever)
        upstream_thread.start()
        try:
            assert get_json(base + '/health')[1]['inference']['llm_ready']
            image = 'a' * 200000
            request = Request(base + '/v1/detect',
                              data=json.dumps({'image_base64': image}).encode(),
                              headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=3) as response:
                assert json.load(response) == {
                    'path': '/v1/detect', 'image_base64': image}
            request = Request(base + '/v1/chat',
                              data=json.dumps({'text': '你好'}).encode(),
                              headers={'Content-Type': 'application/json'})
            try:
                urlopen(request, timeout=2)
            except HTTPError as exc:
                assert exc.code == 502
                assert json.load(exc)['detail'] == '上游错误'
            else:
                raise AssertionError('必须保留推理服务返回的错误状态')
        finally:
            upstream.shutdown()
            upstream.server_close()
            upstream_thread.join(timeout=5)

        assert get_json(base + '/health')[1]['inference']['state'] == 'unavailable'


def test_log_download_stays_on_9100(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    log_root = tmp_path / 'logs'
    (log_root / 'index').mkdir(parents=True)
    bag = log_root / 'sample.mcap'
    bag.write_bytes(b'MCAP TEST')
    database = log_root / 'index' / 'logs.sqlite3'
    with sqlite3.connect(database) as conn:
        conn.execute('CREATE TABLE messages (id INTEGER PRIMARY KEY, bag_file TEXT)')
        conn.execute('INSERT INTO messages VALUES (1, ?)', (str(bag),))
    log_config = tmp_path / 'logging.yaml'
    log_config.write_text(yaml.safe_dump({'logging': {'root': str(log_root)}}))
    web_config = tmp_path / 'web.yaml'
    web_config.write_text(yaml.safe_dump({'web': {
        'host': '127.0.0.1', 'port': 9100,
        'inference_socket_name': 'inference.sock',
        'log_config': str(log_config), 'request_timeout_seconds': 2,
        'chat_timeout_seconds': 2, 'health_timeout_seconds': 1}}))
    with web_server(web_config) as base:
        assert get_json(base + '/health')[1]['logs']['state'] == 'ok'
        with urlopen(base + '/log/api/logs/1/mcap', timeout=2) as response:
            assert response.read() == b'MCAP TEST'


def test_web_gateway_does_not_filter_client_networks(tmp_path, monkeypatch):
    """开放 IPv4 网卡后，来源地址不影响网页和 API 访问。"""
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    log_config = tmp_path / 'logging.yaml'
    log_config.write_text(yaml.safe_dump({'logging': {
        'root': str(tmp_path / 'logs')}}))
    web_config = tmp_path / 'web.yaml'
    web_config.write_text(yaml.safe_dump({'web': {
        'host': '0.0.0.0', 'port': 9100,
        'inference_socket_name': 'inference.sock',
        'log_config': str(log_config), 'request_timeout_seconds': 2,
        'chat_timeout_seconds': 2, 'health_timeout_seconds': 1}}))
    with web_server(web_config) as base:
        assert get_json(base + '/health')[0] == 200
        # Uvicorn 信任本机转发头；在测试中模拟其他网段来源。
        remote = Request(base + '/health', headers={
            'X-Forwarded-For': '10.0.0.12'})
        with urlopen(remote, timeout=2) as response:
            assert response.status == 200


def test_console_isolated_socket_does_not_change_existing_gateway(tmp_path,
                                                                   monkeypatch):
    """控制台离线时原网关仍可用；上线后只转发 /robot/。"""
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    log_config = tmp_path / 'logging.yaml'
    log_config.write_text(yaml.safe_dump({'logging': {
        'root': str(tmp_path / 'logs')}}))
    console_socket = tmp_path / 'console.sock'
    web_config = tmp_path / 'web.yaml'
    web_config.write_text(yaml.safe_dump({'web': {
        'host': '127.0.0.1', 'port': 9100,
        'inference_socket_name': 'inference.sock',
        'console_socket': str(console_socket),
        'log_config': str(log_config), 'request_timeout_seconds': 2,
        'chat_timeout_seconds': 2, 'health_timeout_seconds': 1}}))
    with web_server(web_config) as base:
        assert get_json(base + '/health')[0] == 200
        try:
            get_json(base + '/robot/api/status')
        except HTTPError as exc:
            assert exc.code == 503
        else:
            raise AssertionError('控制台离线时应返回 503')
        upstream = UnixHTTPServer(str(console_socket), InferenceHandler)
        thread = threading.Thread(target=upstream.serve_forever)
        thread.start()
        try:
            status, payload = get_json(base + '/robot/api/status')
            assert status == 200
            assert payload['state'] == 'ok'
            assert get_json(base + '/health')[0] == 200
        finally:
            upstream.shutdown()
            upstream.server_close()
            thread.join(timeout=5)


def console_test_config(tmp_path, monkeypatch, **overrides):
    """生成可缩短等待时间的隔离网关配置。"""
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    log_config = tmp_path / 'logging.yaml'
    log_config.write_text(yaml.safe_dump({'logging': {'root': str(tmp_path / 'logs')}}))
    values = {
        'host': '127.0.0.1', 'port': 9100,
        'inference_socket_name': 'inference.sock',
        'console_socket': str(tmp_path / 'console.sock'),
        'log_config': str(log_config), 'request_timeout_seconds': 2,
        'chat_timeout_seconds': 2, 'health_timeout_seconds': 1,
        **overrides}
    config = tmp_path / 'web.yaml'
    config.write_text(yaml.safe_dump({'web': values}))
    return config


@pytest.mark.parametrize('path,error,status,message', [
    ('api/save-map', TimeoutError, 504, '后台可能仍在保存'),
    ('api/status', TimeoutError, 504, '响应超时'),
    ('api/status', FileNotFoundError, 503, '未启动'),
    ('api/status', ConnectionRefusedError, 503, '未启动'),
    ('api/save-map', HTTPException, 502, '连接中断'),
])
def test_console_transport_errors_are_not_all_reported_as_offline(
        tmp_path, monkeypatch, path, error, status, message):
    config = console_test_config(tmp_path, monkeypatch)
    budgets = []

    def upstream(socket_path, method, target, body, timeout, content_type):
        budgets.append(timeout)
        raise error('测试连接异常')

    monkeypatch.setattr('robot_web.server.call_inference', upstream)
    with web_server(config) as base:
        request = Request(base + '/robot/' + path, data=b'{}')
        with pytest.raises(HTTPError) as caught:
            urlopen(request, timeout=2)
        assert caught.value.code == status
        detail = json.load(caught.value)['detail']
        assert message in detail
        if error is TimeoutError:
            assert '未启动' not in detail
    assert budgets == [120 if path == 'api/save-map' else 20]


def test_slow_save_uses_its_own_longer_timeout(tmp_path, monkeypatch):
    config = console_test_config(
        tmp_path, monkeypatch, console_request_timeout_seconds=0.01,
        console_save_timeout_seconds=1)

    class SlowSaveHandler(InferenceHandler):
        def do_POST(self):
            self.rfile.read(int(self.headers['Content-Length']))
            time.sleep(0.12)
            self.reply(200, {'success': True, 'message': '/maps/saved/map.yaml'})

    upstream = UnixHTTPServer(str(tmp_path / 'console.sock'), SlowSaveHandler)
    thread = threading.Thread(target=upstream.serve_forever)
    thread.start()
    try:
        with web_server(config) as base:
            request = Request(base + '/robot/api/save-map', data=b'{}')
            with urlopen(request, timeout=2) as response:
                assert response.status == 200
                assert json.load(response)['success']
    finally:
        upstream.shutdown()
        upstream.server_close()
        thread.join(timeout=5)
