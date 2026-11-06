"""作用：验证 9100 驾驶持续连接与 Unix Socket 后端的真实转发和断线停车；用法：pytest 本文件。"""

import asyncio
from contextlib import contextmanager
import json
import threading
import time
from types import SimpleNamespace

import uvicorn
from websockets.legacy.client import connect
from websockets.exceptions import InvalidStatusCode
import pytest

from robot_web.console import create_app as console_app
from robot_web.control import ControlState
from test_web_gateway import console_test_config, web_server


@contextmanager
def backend(path, robot):
    server=uvicorn.Server(uvicorn.Config(console_app(robot), uds=str(path), log_level='error'))
    thread=threading.Thread(target=server.run)
    thread.start()
    try:
        for _ in range(200):
            if server.started:
                break
            time.sleep(.01)
        assert server.started
        yield
    finally:
        server.should_exit=True
        thread.join(timeout=5)
        assert not thread.is_alive()


def test_gateway_stream_held_input_release_disconnect_and_origin(tmp_path, monkeypatch):
    config=console_test_config(tmp_path,monkeypatch)
    robot=SimpleNamespace(control=ControlState(),saving_map=False)
    with backend(tmp_path/'console.sock',robot),web_server(config) as url:
        async def exercise():
            target=url.replace('http:','ws:')+'/robot/api/drive-stream'
            async with connect(target,origin=url) as ws:
                for sequence in range(1,9):
                    await ws.send(json.dumps({'linear':.07,'angular':.1,'client_id':'page','sequence':sequence}))
                    reply = json.loads(await ws.recv())
                    assert reply['ok'] and reply['accepted']
                    assert reply['timeout_seconds'] == robot.control.command_timeout
                    assert robot.control.manual_velocity()==(.07,.1)
                    await asyncio.sleep(.1)
                await ws.send(json.dumps({'linear':0,'angular':0,'client_id':'page','sequence':9}))
                await ws.recv()
                assert robot.control.manual_velocity()==(0,0)
                await ws.send(json.dumps({'linear':.07,'angular':0,'client_id':'page','sequence':10}))
                await ws.recv()
            for _ in range(50):
                if robot.control.manual_velocity()==(0,0):
                    break
                await asyncio.sleep(.01)
            assert robot.control.manual_velocity()==(0,0)
            with pytest.raises(InvalidStatusCode):
                async with connect(target,origin='http://unrelated.example'):
                    pass
        asyncio.run(exercise())
