#!/usr/bin/env python3
"""局域网文字交互、多用户租约和 ROS 状态汇聚 HTTP 服务."""

import asyncio
import base64
from copy import deepcopy
from dataclasses import dataclass, field
import json
import os
import re
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request as UrlRequest, urlopen
import uuid

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from robot.mission.multi_user import MultiUserMissionState
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class SharedRobotState:
    """在线程间共享最新状态；图像和地图只缓存一份."""

    def __init__(self):
        self.lock = threading.RLock()
        self.detections = {'detections': []}
        self.health = {
            'web': {'state': 'ok'},
            'camera': {'state': 'waiting'},
            'semantic': {'state': 'waiting'},
            'slam': {'state': 'waiting'},
            'nav2': {'state': 'waiting'},
        }
        self.mission_ros = {'state': 'stopped', 'message': '等待任务'}
        self.map = None
        self.frame = None
        self.frame_stamp = 0.0

    def public_snapshot(self, lease, client_id=''):
        with self.lock:
            public_lease = deepcopy(lease)
            controller_id = public_lease.pop('controller_id', '')
            public_lease['is_controller'] = bool(
                client_id and controller_id == client_id)
            return {
                'type': 'state',
                'lease': public_lease,
                'mission': deepcopy(self.mission_ros),
                'health': deepcopy(self.health),
                'detections': deepcopy(self.detections),
                'server_time': time.time(),
            }


class BrainWebBridge(Node):
    """只负责 ROS 消息与网页共享状态互通，不在回调中执行推理."""

    def __init__(self, shared):
        super().__init__('brain_web')
        defaults = {
            'http_host': '0.0.0.0',
            'http_port': 8080,
            'inference_url': 'http://127.0.0.1:9100',
            'chat_queue_size': 8,
            'lease_grace_seconds': 10.0,
            'motion_enabled': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.shared = shared
        self.notify = None
        self.mission_pub = self.create_publisher(
            String, '/brain/mission_request', 20)
        self.sample_pub = self.create_publisher(
            String, '/brain/sample_request', 10)
        self.create_subscription(
            String, '/perception/semantic_detections',
            self._detections_callback, 10)
        self.create_subscription(
            String, '/perception/semantic_status',
            self._semantic_status_callback, 10)
        self.create_subscription(
            String, '/mission/status', self._mission_callback, 20)
        self.create_subscription(
            String, '/brain/sample_status', self._sample_status_callback, 10)
        self.create_subscription(
            CompressedImage, '/stereo/left/image_rect/compressed',
            self._frame_callback, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

    def publish_mission(self, payload):
        self.mission_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False)))

    def request_sample(self, payload):
        self.sample_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False)))

    def _detections_callback(self, msg):
        payload = self._json(msg.data)
        if payload is None:
            return
        with self.shared.lock:
            self.shared.detections = payload
        self._changed()

    def _semantic_status_callback(self, msg):
        payload = self._json(msg.data)
        if payload is None:
            return
        with self.shared.lock:
            self.shared.health['semantic'] = payload
        self._changed()

    def _mission_callback(self, msg):
        payload = self._json(msg.data)
        if payload is None:
            return
        with self.shared.lock:
            self.shared.mission_ros = payload
            state = payload.get('state', '')
            self.shared.health['nav2'] = {
                'state': ('ok' if state not in (
                    'waiting_map', 'waiting_tf', 'planning_failed')
                          else state),
                'message': payload.get('message', ''),
            }
        self._changed()

    def _sample_status_callback(self, msg):
        payload = self._json(msg.data)
        if payload is None:
            return
        with self.shared.lock:
            self.shared.health['dataset'] = payload
        self._changed()

    def _frame_callback(self, msg):
        with self.shared.lock:
            self.shared.frame = bytes(msg.data)
            self.shared.frame_stamp = time.time()
            self.shared.health['camera'] = {'state': 'ok'}
        self._changed()

    def _map_callback(self, msg):
        # 网页用 Canvas 绘制，地图只保留最新一份，不向每个用户重复转换图像。
        stride = max(1, int(max(msg.info.width, msg.info.height) / 320))
        values = list(msg.data)
        sampled = []
        for row in range(0, msg.info.height, stride):
            start = row * msg.info.width
            sampled.extend(values[start:start + msg.info.width:stride])
        with self.shared.lock:
            self.shared.map = {
                'width': (msg.info.width + stride - 1) // stride,
                'height': (msg.info.height + stride - 1) // stride,
                'resolution': msg.info.resolution * stride,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                },
                'data': sampled,
            }
            self.shared.health['slam'] = {
                'state': 'ok',
                'size': [msg.info.width, msg.info.height],
            }
        self._changed()

    @staticmethod
    def _json(text):
        try:
            return json.loads(text)
        except (TypeError, ValueError, json.JSONDecodeError):
            return None

    def _changed(self):
        callback = self.notify
        if callback is not None:
            callback()


@dataclass
class BrowserConnection:
    """每个浏览器独立发送队列，慢客户端不会反压其他连接."""

    websocket: object
    queue: asyncio.Queue = field(
        default_factory=lambda: asyncio.Queue(maxsize=20))
    sender: object = None


class SocketHub:
    """共享状态广播与私有聊天路由."""

    def __init__(self, shared, lease):
        self.shared = shared
        self.lease = lease
        self.connections = {}
        self.lock = asyncio.Lock()

    async def add(self, client_id, websocket):
        connection = BrowserConnection(websocket)
        connection.sender = asyncio.create_task(self._sender(connection))
        async with self.lock:
            old = self.connections.pop(client_id, None)
            self.connections[client_id] = connection
        if old is not None:
            old.sender.cancel()
        await self.send(client_id, self.shared.public_snapshot(
            self.lease.connect(client_id), client_id))

        if old is not None:
            await old.websocket.close(code=1000)

    async def remove(self, client_id, websocket):
        async with self.lock:
            current = self.connections.get(client_id)
            connection = (self.connections.pop(client_id, None)
                          if current and current.websocket is websocket
                          else None)
        if connection is None:
            return
        self.lease.disconnect(client_id)
        connection.sender.cancel()

    async def broadcast(self):
        async with self.lock:
            client_ids = list(self.connections)
        for client_id in client_ids:
            payload = self.shared.public_snapshot(
                self.lease.snapshot(), client_id)
            await self.send(client_id, payload, replace_shared=True)

    async def send(self, client_id, payload, replace_shared=False):
        async with self.lock:
            connection = self.connections.get(client_id)
        if connection is None:
            return
        if replace_shared and connection.queue.full():
            try:
                connection.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            connection.queue.put_nowait(payload)
        except asyncio.QueueFull:
            # 私有消息队列异常积压时关闭慢连接，不阻塞机器人主链。
            await connection.websocket.close(code=1013)

    @staticmethod
    async def _sender(connection):
        while True:
            payload = await connection.queue.get()
            await connection.websocket.send_json(payload)


def infer_mission(text, detections):
    """确定性解析高风险运动命令，大模型不能直接取得控制权."""
    normalized = str(text).strip()
    if any(word in normalized for word in ('立即停止', '紧急停止')):
        return {'task': 'stop', 'candidates': []}
    if any(word in normalized for word in ('开始探索', '自主探索', '自主建图')):
        return {'task': 'explore', 'candidates': []}
    if '跟随' in normalized:
        candidates = [item for item in detections
                      if str(item.get('class_name', '')).lower() == 'person']
        return {'task': 'follow_person', 'candidates': candidates}
    match = re.search(r'(?:前往|去|到)(?:那个|这个|一下)?\s*([^，。！？ ]+)', normalized)
    if match:
        label = match.group(1)
        candidates = [item for item in detections if
                      label in str(item.get('label_zh', '')) or
                      label.lower() in str(item.get('class_name', '')).lower()]
        return {'task': 'goto_object', 'label': label,
                'candidates': candidates}
    return None


def create_app(bridge, shared):
    """创建 FastAPI 应用；导入放在函数内以便纯算法测试不依赖网页组件."""
    from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse, Response

    grace = float(bridge.get_parameter('lease_grace_seconds').value)
    queue_size = int(bridge.get_parameter('chat_queue_size').value)
    inference_url = str(bridge.get_parameter('inference_url').value).rstrip('/')
    lease = MultiUserMissionState(grace_seconds=grace)
    hub = SocketHub(shared, lease)
    app = FastAPI(title='机器人本地大脑', docs_url=None, redoc_url=None)
    app.state.chat_queue = asyncio.Queue(maxsize=queue_size)
    app.state.missions = {}
    app.state.loop = None
    web_dir = os.path.join(
        get_package_share_directory('robot'), 'web')

    async def publish_state():
        await hub.broadcast()

    def notify_from_ros():
        loop = app.state.loop
        if loop is not None and not loop.is_closed():
            loop.call_soon_threadsafe(
                lambda: asyncio.create_task(publish_state()))

    async def chat_worker():
        while True:
            job = await app.state.chat_queue.get()
            await hub.send(job['client_id'], {
                'type': 'chat_status', 'request_id': job['request_id'],
                'state': 'running', 'position': 0})
            try:
                answer = await asyncio.to_thread(
                    _call_chat_service, inference_url, job, shared)
                payload = {
                    'type': 'chat_result',
                    'request_id': job['request_id'],
                    'answer': answer, 'state': 'completed'}
            except Exception as exc:
                payload = {
                    'type': 'chat_result',
                    'request_id': job['request_id'],
                    'answer': f'端侧模型暂不可用：{exc}', 'state': 'error'}
            await hub.send(job['client_id'], payload)
            app.state.chat_queue.task_done()

    async def lease_watchdog():
        while True:
            await asyncio.sleep(0.5)
            expired = lease.tick()
            if expired is not None:
                bridge.publish_mission({'type': 'stop', 'reason': 'lease_expired'})
                await hub.broadcast()

    @app.on_event('startup')
    async def startup():
        app.state.loop = asyncio.get_running_loop()
        bridge.notify = notify_from_ros
        app.state.workers = [
            asyncio.create_task(chat_worker()),
            asyncio.create_task(lease_watchdog()),
        ]

    @app.on_event('shutdown')
    async def shutdown():
        bridge.notify = None
        for worker in app.state.workers:
            worker.cancel()

    @app.get('/')
    async def index():
        return FileResponse(os.path.join(web_dir, 'index.html'))

    @app.get('/app.js')
    async def javascript():
        return FileResponse(os.path.join(web_dir, 'app.js'),
                            media_type='application/javascript')

    @app.get('/style.css')
    async def stylesheet():
        return FileResponse(os.path.join(web_dir, 'style.css'),
                            media_type='text/css')

    @app.get('/api/frame.jpg')
    async def frame():
        with shared.lock:
            data = shared.frame
        if data is None:
            return Response(status_code=204)
        return Response(content=data, media_type='image/jpeg', headers={
            'Cache-Control': 'no-store'})

    @app.get('/api/map')
    async def map_data():
        with shared.lock:
            payload = deepcopy(shared.map)
        if payload is None:
            return JSONResponse({'state': 'waiting'}, status_code=503)
        return payload

    @app.get('/api/detections')
    async def detections():
        with shared.lock:
            return deepcopy(shared.detections)

    @app.get('/api/health')
    async def health():
        return shared.public_snapshot(lease.snapshot())

    @app.post('/api/dataset/capture')
    async def capture_sample(request: Request):
        body = await request.json()
        bridge.request_sample({
            'client_id': str(body.get('client_id', '')),
            'request_id': str(body.get('request_id', '')),
            'scene_id': str(body.get('scene_id', 'scene')),
        })
        return {
            'success': True,
            'message': '已请求保存当前图像、深度、TF和自动检测结果',
        }

    @app.post('/api/chat')
    async def chat(request: Request):
        body = await request.json()
        client_id = str(body.get('client_id', ''))
        request_id = str(body.get('request_id', '') or uuid.uuid4())
        text = str(body.get('text', '')).strip()
        if not client_id or not text:
            return JSONResponse(
                {'success': False, 'message': 'client_id 和 text 不能为空'},
                status_code=400)
        with shared.lock:
            current_detections = deepcopy(
                shared.detections.get('detections', []))
        mission = infer_mission(text, current_detections)
        if mission is not None:
            if mission['task'] == 'stop':
                result = lease.stop(client_id, request_id)
                bridge.publish_mission({'type': 'stop', 'client_id': client_id})
                await hub.broadcast()
                return result
            preview = _create_preview(
                app, client_id, mission['task'], mission.get('candidates', []),
                mission.get('label', ''))
            if (preview['task'] == 'explore'
                    or preview.get('selected_target_id')):
                bridge.publish_mission({
                    'type': 'preview', 'mission_id': preview['mission_id'],
                    'task': preview['task'],
                    'target_id': preview.get('selected_target_id', ''),
                })
            return {'success': True, 'kind': 'mission_preview', **preview}
        job = {'client_id': client_id, 'request_id': request_id, 'text': text}
        try:
            app.state.chat_queue.put_nowait(job)
        except asyncio.QueueFull:
            return JSONResponse({
                'success': False, 'state': 'busy',
                'message': '端侧模型队列已满，请稍后重试'}, status_code=503)
        position = app.state.chat_queue.qsize()
        await hub.send(client_id, {
            'type': 'chat_status', 'request_id': request_id,
            'state': 'queued', 'position': position})
        return {'success': True, 'state': 'queued',
                'request_id': request_id, 'position': position}

    @app.post('/api/missions/preview')
    async def mission_preview(request: Request):
        body = await request.json()
        client_id = str(body.get('client_id', ''))
        task = str(body.get('task', ''))
        if task not in ('explore', 'follow_person', 'goto_object'):
            return JSONResponse(
                {'success': False, 'message': '任务类型无效'}, status_code=400)
        target_id = str(body.get('target_id', ''))
        with shared.lock:
            all_detections = deepcopy(
                shared.detections.get('detections', []))
        candidates = [item for item in all_detections
                      if not target_id or str(item.get('id')) == target_id]
        if task == 'follow_person':
            candidates = [item for item in candidates if
                          str(item.get('class_name', '')).lower() == 'person']
        preview = _create_preview(app, client_id, task, candidates, '')
        if target_id:
            preview['selected_target_id'] = target_id
        if task == 'explore' or preview.get('selected_target_id'):
            bridge.publish_mission({
                'type': 'preview', 'mission_id': preview['mission_id'],
                'task': task,
                'target_id': preview.get('selected_target_id', '')})
        return {'success': True, **preview}

    @app.post('/api/missions/{mission_id}/confirm')
    async def mission_confirm(mission_id: str, request: Request):
        body = await request.json()
        client_id = str(body.get('client_id', ''))
        request_id = str(body.get('request_id', ''))
        mission = app.state.missions.get(mission_id)
        if mission is None or mission.get('client_id') != client_id:
            return JSONResponse(
                {'success': False, 'message': '任务预览不存在或不属于当前页面'},
                status_code=404)
        target_id = str(body.get(
            'target_id', mission.get('selected_target_id', '')))
        candidate_ids = {
            str(item.get('id', '')) for item in mission.get('candidates', [])}
        if mission['task'] != 'explore' and target_id not in candidate_ids:
            return JSONResponse({
                'success': False,
                'message': '必须从当前页面的候选目标中选择一个有效目标',
            }, status_code=400)
        result = lease.confirm(
            client_id, request_id, mission_id, mission['task'])
        if not result['success']:
            return JSONResponse(result, status_code=409)
        bridge.publish_mission({
            'type': 'confirm', 'mission_id': mission_id,
            'task': mission['task'], 'target_id': target_id,
            'client_id': client_id, 'request_id': request_id})
        await hub.broadcast()
        return result

    @app.post('/api/missions/cancel')
    async def mission_cancel(request: Request):
        body = await request.json()
        result = lease.cancel(
            str(body.get('client_id', '')),
            str(body.get('request_id', '')))
        if result['success']:
            bridge.publish_mission({'type': 'cancel'})
            await hub.broadcast()
            return result
        return JSONResponse(result, status_code=409)

    @app.post('/api/control/release')
    async def release(request: Request):
        body = await request.json()
        result = lease.release(
            str(body.get('client_id', '')),
            str(body.get('request_id', '')))
        if result['success']:
            bridge.publish_mission({'type': 'release'})
            await hub.broadcast()
            return result
        return JSONResponse(result, status_code=409)

    @app.post('/api/stop')
    async def stop(request: Request):
        body = await request.json()
        result = lease.stop(
            str(body.get('client_id', '')),
            str(body.get('request_id', '')))
        bridge.publish_mission({'type': 'stop'})
        await hub.broadcast()
        return result

    @app.websocket('/ws/state')
    async def websocket_state(websocket: WebSocket):
        client_id = str(websocket.query_params.get('client_id', ''))
        if not client_id:
            await websocket.close(code=1008)
            return
        await websocket.accept()
        await hub.add(client_id, websocket)
        await hub.broadcast()
        try:
            while True:
                message = await websocket.receive_json()
                if message.get('type') == 'heartbeat':
                    lease.heartbeat(client_id)
        except WebSocketDisconnect:
            pass
        finally:
            await hub.remove(client_id, websocket)
            await hub.broadcast()

    return app


def _create_preview(app, client_id, task, candidates, label):
    mission_id = str(uuid.uuid4())
    selected = str(candidates[0].get('id', '')) if len(candidates) == 1 else ''
    preview = {
        'mission_id': mission_id,
        'client_id': client_id,
        'task': task,
        'label': label,
        'candidates': candidates,
        'selected_target_id': selected,
        'created_at': time.time(),
    }
    app.state.missions[mission_id] = preview
    # 预览仅短期存在，防止无人确认的任务长期占用内存。
    cutoff = time.time() - 300.0
    expired = [key for key, item in app.state.missions.items()
               if item.get('created_at', 0.0) < cutoff]
    for key in expired:
        app.state.missions.pop(key, None)
    return preview


def _call_chat_service(base_url, job, shared):
    with shared.lock:
        frame = shared.frame
        detections = deepcopy(shared.detections.get('detections', []))
    body = {
        'request_id': job['request_id'],
        'text': job['text'],
        'detections': detections,
    }
    if frame is not None:
        body['image_base64'] = base64.b64encode(frame).decode('ascii')
    request = UrlRequest(
        base_url + '/v1/chat',
        data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urlopen(request, timeout=30.0) as response:
            payload = json.loads(response.read().decode('utf-8'))
    except (HTTPError, URLError, TimeoutError) as exc:
        raise RuntimeError(str(exc)) from exc
    return str(payload.get('answer', '模型没有返回文字回答'))


def main(args=None):
    # FastAPI 在主线程运行，ROS executor 放到后台，二者通过线程安全快照通信。
    import uvicorn

    rclpy.init(args=args)
    shared = SharedRobotState()
    bridge = BrainWebBridge(shared)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    app = create_app(bridge, shared)
    host = str(bridge.get_parameter('http_host').value)
    port = int(bridge.get_parameter('http_port').value)
    try:
        uvicorn.run(app, host=host, port=port, log_level='info')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
