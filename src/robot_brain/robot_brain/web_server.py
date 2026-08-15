#!/usr/bin/env python3
"""使用方法：ros2 run robot_brain brain_web 启动局域网 HTTP 文字交互和任务确认网页。"""

import asyncio
from copy import deepcopy
import os
import threading
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_brain.mission_manager import MissionManager
from robot_brain.qwen_client import QwenClient
from robot_brain.ros_bridge import RosBridge, SharedRobotState
from robot_brain.tool_dispatcher import dispatch


async def await_ros_future(future, timeout=10.0):
    """在 FastAPI 事件循环中等待由独立 ROS executor 推进的 Future。"""
    started = asyncio.get_running_loop().time()
    while not future.done():
        if asyncio.get_running_loop().time() - started > timeout:
            raise TimeoutError('ROS 请求超时')
        await asyncio.sleep(0.01)
    result = future.result()
    if result is None:
        raise RuntimeError('ROS 请求没有返回结果')
    return result


class SocketHub:
    """按 client_id 路由私有消息，共享状态对慢客户端只保留最新值。"""

    def __init__(self, shared, missions):
        self.shared = shared
        self.missions = missions
        self.connections = {}
        self.lock = asyncio.Lock()

    async def add(self, client_id, websocket):
        async with self.lock:
            self.connections.setdefault(client_id, set()).add(websocket)
        self.missions.lease.connect(client_id)

    async def remove(self, client_id, websocket):
        async with self.lock:
            sockets = self.connections.get(client_id, set())
            sockets.discard(websocket)
            if not sockets:
                self.connections.pop(client_id, None)
                self.missions.lease.disconnect(client_id)

    async def send(self, client_id, payload):
        async with self.lock:
            sockets = list(self.connections.get(client_id, set()))
        for socket in sockets:
            try:
                await socket.send_json(payload)
            except Exception:
                pass

    async def broadcast(self):
        async with self.lock:
            targets = [(key, list(value)) for key, value in self.connections.items()]
        base = self.shared.snapshot()
        for client_id, sockets in targets:
            lease = self.missions.lease.snapshot()
            controller = lease.pop('controller_id', '')
            lease['is_controller'] = controller == client_id
            payload = {'type': 'state', 'lease': lease, **deepcopy(base)}
            payload['health']['nav2'] = payload['health'].get('navigation', {})
            for socket in sockets:
                try:
                    await socket.send_json(payload)
                except Exception:
                    pass


def create_app(bridge, shared):
    """创建 HTTP 应用；所有 ROS 操作通过 bridge 的固定方法完成。"""
    from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse, Response

    grace = float(bridge.get_parameter('lease_grace_seconds').value)
    queue_size = int(bridge.get_parameter('chat_queue_size').value)
    qwen = QwenClient(bridge.get_parameter('inference_url').value)
    missions = MissionManager(grace_seconds=grace)
    hub = SocketHub(shared, missions)
    app = FastAPI(title='机器人本地大脑', docs_url=None, redoc_url=None)
    app.state.chat_queue = asyncio.Queue(maxsize=queue_size)
    app.state.loop = None
    web_dir = os.path.join(get_package_share_directory('robot_brain'), 'web')

    async def plan_preview(preview):
        target_id = preview.get('selected_target_id', '')
        if preview['task'] != 'explore' and not target_id:
            return preview
        goal_future = bridge.send_plan(
            preview['mission_id'], preview['task'], target_id,
            preview.get('label', ''))
        goal_handle = await await_ros_future(goal_future)
        if not goal_handle.accepted:
            raise RuntimeError('任务规划请求被拒绝')
        wrapped = await await_ros_future(goal_handle.get_result_async(), 15.0)
        result = wrapped.result
        preview['plan'] = {
            'success': result.success, 'error_code': result.error_code,
            'message': result.message,
            'goal': {'x': result.goal.pose.position.x,
                     'y': result.goal.pose.position.y},
            'path_points': len(result.path.poses),
        }
        if not result.success:
            raise RuntimeError(result.message)
        return preview

    def candidates_for(task, label='', target_id=''):
        with shared.lock:
            values = deepcopy(shared.detections.get('detections', []))
        if target_id:
            values = [item for item in values if item['id'] == target_id]
        if task == 'follow_person':
            values = [item for item in values if item['class_name'].lower() == 'person']
        if task == 'goto_object' and label:
            word = label.lower()
            values = [item for item in values
                      if word in item['class_name'].lower()
                      or label in item.get('label_zh', '')]
        return values

    async def chat_worker():
        while True:
            job = await app.state.chat_queue.get()
            await hub.send(job['client_id'], {
                'type': 'chat_status', 'request_id': job['request_id'],
                'state': 'running', 'position': 0})
            try:
                with shared.lock:
                    image = shared.frame
                    detections = deepcopy(shared.detections.get('detections', []))
                model = await asyncio.to_thread(
                    qwen.chat, job['request_id'], job['text'], image, detections)
                preview = None
                if model.action is not None:
                    tool = dispatch(model.action)
                    values = candidates_for(tool['task'], tool['label'])
                    selected = values[0]['id'] if len(values) == 1 else ''
                    preview = missions.create_preview(
                        job['client_id'], tool['task'], tool['label'], selected)
                    preview['candidates'] = values
                    if tool['task'] == 'explore' or selected:
                        await plan_preview(preview)
                await hub.send(job['client_id'], {
                    'type': 'chat_result', 'request_id': job['request_id'],
                    'answer': model.answer, 'preview': preview,
                    'state': 'completed'})
            except Exception as exc:
                await hub.send(job['client_id'], {
                    'type': 'chat_result', 'request_id': job['request_id'],
                    'answer': f'端侧模型或任务预演暂不可用：{exc}',
                    'state': 'error'})
            finally:
                app.state.chat_queue.task_done()

    async def lease_watchdog():
        while True:
            await asyncio.sleep(0.5)
            if missions.lease.tick() is not None:
                await await_ros_future(bridge.stop(), timeout=1.0)
                await hub.broadcast()

    def notify_from_ros():
        loop = app.state.loop
        if loop is not None and not loop.is_closed():
            loop.call_soon_threadsafe(lambda: asyncio.create_task(hub.broadcast()))

    @app.on_event('startup')
    async def startup():
        app.state.loop = asyncio.get_running_loop()
        bridge.notify = notify_from_ros
        app.state.workers = [asyncio.create_task(chat_worker()),
                             asyncio.create_task(lease_watchdog())]

    @app.on_event('shutdown')
    async def shutdown():
        bridge.notify = None
        for worker in app.state.workers:
            worker.cancel()

    @app.get('/')
    async def index():
        return FileResponse(
            os.path.join(web_dir, 'index.html'),
            headers={'Cache-Control': 'no-store'})

    @app.get('/app.js')
    async def javascript():
        return FileResponse(
            os.path.join(web_dir, 'app.js'), media_type='application/javascript',
            headers={'Cache-Control': 'no-store'})

    @app.get('/style.css')
    async def stylesheet():
        return FileResponse(
            os.path.join(web_dir, 'style.css'), media_type='text/css',
            headers={'Cache-Control': 'no-store'})

    @app.get('/api/frame.jpg')
    async def frame():
        with shared.lock:
            data = shared.frame
        return (Response(status_code=204) if data is None else
                Response(content=data, media_type='image/jpeg',
                         headers={'Cache-Control': 'no-store'}))

    @app.get('/api/map')
    async def map_data():
        with shared.lock:
            value = deepcopy(shared.map)
        if value is None:
            return JSONResponse({'state': 'waiting'}, status_code=503)
        return value

    @app.get('/api/detections')
    async def detections():
        with shared.lock:
            return deepcopy(shared.detections)

    @app.post('/api/detections/refresh')
    async def detections_refresh():
        with shared.lock:
            before_stamp = shared.detections.get('stamp_ns')
        response = await await_ros_future(bridge.detect(True), timeout=3.0)
        # DetectObjects 的 force_refresh 只安排下一帧，网页在这里等待真正的新结果，
        # 避免把上一次的数量误报成“当前画面”识别结果。
        deadline = asyncio.get_running_loop().time() + 5.0
        refreshed = False
        current = None
        while asyncio.get_running_loop().time() < deadline:
            with shared.lock:
                current = deepcopy(shared.detections)
            stamp = current.get('stamp_ns')
            if stamp is not None and stamp != before_stamp:
                refreshed = True
                break
            await asyncio.sleep(0.1)
        if current is None:
            with shared.lock:
                current = deepcopy(shared.detections)
        values = current.get('detections', [])
        if refreshed:
            message = f'当前画面识别完成，共 {len(values)} 个目标'
        elif values:
            message = f'新结果等待超时，显示上次识别的 {len(values)} 个目标'
        else:
            message = response.message or '尚未收到识别结果'
        return {
            'success': bool(response.success or refreshed),
            'refreshed': refreshed,
            'message': message,
            'model': current.get('model', ''),
            'latency_ms': current.get('latency_ms', 0.0),
            'detections': values,
        }

    @app.get('/api/health')
    async def health():
        return shared.snapshot()

    @app.post('/api/chat')
    async def chat(request: Request):
        body = await request.json()
        client_id = str(body.get('client_id', ''))
        text = str(body.get('text', '')).strip()
        request_id = str(body.get('request_id', '') or uuid.uuid4())
        if not client_id or not text:
            return JSONResponse({'message': 'client_id 和 text 不能为空'}, status_code=400)
        try:
            app.state.chat_queue.put_nowait({
                'client_id': client_id, 'request_id': request_id, 'text': text})
        except asyncio.QueueFull:
            return JSONResponse({'message': '端侧模型队列已满，请稍后重试'}, status_code=503)
        position = app.state.chat_queue.qsize()
        await hub.send(client_id, {'type': 'chat_status', 'request_id': request_id,
                                   'state': 'queued', 'position': position})
        return {'success': True, 'state': 'queued', 'request_id': request_id,
                'position': position}

    @app.post('/api/missions/preview')
    async def mission_preview(request: Request):
        body = await request.json()
        client_id = str(body.get('client_id', ''))
        task = str(body.get('task', ''))
        if task not in ('goto_object', 'follow_person', 'explore'):
            return JSONResponse({'message': '任务类型无效'}, status_code=400)
        values = candidates_for(task, str(body.get('label', '')),
                                str(body.get('target_id', '')))
        selected = values[0]['id'] if len(values) == 1 else str(body.get('target_id', ''))
        preview = missions.create_preview(client_id, task, str(body.get('label', '')), selected)
        preview['candidates'] = values
        if task == 'explore' or selected:
            try:
                await plan_preview(preview)
            except Exception as exc:
                preview['plan'] = {'success': False, 'message': str(exc)}
        return {'success': True, **preview}

    @app.post('/api/missions/{mission_id}/confirm')
    async def mission_confirm(mission_id: str, request: Request):
        body = await request.json()
        client_id = str(body.get('client_id', ''))
        request_id = str(body.get('request_id', ''))
        preview = missions.owned_preview(mission_id, client_id)
        if preview is None:
            return JSONResponse({'message': '任务预览不存在或不属于当前页面'}, status_code=404)
        selected = str(body.get('target_id', preview.get('selected_target_id', '')))
        if preview['task'] != 'explore' and not selected:
            return JSONResponse({'message': '必须选择一个目标'}, status_code=400)
        candidate_ids = {str(item.get('id', '')) for item in preview.get('candidates', [])}
        if preview['task'] != 'explore' and selected not in candidate_ids:
            return JSONResponse({'message': '所选目标不属于当前任务预览'}, status_code=400)
        if selected != preview.get('selected_target_id'):
            preview['selected_target_id'] = selected
            await plan_preview(preview)
        lease_result = missions.lease.confirm(
            client_id, request_id, mission_id, preview['task'])
        if not lease_result['success']:
            return JSONResponse(lease_result, status_code=409)
        result = await await_ros_future(bridge.confirm(mission_id), timeout=3.0)
        if not result.accepted:
            missions.lease.cancel(client_id, request_id + '-rollback')
            return JSONResponse({'message': result.message}, status_code=409)
        await hub.broadcast()
        return {**lease_result, 'message': result.message}

    @app.post('/api/missions/cancel')
    async def mission_cancel(request: Request):
        body = await request.json()
        result = missions.lease.cancel(str(body.get('client_id', '')),
                                       str(body.get('request_id', '')))
        if not result['success']:
            return JSONResponse(result, status_code=409)
        await await_ros_future(bridge.cancel(), timeout=1.0)
        await hub.broadcast()
        return result

    @app.post('/api/control/release')
    async def release(request: Request):
        body = await request.json()
        result = missions.lease.release(str(body.get('client_id', '')),
                                        str(body.get('request_id', '')))
        if not result['success']:
            return JSONResponse(result, status_code=409)
        await await_ros_future(bridge.stop(), timeout=1.0)
        await hub.broadcast()
        return result

    @app.post('/api/stop')
    async def stop(request: Request):
        body = await request.json()
        result = missions.lease.stop(str(body.get('client_id', '')),
                                     str(body.get('request_id', '')))
        await await_ros_future(bridge.stop(), timeout=1.0)
        await hub.broadcast()
        return result

    @app.post('/api/dataset/capture')
    async def capture(request: Request):
        body = await request.json()
        result = await await_ros_future(bridge.capture(
            str(body.get('request_id', '')), str(body.get('scene_id', 'scene'))), 5.0)
        status = 200 if result.success else 503
        return JSONResponse({'success': result.success, 'message': result.message,
                             'output_path': result.output_path}, status_code=status)

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
                    missions.lease.heartbeat(client_id)
        except WebSocketDisconnect:
            pass
        finally:
            await hub.remove(client_id, websocket)
            await hub.broadcast()

    return app


def main(args=None):
    """在 ROS executor 旁启动 Uvicorn，HTTP只监听配置中的局域网地址。"""
    import uvicorn
    rclpy.init(args=args)
    shared = SharedRobotState()
    bridge = RosBridge(shared)
    bridge.declare_parameter('http_host', '0.0.0.0')
    bridge.declare_parameter('http_port', 8080)
    bridge.declare_parameter('inference_url', 'http://127.0.0.1:9100')
    bridge.declare_parameter('chat_queue_size', 8)
    bridge.declare_parameter('lease_grace_seconds', 10.0)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        uvicorn.run(create_app(bridge, shared),
                    host=str(bridge.get_parameter('http_host').value),
                    port=int(bridge.get_parameter('http_port').value))
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
