#!/usr/bin/env python3
"""使用方法：ros2 run robot_brain brain_web 启动局域网 HTTP 文字交互和任务确认网页。"""

import asyncio
from copy import deepcopy
import json
import os
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_brain.action_schema import ModelAction, ModelResponse
from robot_brain.command_policy import CommandPolicy
from robot_brain.mission_manager import MissionManager
from robot_brain.qwen_client import QwenClient
from robot_brain.ros_bridge import RosBridge, SharedRobotState
from robot_brain.target_resolver import TargetResolver
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
    qwen = QwenClient(
        bridge.get_parameter('inference_url').value,
        timeout=float(bridge.get_parameter('chat_timeout').value),
        logger=bridge.get_logger(),
        log_directory=str(
            bridge.get_parameter('qwen_log_directory').value))
    missions = MissionManager(grace_seconds=grace)
    hub = SocketHub(shared, missions)
    app = FastAPI(title='机器人本地大脑', docs_url=None, redoc_url=None)
    app.state.chat_queue = asyncio.Queue(maxsize=queue_size)
    app.state.chat_history = {}
    app.state.chat_history_turns = int(
        bridge.get_parameter('chat_history_turns').value)
    action_refresh_timeout = float(
        bridge.get_parameter('action_scene_refresh_timeout').value)
    app.state.loop = None
    web_dir = os.path.join(get_package_share_directory('robot_brain'), 'web')

    async def plan_preview(preview):
        """生成并保存路径预演结果；规划失败也作为业务结果返回。"""
        target_id = preview.get('selected_target_id', '')
        if preview['task'] != 'explore' and not target_id:
            preview['plan'] = {
                'success': False, 'error_code': 'TARGET_REQUIRED',
                'message': '必须先选择一个目标',
            }
            return preview
        try:
            goal_future = bridge.send_plan(
                preview['mission_id'], preview['task'], target_id,
                preview.get('label', ''))
            goal_handle = await await_ros_future(goal_future)
            if not goal_handle.accepted:
                preview['plan'] = {
                    'success': False, 'error_code': 'PLAN_REJECTED',
                    'message': '任务规划请求被拒绝',
                }
                return preview
            wrapped = await await_ros_future(
                goal_handle.get_result_async(), 15.0)
            result = wrapped.result
            preview['plan'] = {
                'success': bool(result.success),
                'error_code': result.error_code,
                'message': result.message,
                'goal': {'x': result.goal.pose.position.x,
                         'y': result.goal.pose.position.y},
                'path_points': len(result.path.poses),
            }
        except Exception as exc:
            preview['plan'] = {
                'success': False, 'error_code': 'PLAN_TRANSPORT_ERROR',
                'message': f'路径预演请求失败：{exc}',
            }
        return preview

    def candidates_for(task, label='', target_id='', source=None):
        if source is None:
            with shared.lock:
                values = deepcopy(shared.detections.get('detections', []))
        else:
            values = deepcopy(list(source))
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

    async def refresh_action_scene(after_stamp):
        """Qwen 释放 NPU 后只等待一份新场景，不积压推理期间的旧图。"""
        try:
            await await_ros_future(bridge.detect(True), timeout=3.0)
            snapshot = await asyncio.to_thread(
                shared.wait_for_detection_after, after_stamp,
                action_refresh_timeout)
            # 让其他 ROS 订阅者先消费同一份类型化检测，再发起路径 Action。
            if snapshot is not None:
                await asyncio.sleep(0.05)
            return snapshot
        except Exception as exc:
            bridge.get_logger().warning(f'动作前视觉刷新失败：{exc}')
            return None

    async def chat_worker():
        while True:
            job = await app.state.chat_queue.get()
            request_started = time.perf_counter()
            timings = {
                '队列等待': (request_started - job['queued_at']) * 1000.0,
            }
            await hub.send(job['client_id'], {
                'type': 'chat_status', 'request_id': job['request_id'],
                'state': 'running', 'position': 0})
            try:
                snapshot_started = time.perf_counter()
                image = job['image']
                source_scene = job['scene_snapshot']
                image_stamp_ns = source_scene.image_stamp_ns
                detections = source_scene.detection_list()
                image_size = source_scene.image_size
                detection_stamp_ns = source_scene.detection_stamp_ns
                history = list(app.state.chat_history.get(
                    job['client_id'], []))
                timings['状态快照与历史准备'] = (
                    time.perf_counter() - snapshot_started) * 1000.0
                qwen_started = time.perf_counter()
                proposal = await asyncio.to_thread(
                    qwen.chat, job['request_id'], job['text'], image,
                    detections, history, image_size, image_stamp_ns,
                    detection_stamp_ns, source_scene.snapshot_id,
                    source_scene.state)
                timings['Qwen客户端调用'] = (
                    time.perf_counter() - qwen_started) * 1000.0
                policy = CommandPolicy.authorize(
                    job['text'], proposal, detections)
                model = ModelResponse(
                    answer=policy.answer, action=policy.action)
                preview = None
                action_scene = None
                preview_started = time.perf_counter()
                if model.action is not None:
                    tool = dispatch(model.action)
                    if tool['task'] == 'explore':
                        resolution = TargetResolver.resolve(
                            model.action, detections)
                        action_scene = source_scene
                    else:
                        await hub.send(job['client_id'], {
                            'type': 'chat_status',
                            'request_id': job['request_id'],
                            'state': 'refreshing_vision', 'position': 0})
                        action_scene = await refresh_action_scene(
                            source_scene.detection_stamp_ns)
                        if action_scene is None:
                            resolution = None
                            model = ModelResponse(
                                answer='最新视觉刷新超时，未生成机器人任务。')
                        else:
                            resolution = TargetResolver.resolve(
                                model.action, action_scene.detection_list())
                    if resolution is not None and not resolution.success:
                        model = ModelResponse(answer=resolution.message)
                    elif resolution is not None:
                        values = resolution.candidate_list()
                        selected = resolution.selected_target_id
                        preview = missions.create_preview(
                            job['client_id'], tool['task'], tool['label'],
                            selected,
                            snapshot_id=action_scene.snapshot_id,
                            scene_stamp_ns=action_scene.detection_stamp_ns)
                        preview['candidates'] = values
                        if tool['task'] == 'explore' or selected:
                            await plan_preview(preview)
                if model.action is None:
                    model = qwen._ground_spatial_answer(
                        model, job['text'], detections, image_size)
                timings['任务解析与路径预演'] = (
                    time.perf_counter() - preview_started) * 1000.0
                turns = history + [
                    {'role': 'user', 'text': job['text']},
                    {'role': 'assistant', 'text': model.answer},
                ]
                keep = max(0, app.state.chat_history_turns) * 2
                app.state.chat_history[job['client_id']] = (
                    turns[-keep:] if keep else [])
                action_log = None if model.action is None else {
                    'name': model.action.name,
                    'arguments': model.action.arguments,
                }
                bridge.get_logger().info(
                    '[Qwen授权结果] request_id={} policy={} answer={} action={}'.format(
                        job['request_id'], policy.reason_code,
                        json.dumps(model.answer, ensure_ascii=False),
                        json.dumps(action_log, ensure_ascii=False)))
                timings['端到端总时长'] = (
                    time.perf_counter() - job['queued_at']) * 1000.0
                qwen.log_timings(job['request_id'], timings)
                qwen.log_policy_result(
                    job['request_id'], policy, source_scene, action_scene)
                qwen.log_parsed_result(job['request_id'], model)
                await hub.send(job['client_id'], {
                    'type': 'chat_result', 'request_id': job['request_id'],
                    'answer': model.answer, 'preview': preview,
                    'state': 'completed'})
            except Exception as exc:
                timings['失败前端到端总时长'] = (
                    time.perf_counter() - job['queued_at']) * 1000.0
                qwen.log_timings(job['request_id'], timings)
                bridge.get_logger().error(
                    '[Qwen失败] request_id={} 用户输入={} error={}'.format(
                        job['request_id'],
                        json.dumps(job['text'], ensure_ascii=False), exc))
                qwen.log_error(job['request_id'], exc)
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

    async def inference_health_watchdog():
        """定期确认网关及本地 Qwen 可用。"""
        while True:
            try:
                health = await asyncio.to_thread(qwen.health)
                qwen_ok = bool(health.get('llm_ready'))
                value = {
                    'state': 'ok' if qwen_ok else 'error',
                    'model': health.get('local_llm_model', ''),
                    'gateway_state': health.get('state', 'unknown'),
                }
            except Exception as exc:
                value = {'state': 'error', 'message': str(exc)}
            with shared.lock:
                shared.health['qwen'] = value
            await hub.broadcast()
            await asyncio.sleep(5.0)

    def notify_from_ros():
        loop = app.state.loop
        if loop is not None and not loop.is_closed():
            loop.call_soon_threadsafe(lambda: asyncio.create_task(hub.broadcast()))

    @app.on_event('startup')
    async def startup():
        app.state.loop = asyncio.get_running_loop()
        bridge.notify = notify_from_ros
        app.state.workers = [
            asyncio.create_task(chat_worker()),
            asyncio.create_task(lease_watchdog()),
            asyncio.create_task(inference_health_watchdog()),
        ]

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
        # 在提交时冻结同一份配对帧和检测结果，排队期间不再替换。
        with shared.lock:
            frozen_image = shared.detection_frame
            frozen_scene = shared.freeze_scene()
        job = {
            'client_id': client_id,
            'request_id': request_id,
            'text': text,
            'queued_at': time.perf_counter(),
            'image': frozen_image,
            'scene_snapshot': frozen_scene,
        }
        try:
            app.state.chat_queue.put_nowait(job)
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
        scene = shared.freeze_scene()
        values = candidates_for(
            task, str(body.get('label', '')),
            str(body.get('target_id', '')), scene.detection_list())
        selected = values[0]['id'] if len(values) == 1 else str(body.get('target_id', ''))
        preview = missions.create_preview(
            client_id, task, str(body.get('label', '')), selected,
            snapshot_id=scene.snapshot_id,
            scene_stamp_ns=scene.detection_stamp_ns)
        preview['candidates'] = values
        if task == 'explore' or selected:
            await plan_preview(preview)
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
        preview['selected_target_id'] = selected
        if preview['task'] != 'explore':
            fresh_scene = await refresh_action_scene(
                preview.get('scene_stamp_ns'))
            if fresh_scene is None:
                return JSONResponse({
                    'success': False, 'error_code': 'VISION_REFRESH_TIMEOUT',
                    'message': '任务未确认：最新视觉刷新超时',
                }, status_code=422)
            action = ModelAction(
                preview['task'], {'label': preview.get('label', '')}
                if preview['task'] == 'goto_object' else {})
            resolution = TargetResolver.resolve(
                action, fresh_scene.detection_list(), selected)
            if not resolution.success:
                return JSONResponse({
                    'success': False, 'error_code': 'TARGET_NOT_FOUND',
                    'message': f'任务未确认：{resolution.message}',
                }, status_code=422)
            if selected != resolution.selected_target_id:
                # 跟踪 ID 可能因 Qwen 或用户确认耗时而更新；只有最新场景仍
                # 唯一匹配时才安全重绑，多目标时必须回到页面重新选择。
                if len(resolution.candidates) != 1:
                    return JSONResponse({
                        'success': False, 'error_code': 'TARGET_CHANGED',
                        'message': '任务未确认：目标已变化，请重新生成任务预演',
                        'candidates': resolution.candidate_list(),
                    }, status_code=409)
                selected = resolution.selected_target_id
                preview['selected_target_id'] = selected
            preview['candidates'] = resolution.candidate_list()
            preview['snapshot_id'] = fresh_scene.snapshot_id
            preview['scene_stamp_ns'] = fresh_scene.detection_stamp_ns
        # 确认前始终用当前地图和目标重新规划，同时重建 ROS
        # 任务缓存，避免旧预演曾失败或被停止清理后继续确认。
        await plan_preview(preview)
        plan = preview.get('plan', {})
        if not plan.get('success'):
            message = str(plan.get('message') or '当前目标没有可达路径')
            return JSONResponse({
                'success': False,
                'error_code': plan.get('error_code', 'PLAN_FAILED'),
                'message': f'任务未确认：{message}',
                'plan': plan,
            }, status_code=422)
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
    bridge.declare_parameter('chat_timeout', 180.0)
    bridge.declare_parameter('chat_history_turns', 2)
    bridge.declare_parameter('action_scene_refresh_timeout', 5.0)
    bridge.declare_parameter('qwen_log_directory', '/workspace/qwen_logs')
    bridge.declare_parameter('lease_grace_seconds', 10.0)
    bridge.declare_parameter('web_log_level', 'warn')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        web_log_level = str(
            bridge.get_parameter('web_log_level').value).lower()
        web_log_level = {
            'warn': 'warning',
            'fatal': 'critical',
        }.get(web_log_level, web_log_level)
        uvicorn.run(create_app(bridge, shared),
                    host=str(bridge.get_parameter('http_host').value),
                    port=int(bridge.get_parameter('http_port').value),
                    log_level=web_log_level,
                    access_log=web_log_level in ('debug', 'info'))
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
