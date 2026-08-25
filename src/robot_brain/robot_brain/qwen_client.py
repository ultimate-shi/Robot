"""使用方法：Web 聊天线程调用 QwenClient，经 localhost 网关获取严格 JSON。"""

import ast
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from zoneinfo import ZoneInfo

from robot_brain.action_schema import ModelResponse, parse_model_response
from robot_brain.command_policy import CommandPolicy


SYSTEM_INSTRUCTION = '\n'.join((
    '你是机器人的视觉问答和意图分类助手。',
    '用户会提供已经完成物体识别的图片场景。scene 字段表示你看到的'
    '物体，current_request 表示用户当前的问题或指令。',
    'scene 为空数组 [] 时，表示当前没有检测到任何物体；'
    '此时不得编造物体、位置或机器人动作。',
    '只输出一行合法 JSON，顶层必须且只能有 answer 和 action。'
    'answer 中直接放给用户的回答。action 只能是 null 或以下对象：',
    '{"name":"goto_object","arguments":{"label":"scene中的物体名称"}}',
    '{"name":"follow_person","arguments":{}}',
    '用户明确要求跟随人员时选 follow_person。用户明确要求前往'
    '某个物体，且 scene 中存在该物体时，才能选 goto_object。'
    'scene 中没有目标时，action 必须为 null，answer 必须明确说未检测到该目标。'
    '其他所有情况都选 null。'
    'action 为 null 时，answer 禁止声称将生成、已生成任务预演，'
    '也禁止要求用户确认。'
    '“在哪里/什么位置”是问答，不是前往命令，action 必须为 null。'
    '“画面里有什么/看到了什么”必须依据 scene 简短列出目标，'
    'action 必须为 null。'
    'action 非 null 时，answer 只能说明将生成任务预演'
    '并等待确认，不能声称已执行。'
    '示例：',
    '问候 -> {"answer":"你好。","action":null}',
    '前往杯子 -> {"answer":"将生成前往杯子的任务预演，请确认。",'
    '"action":{"name":"goto_object","arguments":{"label":"杯子"}}}',
    '跟着我 -> {"answer":"将生成跟随人员的任务预演，请确认。",'
    '"action":{"name":"follow_person","arguments":{}}}',
    'scene为[]时前往电视 -> {"answer":"当前未检测到电视。","action":null}',
    '普通问答 -> {"answer":"直接回答用户的问题。","action":null}',
))


class QwenClient:
    """访问宿主机推理网关，不包含任何 ROS 能力。"""

    def __init__(self, base_url, timeout=180.0, logger=None,
                 log_directory=''):
        self.base_url = str(base_url).rstrip('/')
        self.timeout = float(timeout)
        self.logger = logger
        self.log_directory = Path(log_directory) if log_directory else None
        self.log_paths = {}

    def health(self):
        """读取推理网关健康状态，供网页显示 Qwen 是否真正可用。"""
        request = Request(self.base_url + '/health', method='GET')
        try:
            with urlopen(request, timeout=2.0) as response:
                return json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'推理网关健康检查失败: {exc}') from exc

    def chat(self, request_id, text, image=None, detections=None,
             history=None, image_size=None, image_stamp_ns=None,
             detection_stamp_ns=None, snapshot_id='', scene_state='valid'):
        client_started = time.perf_counter()
        frozen_detections = list(detections or [])[:20]
        scene = self._compact_scene(frozen_detections, image_size)
        recent = []
        for turn in list(history or [])[-4:]:
            role = 'user' if turn.get('role') == 'user' else 'assistant'
            recent.append({
                'role': role,
                'text': str(turn.get('text', ''))[:160],
            })
        prompt_payload = {
            'schema_version': 'llm_scene.v1',
            'snapshot_id': str(snapshot_id or f'scene-{detection_stamp_ns}'),
            'scene_state': str(scene_state),
            'history_for_reference_only': recent,
            'scene': scene,
            'current_request': str(text),
        }
        prompt = json.dumps(
            prompt_payload, ensure_ascii=False, separators=(',', ':'))
        body = {
            'request_id': request_id,
            'system': SYSTEM_INSTRUCTION,
            'text': prompt,
            'detections': scene,
        }
        image_digest = hashlib.sha256(image).hexdigest() if image else ''
        self._log_info(
            '[Qwen请求] request_id={} 用户输入={}'.format(
                request_id, json.dumps(str(text), ensure_ascii=False)))
        self._log_info(
            '[Qwen输入] request_id={} prompt={} detections={} '
            'image_bytes={} image_sha256={}'.format(
                request_id,
                json.dumps(body['text'], ensure_ascii=False),
                json.dumps(scene, ensure_ascii=False),
                len(image or b''), image_digest))
        self._start_file_log(
            request_id, text, body['system'], body['text'], scene,
            frozen_detections, image, image_digest, image_size,
            image_stamp_ns, detection_stamp_ns)
        prepare_ms = (time.perf_counter() - client_started) * 1000.0
        http_started = time.perf_counter()
        request = Request(
            self.base_url + '/v1/chat', data=json.dumps(body).encode('utf-8'),
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urlopen(request, timeout=self.timeout) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, TimeoutError) as exc:
            self.log_timings(request_id, {
                '提示词与请求准备': prepare_ms,
                '网关HTTP与Qwen推理（失败）': (
                    time.perf_counter() - http_started) * 1000.0,
                'QwenClient总时长（失败）': (
                    time.perf_counter() - client_started) * 1000.0,
            })
            raise RuntimeError(f'Qwen服务不可用: {exc}') from exc
        http_ms = (time.perf_counter() - http_started) * 1000.0
        raw = payload.get('answer', payload.get('content', ''))
        self._log_info(
            '[Qwen原始返回] request_id={} model={} raw={}'.format(
                request_id, payload.get('model', ''),
                json.dumps(raw, ensure_ascii=False)))
        self._append_file_log(request_id, 'Qwen 原始返回', {
            'model': payload.get('model', ''),
            'metrics': payload.get('metrics', {}),
            'raw': raw,
        })
        parse_started = time.perf_counter()

        def finish(result, parse_mode):
            self._append_file_log(request_id, '解析路径', parse_mode)
            self.log_timings(request_id, {
                '提示词与请求准备': prepare_ms,
                '网关HTTP与Qwen推理': http_ms,
                '响应解析与位置落地': (
                    time.perf_counter() - parse_started) * 1000.0,
                'QwenClient总时长': (
                    time.perf_counter() - client_started) * 1000.0,
            })
            return result

        model, parse_mode = self._parse_with_repair(raw)
        if model is None:
            model = self._salvage_plain_answer(raw)
            if model is not None:
                return finish(model, 'answer_salvage')
            return finish(ModelResponse(
                answer='模型返回格式异常，未执行任何机器人动作。'),
                'rejected')
        # 这里只返回不可信模型提案；用户命令授权和最新视觉重验由 Web 编排层完成。
        return finish(model, parse_mode)

    def _log_info(self, message):
        """通过 ROS logger 输出请求审计日志；未提供 logger 时保持安静。"""
        if self.logger is not None:
            self.logger.info(message)

    def _start_file_log(self, request_id, user_text, system_instruction,
                        user_prompt, scene, raw_detections, image,
                        image_digest, image_size, image_stamp_ns,
                        detection_stamp_ns):
        """创建 TXT 与带 YOLO 框的 JPG；日志失败不能阻断机器人问答。"""
        if self.log_directory is None:
            return
        try:
            self.log_directory.mkdir(parents=True, exist_ok=True)
            safe_id = re.sub(r'[^A-Za-z0-9_-]+', '_', str(request_id))[:64]
            timestamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
                '%Y%m%d_%H%M%S_%f')[:-3]
            stem = f'{timestamp}_{safe_id or "request"}'
            text_path = self.log_directory / f'{stem}.txt'
            image_path = self.log_directory / f'{stem}.jpg'
            drawing_errors = []
            if (image and image_stamp_ns is not None
                    and image_stamp_ns == detection_stamp_ns):
                try:
                    annotated, drawing_errors = self._annotate_yolo_image(
                        image, raw_detections)
                    image_path.write_bytes(annotated)
                    image_name = image_path.name
                except Exception as exc:
                    image_name = '无'
                    drawing_errors.append(f'标注图片生成失败：{exc}')
            else:
                image_name = '无'
                if image and image_stamp_ns != detection_stamp_ns:
                    drawing_errors.append(
                        '相机帧与检测时间戳不一致，拒绝保存错位图片')
                elif not image:
                    drawing_errors.append('没有与检测时间戳配对的相机帧')
            content = (
                '使用方法：本文件记录单次网页 Qwen 问答及解析结果。\n'
                f'时间：{timestamp}\n'
                f'request_id：{request_id}\n'
                f'用户输入：{user_text}\n'
                f'图片文件：{image_name}\n'
                '图片用途：YOLO 标注审计图，未发送给 Qwen\n'
                f'源图字节数：{len(image or b"")}\n'
                f'图片 SHA256：{image_digest}\n\n'
                f'相机帧时间戳：{image_stamp_ns}\n'
                f'检测时间戳：{detection_stamp_ns}\n'
                f'检测数量：{len(scene)}\n'
                '绘制告警：'
                f'{json.dumps(drawing_errors, ensure_ascii=False)}\n\n'
                '给 Qwen 的 system 提示词：\n'
                f'{system_instruction}\n\n'
                f'给 Qwen 的 user 内容：\n{user_prompt}\n')
            text_path.write_text(content, encoding='utf-8')
            self.log_paths[str(request_id)] = text_path
            if drawing_errors:
                self._log_warning(
                    '[Qwen审计图片] request_id={} warnings={}'.format(
                        request_id, json.dumps(
                            drawing_errors, ensure_ascii=False)))
            self._log_info(f'[Qwen文件日志] request_id={request_id} '
                           f'txt={text_path} image={image_name}')
        except OSError as exc:
            self._log_error(
                f'[Qwen文件日志失败] request_id={request_id} error={exc}')

    @classmethod
    def _compact_scene(cls, detections, image_size):
        """把感知消息压缩为适合 3B 小模型的稳定 JSON 字段。"""
        scene = []
        for item in list(detections)[:20]:
            label = str(
                item.get('label_zh') or item.get('class_name')
                or '未知物体').strip()
            distance = item.get('distance')
            try:
                distance = (None if distance is None
                            else round(float(distance), 2))
            except (TypeError, ValueError):
                distance = None
            try:
                confidence = round(float(item.get('confidence', 0.0)), 2)
            except (TypeError, ValueError):
                confidence = 0.0
            scene.append({
                'id': str(item.get('id', '')),
                'label': label,
                'class_name': str(item.get('class_name', '')),
                'confidence': confidence,
                'distance_m': distance,
                'position': cls._box_position(item, image_size) or 'unknown',
            })
        return scene

    @staticmethod
    def _annotate_yolo_image(image, detections):
        """在配对 JPEG 上绘制稳定的 ASCII 标签，返回编码后的 JPG 与告警。"""
        import cv2
        import numpy as np

        encoded = np.frombuffer(image, dtype=np.uint8)
        canvas = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if canvas is None:
            raise OSError('无法解码 YOLO 审计源图')
        height, width = canvas.shape[:2]
        errors = []
        drawn = 0
        for index, item in enumerate(detections):
            try:
                values = [float(value) for value in item.get('bbox', [])]
                if len(values) != 4 or not all(math.isfinite(v) for v in values):
                    raise ValueError('bbox 不是四个有限数值')
                x1, y1, x2, y2 = values
                x1 = int(round(min(max(x1, 0.0), width - 1)))
                y1 = int(round(min(max(y1, 0.0), height - 1)))
                x2 = int(round(min(max(x2, 0.0), width - 1)))
                y2 = int(round(min(max(y2, 0.0), height - 1)))
                if x2 <= x1 or y2 <= y1:
                    raise ValueError('裁剪后检测框面积为零')
                identity = str(item.get('id') or index)
                digest = hashlib.sha256(identity.encode('utf-8')).digest()
                color = tuple(int(96 + byte % 160) for byte in digest[:3])
                confidence = round(float(item.get('confidence', 0.0)) * 100)
                class_name = str(item.get('class_name') or 'object')
                distance = item.get('distance')
                distance_text = ('unknown' if distance is None
                                 else f'{float(distance):.2f}m')
                label = f'{identity} {class_name} {confidence}% {distance_text}'
                thickness = max(2, width // 320)
                scale = max(0.45, width / 1280.0)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
                (text_width, text_height), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
                text_top = max(0, y1 - text_height - baseline - 6)
                text_right = min(width - 1, x1 + text_width + 6)
                cv2.rectangle(
                    canvas, (x1, text_top),
                    (text_right, min(height - 1, text_top + text_height
                                     + baseline + 6)), color, -1)
                cv2.putText(
                    canvas, label, (x1 + 3, text_top + text_height + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness,
                    cv2.LINE_AA)
                drawn += 1
            except (TypeError, ValueError) as exc:
                errors.append(f'第 {index + 1} 项未绘制：{exc}')
        if not detections:
            label = 'YOLO: 0 objects'
            cv2.rectangle(canvas, (0, 0), (230, 34), (64, 220, 128), -1)
            cv2.putText(canvas, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0), 2, cv2.LINE_AA)
        elif drawn == 0:
            errors.append('没有任何有效检测框被绘制')
        success, output = cv2.imencode(
            '.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            raise OSError('YOLO 审计图片 JPEG 编码失败')
        return output.tobytes(), errors

    def _append_file_log(self, request_id, title, value):
        path = self.log_paths.get(str(request_id))
        if path is None:
            return
        try:
            serialized = (value if isinstance(value, str) else json.dumps(
                value, ensure_ascii=False, indent=2))
            with path.open('a', encoding='utf-8') as stream:
                stream.write(f'\n{title}：\n{serialized}\n')
        except OSError as exc:
            self._log_error(
                f'[Qwen文件日志失败] request_id={request_id} error={exc}')

    def log_parsed_result(self, request_id, model):
        """追加最终用户答案和经过白名单处理后的动作。"""
        action = None if model.action is None else {
            'name': model.action.name,
            'arguments': model.action.arguments,
        }
        self._append_file_log(request_id, '最终解析结果', {
            'answer': model.answer,
            'action': action,
        })
        self.log_paths.pop(str(request_id), None)

    def log_policy_result(self, request_id, result, source_snapshot,
                          action_snapshot=None):
        """记录确定性授权结果以及回答、动作各自使用的场景快照。"""
        action = None if result.action is None else {
            'name': result.action.name,
            'arguments': result.action.arguments,
        }
        self._append_file_log(request_id, '动作授权结果', {
            'reason_code': result.reason_code,
            'source': result.source,
            'action': action,
            'answer_snapshot': source_snapshot.audit_dict(),
            'action_snapshot': (
                None if action_snapshot is None else action_snapshot.audit_dict()),
        })

    def log_timings(self, request_id, timings):
        """追加各阶段毫秒耗时，并同步输出一行终端日志。"""
        rounded = {
            name: round(float(duration), 1)
            for name, duration in timings.items()
        }
        self._append_file_log(request_id, '步骤耗时（毫秒）', rounded)
        self._log_info(
            '[Qwen耗时] request_id={} timings={}'.format(
                request_id, json.dumps(rounded, ensure_ascii=False)))

    def log_error(self, request_id, error):
        """追加失败原因并结束本轮文件记录。"""
        self._append_file_log(request_id, '失败原因', str(error))
        self.log_paths.pop(str(request_id), None)

    def _log_error(self, message):
        if self.logger is not None:
            self.logger.error(message)

    def _log_warning(self, message):
        if self.logger is not None:
            self.logger.warning(message)

    @classmethod
    def _ground_spatial_answer(cls, model, text, detections, image_size):
        """位置问答使用实时检测框落地，避免小模型给出含糊答案。"""
        if model.action is not None or not any(word in str(text) for word in (
                '哪里', '哪儿', '哪个位置', '什么位置', '位置')):
            return model
        matches = []
        question = str(text).lower()
        for item in detections:
            labels = {
                str(item.get('label_zh', '')).strip(),
                str(item.get('class_name', '')).strip(),
            }
            labels.discard('')
            person_match = ('人' in question
                            and str(item.get('class_name', '')).lower()
                            == 'person')
            if person_match or any(label.lower() in question for label in labels):
                position = cls._box_position(item, image_size)
                if position:
                    name = item.get('label_zh') or item.get('class_name')
                    matches.append(f'{name}位于画面{position}')
        if not matches:
            return model
        return ModelResponse(answer='；'.join(matches) + '。')

    @staticmethod
    def _box_position(item, image_size):
        """把像素检测框转换为用户易懂的九宫格方位。"""
        box = item.get('bbox')
        try:
            width = float((image_size or {})['width'])
            height = float((image_size or {})['height'])
            x1, y1, x2, y2 = [float(value) for value in box]
        except (KeyError, TypeError, ValueError):
            return ''
        if width <= 0.0 or height <= 0.0:
            return ''
        center_x = (x1 + x2) / (2.0 * width)
        center_y = (y1 + y2) / (2.0 * height)
        horizontal = '左侧' if center_x < 1.0 / 3.0 else (
            '右侧' if center_x > 2.0 / 3.0 else '中间')
        vertical = '上方' if center_y < 1.0 / 3.0 else (
            '下方' if center_y > 2.0 / 3.0 else '中部')
        return horizontal + vertical

    @staticmethod
    def _parse_with_repair(raw):
        """执行一次本地格式修复；所有候选仍必须通过严格动作 Schema。"""
        try:
            return parse_model_response(raw), 'direct'
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

        decoded = None
        mode = 'json_extract'
        if isinstance(raw, str):
            text = raw.strip()
            if text.startswith('```') and text.endswith('```'):
                lines = text.splitlines()
                text = '\n'.join(lines[1:-1]).strip()
            try:
                decoded = json.loads(text)
            except (TypeError, ValueError, json.JSONDecodeError):
                decoded = QwenClient._extract_unique_json(text)
            if decoded is None:
                try:
                    literal = ast.literal_eval(text)
                    decoded = literal if isinstance(literal, dict) else None
                except (SyntaxError, ValueError):
                    decoded = None
        elif isinstance(raw, dict):
            decoded = raw

        if decoded is not None:
            try:
                return parse_model_response(decoded), mode
            except (TypeError, ValueError, json.JSONDecodeError):
                pass
            try:
                if (isinstance(decoded, dict)
                        and set(decoded) == {'name', 'arguments'}):
                    return parse_model_response({
                        'answer': '已生成任务预演，请检查后确认。',
                        'action': decoded,
                    }), 'action_wrap'
            except (TypeError, ValueError, json.JSONDecodeError):
                pass
        return None, 'rejected'

    @staticmethod
    def _extract_unique_json(text):
        """从解释文字中提取唯一响应对象；多个完整响应时拒绝猜测。"""
        decoder = json.JSONDecoder()
        candidates = []
        for index, character in enumerate(str(text)):
            if character != '{':
                continue
            try:
                value, _ = decoder.raw_decode(str(text)[index:])
            except json.JSONDecodeError:
                continue
            if not isinstance(value, dict):
                continue
            keys = set(value)
            if keys == {'answer', 'action'}:
                candidates.append(value)
        return candidates[0] if len(candidates) == 1 else None

    @staticmethod
    def _salvage_plain_answer(raw):
        """保留格式错误响应中的文字答案，但绝不执行其中的非法动作。"""
        payload = raw
        if isinstance(raw, str):
            text = raw.strip()
            if text.startswith('```') and text.endswith('```'):
                lines = text.splitlines()
                text = '\n'.join(lines[1:-1]).strip()
            try:
                payload = json.loads(text)
            except (TypeError, ValueError, json.JSONDecodeError):
                payload = QwenClient._extract_unique_json(text)
            if payload is None:
                try:
                    payload = ast.literal_eval(text)
                except (SyntaxError, ValueError):
                    return None
        if not isinstance(payload, dict):
            return None
        answer = payload.get('answer')
        if not isinstance(answer, str) or not answer.strip():
            return None
        return ModelResponse(answer=answer.strip())

    @staticmethod
    def _repair_action_only(raw):
        """只修复完整白名单动作对象，其他非标准输出按无动作处理。"""
        model, mode = QwenClient._parse_with_repair(raw)
        return model if mode == 'action_wrap' else None

    @staticmethod
    def _explicit_action(text, detections):
        """兼容旧测试和调用者，实际命令策略由 CommandPolicy 独立维护。"""
        return CommandPolicy.explicit_action(text, detections)

    @staticmethod
    def _validate_scene_action(model, detections):
        """goto_object 必须引用本轮实际检测标签。"""
        if model.action is None or model.action.name != 'goto_object':
            return model
        requested = str(model.action.arguments.get('label', '')).strip().lower()
        for item in detections:
            labels = {
                str(item.get('label_zh', '')).strip().lower(),
                str(item.get('class_name', '')).strip().lower(),
            }
            labels.discard('')
            if requested in labels:
                return model
        display = model.action.arguments.get('label', '目标物体')
        return ModelResponse(
            answer=f'本轮 YOLO 未检测到{display}，未生成机器人动作。')

    @staticmethod
    def _gate_action_by_request(model, text, detections):
        """兼容入口：委托独立策略层校验用户原始命令。"""
        result = CommandPolicy.authorize(text, model, detections)
        return ModelResponse(answer=result.answer, action=result.action)

    @staticmethod
    def _preview_answer(action):
        return CommandPolicy.preview_answer(action)
