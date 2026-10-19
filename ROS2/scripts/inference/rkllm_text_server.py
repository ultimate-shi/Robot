#!/usr/bin/env python3
"""使用方法：由 start_yolo_gateway.sh 启动。

本程序常驻加载纯文本 RKLLM 模型并提供本机 HTTP 接口。
"""

import argparse
import codecs
import ctypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
import time


RKLLM_RUN_NORMAL = 0
RKLLM_RUN_FINISH = 2
RKLLM_RUN_ERROR = 3
RKLLM_INPUT_PROMPT = 0
RKLLM_INFER_GENERATE = 0


class RKLLMExtendParam(ctypes.Structure):
    _fields_ = [
        ('base_domain_id', ctypes.c_int32),
        ('embed_flash', ctypes.c_int8),
        ('enabled_cpus_num', ctypes.c_int8),
        ('enabled_cpus_mask', ctypes.c_uint32),
        ('n_batch', ctypes.c_uint8),
        ('use_cross_attn', ctypes.c_int8),
        ('reserved', ctypes.c_uint8 * 104),
    ]


class RKLLMParam(ctypes.Structure):
    _fields_ = [
        ('model_path', ctypes.c_char_p),
        ('max_context_len', ctypes.c_int32),
        ('max_new_tokens', ctypes.c_int32),
        ('top_k', ctypes.c_int32),
        ('n_keep', ctypes.c_int32),
        ('top_p', ctypes.c_float),
        ('temperature', ctypes.c_float),
        ('repeat_penalty', ctypes.c_float),
        ('frequency_penalty', ctypes.c_float),
        ('presence_penalty', ctypes.c_float),
        ('mirostat', ctypes.c_int32),
        ('mirostat_tau', ctypes.c_float),
        ('mirostat_eta', ctypes.c_float),
        ('skip_special_token', ctypes.c_bool),
        ('is_async', ctypes.c_bool),
        ('img_start', ctypes.c_char_p),
        ('img_end', ctypes.c_char_p),
        ('img_content', ctypes.c_char_p),
        ('extend_param', RKLLMExtendParam),
    ]


class RKLLMEmbedInput(ctypes.Structure):
    _fields_ = [('embed', ctypes.POINTER(ctypes.c_float)),
                ('n_tokens', ctypes.c_size_t)]


class RKLLMTokenInput(ctypes.Structure):
    _fields_ = [('input_ids', ctypes.POINTER(ctypes.c_int32)),
                ('n_tokens', ctypes.c_size_t)]


class RKLLMMultiModelInput(ctypes.Structure):
    _fields_ = [
        ('prompt', ctypes.c_char_p),
        ('image_embed', ctypes.POINTER(ctypes.c_float)),
        ('n_image_tokens', ctypes.c_size_t),
        ('n_image', ctypes.c_size_t),
        ('image_width', ctypes.c_size_t),
        ('image_height', ctypes.c_size_t),
    ]


class RKLLMInputUnion(ctypes.Union):
    _fields_ = [
        ('prompt_input', ctypes.c_char_p),
        ('embed_input', RKLLMEmbedInput),
        ('token_input', RKLLMTokenInput),
        ('multimodal_input', RKLLMMultiModelInput),
    ]


class RKLLMInput(ctypes.Structure):
    _fields_ = [
        ('role', ctypes.c_char_p),
        ('enable_thinking', ctypes.c_bool),
        ('input_type', ctypes.c_int),
        ('input_data', RKLLMInputUnion),
    ]


class RKLLMInferParam(ctypes.Structure):
    _fields_ = [
        ('mode', ctypes.c_int),
        ('lora_params', ctypes.c_void_p),
        ('prompt_cache_params', ctypes.c_void_p),
        ('keep_history', ctypes.c_int),
    ]


class RKLLMResultLastHiddenLayer(ctypes.Structure):
    _fields_ = [('hidden_states', ctypes.POINTER(ctypes.c_float)),
                ('embd_size', ctypes.c_int), ('num_tokens', ctypes.c_int)]


class RKLLMResultLogits(ctypes.Structure):
    _fields_ = [('logits', ctypes.POINTER(ctypes.c_float)),
                ('vocab_size', ctypes.c_int), ('num_tokens', ctypes.c_int)]


class RKLLMPerfStat(ctypes.Structure):
    _fields_ = [
        ('prefill_time_ms', ctypes.c_float), ('prefill_tokens', ctypes.c_int),
        ('generate_time_ms', ctypes.c_float), ('generate_tokens', ctypes.c_int),
        ('memory_usage_mb', ctypes.c_float),
    ]


class RKLLMResult(ctypes.Structure):
    _fields_ = [
        ('text', ctypes.c_char_p), ('token_id', ctypes.c_int),
        ('last_hidden_layer', RKLLMResultLastHiddenLayer),
        ('logits', RKLLMResultLogits), ('perf', RKLLMPerfStat),
    ]


class RKLLMTextModel:
    """按 Rockchip 1.2.1 ABI 初始化一次模型，并串行处理所有文本请求。"""

    def __init__(self, model_path, runtime_path, context_length=4096,
                 max_new_tokens=96):
        self.model_path = Path(model_path)
        self.runtime_path = Path(runtime_path)
        self.lock = threading.Lock()
        self.output = []
        self.decoder = None
        self.callback_error = ''
        self.first_token_at = None
        self.last_metrics = {}
        load_started_at = time.time()
        self.library = ctypes.CDLL(str(self.runtime_path))
        self.handle = ctypes.c_void_p()
        self.callback_type = ctypes.CFUNCTYPE(
            ctypes.c_int, ctypes.POINTER(RKLLMResult), ctypes.c_void_p,
            ctypes.c_int)
        self.callback = self.callback_type(self._callback)

        create_default = self.library.rkllm_createDefaultParam
        create_default.argtypes = []
        create_default.restype = RKLLMParam
        params = create_default()
        params.model_path = str(self.model_path).encode('utf-8')
        params.max_context_len = int(context_length)
        params.max_new_tokens = int(max_new_tokens)
        params.top_k = 1
        params.top_p = 0.1
        params.temperature = 0.1
        params.repeat_penalty = 1.05
        params.skip_special_token = True
        params.is_async = False
        params.extend_param.base_domain_id = 0
        params.extend_param.embed_flash = 1
        params.extend_param.n_batch = 1
        params.extend_param.use_cross_attn = 0
        params.extend_param.enabled_cpus_num = 4
        params.extend_param.enabled_cpus_mask = sum(1 << item for item in range(4, 8))

        initialize = self.library.rkllm_init
        initialize.argtypes = [ctypes.POINTER(ctypes.c_void_p),
                               ctypes.POINTER(RKLLMParam), self.callback_type]
        initialize.restype = ctypes.c_int
        result = initialize(ctypes.byref(self.handle), ctypes.byref(params),
                            self.callback)
        if result != 0:
            raise RuntimeError(f'RKLLM 初始化失败，错误码 {result}')
        self.initialized_at = time.time()
        self.load_duration_seconds = self.initialized_at - load_started_at

        self.run_api = self.library.rkllm_run
        self.run_api.argtypes = [ctypes.c_void_p, ctypes.POINTER(RKLLMInput),
                                 ctypes.POINTER(RKLLMInferParam), ctypes.c_void_p]
        self.run_api.restype = ctypes.c_int
        self.set_template = self.library.rkllm_set_chat_template
        self.set_template.argtypes = [ctypes.c_void_p, ctypes.c_char_p,
                                      ctypes.c_char_p, ctypes.c_char_p]
        self.set_template.restype = ctypes.c_int
        self.destroy_api = self.library.rkllm_destroy
        self.destroy_api.argtypes = [ctypes.c_void_p]
        self.destroy_api.restype = ctypes.c_int

    def _callback(self, result, _userdata, state):
        if state == RKLLM_RUN_NORMAL and result and result.contents.text:
            if self.first_token_at is None:
                self.first_token_at = time.perf_counter()
            chunk = ctypes.string_at(result.contents.text)
            self.output.append(self.decoder.decode(chunk, final=False))
        elif state == RKLLM_RUN_FINISH and self.decoder is not None:
            self.output.append(self.decoder.decode(b'', final=True))
        elif state == RKLLM_RUN_ERROR:
            self.callback_error = 'RKLLM 回调报告推理错误'
        return 0

    def chat(self, system, prompt):
        """使用 Qwen2.5 模板执行单轮推理；历史已由上游压缩。"""
        with self.lock:
            system_template = (
                '<|im_start|>system\n' + str(system).strip()
                + '<|im_end|>\n')
            result = self.set_template(
                self.handle, system_template.encode('utf-8'),
                b'<|im_start|>user\n',
                b'<|im_end|>\n<|im_start|>assistant\n')
            if result != 0:
                raise RuntimeError(f'设置 Qwen chat template 失败，错误码 {result}')
            self.output = []
            self.decoder = codecs.getincrementaldecoder('utf-8')('replace')
            self.callback_error = ''
            self.first_token_at = None
            model_input = RKLLMInput()
            ctypes.memset(ctypes.byref(model_input), 0, ctypes.sizeof(model_input))
            model_input.role = b'user'
            model_input.enable_thinking = False
            model_input.input_type = RKLLM_INPUT_PROMPT
            model_input.input_data.prompt_input = str(prompt).encode('utf-8')
            infer = RKLLMInferParam()
            ctypes.memset(ctypes.byref(infer), 0, ctypes.sizeof(infer))
            infer.mode = RKLLM_INFER_GENERATE
            infer.keep_history = 0
            inference_started = time.perf_counter()
            result = self.run_api(
                self.handle, ctypes.byref(model_input), ctypes.byref(infer), None)
            inference_finished = time.perf_counter()
            self.last_metrics = {
                'first_token_ms': (None if self.first_token_at is None else
                                   round((self.first_token_at
                                          - inference_started) * 1000.0, 1)),
                'total_ms': round(
                    (inference_finished - inference_started) * 1000.0, 1),
            }
            if result != 0:
                raise RuntimeError(f'RKLLM 推理失败，错误码 {result}')
            if self.callback_error:
                raise RuntimeError(self.callback_error)
            return ''.join(self.output).strip()

    def close(self):
        if self.handle:
            self.destroy_api(self.handle)
            self.handle = None


class RequestHandler(BaseHTTPRequestHandler):
    """提供只监听回环地址的最小 OpenAI 兼容接口。"""

    model = None
    model_name = 'qwen2.5-3b-instruct-w8a8-rk3588'

    def log_message(self, message, *args):
        print('[RKLLM HTTP] ' + message % args, flush=True)

    def _json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/health':
            self._json(200, {
                'state': 'ok', 'model': self.model_name,
                'loaded_at': self.model.initialized_at,
                'load_duration_seconds': self.model.load_duration_seconds,
                'runtime_version': '1.2.1',
                'runtime_path': str(self.model.runtime_path),
                'process_pid': os.getpid(),
                'resident': True,
            })
        else:
            self._json(404, {'message': '接口不存在'})

    def do_POST(self):
        if self.path != '/v1/chat/completions':
            self._json(404, {'message': '接口不存在'})
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            payload = json.loads(self.rfile.read(length).decode('utf-8'))
            messages = payload.get('messages')
            if not isinstance(messages, list):
                raise ValueError('messages 必须是数组')
            system = ''
            users = []
            for message in messages:
                if not isinstance(message, dict):
                    continue
                if message.get('role') == 'system':
                    system = str(message.get('content', ''))
                elif message.get('role') == 'user':
                    users.append(str(message.get('content', '')))
            if not users:
                raise ValueError('至少需要一条 user 消息')
            answer = self.model.chat(system, users[-1])
            self._json(200, {
                'id': 'robot-rkllm', 'object': 'chat.completion',
                'model': self.model_name,
                'choices': [{'index': 0, 'message': {
                    'role': 'assistant', 'content': answer,
                }, 'finish_reason': 'stop'}],
                'robot_metrics': self.model.last_metrics,
            })
        except (ValueError, json.JSONDecodeError) as exc:
            self._json(400, {'message': str(exc)})
        except Exception as exc:
            self._json(500, {'message': str(exc)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--runtime', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9101)
    parser.add_argument('--context-length', type=int, default=4096)
    parser.add_argument('--max-new-tokens', type=int, default=96)
    args = parser.parse_args()
    RequestHandler.model = RKLLMTextModel(
        args.model, args.runtime, args.context_length, args.max_new_tokens)
    server = ThreadingHTTPServer((args.host, args.port), RequestHandler)
    try:
        print(f'RKLLM 文本服务：http://{args.host}:{args.port}', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        RequestHandler.model.close()


if __name__ == '__main__':
    main()
