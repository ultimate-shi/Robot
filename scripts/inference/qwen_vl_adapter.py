#!/usr/bin/env python3
"""使用方法：由 brain_inference_server.py 调用，把 VLM_NPU 封装为视觉问答。"""

import base64
import binascii
import os
from pathlib import Path
import re
import subprocess
import tempfile


class LocalQwenRunner:
    """为每张最新画面启动一次 Qwen，避免复用只绑定启动图片的旧进程。"""

    def __init__(self):
        script_dir = Path(__file__).resolve().parent
        repo_dir = script_dir.parent.parent
        qwen_dir = Path(os.environ.get(
            'ROBOT_QWEN_DIR', repo_dir / 'model' / 'qwen2.5-vl'))
        self.vision_model = Path(os.environ.get(
            'ROBOT_QWEN_VISION_MODEL',
            qwen_dir / 'qwen2.5-vl-3b_vision_rk3588.rknn'))
        self.llm_model = Path(os.environ.get(
            'ROBOT_QWEN_LLM_MODEL',
            qwen_dir / 'qwen2.5-vl-3b-instruct_w8a8_rk3588.rkllm'))
        self.executable = Path(os.environ.get(
            'ROBOT_QWEN_EXECUTABLE', qwen_dir / 'bin' / 'VLM_NPU'))
        self.runtime_dir = Path(os.environ.get(
            'ROBOT_QWEN_RUNTIME_DIR', qwen_dir / 'lib'))
        self.max_new_tokens = int(os.environ.get(
            'ROBOT_QWEN_MAX_NEW_TOKENS', '128'))
        self.context_length = int(os.environ.get(
            'ROBOT_QWEN_CONTEXT_LENGTH', '4096'))
        self.timeout = float(os.environ.get('ROBOT_QWEN_TIMEOUT', '180'))
        self.model_name = os.environ.get(
            'ROBOT_QWEN_MODEL_NAME', 'qwen2.5-vl-3b-w8a8-local')

    @property
    def available(self):
        """检查可执行程序、两个模型和私有 Runtime 是否全部存在。"""
        required = (
            self.executable, self.vision_model, self.llm_model,
            self.runtime_dir / 'librkllmrt.so',
            self.runtime_dir / 'librknnrt.so',
        )
        return all(path.is_file() and path.stat().st_size > 0
                   for path in required) and os.access(self.executable, os.X_OK)

    def chat(self, text, image_base64):
        """运行一次图文问答；使用参数数组和标准输入，不经过 shell。"""
        if not self.available:
            raise RuntimeError('本地 Qwen 模型、Runtime 或 VLM_NPU 不完整')
        if not image_base64:
            raise RuntimeError('尚未收到右目画面，Qwen 暂时无法开始对话')
        try:
            image = base64.b64decode(image_base64, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise RuntimeError('Qwen 输入图像的 base64 无效') from exc
        if not image:
            raise RuntimeError('Qwen 输入图像为空')

        prompt = str(text).strip()
        if '<image>' not in prompt:
            prompt = '<image>\n' + prompt
        process_env = os.environ.copy()
        previous_library_path = process_env.get('LD_LIBRARY_PATH', '')
        process_env['LD_LIBRARY_PATH'] = str(self.runtime_dir) + (
            ':' + previous_library_path if previous_library_path else '')
        process_env.setdefault('RKLLM_LOG_LEVEL', '1')

        with tempfile.TemporaryDirectory(prefix='robot-qwen-') as temp_dir:
            image_path = Path(temp_dir) / 'right_rect.jpg'
            image_path.write_bytes(image)
            command = [
                str(self.executable), str(image_path),
                str(self.vision_model), str(self.llm_model),
                str(self.max_new_tokens), str(self.context_length),
            ]
            try:
                completed = subprocess.run(
                    command, input=prompt + '\nexit\n',
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding='utf-8', errors='replace',
                    timeout=self.timeout, env=process_env, check=False)
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError(
                    f'本地 Qwen 推理超过 {self.timeout:.0f} 秒') from exc
        if completed.returncode != 0:
            tail = completed.stdout.strip()[-500:]
            raise RuntimeError(
                f'本地 Qwen 退出码 {completed.returncode}: {tail}')
        return self.extract_answer(completed.stdout)

    @staticmethod
    def extract_answer(output):
        """从 Runtime 日志中提取 Answer 区段，保留模型生成的多行 JSON。"""
        pattern = re.compile(
            r'(?:^|\n)User:\s*Answer:\s*(.*?)'
            r'(?=\nI rkllm:|\nUser:|\Z)', re.DOTALL)
        answers = [match.group(1).strip() for match in pattern.finditer(output)]
        answers = [answer for answer in answers if answer]
        if not answers:
            raise RuntimeError('本地 Qwen 输出中没有找到 Answer 区段')
        answer = answers[-1]
        if answer.startswith('```') and answer.endswith('```'):
            lines = answer.splitlines()
            answer = '\n'.join(lines[1:-1]).strip()
        return answer
