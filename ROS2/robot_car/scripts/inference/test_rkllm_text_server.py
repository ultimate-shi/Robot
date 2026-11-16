"""使用方法：pytest 运行本文件，验证 RKLLM 只初始化一次并复用。"""

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import rkllm_text_server as server  # noqa: E402


class FakeFunction:
    def __init__(self, callback):
        self.callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.callback(*args)


class FakeLibrary:
    def __init__(self):
        self.init_count = 0
        self.run_count = 0
        self.template = None
        self.model_callback = None
        self.rkllm_createDefaultParam = FakeFunction(
            lambda: server.RKLLMParam())
        self.rkllm_init = FakeFunction(self._initialize)
        self.rkllm_run = FakeFunction(self._run)
        self.rkllm_set_chat_template = FakeFunction(self._set_template)
        self.rkllm_destroy = FakeFunction(lambda _handle: 0)

    def _initialize(self, _handle, _params, callback):
        self.init_count += 1
        self.model_callback = callback
        return 0

    def _set_template(self, _handle, system, prefix, postfix):
        self.template = (system, prefix, postfix)
        return 0

    def _run(self, _handle, _model_input, _infer, _userdata):
        self.run_count += 1
        result = server.RKLLMResult()
        result.text = b'{"answer":"ok","action":null}'
        self.model_callback(
            server.ctypes.pointer(result), None, server.RKLLM_RUN_NORMAL)
        self.model_callback(
            server.ctypes.pointer(result), None, server.RKLLM_RUN_FINISH)
        return 0


def test_model_is_initialized_once_and_reused(monkeypatch):
    library = FakeLibrary()
    monkeypatch.setattr(server.ctypes, 'CDLL', lambda _path: library)
    model = server.RKLLMTextModel('/model.rkllm', '/librkllmrt.so')

    first = model.chat('system-rule', 'first')
    second = model.chat('system-rule', 'second')

    assert first == '{"answer":"ok","action":null}'
    assert second == first
    assert library.init_count == 1
    assert library.run_count == 2
    assert library.template[0].decode().startswith(
        '<|im_start|>system\nsystem-rule')
