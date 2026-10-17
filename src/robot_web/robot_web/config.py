"""作用：读取统一网页入口配置；使用方法：由网页入口和推理进程共同调用。"""

import os
from pathlib import Path

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'web.yaml'


def load_config(path=None):
    config_path = Path(path or os.environ.get('ROBOT_WEB_CONFIG', DEFAULT_CONFIG))
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)['web']
    if config['host'] not in ('127.0.0.1', '0.0.0.0'):
        raise ValueError('网页入口只能监听本机或 IPv4 网卡')
    port = int(config['port'])
    if not 1 <= port <= 65535:
        raise ValueError('网页入口端口无效')
    socket_name = config['inference_socket_name']
    if not socket_name or Path(socket_name).name != socket_name:
        raise ValueError('推理 Socket 名称只能是文件名')
    runtime_dir = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'))
    config['port'] = port
    config['inference_socket'] = runtime_dir / socket_name
    log_config = Path(config['log_config'])
    config['log_config'] = (log_config if log_config.is_absolute() else
                            config_path.parent / log_config).resolve()
    config['request_timeout_seconds'] = float(config['request_timeout_seconds'])
    config['chat_timeout_seconds'] = float(config['chat_timeout_seconds'])
    config['health_timeout_seconds'] = float(config['health_timeout_seconds'])
    return config
