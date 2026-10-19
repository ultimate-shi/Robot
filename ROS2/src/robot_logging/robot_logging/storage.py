"""使用方法：采集器和索引器调用本模块维护会话目录与磁盘配额。"""

import json
import os
from pathlib import Path
import shutil
import time
from uuid import uuid4

import yaml


KINDS = ('routine', 'incidents')


def load_config(path):
    """读取日志参数，拒绝会导致误删其他路径的根目录。"""
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)['logging']
    original_root = Path(config['root'])
    if not original_root.is_absolute():
        raise ValueError('日志根目录必须是独立的绝对路径')
    root = original_root.resolve()
    if not root.is_absolute() or str(root) in ('/', '/var', '/var/lib'):
        raise ValueError('日志根目录必须是独立的绝对路径')
    config['root'] = root
    return config


def ensure_layout(root):
    root = Path(root)
    for part in (*KINDS, 'index'):
        (root / part).mkdir(parents=True, exist_ok=True)


def new_session(root, kind, mode, reason=''):
    """创建独立 bag 目录的父目录；rosbag2 自己创建其中的 bag 目录。"""
    if kind not in KINDS or mode not in ('real', 'virtual'):
        raise ValueError('无效的日志类别或运行模式')
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    path = Path(root) / kind / mode / f'{stamp}_{uuid4().hex[:8]}'
    path.mkdir(parents=True)
    (path / '.active').touch()
    (path / 'session.json').write_text(json.dumps({
        'kind': kind, 'mode': mode, 'reason': reason,
        'started_at': time.time(),
    }, ensure_ascii=False), encoding='utf-8')
    return path


def finish_session(path):
    marker = Path(path) / '.active'
    marker.unlink(missing_ok=True)


def directory_bytes(path):
    total = 0
    for base, _, names in os.walk(path):
        for name in names:
            try:
                total += (Path(base) / name).stat().st_size
            except FileNotFoundError:
                pass
    return total


def sessions(root):
    for kind in KINDS:
        for mode in ('real', 'virtual'):
            for path in (Path(root) / kind / mode).glob('*'):
                if path.is_dir():
                    yield path


def recover_abandoned(root):
    """整车重启时关闭无录制进程的会话标记，使其可以索引和按期清理。"""
    active_bags = set()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            command = (entry / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for arg in command:
            if arg.endswith(b'/bag'):
                active_bags.add(arg.decode('utf-8', errors='replace'))
    recovered = []
    for path in sessions(root):
        if (path / '.active').exists() and str(path / 'bag') not in active_bags:
            finish_session(path)
            recovered.append(path)
    return recovered


def space_ok(config, required_bytes=0):
    root = config['root']
    return (directory_bytes(root) + required_bytes <= int(config['max_total_bytes'])
            and shutil.disk_usage(root).free - required_bytes >= int(config['min_free_bytes']))


def prune(config, active=()):
    """先清过期会话，再按时间清最旧完整会话；绝不删除活动会话。"""
    root = config['root']
    active = {Path(path).resolve() for path in active}
    now = time.time()
    removed = []
    available = sorted(
        (path for path in sessions(root)
         if path.resolve() not in active and not (path / '.active').exists()),
        key=lambda path: path.stat().st_mtime)
    for path in available:
        expired = now - path.stat().st_mtime > int(config['retention_days']) * 86400
        if expired or not space_ok(config):
            shutil.rmtree(path)
            removed.append(path)
    return removed
