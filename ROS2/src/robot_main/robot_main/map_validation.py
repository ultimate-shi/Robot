"""作用：在启动整车前验证静态地图文件及图像路径；使用方法：由 robot.launch.py 调用。"""

from pathlib import Path

import yaml


def validate_map_yaml(filename):
    """返回当前运行环境可读取的绝对路径；错误时在启动硬件前给出明确原因。"""
    path = Path(filename).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(
            f'地图 YAML 不存在或不可读取：{path}；请使用当前 ROS 运行环境中的路径，'
            '容器启动通常为 /workspace/maps/.../map.yaml')
    try:
        with path.open(encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'地图 YAML 无法读取或解析：{path}：{exc}') from exc
    if not isinstance(metadata, dict) or not isinstance(metadata.get('image'), str):
        raise RuntimeError(f'地图 YAML 缺少 image 路径：{path}')
    image = Path(metadata['image']).expanduser()
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise RuntimeError(f'地图图像不存在或不可读取：{image}；地图 YAML：{path}')
    return str(path)
