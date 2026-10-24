"""作用：校验并复制 RTAB-Map SQLite 数据库；使用方法：快照保存及重定位启动时调用。"""

from contextlib import closing
from pathlib import Path
import sqlite3
import hashlib
import json


def file_digest(filename):
    with open(filename, 'rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def validate_bundle(map_yaml, database):
    """新快照校验地图和数据库成套关系，防止选错库或文件损坏。"""
    database = validate_database(database)
    manifest = Path(map_yaml).with_suffix('.json')
    if manifest.is_file():
        metadata = json.loads(manifest.read_text(encoding='utf-8'))
        if not isinstance(metadata, dict):
            raise ValueError(f'地图快照清单格式错误：{manifest}')
        digests = metadata.get('files_sha256', {})
        if not isinstance(digests, dict):
            raise ValueError(f'地图快照校验字段格式错误：{manifest}')
        if digests:
            for filename in (Path(map_yaml), Path(map_yaml).with_suffix('.pgm'), Path(database)):
                expected = digests.get(filename.name)
                if not expected or file_digest(filename) != expected:
                    raise ValueError(f'地图与数据库不配套或文件已变化：{filename}；请使用同一次保存的完整目录')
    return database


def validate_database(filename):
    path = Path(filename).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'缺少地图配套数据库：{path}；旧版仅有 PGM/PLY 的快照需重新建图保存')
    try:
        with closing(sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)) as connection:
            if connection.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
                raise ValueError(f'地图数据库完整性检查失败：{path}')
            count = connection.execute('SELECT COUNT(*) FROM Node WHERE id > 0').fetchone()[0]
            if count == 0:
                raise ValueError('地图数据库没有可用于重定位的节点，请先完成建图')
    except sqlite3.DatabaseError as exc:
        raise ValueError(f'不是有效的 RTAB-Map 地图数据库或文件已损坏：{path}') from exc
    return str(path)


def copy_database(source, destination):
    """使用 SQLite 备份接口保留事务一致性，不直接复制仍在写入的数据库文件。"""
    source = Path(validate_database(source))
    with closing(sqlite3.connect(source.as_uri() + '?mode=ro', uri=True)) as reader:
        with closing(sqlite3.connect(str(destination))) as writer:
            reader.backup(writer)
    validate_database(destination)
