"""作用：验证保存调用内存落盘、恢复建图及原子公开完整地图；使用方法：pytest -q 本文件。"""

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import sqlite3
import struct
import os
import stat

import pytest
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from robot_navigation.mapping.database import copy_database, validate_bundle
from robot_navigation.mapping.snapshot_manager import SnapshotManager, snapshot_identity, apply_snapshot_permissions


class Service:
    def __init__(self, name, events, callback=lambda: SimpleNamespace()):
        self.srv_name, self.events, self.callback = name, events, callback

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.events.append(self.srv_name)
        future = Future()
        try:
            future.set_result(self.callback())
        except Exception as exc:
            future.set_exception(exc)
        return future


def manager(tmp_path, fail_backup=False):
    node = SnapshotManager.__new__(SnapshotManager)
    node.database_path = str(tmp_path / 'active.db')
    with sqlite3.connect(node.database_path) as connection:
        connection.execute('CREATE TABLE Node (id INTEGER PRIMARY KEY)')
        connection.execute('INSERT INTO Node VALUES (1)')
    node.database_timeout = node.resume_timeout = 1.0
    node.backup_future = None
    node.save_directory = str(tmp_path)
    node.snapshot_timezone = 'Asia/Shanghai'
    node.snapshot_uid, node.snapshot_gid = os.getuid(), os.getgid()
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 0.05
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, 100, -1, 0]
    node.latest_map = grid
    node.latest_cloud = PointCloud2()
    cloud = node.latest_cloud
    cloud.header.frame_id = 'map'
    cloud.width = cloud.height = 1
    cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.point_step = cloud.row_step = 12
    cloud.data = struct.pack('<fff', 1.0, 2.0, 0.2)
    events = []
    node.status_pub = SimpleNamespace(publish=lambda msg: None)
    node.pause_client = Service('pause', events)
    node.resume_client = Service('resume', events)
    node.map_client = Service('get_map', events, lambda: SimpleNamespace(map=grid))

    def backup():
        if fail_backup:
            raise ValueError('备份失败')
        copy_database(node.database_path, node.database_path + '.back')
        return SimpleNamespace()

    node.backup_client = Service('backup', events, backup)
    return node, events


@pytest.mark.parametrize('fail_backup', [False, True])
def test_database_save_is_atomic_and_always_resumes_mapping(tmp_path, fail_backup):
    node, events = manager(tmp_path, fail_backup)
    response = node._save_snapshot(None, Trigger.Response())
    assert events == ['pause', 'get_map', 'backup', 'resume']
    directories = list(tmp_path.glob('map_*'))
    assert response.success is not fail_backup
    if fail_backup:
        assert not directories
        assert '备份失败' in response.message
    else:
        assert len(directories) == 1
        directory = directories[0]
        assert {p.suffix for p in directory.iterdir()} == {'.yaml', '.pgm', '.ply', '.json', '.db'}
        assert validate_bundle(directory / 'map.yaml', directory / 'map.db')
        assert Path(response.message).is_file()
        assert stat.S_IMODE(directory.stat().st_mode) == 0o755
        for item in directory.iterdir():
            assert stat.S_IMODE(item.stat().st_mode) == 0o644
            assert item.stat().st_uid == node.snapshot_uid
            assert item.stat().st_gid == node.snapshot_gid


def test_permissions_give_current_user_write_access_without_world_write(tmp_path):
    directory = tmp_path / 'map'
    directory.mkdir(mode=0o700)
    path = directory / 'map.yaml'
    path.write_text('image: map.pgm\n')
    path.chmod(0o600)
    apply_snapshot_permissions(directory, os.getuid(), os.getgid())
    assert stat.S_IMODE(directory.stat().st_mode) == 0o755
    assert stat.S_IMODE(path.stat().st_mode) == 0o644


def test_container_root_uses_workspace_owner_and_explicit_ids_override(monkeypatch):
    monkeypatch.setattr(os, 'getuid', lambda: 0)
    monkeypatch.setattr(os, 'getgid', lambda: 0)
    monkeypatch.setattr(os.path, 'exists', lambda _: True)
    monkeypatch.setattr(os, 'stat', lambda path: SimpleNamespace(
        st_uid=1234 if path == '/workspace' else 0,
        st_gid=2345 if path == '/workspace' else 0))
    assert snapshot_identity('/workspace/maps') == (1234, 2345)
    assert snapshot_identity('/workspace/maps', 3456, 4567) == (3456, 4567)
