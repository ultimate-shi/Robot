"""作用：验证异常恢复不会破坏原库或公开半成品；用法：Jazzy 环境 pytest -q 本文件。"""

from concurrent.futures import Future
import fcntl
import json
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest
from std_srvs.srv import Trigger

from robot_navigation.mapping.database import file_digest, validate_bundle
from robot_navigation.mapping.recovery import assert_databases_closed, load_options, recover_snapshot
from robot_navigation.mapping.snapshot_manager import SnapshotManager


def database(path, stamp=1, nodes=1):
    with sqlite3.connect(path) as connection:
        connection.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY, stamp FLOAT)')
        connection.executemany('INSERT INTO Node VALUES (?, ?)', [(i + 1, stamp) for i in range(nodes)])
    return path


@pytest.fixture
def options(tmp_path):
    values = load_options(Path(__file__).parents[1] / 'config' / 'mapping_recovery.yaml')
    # 真正调用子进程，模拟导出文件及退出码，覆盖临时目录和超时边界。
    exporter = tmp_path / 'exporter'
    exporter.write_text('#!/usr/bin/env python3\n'
                        'import pathlib, sys\n'
                        'p = pathlib.Path(sys.argv[sys.argv.index("--output_dir") + 1])\n'
                        '(p/"map.pgm").write_bytes(b"P5\\n2 2\\n255\\n" + bytes([0,254,205,254]))\n'
                        '(p/"map.yaml").write_text("image: map.pgm\\nresolution: 0.02\\norigin: [0, 0, 0]\\n")\n')
    exporter.chmod(0o755)
    values['export_executable'] = str(exporter)
    values['minimum_free_bytes'] = 0
    return values


def test_recovers_latest_valid_database_without_mutating_sources(tmp_path, options):
    source = database(tmp_path / 'active.db', stamp=10)
    backup = database(tmp_path / 'active.db.back', stamp=20, nodes=2)
    digests = [file_digest(p) for p in (source, backup)]
    result = Path(recover_snapshot(source, tmp_path / 'maps', tmp_path / 'preview', options))
    assert validate_bundle(result, result.with_suffix('.db'))
    metadata = json.loads(result.with_suffix('.json').read_text())
    assert metadata['source_database'] == str(backup)
    assert metadata['node_count'] == 2
    assert metadata['cloud_available'] is False
    assert not result.with_suffix('.ply').exists()
    assert [file_digest(p) for p in (source, backup)] == digests


def test_corrupt_active_database_falls_back_to_valid_backup(tmp_path, options):
    source = tmp_path / 'active.db'
    source.write_bytes(b'corrupt')
    database(tmp_path / 'active.db.back')
    result = Path(recover_snapshot(source, tmp_path / 'maps', tmp_path, options))
    assert json.loads(result.with_suffix('.json').read_text())['source_validation_errors']
    assert source.read_bytes() == b'corrupt'


def test_failed_export_has_no_public_or_partial_bundle(tmp_path, options):
    source = database(tmp_path / 'active.db')
    options['export_executable'] = '/bin/false'
    with pytest.raises(ValueError, match='导出失败'):
        recover_snapshot(source, tmp_path / 'maps', tmp_path, options)
    assert not list((tmp_path / 'maps').glob('map_*'))
    assert not list((tmp_path / 'maps').glob('.mapping-recovery-*'))


def test_timeout_cleans_up_export_and_preserves_source(tmp_path, options):
    source = database(tmp_path / 'active.db')
    before = file_digest(source)
    Path(options['export_executable']).write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(5)\n')
    options['export_timeout'] = 0.05
    with pytest.raises(ValueError, match='导出失败'):
        recover_snapshot(source, tmp_path / 'maps', tmp_path, options)
    assert file_digest(source) == before
    assert not list((tmp_path / 'maps').glob('map_*'))


def test_running_database_is_rejected(tmp_path):
    source = database(tmp_path / 'active.db')
    process = subprocess.Popen([sys.executable, '-u', '-c',
        'import sqlite3, sys; c=sqlite3.connect(sys.argv[1]); c.execute("SELECT * FROM Node").fetchall(); '
        'print("ready", flush=True); sys.stdin.read()', str(source)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    try:
        assert process.stdout.readline().strip() == b'ready'
        with pytest.raises(ValueError, match='仍被进程'):
            assert_databases_closed([source])
    finally:
        process.communicate(timeout=5)


def test_concurrent_recovery_and_insufficient_space_fail_closed(tmp_path, options, monkeypatch):
    source = database(tmp_path / 'active.db')
    root = tmp_path / 'maps'
    root.mkdir()
    with open(root / '.mapping_recovery.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(ValueError, match='已有地图恢复'):
            recover_snapshot(source, root, tmp_path, options)
    options['minimum_free_bytes'] = 10**30
    with pytest.raises(ValueError, match='磁盘空间不足'):
        recover_snapshot(source, root, tmp_path, options)
    assert not list(root.glob('map_*'))


def test_save_service_falls_back_only_when_all_online_services_are_gone():
    from types import SimpleNamespace
    node = SnapshotManager.__new__(SnapshotManager)
    client = SimpleNamespace(service_is_ready=lambda: False)
    node.pause_client = node.resume_client = node.map_client = node.backup_client = client
    node._recover_snapshot = lambda request, response: Trigger.Response(success=True, message='recovered')
    assert node._save_snapshot(None, Trigger.Response()).message == 'recovered'
    node.pause_client = SimpleNamespace(service_is_ready=lambda: True)
    node.snapshot_timezone = 'Asia/Shanghai'
    node.save_directory = '/tmp'
    node._handle_snapshot = lambda *args, **kwargs: Trigger.Response(success=False, message='online')
    assert node._save_snapshot(None, Trigger.Response()).message == 'online'


def test_dead_process_pending_future_does_not_block_recovery(tmp_path, options, monkeypatch):
    from types import SimpleNamespace
    node = SnapshotManager.__new__(SnapshotManager)
    client = SimpleNamespace(service_is_ready=lambda: False)
    node.pause_client = node.resume_client = node.map_client = node.backup_client = client
    node.backup_future = Future()
    node._publish_status = lambda *args: None
    node.latest_map = node.latest_cloud = None
    node.database_path = str(database(tmp_path / 'active.db'))
    node.save_directory = str(tmp_path / 'maps')
    node.preview_directory = str(tmp_path / 'preview')
    node.recovery_config_file = 'test'
    monkeypatch.setattr('robot_navigation.mapping.recovery.load_options', lambda _: options)
    response = node._recover_snapshot(None, Trigger.Response())
    assert response.success, response.message
    assert validate_bundle(response.message, Path(response.message).with_suffix('.db'))


@pytest.mark.parametrize('matching', [True, False])
def test_cached_cloud_requires_matching_map_and_database(tmp_path, options, matching):
    source = database(tmp_path / 'active.db')
    preview = tmp_path / 'preview'
    preview.mkdir()
    (preview / 'current.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0,254,205,254] if matching else [254]*4))
    (preview / 'current.yaml').write_text('image: current.pgm\nresolution: 0.02\norigin: [0, 0, 0]\n')
    (preview / 'current.json').write_text(json.dumps({'database_path': str(source)}))
    (preview / 'current.ply').write_bytes(b'ply\nformat ascii 1.0\nelement vertex 0\nend_header\n')
    result = Path(recover_snapshot(source, tmp_path / 'maps', preview, options))
    assert result.with_suffix('.ply').exists() is matching
    assert json.loads(result.with_suffix('.json').read_text())['cloud_available'] is matching


def test_live_service_blocks_offline_recovery():
    from types import SimpleNamespace
    node = SnapshotManager.__new__(SnapshotManager)
    node.pause_client = node.resume_client = node.map_client = node.backup_client = SimpleNamespace(service_is_ready=lambda: True)
    node._publish_status = lambda *args: None
    response = node._recover_snapshot(None, Trigger.Response())
    assert not response.success
    assert '仍在线' in response.message


def test_cloud_export_uses_separate_command_without_save_in_db(tmp_path, options):
    source = database(tmp_path / 'active.db')
    exporter = Path(options['export_executable'])
    exporter.write_text(exporter.read_text() +
                        'if "--cloud" in sys.argv:\n'
                        '    assert "--save_in_db" not in sys.argv\n'
                        '    (p/"map_cloud.ply").write_text("ply\\nformat ascii 1.0\\nelement vertex 0\\nend_header\\n")\n')
    options['export_cloud'] = True
    result = Path(recover_snapshot(source, tmp_path / 'maps', tmp_path / 'preview', options))
    assert result.with_suffix('.ply').is_file()
    assert not (result.parent / 'map_cloud.ply').exists()
    assert json.loads(result.with_suffix('.json').read_text())['cloud_source'] == 'database'
