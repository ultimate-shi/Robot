"""使用方法：在 Jazzy 容器运行 pytest -q src/robot_logging/test/test_logging_pipeline.py。"""

import sqlite3
import time

from mcap.writer import Writer
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import yaml

from robot_logging.indexer import SCHEMA, category_name, connect, index_file, scan, sync_deleted
from robot_logging.storage import (
    ensure_layout, new_session, finish_session, prune, recover_abandoned,
)
from robot_logging.web import create_app


def test_mcap_index_idempotent_and_deleted_file(tmp_path):
    ensure_layout(tmp_path)
    session = new_session(tmp_path, 'routine', 'virtual')
    bag = session / 'small.mcap'
    with bag.open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'string data\n')
        channel = writer.register_channel('/mission/status', 'cdr', schema)
        writer.add_message(channel, 1234567890000000000,
                           serialize_message(String(data='测试导航失败')),
                           1234567890000000000)
        writer.finish()
    db = connect(tmp_path / 'index' / 'logs.sqlite3')
    config = {'root': tmp_path, 'index_max_message_bytes': 16384,
              'index_max_bytes': 1000000}
    scan(db, config)
    assert db.execute('SELECT count(*) FROM messages').fetchone()[0] == 0
    finish_session(session)
    scan(db, config)
    assert db.execute('SELECT count(*) FROM messages').fetchone()[0] == 1
    assert index_file(db, bag, 16384) == 0
    row = db.execute('SELECT name, payload FROM messages').fetchone()
    assert row[0] == '/mission/status'
    assert '测试导航失败' in row[1]
    bag.unlink()
    sync_deleted(db)
    assert db.execute('SELECT count(*) FROM messages').fetchone()[0] == 0
    db.close()


def test_retention_never_deletes_active_session(tmp_path):
    ensure_layout(tmp_path)
    old = new_session(tmp_path, 'routine', 'real')
    active = new_session(tmp_path, 'incidents', 'real')
    finish_session(old)
    past = time.time() - 9 * 86400
    import os
    os.utime(old, (past, past))
    os.utime(active, (past, past))
    config = {'root': tmp_path, 'retention_days': 7,
              'max_total_bytes': 5000000000, 'min_free_bytes': 0}
    prune(config, [active])
    assert not old.exists()
    assert active.exists()


def test_quota_removes_oldest_completed_session(tmp_path):
    ensure_layout(tmp_path)
    old = new_session(tmp_path, 'routine', 'virtual')
    (old / 'data').write_bytes(b'x' * 80)
    finish_session(old)
    newer = new_session(tmp_path, 'routine', 'virtual')
    (newer / 'data').write_bytes(b'y' * 80)
    finish_session(newer)
    import os
    past = time.time() - 60
    os.utime(old, (past, past))
    config = {'root': tmp_path, 'retention_days': 7,
              'max_total_bytes': 200, 'min_free_bytes': 0}
    prune(config)
    assert not old.exists()
    assert newer.exists()


def test_standard_event_category():
    assert category_name('/mission/confirm/_service_event') == (
        'service', '/mission/confirm')
    assert category_name('/mission/plan/_action/feedback') == (
        'action', '/mission/plan')


def test_recover_abandoned_session(tmp_path):
    ensure_layout(tmp_path)
    stale = new_session(tmp_path, 'routine', 'virtual')
    assert recover_abandoned(tmp_path) == [stale]
    assert not (stale / '.active').exists()


def test_cursor_pagination_and_keyword_index(tmp_path):
    ensure_layout(tmp_path)
    config_file = tmp_path / 'logging.yaml'
    config_file.write_text(yaml.safe_dump({'logging': {'root': str(tmp_path)}}))
    conn = connect(tmp_path / 'index' / 'logs.sqlite3')
    with conn:
        conn.executemany('''INSERT INTO messages
            (ts_ns, category, name, type_name, summary, payload, bag_file,
             ordinal, size_bytes) VALUES (?, 'topic', '/status', 'std_msgs/msg/String',
             '', ?, 'bag.mcap', ?, 10)''',
            [(1000 + i // 2, '测试导航失败 100% 完成' if i == 17 else '正常', i)
             for i in range(205)])
    app = create_app(config_file)
    endpoint = next(route.endpoint for route in app.routes if route.path == '/api/logs')

    def query(before=None, keyword=''):
        return endpoint(
            start_ns=None, end_ns=None, topic='', service='', action='',
            keyword=keyword, limit=100,
            before_id=before['before_id'] if before else None,
            before_ts_ns=before['before_ts_ns'] if before else None,
            page=1, page_size=100, cursor_mode=True)

    first = query()
    second = query(first['next_cursor'])
    third = query(second['next_cursor'])
    assert [len(page['items']) for page in (first, second, third)] == [100, 100, 5]
    assert third['next_cursor'] is None
    assert len({item['id'] for page in (first, second, third)
                for item in page['items']}) == 205
    assert first['total'] is None
    assert len(query(keyword='导航失败')['items']) == 1
    assert len(query(keyword='错误')['items']) == 0
    assert len(query(keyword='100%')['items']) == 1
    assert query(keyword='正常')['items'][0]['summary'] == '正常'
    with conn:
        conn.execute('DELETE FROM messages WHERE ordinal=17')
    assert not query(keyword='导航失败')['items']
    conn.close()


def test_existing_index_migrates_to_fts(tmp_path):
    ensure_layout(tmp_path)
    database = tmp_path / 'index' / 'logs.sqlite3'
    legacy = sqlite3.connect(database)
    legacy.executescript(SCHEMA)
    legacy.execute('''INSERT INTO messages
        (ts_ns, category, name, type_name, summary, payload, bag_file,
         ordinal, size_bytes) VALUES (1, 'topic', '/status', 'std_msgs/msg/String',
         '导航发生错误', '导航发生错误', 'bag.mcap', 0, 10)''')
    legacy.commit()
    legacy.close()
    upgraded = connect(database)
    assert upgraded.execute('SELECT summary FROM messages').fetchone()[0] == ''
    assert upgraded.execute("SELECT count(*) FROM messages_fts WHERE payload LIKE '%发生错%'").fetchone()[0] == 1
    upgraded.close()
