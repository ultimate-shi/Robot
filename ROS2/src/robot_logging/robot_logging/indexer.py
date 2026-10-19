"""使用方法：ros2 run robot_logging log_indexer --config-file 日志参数文件。"""

import argparse
import logging
import os
from pathlib import Path
import sqlite3
import time

from robot_logging.storage import KINDS, ensure_layout, load_config, sessions


SCHEMA = '''
CREATE TABLE IF NOT EXISTS messages (
  id INTEGER PRIMARY KEY,
  ts_ns INTEGER NOT NULL,
  category TEXT NOT NULL,
  name TEXT NOT NULL,
  type_name TEXT NOT NULL,
  node TEXT NOT NULL DEFAULT '',
  level INTEGER,
  summary TEXT NOT NULL,
  payload TEXT NOT NULL,
  bag_file TEXT NOT NULL,
  ordinal INTEGER NOT NULL,
  size_bytes INTEGER NOT NULL,
  UNIQUE(bag_file, ordinal)
);
CREATE INDEX IF NOT EXISTS idx_messages_time ON messages(ts_ns DESC);
CREATE INDEX IF NOT EXISTS idx_messages_filter ON messages(category, name, ts_ns DESC);
CREATE TABLE IF NOT EXISTS indexed_files (
  path TEXT PRIMARY KEY,
  size_bytes INTEGER NOT NULL,
  indexed_at INTEGER NOT NULL
);
'''

FTS_SCHEMA = '''
CREATE VIRTUAL TABLE IF NOT EXISTS messages_fts USING fts5(
  payload, content='messages', content_rowid='id',
  tokenize='trigram', detail='none', columnsize=0
);
CREATE TRIGGER IF NOT EXISTS messages_ai AFTER INSERT ON messages BEGIN
  INSERT INTO messages_fts(rowid, payload) VALUES (new.id, new.payload);
END;
CREATE TRIGGER IF NOT EXISTS messages_ad AFTER DELETE ON messages BEGIN
  INSERT INTO messages_fts(messages_fts, rowid, payload)
    VALUES ('delete', old.id, old.payload);
END;
CREATE TRIGGER IF NOT EXISTS messages_au AFTER UPDATE ON messages BEGIN
  INSERT INTO messages_fts(messages_fts, rowid, payload)
    VALUES ('delete', old.id, old.payload);
  INSERT INTO messages_fts(rowid, payload) VALUES (new.id, new.payload);
END;
'''


def connect(database):
    fresh = not Path(database).exists()
    conn = sqlite3.connect(database)
    if fresh:
        # 清理过期会话后归还页面给文件系统，而不是只留在 SQLite 空闲页中。
        conn.execute('PRAGMA auto_vacuum=INCREMENTAL')
        conn.execute('VACUUM')
    conn.execute('PRAGMA journal_mode=WAL')
    conn.execute('PRAGMA busy_timeout=5000')
    had_fts = conn.execute("SELECT 1 FROM sqlite_master WHERE name='messages_fts'").fetchone()
    conn.executescript(SCHEMA)
    if not had_fts and not fresh:
        # 旧索引把相同文本同时存入摘要和正文；升级时仅保留正文。
        with conn:
            conn.execute("UPDATE messages SET summary='' WHERE payload != ''")
    conn.executescript(FTS_SCHEMA)
    if not had_fts:
        with conn:
            conn.execute("INSERT INTO messages_fts(messages_fts) VALUES ('rebuild')")
    return conn


def category_name(topic):
    if topic.endswith('/_service_event'):
        return 'service', topic[:-len('/_service_event')]
    if '/_action/' in topic:
        return 'action', topic.split('/_action/', 1)[0]
    return 'topic', topic


def decode(schema, message, max_bytes):
    """只反序列化小消息，避免把图像和点云展开成庞大的文本。"""
    if len(message.data) > max_bytes or schema is None:
        return '', '', None
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        ros_type = get_message(schema.name)
        decoded = deserialize_message(message.data, ros_type)
        if schema.name == 'rcl_interfaces/msg/Log':
            return decoded.msg[:max_bytes], decoded.name, int(decoded.level)
        return str(decoded)[:max_bytes], '', None
    except (ImportError, LookupError, RuntimeError, ValueError, AttributeError) as exc:
        logging.debug('消息解码失败：%s', exc)
        return '', '', None


def index_file(conn, file_path, max_bytes):
    """单文件原子写入，重试或进程重启不会产生重复消息。"""
    from mcap.reader import make_reader

    file_path = Path(file_path).resolve()
    size = file_path.stat().st_size
    if conn.execute('SELECT 1 FROM indexed_files WHERE path=? AND size_bytes=?',
                    (str(file_path), size)).fetchone():
        return 0
    count = 0
    with file_path.open('rb') as stream, conn:
        conn.execute('DELETE FROM messages WHERE bag_file=?', (str(file_path),))
        reader = make_reader(stream)
        batch = []
        for ordinal, (schema, channel, message) in enumerate(reader.iter_messages()):
            category, name = category_name(channel.topic)
            payload, node, level = decode(schema, message, max_bytes)
            summary = '' if payload else f'{len(message.data)} B 原始消息；请下载 MCAP 回放'
            batch.append((
                int(message.log_time), category, name,
                schema.name if schema is not None else '', node, level,
                summary, payload, str(file_path), ordinal, len(message.data)))
            if len(batch) >= 500:
                conn.executemany('''INSERT OR IGNORE INTO messages
                    (ts_ns, category, name, type_name, node, level, summary,
                     payload, bag_file, ordinal, size_bytes)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', batch)
                count += len(batch)
                batch.clear()
        if batch:
            conn.executemany('''INSERT OR IGNORE INTO messages
                (ts_ns, category, name, type_name, node, level, summary,
                 payload, bag_file, ordinal, size_bytes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', batch)
            count += len(batch)
        conn.execute('''INSERT OR REPLACE INTO indexed_files
            (path, size_bytes, indexed_at) VALUES (?, ?, ?)''',
                     (str(file_path), size, int(time.time())))
    return count


def sync_deleted(conn):
    missing = [row[0] for row in conn.execute('SELECT path FROM indexed_files')
               if not Path(row[0]).is_file()]
    if not missing:
        return
    with conn:
        conn.executemany('DELETE FROM messages WHERE bag_file=?',
                         [(path,) for path in missing])
        conn.executemany('DELETE FROM indexed_files WHERE path=?',
                         [(path,) for path in missing])
    conn.execute('PRAGMA incremental_vacuum')
    conn.execute('PRAGMA wal_checkpoint(TRUNCATE)')


def scan(conn, config):
    root = config['root']
    database = Path(conn.execute('PRAGMA database_list').fetchone()[2])
    for kind in KINDS:
        for mode in ('real', 'virtual'):
            for session in (root / kind / mode).glob('*'):
                if not session.is_dir() or (session / '.active').exists():
                    continue
                for file_path in sorted(session.glob('**/*.mcap')):
                    if database.stat().st_size >= int(config['index_max_bytes']):
                        logging.warning('日志索引达到配额，暂停新增索引：%s', database)
                        sync_deleted(conn)
                        return
                    try:
                        count = index_file(conn, file_path,
                                           int(config['index_max_message_bytes']))
                        if count:
                            logging.info('已索引 %s：%d 条', file_path, count)
                    except (OSError, ValueError, RuntimeError) as exc:
                        logging.warning('索引失败 %s：%s', file_path, exc)
    sync_deleted(conn)


def main():
    parser = argparse.ArgumentParser(description='索引已关闭的 MCAP 片段')
    parser.add_argument('--config-file', required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    os.umask(0o002)
    config = load_config(args.config_file)
    if config.get('require_mount', False) and not os.path.ismount(config['root']):
        raise RuntimeError(f"日志目录没有挂载到容器：{config['root']}")
    ensure_layout(config['root'])
    conn = connect(config['root'] / 'index' / 'logs.sqlite3')
    try:
        while True:
            scan(conn, config)
            time.sleep(float(config['poll_seconds']))
    except KeyboardInterrupt:
        # launch 退出时，等待录制器写完 MCAP 页脚并移除活动标记，再索引最后一段。
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline and any(
                (path / '.active').exists() for path in sessions(config['root'])):
            time.sleep(0.5)
        scan(conn, config)
    finally:
        conn.close()
