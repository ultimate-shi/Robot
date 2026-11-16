"""作用：验证持久数据库的完整性、事务复制和成套校验；使用方法：pytest -q 本文件。"""

import json
import sqlite3

import pytest
from robot_navigation.mapping.database import copy_database, file_digest, validate_bundle, validate_database


def make_database(path):
    with sqlite3.connect(path) as connection:
        connection.execute('CREATE TABLE Node (id INTEGER PRIMARY KEY)')
        connection.execute('INSERT INTO Node VALUES (1)')
    return path


def test_database_copy_and_invalid_database_fail_closed(tmp_path):
    source = make_database(tmp_path / 'map.db')
    target = tmp_path / 'copy.db'
    copy_database(source, target)
    assert validate_database(target) == str(target)
    with sqlite3.connect(target) as connection:
        assert connection.execute('SELECT id FROM Node').fetchall() == [(1,)]
    with pytest.raises(ValueError, match='缺少'):
        validate_database(tmp_path / 'missing.db')
    with sqlite3.connect(source) as connection:
        connection.execute('DELETE FROM Node')
    with pytest.raises(ValueError, match='没有'):
        validate_database(source)
    target.write_bytes(b'broken database')
    with pytest.raises((ValueError, sqlite3.DatabaseError)):
        validate_database(target)


def test_new_bundle_rejects_modified_or_mismatched_database(tmp_path):
    database = make_database(tmp_path / 'map.db')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\n')
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'PGM')
    metadata = {'files_sha256': {path.name: file_digest(path)
                               for path in (database, map_yaml, image)}}
    (tmp_path / 'map.json').write_text(json.dumps(metadata))
    assert validate_bundle(map_yaml, database) == str(database)
    with sqlite3.connect(database) as connection:
        connection.execute('INSERT INTO Node VALUES (2)')
    with pytest.raises(ValueError, match='不配套'):
        validate_bundle(map_yaml, database)
