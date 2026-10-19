"""使用方法：由推理网关挂载，在 9100/log/ 提供只读日志查询。"""

from contextlib import contextmanager
from pathlib import Path
import sqlite3

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, HTMLResponse

from robot_logging.storage import load_config


def create_app(config_file):
    config = load_config(config_file)
    database = config['root'] / 'index' / 'logs.sqlite3'
    html_path = Path(__file__).resolve().parent / 'web' / 'index.html'
    app = FastAPI(title='机器人日志查询', docs_url=None, redoc_url=None)

    @contextmanager
    def read_db():
        if not database.is_file():
            raise HTTPException(503, '日志索引尚未生成')
        conn = sqlite3.connect(f'file:{database}?mode=ro', uri=True)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
        finally:
            conn.close()

    @app.get('/', response_class=HTMLResponse)
    def home():
        return html_path.read_text(encoding='utf-8')

    @app.get('/api/logs')
    def logs(start_ns: int | None = None, end_ns: int | None = None,
             topic: str = '', service: str = '', action: str = '',
             keyword: str = '', limit: int = Query(100, ge=1, le=500),
             before_id: int | None = None, before_ts_ns: int | None = None,
             page: int = Query(1, ge=1), page_size: int | None = None,
             cursor_mode: bool = False):
        # 旧页码 API 保持可用；网页的游标查询无需逐页跳过或计算总数。
        if page_size is not None and page_size not in (100, 200, 500, 1000):
            raise HTTPException(422, '每页条数只能为 100、200、500 或 1000')
        if (before_id is None) != (before_ts_ns is None):
            raise HTTPException(422, '游标必须同时包含时间和消息 ID')
        size = page_size if page_size is not None else limit
        filters = []
        values = []
        if start_ns is not None:
            filters.append('ts_ns >= ?'); values.append(start_ns)
        if end_ns is not None:
            filters.append('ts_ns <= ?'); values.append(end_ns)
        names = [(category, name) for category, name in
                 (('topic', topic), ('service', service), ('action', action)) if name]
        if names:
            filters.append('(' + ' OR '.join('(category = ? AND name = ?)' for _ in names) + ')')
            for category, name in names:
                values.extend((category, name))
        if keyword:
            escaped = keyword.replace('\\', '\\\\').replace('%', '\\%').replace('_', '\\_')
            if len(keyword) >= 3:
                # 三字以上由 trigram 全文索引缩小候选集；短词仍能精确搜索。
                filters.append('id IN (SELECT rowid FROM messages_fts WHERE payload LIKE ?)')
                values.append(f'%{keyword}%')
            filters.append('payload LIKE ? ESCAPE \'\\\'')
            values.append(f'%{escaped}%')
        if before_id is not None and before_ts_ns is not None:
            filters.append('(ts_ns, id) < (?, ?)')
            values.extend((before_ts_ns, before_id))
        where = ' WHERE ' + ' AND '.join(filters) if filters else ''
        select = '''SELECT id, ts_ns, category, name, type_name, node, level,
            CASE WHEN summary = '' THEN substr(payload, 1, 512) ELSE summary END AS summary,
            size_bytes FROM messages'''
        with read_db() as conn:
            if cursor_mode:
                rows = conn.execute(select + where +
                                    ' ORDER BY ts_ns DESC, id DESC LIMIT ?',
                                    (*values, size + 1)).fetchall()
                has_more = len(rows) > size
                rows = rows[:size]
                total = None
                total_pages = None
            else:
                total = conn.execute('SELECT count(*) FROM messages' + where,
                                     values).fetchone()[0]
                rows = conn.execute(select + where +
                                    ' ORDER BY ts_ns DESC, id DESC LIMIT ? OFFSET ?',
                                    (*values, size, (page - 1) * size)).fetchall()
                has_more = page * size < total
                total_pages = (total + size - 1) // size
        return {'items': [dict(row) for row in rows],
                'page': page, 'page_size': size, 'total': total,
                'total_pages': total_pages,
                'next_cursor': {'before_id': rows[-1]['id'],
                                'before_ts_ns': rows[-1]['ts_ns']}
                if rows and has_more else None}

    @app.get('/api/logs/{message_id}')
    def detail(message_id: int):
        with read_db() as conn:
            row = conn.execute('SELECT * FROM messages WHERE id=?', (message_id,)).fetchone()
        if row is None:
            raise HTTPException(404, '日志不存在或已过期')
        result = dict(row)
        if not result['summary'] and result['payload']:
            result['summary'] = result['payload'][:512]
        return result

    @app.get('/api/logs/{message_id}/mcap')
    def download(message_id: int):
        with read_db() as conn:
            row = conn.execute('SELECT bag_file FROM messages WHERE id=?',
                               (message_id,)).fetchone()
        if row is None:
            raise HTTPException(404, '日志不存在或已过期')
        path = Path(row['bag_file']).resolve()
        root = config['root'].resolve()
        if not path.is_relative_to(root) or path.suffix != '.mcap' or not path.is_file():
            raise HTTPException(404, 'MCAP 片段不存在')
        return FileResponse(path, filename=path.name,
                            media_type='application/octet-stream')

    return app
