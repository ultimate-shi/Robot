"""作用：区分画面短时延迟和持续断流；使用方法：状态 API 传入最近接收时间及配置阈值，不改变导航许可。"""


def stream_status(last_seen, now, fresh_timeout, dropout_timeout):
    if last_seen is None:
        return {'fresh': False, 'state': 'waiting', 'age_seconds': None}
    age = max(0.0, now-last_seen)
    state = 'healthy' if age <= fresh_timeout else 'delayed' if age <= dropout_timeout else 'lost'
    return {'fresh': state == 'healthy', 'state': state, 'age_seconds': round(age, 1)}
