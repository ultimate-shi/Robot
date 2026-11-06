"""作用：读取录制/索引话题白名单并精简索引事件；使用方法：由 manager 和 indexer 加载各自 YAML。"""

import hashlib
import json
import math
from pathlib import Path
import re

import yaml


CATEGORIES = ('常用', '按需', '不建议')


class TopicPolicy:
    """每项显式 enabled；分类只用于维护建议，启用项按配置顺序匹配。"""

    def __init__(self, path):
        self.path = Path(path)
        self.config = yaml.safe_load(self.path.read_text(encoding='utf-8'))
        groups = self.config['categories']
        if set(groups) != set(CATEGORIES):
            raise ValueError('话题配置必须包含常用、按需、不建议三个分类')
        self.rules = []
        patterns = []
        for category in CATEGORIES:
            for entry in groups[category]:
                if type(entry.get('enabled')) is not bool:
                    raise ValueError('每个话题必须显式设置 enabled: true/false')
                if ('topic' in entry) == ('regex' in entry):
                    raise ValueError('每个话题只能设置 topic 或 regex 其中一项')
                expression = re.escape(entry['topic']) if 'topic' in entry else entry['regex']
                pattern = re.compile(expression)
                if entry['enabled']:
                    self.rules.append((pattern, entry))
                    patterns.append(expression)
        if not patterns:
            raise ValueError('至少需要启用一个话题，禁止空白名单退化成全量采集')
        self.regex = '^(?:' + '|'.join(patterns) + ')$'
        self.fingerprint = hashlib.sha256(json.dumps(
            self.config, sort_keys=True, ensure_ascii=False).encode()).hexdigest()

    def match(self, topic):
        return next((rule for pattern, rule in self.rules if pattern.fullmatch(topic)), None)


class IndexSelector:
    """只解码入选话题；状态变化与周期采样分开，故障变化不受采样间隔限制。"""

    def __init__(self, policy, max_decode_bytes=65536):
        self.policy = policy
        self.max_decode_bytes = max_decode_bytes
        self.previous = {}
        self.types = {}
        for _, rule in policy.rules:
            if rule.get('mode', 'changes') not in ('all', 'changes', 'sample', 'rosout', 'diagnostics'):
                raise ValueError('不支持的索引 mode')
            for key in ('interval_seconds', 'heartbeat_seconds', 'repeat_seconds'):
                value = rule.get(key, 0)
                if not math.isfinite(value) or value < 0:
                    raise ValueError(f'{key} 必须是非负有限值')

    @staticmethod
    def _field(data, path):
        for key in path.split('.'):
            if not isinstance(data, dict) or key not in data:
                raise ValueError(f'索引 fields 字段不存在：{path}')
            data = data[key]
        return data

    def select(self, topic, schema, message, max_bytes):
        rule = self.policy.match(topic)
        if rule is None or schema is None or len(message.data) > self.max_decode_bytes:
            return None
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message

        if schema.name not in self.types:
            self.types[schema.name] = get_message(schema.name)
        decoded = deserialize_message(message.data, self.types[schema.name])
        node, level = '', None
        mode = rule.get('mode', 'changes')
        data = message_to_ordereddict(decoded)
        # String 状态中的 JSON 按真实字段比较，避免 elapsed、测距等连续变化污染事件索引。
        if schema.name == 'std_msgs/msg/String':
            try:
                data = json.loads(decoded.data)
            except ValueError:
                data = {'data': decoded.data}
        if mode == 'rosout':
            node, level = decoded.name, int(decoded.level)
            if (level < int(rule.get('min_level', 30)) and not
                    (level >= 20 and node.split('.')[-1] in rule.get('info_nodes', []))):
                return None
            data = {'node': node, 'level': level, 'message': decoded.msg}
            signature = data
            key = (topic, node)
            interval = rule.get('repeat_seconds', 0)
        elif mode == 'diagnostics':
            statuses = []
            for item, original in zip(data['status'], decoded.status):
                # DiagnosticStatus.level 是 ROS byte，Jazzy Python 将其表示为单字节 bytes。
                value = original.level
                item['level'] = value[0] if isinstance(value, (bytes, bytearray)) else int(value)
                if item['level'] > 0:
                    statuses.append(item)
            signature = [{key: item[key] for key in ('name', 'level', 'message')} for item in statuses]
            data = {'status': statuses, 'message': '诊断异常' if statuses else '诊断恢复正常'}
            key = topic
            interval = rule.get('heartbeat_seconds', 60)
            if not statuses and key not in self.previous:
                return None
        else:
            fields = rule.get('fields', [])
            if fields:
                signature = {field: self._field(data, field) for field in fields}
            else:
                signature = {key: value for key, value in data.items() if key != 'header'} if isinstance(data, dict) else data
            key = topic
            interval = rule.get('interval_seconds', 1) if mode == 'sample' else rule.get('heartbeat_seconds', 60)
        signature = json.dumps(signature, sort_keys=True, ensure_ascii=False)
        previous = self.previous.get(key)
        now = message.log_time
        if previous is not None:
            last_time, last_value = previous
            if mode == 'sample' and now-last_time < interval*1e9:
                return None
            if mode in ('changes', 'diagnostics', 'rosout') and signature == last_value and now-last_time < interval*1e9:
                return None
        self.previous[key] = (now, signature)
        payload = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
        payload = payload.encode('utf-8')[:max_bytes].decode('utf-8', errors='ignore')
        return payload, node, level
