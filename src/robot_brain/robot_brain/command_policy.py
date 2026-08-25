"""使用方法：Web 聊天在模型返回后调用 CommandPolicy，未经授权的动作不得调度。"""

from robot_brain.action_schema import ModelAction, ModelResponse
from robot_brain.contracts import PolicyResult


class CommandPolicy:
    """结合用户原话和冻结视觉证据授权动作，避免只相信小模型分类。"""

    NEGATIVE_WORDS = (
        '不要', '别', '无需', '不用', '不需要', '不想', '不去',
        '先不', '暂时不', '取消', '停止',
    )
    LOCATION_QUESTIONS = (
        '在哪里', '在哪儿', '哪儿', '哪里', '什么位置', '哪个位置',
        '在什么地方', '位置在哪',
    )
    GENERAL_QUESTIONS = (
        '吗', '么', '能不能', '可不可以', '是否', '会不会', '能否')
    GOTO_WORDS = (
        '前往', '靠近', '导航到', '走到', '去找', '帮我找', '过去')
    GOTO_SUFFIXES = ('的地方', '那里', '那边', '旁边', '附近', '跟前')

    @classmethod
    def authorize(cls, text, proposal, detections):
        """返回唯一可进入任务层的动作；模型 action 只是待验证提案。"""
        command = str(text).strip()
        values = list(detections or [])
        if not command:
            return PolicyResult(
                proposal.answer, None, 'EMPTY_REQUEST', 'policy')
        if any(word in command for word in cls.NEGATIVE_WORDS):
            return PolicyResult(
                '已识别为否定或取消表达，未生成机器人动作。', None,
                'NEGATED_COMMAND', 'policy')
        if any(word in command for word in cls.LOCATION_QUESTIONS):
            if cls._goto_target_missing(proposal.action, values):
                label = proposal.action.arguments['label'].strip()
                return PolicyResult(
                    f'当前画面未检测到{label}。', None,
                    'LOCATION_TARGET_NOT_DETECTED', 'policy')
            return PolicyResult(
                ('这是位置询问，未生成机器人动作。'
                 if proposal.action is not None else proposal.answer),
                None, 'LOCATION_QUESTION', 'policy')
        if any(word in command for word in cls.GENERAL_QUESTIONS):
            return PolicyResult(
                ('这是问答内容，未生成机器人动作。'
                 if proposal.action is not None else proposal.answer),
                None, 'QUESTION_NOT_COMMAND', 'policy')

        expected = cls.explicit_action(command, values)
        if expected is None:
            if proposal.action is None:
                if cls._claims_preview(proposal.answer):
                    return PolicyResult(
                        '当前画面未识别到与命令匹配的目标，'
                        '未生成任务预演。请先刷新识别后再试。',
                        None, 'UNGROUNDED_ACTION_CLAIM', 'policy')
                return PolicyResult(
                    proposal.answer, None, 'PLAIN_ANSWER', 'model')
            if (cls._looks_like_goto_command(command)
                    and proposal.action.name == 'goto_object'
                    and proposal.action.arguments['label'].lower()
                    in command.lower()
                    and cls._goto_target_missing(proposal.action, values)):
                label = proposal.action.arguments['label'].strip()
                return PolicyResult(
                    f'当前画面未检测到{label}，未生成任务预演。',
                    None, 'TARGET_NOT_DETECTED', 'policy')
            return PolicyResult(
                '这不是明确的机器人任务命令，未生成机器人动作。',
                None, 'COMMAND_NOT_EXPLICIT', 'policy')

        # 明确命令以本地白名单结果为准；小模型遗漏或误分类时不二次推理。
        source = 'model' if cls._same_action(proposal.action, expected) else (
            'deterministic_fallback')
        return PolicyResult(
            cls.preview_answer(expected), expected,
            'AUTHORIZED', source)

    @classmethod
    def explicit_action(cls, text, detections):
        """识别保守的明确命令；前往动作必须同时包含本轮实际目标标签。"""
        command = str(text).strip()
        if not command or any(word in command for word in cls.NEGATIVE_WORDS):
            return None
        if '探索' in command and any(word in command for word in (
                '开始', '进行', '执行', '预演', '自主')):
            return ModelAction('explore', {})
        if any(word in command for word in ('跟随', '跟着')) and any(
                word in command for word in ('人', '人员', '我', '他', '她')):
            return ModelAction('follow_person', {})

        matches = []
        lowered = command.lower()
        for item in detections:
            labels = []
            for key in ('label_zh', 'class_name'):
                label = str(item.get(key, '')).strip()
                if label and label.lower() in lowered:
                    labels.append(label)
            if not labels:
                continue
            label = labels[0]
            label_index = lowered.find(label.lower())
            strong_verb = any(
                0 <= command.find(word) < label_index
                for word in cls.GOTO_WORDS)
            plain_go = (
                command.find('去') >= 0 and command.find('去') < label_index)
            destination_suffix = any(
                word in command for word in cls.GOTO_SUFFIXES)
            plain_arrive = (
                command.find('到') >= 0 and command.find('到') < label_index)
            if strong_verb or (plain_go and destination_suffix) or (
                    plain_arrive and destination_suffix):
                matches.append(label)
        if matches:
            return ModelAction('goto_object', {'label': matches[0]})

        if any(word in command for word in ('去那里', '去那边', '过去')):
            if len(detections) == 1:
                item = detections[0]
                label = str(
                    item.get('label_zh') or item.get('class_name')
                    or '').strip()
                if label:
                    return ModelAction('goto_object', {'label': label})
        return None

    @staticmethod
    def _same_action(left, right):
        return (left is not None and left.name == right.name
                and left.arguments == right.arguments)

    @staticmethod
    def _claims_preview(answer):
        """识别模型在无动作时误称已经创建任务的回答。"""
        text = str(answer)
        return '任务预演' in text and any(
            word in text for word in ('将生成', '已生成', '请确认'))

    @classmethod
    def _looks_like_goto_command(cls, command):
        """不依赖视觉结果，只判断用户原话是否具有前往命令形式。"""
        text = str(command)
        if any(word in text for word in cls.GOTO_WORDS):
            return True
        return (any(word in text for word in ('去', '到'))
                and any(word in text for word in cls.GOTO_SUFFIXES))

    @staticmethod
    def _goto_target_missing(action, detections):
        """goto_object 目标必须在当前结构化检测中有精确标签。"""
        if action is None or action.name != 'goto_object':
            return False
        requested = str(action.arguments.get('label', '')).strip().lower()
        if not requested:
            return True
        for item in detections:
            labels = {
                str(item.get('label_zh', '')).strip().lower(),
                str(item.get('class_name', '')).strip().lower(),
            }
            if requested in labels:
                return False
        return True

    @staticmethod
    def preview_answer(action):
        labels = {
            'explore': '自主探索',
            'follow_person': '跟随人员',
            'goto_object': '前往物体',
        }
        return f'已生成{labels[action.name]}任务预演，请检查路径后确认。'


def authorize_response(text, proposal: ModelResponse, detections):
    """兼容函数：将策略结果转换回现有 ModelResponse。"""
    result = CommandPolicy.authorize(text, proposal, detections)
    return ModelResponse(answer=result.answer, action=result.action), result
