"""作用：桥接网页关节、车身平衡与状态；使用方法：RobotConsole 初始化后调用 tick/status。"""
import json
import math
import time
from types import SimpleNamespace

from robot_interfaces.msg import BodyBalanceStatus
from robot_interfaces.srv import SetBodyBalance
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from robot_web.posture import angles, heights, body_step, bounded_move, body_angle_step


JOINT_DEFAULTS = {
    'joint_state_topic': '/joint_states',
    'leg_joint_names': ['lap_body_joint_fl', 'lap_body_joint_fr',
                        'lap_body_joint_rl', 'lap_body_joint_rr'],
    'leg_command_topics': ['/lap_fl_position_controller/commands',
                           '/lap_fr_position_controller/commands',
                           '/lap_rl_position_controller/commands',
                           '/lap_rr_position_controller/commands'],
    'head_joint_names': ['head_yaw_joint', 'head_pitch_joint'],
    'head_command_topic': '/head_controller/commands',
    'leg_step_deg': 5.0, 'leg_step_repeat_seconds': 0.3,
    'leg_hardware_sign': [-1.0, 1.0, -1.0, 1.0],
    'leg_min_deg': [-30.0] * 4, 'leg_max_deg': [30.0] * 4,
    'head_min_deg': [-90.0, -30.0], 'head_max_deg': [90.0, 30.0],
    'leg_length_mm': 60.0, 'body_speed_mm_s': 5.0,
    'head_speed_deg_s': 15.0, 'head_pitch_up_sign': -1.0, 'joint_feedback_timeout': 0.5,
    'joint_command_timeout': 0.35, 'balance_status_timeout': 0.5,
    'joint_max_tick_seconds': 0.1, 'joint_target_lead_deg': 5.0,
    'joint_arrival_tolerance_deg': 0.2, 'controller_status_timeout': 5.0,
    'head_lock_status_timeout': 1.0,
    'balance_service_timeout': 3.0, 'balance_max_setpoint_deg': 10.0,
    'balance_service': '/body_balance/set',
    'balance_status_topic': '/body_balance/status',
    'head_lock_status_topic': '/mapping/head_lock_status',
    'head_lock_node': 'head_mapping_lock',
    'head_lock_resume_service': '/mapping/resume_head_lock',
    'camera_raw_color_topic': '/astra/color/image_raw',
    'camera_raw_depth_topic': '/astra/depth/image_raw',
}


class JointControl:
    def __init__(self, node):
        self.node = node
        self.cfg = node.settings
        c = self.cfg
        for group, count in [('leg', 4), ('head', 2)]:
            if any(len(c[key]) != count for key in
                   (f'{group}_joint_names', f'{group}_min_deg', f'{group}_max_deg')):
                raise ValueError('关节名称与限位数量不一致')
            if any(not math.isfinite(a) or not math.isfinite(b) or a >= b
                   for a, b in zip(c[f'{group}_min_deg'], c[f'{group}_max_deg'])):
                raise ValueError('关节限位无效')
        if any(v < -90 for v in c['leg_min_deg']) or any(v > 90 for v in c['leg_max_deg']):
            raise ValueError('大腿限位必须在 -90° 到 90° 内')
        if c['head_pitch_up_sign'] not in (-1.0, 1.0):
            raise ValueError('抬头方向必须为 +1 或 -1')
        if len(c['leg_hardware_sign']) != 4 or any(v not in (-1.0, 1.0) for v in c['leg_hardware_sign']):
            raise ValueError('四腿板端方向须为四项 +1 或 -1，与硬件插件一致')
        if len(c['leg_command_topics']) != 4:
            raise ValueError('四腿命令话题必须各有一项')
        for key in ('leg_step_deg', 'leg_step_repeat_seconds', 'leg_length_mm', 'body_speed_mm_s', 'head_speed_deg_s',
                    'joint_feedback_timeout', 'joint_command_timeout',
                    'balance_status_timeout', 'balance_service_timeout',
                    'joint_max_tick_seconds', 'joint_target_lead_deg',
                    'joint_arrival_tolerance_deg', 'controller_status_timeout', 'head_lock_status_timeout'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} 必须为正数')
        self.feedback = {}
        self.velocities = {}
        self.targets = {'leg': None, 'head': None}
        self.active = set()
        self.direct_groups = set()
        self.intent = {}
        self.intent_stamp = 0.0
        self.sequences = {}
        self.balance = None
        self.pending = None
        self.balance_error = ''
        self.head_lock_stamp = 0.0
        self.head_lock_present = False
        self.head_lock_state = {}
        self.last_tick = time.monotonic()
        self.step_limited = False
        self.limit = False
        self.stop_warning = ''
        self.publishers = [node.create_publisher(Float64MultiArray, topic, 10)
                           for topic in c['leg_command_topics']]
        self.head_pub = node.create_publisher(Float64MultiArray, c['head_command_topic'], 10)
        self.balance_client = node.create_client(SetBodyBalance, c['balance_service'])
        self.head_resume_client = node.create_client(Trigger, c['head_lock_resume_service'])
        node.create_subscription(JointState, c['joint_state_topic'], self.on_joints, 20)
        node.create_subscription(BodyBalanceStatus, c['balance_status_topic'], self.on_balance, 10)
        node.create_subscription(String, c['head_lock_status_topic'], self.on_head_lock, 10)
        self.lower, self.upper = [], []
        for low, high in zip(heights(c['leg_min_deg'], c['leg_length_mm']),
                             heights(c['leg_max_deg'], c['leg_length_mm'])):
            self.lower.append(min(low, high))
            self.upper.append(max(low, high))

    def on_joints(self, msg):
        with self.node.data_lock:
            now = time.monotonic()
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and math.isfinite(msg.position[i]):
                    self.feedback[name] = (math.degrees(msg.position[i]), now)
                    self.velocities[name] = float(msg.velocity[i]) if i < len(msg.velocity) and math.isfinite(msg.velocity[i]) else None
                else:
                    self.feedback.pop(name, None)

    def on_balance(self, msg):
        with self.node.data_lock:
            self.balance = (msg, time.monotonic())

    def on_head_lock(self, msg):
        with self.node.data_lock:
            self.head_lock_stamp = time.monotonic()
            try:
                self.head_lock_state = json.loads(msg.data)
            except (ValueError, TypeError):
                self.head_lock_state = {}

    def fresh(self, group):
        now = time.monotonic()
        return all(name in self.feedback and now - self.feedback[name][1] <=
                   self.cfg['joint_feedback_timeout'] for name in self.cfg[f'{group}_joint_names'])

    def measured(self, group):
        return [self.feedback[name][0] for name in self.cfg[f'{group}_joint_names']]

    def reason(self, group):
        n, c = self.node, self.cfg
        if n.control.stopped:
            return '紧急停止已锁定'
        if n.control.mode != 'manual':
            return '仅手动模式可操作'
        if n.saving_map:
            return '地图保存中'
        if not self.fresh(group):
            return '关节反馈缺失或过期'
        if c['mode'] == 'real':
            if not n.hardware or time.monotonic() - n.hardware[1] > c['joint_feedback_timeout']:
                return '硬件反馈过期'
            hw = n.hardware[0]
            if not hw.outputs_enabled or hw.manual_recovery_required or hw.fault:
                return '硬件输出不可用或故障锁定'
            if not (hw.dmc0_connected and hw.dmc1_connected and hw.se2_connected and hw.watchdog_ok):
                return '控制板连接或心跳异常'
        expected = ([topic.split('/')[1] for topic in c['leg_command_topics']]
                    if group == 'leg' else [c['head_command_topic'].split('/')[1]])
        if not n.controllers or time.monotonic() - n.controllers[1] > c['controller_status_timeout'] or any(
                n.controllers[0].get(name) != 'active' for name in expected):
            return '关节控制器未激活或状态过期'
        if group == 'head':
            self.head_lock_present = any(name == c['head_lock_node'] for name, _ in n.get_node_names_and_namespaces())
            if self.head_lock_present or time.monotonic() - self.head_lock_stamp < c['head_lock_status_timeout']:
                return '建图/定位正在锁定头部归中'
        else:
            if self.pending:
                return '等待自动平衡服务确认'
            if not self.balance or time.monotonic() - self.balance[1] > c['balance_status_timeout']:
                if c['mode'] == 'real' or self.balance_client.service_is_ready():
                    return '自动平衡状态未知或过期'
            elif self.balance[0].enabled:
                return '请先关闭自动平衡'
            if self.balance and self.balance[0].overload_active:
                return '大腿过流保护中'
        return ''

    def publish(self, group, values):
        if group == 'leg':
            for pub, value in zip(self.publishers, values):
                pub.publish(Float64MultiArray(data=[math.radians(value)]))
        else:
            self.head_pub.publish(Float64MultiArray(data=[math.radians(v) for v in values]))

    def hold(self, group):
        self.direct_groups.discard(group)
        self.active.discard(group)
        if self.fresh(group):
            values = self.measured(group)
            self.targets[group] = list(values)
            setattr(self, '_sent_'+group, list(values))
            self.publish(group, values)
            return True
        self.targets[group] = None
        return False

    def clear(self):
        for group in list(self.active):
            self.hold(group)
        self.intent = {}
        self.intent_stamp = 0.0
        self.direct_groups.clear()

    def command(self, body):
        self.step_limited = False
        group = body.group
        reason = self.reason(group)
        if reason:
            raise ValueError(reason)
        values = body.values
        if group == 'head' and body.unit != 'deg':
            raise ValueError('头部目标只能使用角度')
        count = 4 if group == 'leg' else 2
        if len(values) != count or not all(math.isfinite(v) for v in values):
            raise ValueError('关节目标数量或数值无效')
        if group == 'leg' and body.unit == 'hardware_deg':
            # 板端角度先反向映射为 ROS 角度，由既有硬件插件恢复原符号并乘 100。
            values = [v*sign for v, sign in zip(values, self.cfg['leg_hardware_sign'])]
        if group == 'leg' and body.unit == 'height_mm':
            if any(v < lo or v > hi for v, lo, hi in zip(values, self.lower, self.upper)):
                raise ValueError('大腿高度超出限位')
            values = angles(values, self.cfg['leg_length_mm'])
        if any(v < lo - 1e-8 or v > hi + 1e-8 for v, lo, hi in zip(
                values, self.cfg[f'{group}_min_deg'], self.cfg[f'{group}_max_deg'])):
            raise ValueError('关节角度超出限位')
        for key in (('pitch', 'roll', 'lift') if group == 'leg' else ('yaw', 'head_pitch')):
            self.intent[key] = 0
        if group in self.active:
            if group == 'leg' and body.unit == 'hardware_deg':
                # 连按直接替换目标，不插入当前反馈角，避免 3→0→6 式回跳。
                self.active.discard(group)
                self.direct_groups.discard(group)
            else:
                self.hold(group)
        self.targets[group] = list(values)
        setattr(self, '_sent_'+group, self.measured(group))
        self.active.add(group)
        if group == 'leg' and body.unit == 'hardware_deg':
            self.direct_groups.add(group)
            self.publish(group, values)
            setattr(self, '_sent_'+group, list(values))

    def zero(self, body):
        """本组所有关节一次置零，并用序列屏障阻止旧持续输入覆盖。"""
        if not body.client_id or len(body.client_id) > 100:
            raise ValueError('页面标识无效')
        old = self.sequences.get(body.client_id)
        if old and body.sequence <= old[0]:
            raise ValueError('已丢弃过期或乱序的控制指令')
        if body.generation != self.node.motion_generation:
            raise ValueError('控制模式已更新，请等待状态刷新后重新操作')
        group = body.group
        reason = self.reason(group)
        if reason:
            raise ValueError(reason)
        if any(low > 0 or high < 0 for low, high in zip(
                self.cfg[f'{group}_min_deg'], self.cfg[f'{group}_max_deg'])):
            raise ValueError('零位超出关节限位')
        values = [0.0]*len(self.cfg[f'{group}_joint_names'])
        for key in (('pitch', 'roll', 'lift') if group == 'leg' else ('yaw', 'head_pitch')):
            self.intent[key] = 0
        self.targets[group] = values
        self.active.add(group)
        self.direct_groups.add(group)
        setattr(self, '_sent_'+group, list(values))
        self.step_limited = False
        self.sequences[body.client_id] = (body.sequence, time.monotonic())
        self.publish(group, values)

    def step(self, body):
        """按页面序列处理离散角度步进，连按基于上次目标累加。"""
        if not body.client_id or len(body.client_id) > 100:
            raise ValueError('页面标识无效')
        old = self.sequences.get(body.client_id)
        if old and body.sequence <= old[0]:
            raise ValueError('已丢弃过期或乱序的控制指令')
        if body.generation != self.node.motion_generation:
            raise ValueError('控制模式已更新，请等待状态刷新后重新操作')
        reason = self.reason('leg')
        if reason:
            raise ValueError(reason)
        current = self.targets['leg'] if 'leg' in self.active else self.measured('leg')
        targets, limited = body_angle_step(current, body.action, self.cfg['leg_step_deg'],
            self.cfg['leg_length_mm'], self.cfg['leg_min_deg'], self.cfg['leg_max_deg'])
        board = [v*sign for v, sign in zip(targets, self.cfg['leg_hardware_sign'])]
        self.command(SimpleNamespace(group='leg', values=board, unit='hardware_deg'))
        self.step_limited = limited
        self.sequences[body.client_id] = (body.sequence, time.monotonic())

    def motion(self, body):
        if not body.client_id or len(body.client_id) > 100:
            raise ValueError('页面标识无效')
        previous = self.sequences.get(body.client_id)
        if previous and body.sequence <= previous[0]:
            raise ValueError('已丢弃过期或乱序的控制指令')
        if body.hold:
            self.sequences[body.client_id] = (body.sequence, time.monotonic())
            self.clear()
            return
        if body.generation != self.node.motion_generation:
            raise ValueError('控制模式已更新，请等待状态刷新后重新操作')
        values = {name: getattr(body, name) for name in ('pitch', 'roll', 'lift', 'yaw', 'head_pitch')}
        if any(not math.isfinite(v) or abs(v) > 1 for v in values.values()):
            raise ValueError('运动意图必须在 -1 到 1 之间')
        for group, moving in [('leg', any(values[k] for k in ('pitch', 'roll', 'lift'))),
                              ('head', any(values[k] for k in ('yaw', 'head_pitch')))]:
            if moving and self.reason(group):
                raise ValueError(self.reason(group))
        now = time.monotonic()
        self.sequences[body.client_id] = (body.sequence, now)
        # 限制页面序列缓存规模；活跃连接的防乱序记录保留。
        if len(self.sequences) > 256:
            self.sequences = {k: v for k, v in self.sequences.items() if now-v[1] < 60}
        for group, keys in [('leg', ('pitch', 'roll', 'lift')), ('head', ('yaw', 'head_pitch'))]:
            moving = any(values[k] for k in keys)
            was_moving = any(self.intent.get(k, 0) for k in keys)
            if (not moving and was_moving) or (moving and not was_moving and group in self.active):
                self.hold(group)
            if moving and not was_moving:
                setattr(self, '_sent_'+group, self.measured(group))
        self.intent, self.intent_stamp = values, now

    def set_balance(self, body):
        if self.pending:
            raise ValueError('平衡设置正在处理中')
        if body.level not in ('low', 'mid', 'high'):
            raise ValueError('平衡档位无效')
        if any(not math.isfinite(v) or abs(v) > self.cfg['balance_max_setpoint_deg']
               for v in (body.target_roll_deg, body.target_pitch_deg)):
            raise ValueError('姿态目标超出范围')
        if body.level != 'mid' and (body.target_roll_deg or body.target_pitch_deg):
            raise ValueError('低/高档为固定高度，只允许零姿态目标')
        if body.enabled:
            # 已启用的平衡允许调整设置，其余硬件检查沿用手动腿的保护。
            old = self.balance
            self.balance = None
            reason = self.reason('leg')
            self.balance = old
            if reason and reason != '自动平衡状态未知或过期':
                raise ValueError(reason)
            if not old or time.monotonic()-old[1] > self.cfg['balance_status_timeout']:
                raise ValueError('平衡状态过期')
            if old[0].overload_active and body.level != 'low':
                raise ValueError('过流保护只允许低档')
        if not self.balance_client.service_is_ready():
            raise ValueError('自动平衡服务不可用')
        self.clear()
        request = SetBodyBalance.Request()
        request.enabled, request.level = body.enabled, body.level
        request.target_roll_deg = body.target_roll_deg
        request.target_pitch_deg = body.target_pitch_deg
        future = self.balance_client.call_async(request)
        self.pending = (future, time.monotonic(), bool(body.enabled))
        self.balance_error = ''

    def resume_head_lock(self):
        if self.node.control.stopped:
            raise ValueError('请先解除紧急停止')
        if not self.head_resume_client.service_is_ready():
            raise ValueError('头部归中服务不可用')
        return self.head_resume_client.call_async(Trigger.Request())

    def emergency(self):
        self.intent = {}
        self.active.clear()
        missing = [g for g in ('leg', 'head') if not self.hold(g)]
        self.stop_warning = ('停止保持未确认：' + '、'.join(missing) + '反馈不可用') if missing else ''
        # 停止锁话题由平衡节点独立处理，避免等待网页服务才能退出平衡。

    def tick(self):
        now = time.monotonic()
        dt = min(self.cfg['joint_max_tick_seconds'], max(0, now-self.last_tick))
        self.last_tick = now
        if self.pending:
            future, stamp, enabled = self.pending
            if future.done():
                try:
                    response = future.result()
                    if not response.success:
                        self.balance_error = response.message
                    # 丢弃服务响应前的状态，必须收到响应后的新状态才开放手动腿。
                    self.balance = None
                except Exception as exc:
                    self.balance_error = str(exc)
                self.pending = None
            elif now-stamp > self.cfg['balance_service_timeout']:
                self.balance_error = '平衡服务确认超时；保持手动大腿锁定，等待服务返回'
        if self.node.control.stopped:
            self.intent = {}
            self.active.clear()
            self.direct_groups.clear()
        if now-self.intent_stamp > self.cfg['joint_command_timeout'] and any(self.intent.values()):
            self.clear()
        self.limit = False
        for group in ('leg', 'head'):
            keys = ('pitch', 'roll', 'lift') if group == 'leg' else ('yaw', 'head_pitch')
            moving = any(self.intent.get(k, 0) for k in keys)
            if not moving and group not in self.active:
                continue
            if self.reason(group):
                if group in self.active:
                    self.hold(group)
                for key in keys:
                    self.intent[key] = 0
                continue
            current = self.measured(group)
            if group in self.direct_groups and not moving:
                # 板端位置目标已一次送出，仅观察到位；不插入网页端渐进角度。
                if max(abs(a-b) for a,b in zip(current,self.targets[group])) < self.cfg['joint_arrival_tolerance_deg']:
                    self.direct_groups.discard(group)
                    self.active.discard(group)
                continue
            # 使用上次已下发目标推进，限制目标领先实际反馈的距离。
            if moving:
                baseline = self.targets[group] if group in self.active else current
                if group == 'leg':
                    h = heights(baseline, self.cfg['leg_length_mm'])
                    values, limited = body_step(h, self.intent, self.cfg['body_speed_mm_s']*dt,
                                                self.lower, self.upper)
                    target = angles(values, self.cfg['leg_length_mm'])
                else:
                    delta = [self.intent.get(k, 0)*self.cfg['head_speed_deg_s']*dt *
                             (self.cfg['head_pitch_up_sign'] if k == 'head_pitch' else 1) for k in keys]
                    target, limited = bounded_move(baseline, delta, self.cfg['head_min_deg'], self.cfg['head_max_deg'])
                self.limit |= limited
                self.targets[group] = target
                self.active.add(group)
            target = self.targets[group]
            # 关节反馈跟不上时，不继续积累远距离目标。
            speed = (self.cfg['head_speed_deg_s'] if group == 'head' else
                     math.degrees(self.cfg['body_speed_mm_s']/self.cfg['leg_length_mm']))
            previous = getattr(self, '_sent_'+group, current)
            if any(abs(a-b) > 2*self.cfg['joint_target_lead_deg'] for a, b in zip(previous, current)):
                previous = current
            if group == 'leg':
                previous_h = heights(previous, self.cfg['leg_length_mm'])
                target_h = heights(target, self.cfg['leg_length_mm'])
                step = self.cfg['body_speed_mm_s']*dt
                sent_h, _ = bounded_move(previous_h, [b-a for a, b in zip(previous_h, target_h)],
                                        [v-step for v in previous_h], [v+step for v in previous_h])
                sent = angles(sent_h, self.cfg['leg_length_mm'])
            else:
                sent, _ = bounded_move(previous, [b-a for a, b in zip(previous, target)],
                                       [v-speed*dt for v in previous], [v+speed*dt for v in previous])
            if max(abs(a-b) for a, b in zip(sent, current)) > self.cfg['joint_target_lead_deg']:
                self.targets[group] = list(current)
                self.publish(group, current)
                setattr(self, '_sent_'+group, current)
                continue
            self.publish(group, sent)
            setattr(self, '_sent_'+group, sent)
            if not moving and max(abs(a-b) for a, b in zip(sent, target)) < 1e-5 and max(
                    abs(a-b) for a, b in zip(current, target)) < self.cfg['joint_arrival_tolerance_deg']:
                self.active.discard(group)

    def status(self):
        now = time.monotonic()
        self.head_lock_present = any(name == self.cfg['head_lock_node'] for name, _ in
                                     self.node.get_node_names_and_namespaces())
        rows = []
        hw = self.node.hardware
        hardware_valid = self.cfg['mode'] != 'real' or bool(hw and now-hw[1] <= self.cfg['joint_feedback_timeout']
            and hw[0].outputs_enabled and not hw[0].manual_recovery_required and not hw[0].fault
            and hw[0].dmc0_connected and hw[0].dmc1_connected and hw[0].se2_connected and hw[0].watchdog_ok)
        for group in ('leg', 'head'):
            for i, name in enumerate(self.cfg[f'{group}_joint_names']):
                sample = self.feedback.get(name)
                fresh = bool(hardware_valid and sample and now-sample[1] <= self.cfg['joint_feedback_timeout'])
                angle = sample[0] if fresh else None
                target = self.targets[group]
                sign = self.cfg['leg_hardware_sign'][i] if group == 'leg' else 1.0
                rows.append({'name': name, 'group': group, 'index': i,
                             'actual_deg': angle, 'target_deg': target[i] if target else None,
                             'hardware_actual_deg': angle*sign if angle is not None else None,
                             'hardware_target_deg': target[i]*sign if target else None,
                             'hardware_min_deg': min(self.cfg[f'{group}_min_deg'][i]*sign,self.cfg[f'{group}_max_deg'][i]*sign),
                             'hardware_max_deg': max(self.cfg[f'{group}_min_deg'][i]*sign,self.cfg[f'{group}_max_deg'][i]*sign),
                             'extension_direction': sign*(1 if i<2 else -1) if group == 'leg' else 1,
                             'fresh': fresh, 'age_seconds': round(now-sample[1], 2) if sample else None,
                             'min_deg': self.cfg[f'{group}_min_deg'][i],
                             'max_deg': self.cfg[f'{group}_max_deg'][i],
                             'height_mm': heights([angle if j == i else 0 for j in range(4)],
                                                  self.cfg['leg_length_mm'])[i] if group == 'leg' and fresh else None,
                             'target_height_mm': heights(target, self.cfg['leg_length_mm'])[i] if group == 'leg' and target else None,
                             'controller': (self.cfg['leg_command_topics'][i] if group == 'leg'
                                            else self.cfg['head_command_topic']).split('/')[1],
                             'reason': self.reason(group)})
        balance = {'fresh': False, 'enabled': None, 'state': 'unavailable'}
        if self.balance:
            msg, stamp = self.balance
            balance = {k: getattr(msg, k) for k in ('enabled', 'level', 'state', 'target_roll_deg',
                        'target_pitch_deg', 'actual_roll_deg', 'actual_pitch_deg', 'overload_active')}
            balance['fresh'] = now-stamp <= self.cfg['balance_status_timeout']
            for key, value in list(balance.items()):
                if isinstance(value, float) and not math.isfinite(value):
                    balance[key] = None
        balance.update(pending=bool(self.pending), error=self.balance_error)
        if self.balance and balance['fresh'] and balance['enabled']:
            for row, value in zip(rows[:4], self.balance[0].leg_target_rad):
                row['target_deg'] = math.degrees(value) if math.isfinite(value) else None
                row['hardware_target_deg'] = row['target_deg']*self.cfg['leg_hardware_sign'][row['index']] if row['target_deg'] is not None else None
        return {'joints': rows, 'balance': balance, 'limited': self.limit or self.step_limited,
                'leg_step_deg': self.cfg['leg_step_deg'], 'leg_step_repeat_seconds': self.cfg['leg_step_repeat_seconds'],
                'leg_reason': self.reason('leg'), 'head_reason': self.reason('head'),
                'height_min_mm': self.lower, 'height_max_mm': self.upper,
                'balance_max_setpoint_deg': self.cfg['balance_max_setpoint_deg'],
                'stop_warning': self.stop_warning, 'generation': self.node.motion_generation,
                'head_lock': self.head_lock_state if self.head_lock_present or
                now-self.head_lock_stamp < self.cfg['head_lock_status_timeout'] else {},
                'other_joints': {name: {'actual_deg': value, 'velocity_rad_s': self.velocities.get(name), 'fresh': hardware_valid and now-stamp <= self.cfg['joint_feedback_timeout']}
                                 for name, (value, stamp) in self.feedback.items()
                                 if name not in self.cfg['leg_joint_names'] + self.cfg['head_joint_names']}}
