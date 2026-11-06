"""作用：保存网页导航的规划、Action 反馈和最终结果；使用方法：由 RobotConsole 持有并随状态 API 返回。"""
import math
import threading
import time


class NavigationStatus:
    """使用方法：每个导航目标使用独立标识，忽略取消或替换后迟到的反馈。"""

    def __init__(self, clock=time.monotonic, stalled_after=10.0, diagnostic_grace_seconds=2.0):
        self.clock = clock
        self.stalled_after = float(stalled_after)
        if not math.isfinite(self.stalled_after) or self.stalled_after <= 0:
            raise ValueError('导航进度停滞提示时间必须为正数')
        self.lock = threading.RLock()
        self.diagnostic_grace_seconds = float(diagnostic_grace_seconds)
        if not math.isfinite(self.diagnostic_grace_seconds) or self.diagnostic_grace_seconds < 0:
            raise ValueError('导航迟到错误收集窗口必须为非负有限值')
        self.goal_token = None
        self.data = {'phase': 'idle', 'message': '尚未开始导航',
                     'goal': None, 'distance_remaining_m': None,
                     'heading_error_deg': None,
                     'elapsed_seconds': None, 'eta_seconds': None,
                     'recoveries': 0, 'error_code': None,
                     'started_at': None, 'finished_at': None}
        self.feedback_at = None
        self.started_monotonic = None
        self.progress_at = None
        self.progress_distance = None
        self.progress_heading = None
        self.diagnostics = []
        self.finished_monotonic = None
        self.result_message = ''

    def planning(self, goal):
        with self.lock:
            if self.goal_token is None:
                self.data.update(phase='planning', message='正在规划路径', goal=goal,
                                 distance_remaining_m=None, elapsed_seconds=None,
                                 heading_error_deg=None,
                                 eta_seconds=None, recoveries=0, error_code=None,
                                 started_at=None, finished_at=None)
                self.feedback_at = None
                self.started_monotonic = None
                self.progress_at = None
                self.progress_distance = None
                self.progress_heading = None
                self.finished_monotonic = None
                self.diagnostics = []

    def planned(self, goal, path_points):
        with self.lock:
            if self.goal_token is None:
                self.data.update(phase='ready', message=f'已规划 {path_points} 个路径点，等待确认前往',
                                 goal=goal)

    def start(self, token, goal):
        with self.lock:
            self.goal_token = token
            self.feedback_at = None
            self.started_monotonic = self.clock()
            self.progress_at = None
            self.progress_distance = None
            self.progress_heading = None
            self.diagnostics = []
            self.finished_monotonic = None
            self.result_message = ''
            self.data.update(phase='starting', message='正在提交导航目标',
                             goal=goal, distance_remaining_m=None,
                             heading_error_deg=None,
                             elapsed_seconds=None, eta_seconds=None,
                             recoveries=0, error_code=None,
                             started_at=time.time(), finished_at=None)

    def accepted(self, token):
        with self.lock:
            if token == self.goal_token and self.data['phase'] == 'starting':
                self.data['message'] = 'Nav2 已接受目标，等待导航反馈'

    def is_current(self, token):
        with self.lock:
            return self.goal_token == token

    def diagnostic(self, node, level, message, stamp=None):
        """收集本次 Nav2 主/子节点的具体错误，兼容结果先于 rosout 到达。"""
        with self.lock:
            late = (self.data['phase'] == 'failed' and self.finished_monotonic is not None and
                    self.clock()-self.finished_monotonic <= self.diagnostic_grace_seconds)
            name = node.split('.')[-1]
            relevant = (name in ('bt_navigator', 'controller_server', 'planner_server', 'behavior_server') or
                        name.startswith('bt_navigator_'))
            if (not relevant or level < 30 or (self.goal_token is None and not late) or
                    (stamp is not None and self.data['started_at'] is not None and stamp < self.data['started_at'])):
                return
            reasons = (
                ('acknowledge goal request for compute_path_to_pose', '重新规划请求确认超时', 100),
                ('acknowledge goal request', '导航动作请求确认超时', 100),
                ('executing service call', '导航恢复服务调用超时', 100),
                ('Costmap timed out', '代价地图更新超时', 95),
                ('Failed to make progress', '机器人未达到导航进展要求', 90),
                ('Failed to get result', '停止路径跟踪时未取得执行结果', 80),
                ('Failed to cancel action server', '导航动作取消确认失败', 80),
                ('detected collision ahead', '局部路径预测碰撞，控制器停止', 60),
                ('Collision Ahead', '恢复动作预测碰撞', 60),
                ('failed to plan', '无法规划可通行路径', 90),
                ('time allowance', '恢复动作执行超时', 85),
            )
            match = next(((reason, rank) for key, reason, rank in reasons if key.lower() in message.lower()), None)
            if match is None:
                if level < 40 or message.strip() == 'Goal failed' or 'Aborting handle' in message:
                    return
                match = ('Nav2 执行错误', 70)
            detail = {'node': node, 'message': message, 'reason': match[0], 'priority': match[1]}
            self.diagnostics = [item for item in self.diagnostics if item['node'] != node or item['message'] != message]
            self.diagnostics.append(detail)
            self.diagnostics = sorted(self.diagnostics, key=lambda item: item['priority'], reverse=True)[:5]
            if late and not self.result_message:
                self.data['message'] = self._diagnostic_message()

    def _diagnostic_message(self):
        detail = self.diagnostics[0]
        return f"{detail['reason']}；{detail['node']}: {detail['message']}"

    def feedback(self, token, distance, elapsed, eta, recoveries, current_yaw=None):
        with self.lock:
            if token != self.goal_token:
                return False
            self.feedback_at = self.clock()
            distance = self._nonnegative(distance)
            if distance is not None and (self.progress_distance is None or
                                         distance <= self.progress_distance - 0.05):
                self.progress_distance = distance
                self.progress_at = self.feedback_at
            eta = self._nonnegative(eta)
            if distance is not None and distance > 0.1 and eta == 0:
                eta = None
            heading_error = None
            goal_yaw = self.data['goal'].get('yaw') if self.data['goal'] else None
            if current_yaw is not None and goal_yaw is not None:
                difference = math.atan2(math.sin(goal_yaw - current_yaw),
                                        math.cos(goal_yaw - current_yaw))
                heading_error = round(math.degrees(abs(difference)), 1)
                if (self.progress_heading is None or
                        heading_error <= self.progress_heading - 5.0):
                    self.progress_heading = heading_error
                    self.progress_at = self.feedback_at
            self.data.update(
                phase='navigating', message='正在前往目标',
                distance_remaining_m=distance,
                heading_error_deg=heading_error,
                elapsed_seconds=self._nonnegative(elapsed),
                eta_seconds=eta,
                recoveries=max(0, int(recoveries)))
            return True

    @staticmethod
    def _nonnegative(value):
        value = float(value)
        return value if math.isfinite(value) and value >= 0 else None

    def finish(self, token, status, error_code=0, error_msg=''):
        with self.lock:
            if token != self.goal_token:
                return False
            self.goal_token = None
            self.finished_monotonic = self.clock()
            self.result_message = error_msg.strip()
            if status == 4:
                phase, message = 'succeeded', '已到达目标'
                self.data['distance_remaining_m'] = 0.0
                self.data['eta_seconds'] = 0.0
            elif status == 5:
                phase, message = 'canceled', '导航已取消'
            else:
                phase = 'failed'
                if error_msg.strip():
                    message = error_msg.strip()
                elif self.diagnostics:
                    message = self._diagnostic_message()
                elif (self.data['distance_remaining_m'] is not None and
                      self.data['distance_remaining_m'] <= 0.1 and
                      (self.data['heading_error_deg'] or 0) > math.degrees(0.25)):
                    message = '已接近目标，但朝向未达标；Nav2 导航失败'
                else:
                    message = (f'Nav2 终止导航（状态 {status}，错误码 {error_code}），'
                               '未返回具体错误信息；请查看本次导航时段的实时控制台或历史日志')
            self.data.update(phase=phase, message=message,
                             error_code=error_code if phase == 'failed' else None,
                             finished_at=time.time())
            return True

    def fail(self, message):
        with self.lock:
            if self.goal_token is None:
                self.data.update(phase='failed', message=str(message), error_code=None,
                                 finished_at=time.time())

    def cancel(self, message='导航已取消', pending=False):
        with self.lock:
            if self.goal_token is not None or self.data['phase'] in ('planning', 'ready'):
                self.goal_token = None
                self.data.update(phase='canceling' if pending else 'canceled', message=message,
                                 finished_at=time.time())

    def canceled(self):
        with self.lock:
            if self.data['phase'] == 'canceling':
                self.data.update(phase='canceled', message='Nav2 已确认导航结束，旧目标不会恢复',
                                 finished_at=time.time())

    def snapshot(self):
        with self.lock:
            result = dict(self.data)
            result['failure_details'] = [dict(item) for item in self.diagnostics] if result['phase'] == 'failed' else []
            result['feedback_age_seconds'] = (
                round(self.clock() - self.feedback_at, 1)
                if self.goal_token is not None and self.feedback_at is not None else None)
            result['start_age_seconds'] = (
                round(self.clock() - self.started_monotonic, 1)
                if self.goal_token is not None and self.started_monotonic is not None else None)
            progress_age = (self.clock() - self.progress_at
                            if self.goal_token is not None and self.progress_at is not None
                            else None)
            result['no_progress_seconds'] = round(progress_age, 1) if progress_age else None
            result['appears_stalled'] = bool(
                progress_age is not None and progress_age >= self.stalled_after)
            return result
