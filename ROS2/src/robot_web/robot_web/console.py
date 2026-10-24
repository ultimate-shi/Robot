"""作用：在 Jazzy 容器内提供网页控制台和手动直达底盘的速度仲裁；使用方法：ros2 launch robot_web web_console.launch.py。"""
import json
import math
import os
from pathlib import Path
import re
import shutil
import threading
import time
import uuid
from urllib.request import Request as UrlRequest, urlopen

import cv2
from cv_bridge import CvBridge
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from nav_msgs.srv import GetMap
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from pydantic import BaseModel
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time
from robot_interfaces.msg import ChassisState, SemanticDetectionArray
from robot_brain.action_schema import parse_model_response
from robot_brain.command_policy import CommandPolicy
from robot_brain.qwen_client import QwenClient, SYSTEM_INSTRUCTION
from robot_navigation.frontier import frontier_world_candidates, standoff_pose
from sensor_msgs.msg import Image, Imu, Range
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import uvicorn
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener

from robot_web.control import ControlState
from robot_web.navigation_status import NavigationStatus


class ModeBody(BaseModel):
    mode: str


class DriveBody(BaseModel):
    linear: float = 0.0
    angular: float = 0.0
    client_id: str
    sequence: int


class GoalBody(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class TaskBody(BaseModel):
    task: str
    target_id: str = ''
    follow_distance: float = 1.5
    max_speed: float = 0.04


class ModelBody(BaseModel):
    text: str
    provider: str = 'local'


class ConfirmModelBody(BaseModel):
    proposal_id: str


class RobotConsole(Node):
    """手动速度直达底盘话题，导航速度仍经过避障链。"""

    def __init__(self):
        super().__init__('web_console')
        defaults = {
            'socket_path': '/workspace/.runtime/robot-console.sock',
            'mode': 'virtual', 'vision_source': 'real', 'static_map': False,
            'nav_input_topic': '/cmd_vel_nav',
            'safe_input_topic': '/cmd_vel_console',
            'filtered_nav_topic': '/cmd_vel_nav_safe',
            'chassis_cmd_topic': '/cmd_vel_safe',
            'manual_command_timeout': 0.35,
            'nav_command_timeout': 0.5, 'update_rate': 20.0,
            'navigation_stall_notice_seconds': 10.0,
            'map_topic': '/map', 'odom_topic': '/odom',
            'detections_topic': '/perception/semantic_detections',
            'left_image_topic': '/stereo/left/image_rect',
            'right_image_topic': '/stereo/right/image_rect',
            'depth_image_topic': '/stereo/depth/image_visual',
            'mission_status_topic': '/mission/status',
            'safety_status_topic': '/obstacle_avoidance/status',
            'snapshot_service': '/mapping/save_snapshot',
            'imu_topic': '/sensors/imu/data',
            'ultrasonic_topics': ['/ultrasonic/front_left',
                                  '/ultrasonic/front_center',
                                  '/ultrasonic/front_right',
                                  '/ultrasonic/rear_left',
                                  '/ultrasonic/rear_center',
                                  '/ultrasonic/rear_right',
                                  '/ultrasonic/left', '/ultrasonic/right'],
            'local_model_url': 'http://127.0.0.1:9100',
            'external_model_url': '', 'external_model_name': '',
            'external_model_token_env': 'ROBOT_EXTERNAL_MODEL_TOKEN',
            'follow_lost_timeout': 2.0,
            'follow_replan_distance': 0.4,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        self.vision_enabled = self.settings['vision_source'] == self.settings['mode']
        self.control = ControlState(
            float(self.settings['manual_command_timeout']))
        self.data_lock = threading.RLock()
        self.last_nav = Twist()
        self.nav_stamp = 0.0
        self.filtered_nav = Twist()
        self.filtered_nav_stamp = 0.0
        self.map = None
        self.map_request = None
        self.controllers = None
        self.controller_request = None
        self.odom = None
        self.detections = None
        self.hardware = None
        self.safety = None
        self.mission = None
        self.imu = None
        self.ultrasonic = {}
        self.llm_health = None
        self.images = {}
        self.image_seen = {}
        self.frame_requested = {}
        self.logs = []
        self.log_sequence = 0
        self.bridge = CvBridge()
        self.pending_goal = None
        self.active_goal = None
        self.preview = None
        self.navigation_status = NavigationStatus(
            stalled_after=float(self.settings['navigation_stall_notice_seconds']))
        self.task = ''
        self.task_paused = False
        self.target_id = ''
        self.follow_distance = 1.5
        self.follow_speed = 0.04
        self.last_follow_target = None
        self.last_follow_plan = 0.0
        self.model_proposals = {}
        self.start_time = time.monotonic()
        self.output = self.create_publisher(
            Twist, str(self.settings['safe_input_topic']), 10)
        self.chassis_output = self.create_publisher(
            Twist, str(self.settings['chassis_cmd_topic']), 10)
        self.stop_state_pub = self.create_publisher(
            Bool, '/web_console/stop_latched', 10)
        self.create_subscription(Twist, str(self.settings['nav_input_topic']),
                                 self.on_nav, 10)
        self.create_subscription(Twist, str(self.settings['filtered_nav_topic']),
                                 self.on_filtered_nav, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, str(self.settings['map_topic']),
                                 self.on_map, map_qos)
        self.create_subscription(Odometry, str(self.settings['odom_topic']),
                                 lambda msg: self.save('odom', msg), 10)
        if self.vision_enabled:
            self.create_subscription(SemanticDetectionArray,
                                     str(self.settings['detections_topic']),
                                     lambda msg: self.save('detections', msg), 10)
        self.create_subscription(ChassisState, '/hardware/chassis_state',
                                 lambda msg: self.save('hardware', msg), 10)
        self.create_subscription(Imu, str(self.settings['imu_topic']),
                                 lambda msg: self.save('imu', msg), 10)
        for topic in self.settings['ultrasonic_topics']:
            self.create_subscription(
                Range, str(topic),
                lambda msg, topic=topic: self.on_ultrasonic(topic, msg),
                qos_profile_sensor_data)
        self.create_subscription(String, str(self.settings['safety_status_topic']),
                                 lambda msg: self.save('safety', msg.data), 10)
        self.create_subscription(String, str(self.settings['mission_status_topic']),
                                 lambda msg: self.save('mission', msg.data), 10)
        from rcl_interfaces.msg import Log
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        if self.vision_enabled:
            for camera, parameter in [('left', 'left_image_topic'),
                                      ('right', 'right_image_topic'),
                                      ('depth', 'depth_image_topic')]:
                self.create_subscription(
                    Image, str(self.settings[parameter]),
                    lambda msg, camera=camera: self.on_image(camera, msg), image_qos)
        self.planner = ActionClient(self, ComputePathToPose,
                                    '/compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose,
                                      '/navigate_to_pose')
        self.snapshot_client = self.create_client(
            Trigger, str(self.settings['snapshot_service']))
        self.map_client = self.create_client(GetMap, '/map_server/map')
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.mission_stop_client = self.create_client(Trigger, '/mission/stop')
        self.nav_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.wheel_cancel_client = self.create_client(
            CancelGoal, '/execute_wheel_motion/_action/cancel_goal')
        self.wheel_stop_client = self.create_client(
            Trigger, '/hardware/stop_encoder_motion')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1 / float(self.settings['update_rate']), self.control_loop)
        self.create_timer(0.5, self.task_loop)
        self.create_timer(2.0, self.refresh_diagnostics)
        self.create_timer(5.0, lambda: threading.Thread(
            target=self.refresh_model_health, daemon=True).start())

    def save(self, name, value):
        with self.data_lock:
            setattr(self, name, (value, time.monotonic()))

    @staticmethod
    def json_number(value):
        number = float(value)
        return number if math.isfinite(number) else None

    def on_nav(self, msg):
        with self.data_lock:
            self.last_nav = msg
            self.nav_stamp = time.monotonic()

    def on_filtered_nav(self, msg):
        with self.data_lock:
            self.filtered_nav = msg
            self.filtered_nav_stamp = time.monotonic()

    def on_ultrasonic(self, topic, msg):
        with self.data_lock:
            self.ultrasonic[topic] = (float(msg.range), time.monotonic())

    def refresh_model_health(self):
        try:
            with urlopen(str(self.settings['local_model_url']).rstrip('/') +
                         '/health', timeout=2) as response:
                payload = json.load(response)
            self.llm_health = (bool(payload.get('inference', {}).get(
                'llm_ready')), time.monotonic())
        except (OSError, ValueError):
            self.llm_health = (False, time.monotonic())

    def on_map(self, msg):
        self.save('map', msg)

    def refresh_diagnostics(self):
        """使用方法：后台轮询静态地图和控制器状态，网页启动早于硬件时持续重试。"""
        if (self.settings['static_map'] and self.map is None and
                self.map_request is None and self.map_client.service_is_ready()):
            self.map_request = self.map_client.call_async(GetMap.Request())
            self.map_request.add_done_callback(self.on_map_response)
        if (self.controller_request is None and
                self.controller_client.service_is_ready()):
            self.controller_request = self.controller_client.call_async(
                ListControllers.Request())
            self.controller_request.add_done_callback(self.on_controller_response)

    def on_map_response(self, future):
        try:
            response = future.result()
            if response.map.info.width and response.map.info.height:
                self.on_map(response.map)
        except Exception as exc:
            self.get_logger().warning(f'地图服务读取失败，稍后重试：{exc}')
        finally:
            self.map_request = None

    def on_controller_response(self, future):
        try:
            states = {item.name: item.state for item in future.result().controller}
            self.save('controllers', states)
        except Exception as exc:
            self.get_logger().warning(f'控制器状态读取失败，稍后重试：{exc}')
        finally:
            self.controller_request = None

    def on_log(self, msg):
        with self.data_lock:
            self.log_sequence += 1
            self.logs.append({
                'id': self.log_sequence, 'time': msg.stamp.sec,
                'level': int(msg.level), 'node': msg.name,
                'message': msg.msg[:1200]})
            if len(self.logs) > 600:
                del self.logs[:len(self.logs) - 600]

    def on_image(self, camera, msg):
        with self.data_lock:
            self.image_seen[camera] = time.monotonic()
            if time.monotonic() - self.frame_requested.get(camera, 0.0) > 2.0:
                return
            previous = self.images.get(camera)
            if previous and time.monotonic() - previous[1] < 0.1:
                return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, 72])
            if ok:
                with self.data_lock:
                    self.images[camera] = (encoded.tobytes(), time.monotonic(),
                                           msg.header.stamp.sec, msg.header.stamp.nanosec)
        except Exception as exc:
            self.get_logger().warning(f'{camera} 图像编码失败: {exc}')

    def control_loop(self):
        state = self.control.snapshot()
        self.stop_state_pub.publish(Bool(data=state['stopped']))
        chassis_cmd = Twist()
        nav_cmd = Twist()
        if not state['stopped']:
            if state['mode'] == 'manual':
                chassis_cmd.linear.x, chassis_cmd.angular.z = self.control.manual_velocity()
            elif (state['mode'] in ('navigation', 'explore', 'follow')
                  and not self.task_paused):
                with self.data_lock:
                    if time.monotonic() - self.nav_stamp <= float(
                            self.settings['nav_command_timeout']):
                        nav_cmd.linear.x = self.last_nav.linear.x
                        nav_cmd.linear.y = self.last_nav.linear.y
                        nav_cmd.angular.z = self.last_nav.angular.z
                if state['mode'] == 'follow':
                    nav_cmd.linear.x = max(-self.follow_speed,
                                           min(self.follow_speed, nav_cmd.linear.x))
                with self.data_lock:
                    if time.monotonic() - self.filtered_nav_stamp <= float(
                            self.settings['nav_command_timeout']):
                        chassis_cmd = self.filtered_nav
        self.output.publish(nav_cmd)
        self.chassis_output.publish(chassis_cmd)

    def stop_all(self):
        self.control.stop()
        self.cancel_motion()

    def cancel_motion(self):
        """停止网页任务、Nav2 和定距动作；软件锁由调用方决定。"""
        self.task = ''
        self.task_paused = False
        self.target_id = ''
        self.preview = None
        self.navigation_status.cancel()
        self.output.publish(Twist())
        self.chassis_output.publish(Twist())
        with self.data_lock:
            self.nav_stamp = 0.0
            self.last_nav = Twist()
            self.filtered_nav_stamp = 0.0
            self.filtered_nav = Twist()
        if self.active_goal:
            self.active_goal.cancel_goal_async()
            self.active_goal = None
        if self.mission_stop_client.service_is_ready():
            self.mission_stop_client.call_async(Trigger.Request())
        for client in (self.nav_cancel_client, self.wheel_cancel_client):
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
        if self.wheel_stop_client.service_is_ready():
            self.wheel_stop_client.call_async(Trigger.Request())

    def _pose(self, x, y, yaw):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError('目标坐标无效')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    @staticmethod
    def _wait(future, timeout=8):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise TimeoutError('ROS 请求超时')
        if future.exception():
            raise future.exception()
        return future.result()

    def plan(self, x, y, yaw):
        target = {'x': x, 'y': y, 'yaw': yaw}
        self.preview = None
        self.navigation_status.planning(target)
        try:
            if not self.planner.wait_for_server(timeout_sec=1):
                raise RuntimeError('Nav2 规划服务不可用')
            pose = self._pose(x, y, yaw)
            goal = ComputePathToPose.Goal()
            goal.goal = pose
            handle = self._wait(self.planner.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Nav2 拒绝规划目标')
            result = self._wait(handle.get_result_async(), 15).result
            if result.error_code:
                reasons = {
                    202: '定位或坐标变换不可用', 203: '机器人位置不在地图内',
                    204: '目标不在地图内', 205: '机器人当前位置被障碍占据',
                    206: '目标位置被障碍占据', 207: '规划超时',
                    208: '目标没有可通行路径'}
                reason = reasons.get(result.error_code, '路径规划失败')
                detail = f'：{result.error_msg}' if result.error_msg else ''
                raise RuntimeError(f'{reason}{detail}')
            if not result.path.poses:
                raise RuntimeError('Nav2 未找到路径，目标可能不可达')
            path = [{'x': item.pose.position.x, 'y': item.pose.position.y}
                    for item in result.path.poses]
            self.preview = {'id': uuid.uuid4().hex, 'x': x, 'y': y,
                            'yaw': yaw, 'path': path,
                            'created': time.monotonic()}
            self.navigation_status.planned(target, len(path))
            return {key: value for key, value in self.preview.items()
                    if key != 'created'}
        except Exception as exc:
            self.navigation_status.fail(f'路径规划失败：{exc}')
            raise

    def navigate(self, preview_id):
        preview = self.preview
        if (not preview or preview['id'] != preview_id or
                time.monotonic() - preview['created'] > 60):
            self.navigation_status.fail('预演路径不存在或已过期，请重新规划')
            raise ValueError('预演路径不存在或已过期')
        if not self.navigator.wait_for_server(timeout_sec=1):
            self.navigation_status.fail('Nav2 导航服务不可用')
            raise RuntimeError('Nav2 导航服务不可用')
        if self.active_goal:
            self.active_goal.cancel_goal_async()
            self.active_goal = None
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(preview['x'], preview['y'], preview['yaw'])
        token = uuid.uuid4().hex
        self.navigation_status.start(token, {
            'x': preview['x'], 'y': preview['y'], 'yaw': preview['yaw']})
        try:
            handle = self._wait(self.navigator.send_goal_async(
                goal, feedback_callback=lambda message, token=token:
                self.on_goal_feedback(token, message.feedback)))
            if not handle.accepted:
                raise RuntimeError('Nav2 拒绝导航目标')
        except Exception as exc:
            self.navigation_status.finish(token, -1, error_msg=str(exc))
            raise
        self.active_goal = handle
        self.navigation_status.accepted(token)
        handle.get_result_async().add_done_callback(
            lambda future, handle=handle, token=token:
            self.on_goal_result(future, handle, token))
        self.preview = None
        return {'accepted': True}

    def on_goal_feedback(self, token, feedback):
        """使用方法：Nav2 每次反馈都更新距离、耗时和恢复次数；旧目标反馈自动丢弃。"""
        elapsed = feedback.navigation_time
        eta = feedback.estimated_time_remaining
        orientation = feedback.current_pose.pose.orientation
        current_yaw = 2 * math.atan2(orientation.z, orientation.w)
        self.navigation_status.feedback(
            token, feedback.distance_remaining,
            elapsed.sec + elapsed.nanosec / 1e9,
            eta.sec + eta.nanosec / 1e9,
            feedback.number_of_recoveries, current_yaw)

    def on_goal_result(self, future, handle, token):
        try:
            response = future.result()
            status = response.status
            error_code = response.result.error_code
            error_msg = response.result.error_msg
        except Exception as exc:
            status, error_code, error_msg = -1, 0, f'导航结果读取失败：{exc}'
        if self.active_goal is not handle:
            return
        self.active_goal = None
        self.navigation_status.finish(token, status, error_code, error_msg)
        if self.control.stopped or self.task_paused:
            return
        if self.task == 'explore' and status == 4:
            threading.Thread(target=self.advance_explore, daemon=True).start()
        elif self.task == 'navigation':
            self.task = ''

    def robot_pose(self):
        transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        return (transform.transform.translation.x,
                transform.transform.translation.y)

    def advance_explore(self):
        if self.control.stopped or self.task != 'explore' or self.task_paused:
            return
        try:
            with self.data_lock:
                if not self.map or (not self.settings['static_map'] and
                                    time.monotonic() - self.map[1] > 10):
                    raise RuntimeError('地图过期')
                grid = self.map[0]
            pose = self.robot_pose()
            candidates = [item for item in frontier_world_candidates(grid, pose, 8)
                          if item['distance'] > 0.75]
            if not candidates:
                self.task = 'exploration_complete'
                return
            target = candidates[0]
            yaw = math.atan2(target['y'] - pose[1], target['x'] - pose[0])
            result = self.plan(target['x'], target['y'], yaw)
            self.navigate(result['id'])
        except Exception as exc:
            self.get_logger().warning(f'探索暂停：{exc}')
            self.task_paused = True
            self.output.publish(Twist())

    def task_loop(self):
        if self.task != 'follow' or self.task_paused or self.control.stopped:
            return
        with self.data_lock:
            if not self.detections or time.monotonic() - self.detections[1] > float(
                    self.settings['follow_lost_timeout']):
                self.stop_all()
                return
            target = next((d for d in self.detections[0].detections
                           if d.id == self.target_id and d.has_depth and
                           d.has_map_position), None)
        if target is None:
            self.stop_all()
            return
        point = (target.map_position.point.x, target.map_position.point.y)
        if self.last_follow_target and math.dist(point, self.last_follow_target) < float(
                self.settings['follow_replan_distance']):
            return
        if time.monotonic() - self.last_follow_plan < 1:
            return
        self.last_follow_plan = time.monotonic()
        self.last_follow_target = point
        threading.Thread(target=self.advance_follow, args=(point,), daemon=True).start()

    def advance_follow(self, point):
        try:
            goal = standoff_pose(self.robot_pose(), point,
                                self.follow_distance - 0.25, 0.25)
            result = self.plan(goal['x'], goal['y'], goal['yaw'])
            if self.task == 'follow' and not self.control.stopped:
                self.navigate(result['id'])
        except Exception as exc:
            self.get_logger().warning(f'跟随暂停：{exc}')
            self.task_paused = True
            self.output.publish(Twist())

    def start_task(self, task, target_id='', follow_distance=1.5,
                   max_speed=0.04):
        if self.control.stopped:
            raise ValueError('软件停止锁尚未解除')
        if task not in ('explore', 'follow'):
            raise ValueError('任务类型无效')
        if task == 'follow':
            if not 0.8 <= follow_distance <= 3:
                raise ValueError('跟随距离超出范围')
            if not 0.02 <= max_speed <= self.control.linear_max:
                raise ValueError('跟随速度超出范围')
            with self.data_lock:
                detections = (self.detections[0].detections
                              if self.detections and
                              time.monotonic() - self.detections[1] < 2 else [])
                target = next((d for d in detections if d.id == target_id and
                               d.has_depth and d.has_map_position), None)
            if target is None:
                raise ValueError('跟随目标缺少可靠深度与地图坐标')
        self.cancel_motion()
        self.task = task
        self.task_paused = False
        if task == 'explore':
            threading.Thread(target=self.advance_explore, daemon=True).start()
        else:
            self.target_id = target_id
            self.follow_distance = follow_distance
            self.follow_speed = max_speed
            self.last_follow_target = None
        return {'task': self.task}

    def status(self):
        with self.data_lock:
            now = time.monotonic()
            result = {'control': self.control.snapshot(),
                      'mode_source': self.settings['mode'],
                      'vision_enabled': self.vision_enabled,
                      'uptime_seconds': round(now - self.start_time),
                      'task': self.task, 'images': {}, 'map_ready': False,
                      'task_paused': self.task_paused,
                      'navigation': self.navigation_status.snapshot(),
                      'external_model_configured': bool(
                          self.settings['external_model_url']),
                      'detections': []}
            result['local_model_ready'] = bool(
                self.llm_health and self.llm_health[0] and
                now - self.llm_health[1] < 10)
            result['ros_nodes'] = [namespace.rstrip('/') + '/' + name
                                   for name, namespace in
                                   self.get_node_names_and_namespaces()]
            for name, stamp in self.image_seen.items():
                result['images'][name] = {'fresh': now - stamp < 2}
            if self.map:
                map_age = now - self.map[1]
                result['map_ready'] = (self.settings['static_map'] or
                                       map_age < 10)
                result['map_age_seconds'] = round(map_age, 1)
            result['map_source'] = ('已接收地图' if self.map else
                '等待地图服务' if self.settings['static_map'] and
                not self.map_client.service_is_ready() else
                '地图服务已就绪，等待地图数据' if self.settings['static_map'] else
                '等待在线建图')
            result['controllers'] = {
                'available': bool(self.controllers and now - self.controllers[1] < 5),
                'states': self.controllers[0] if self.controllers else {}}
            if self.odom:
                odom, stamp = self.odom
                result['odom'] = {'fresh': now - stamp < 1 and
                                  math.isfinite(odom.twist.twist.linear.x),
                                  'x': self.json_number(odom.pose.pose.position.x),
                                  'y': self.json_number(odom.pose.pose.position.y),
                                  'linear': self.json_number(odom.twist.twist.linear.x),
                                  'angular': self.json_number(odom.twist.twist.angular.z)}
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_link', Time())
                pose = transform.transform.translation
                orientation = transform.transform.rotation
                stamp_ns = (transform.header.stamp.sec * 1_000_000_000 +
                            transform.header.stamp.nanosec)
                age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                result['pose'] = {'fresh': 0 <= age <= 2 and
                                  math.isfinite(pose.x) and math.isfinite(pose.y),
                                  'x': self.json_number(pose.x),
                                  'y': self.json_number(pose.y),
                                  'yaw': self.json_number(
                                      2 * math.atan2(orientation.z, orientation.w))}
            except Exception:
                result['pose'] = {'fresh': False}
            if self.hardware:
                hardware, stamp = self.hardware
                result['hardware'] = {
                    'fresh': now - stamp < 2,
                    'dmc0_connected': hardware.dmc0_connected,
                    'dmc1_connected': hardware.dmc1_connected,
                    'se2_connected': hardware.se2_connected,
                    'connected': all((hardware.dmc0_connected,
                                      hardware.dmc1_connected,
                                      hardware.se2_connected)),
                    'outputs_enabled': hardware.outputs_enabled,
                    'watchdog_ok': hardware.watchdog_ok,
                    'manual_recovery_required': hardware.manual_recovery_required,
                    'fault': hardware.fault}
            result['imu_fresh'] = bool(self.imu and now - self.imu[1] < 2)
            result['ultrasonic'] = {
                topic.rsplit('/', 1)[-1]: {
                    'received': now - sample[1] < 1,
                    'fresh': now - sample[1] < 1 and math.isfinite(sample[0]),
                    'range_m': self.json_number(sample[0])}
                for topic, sample in self.ultrasonic.items()}
            if self.safety:
                result['safety'] = {'fresh': now - self.safety[1] < 2,
                                    'message': self.safety[0]}
            if self.mission:
                result['mission'] = {'fresh': now - self.mission[1] < 3,
                                     'message': self.mission[0]}
            result['system'] = self.system_status()
            if self.detections:
                detection_msg, stamp = self.detections
                result['detection_fresh'] = now - stamp < 2
                if result['detection_fresh']:
                    result['detections'] = [{
                        'id': d.id, 'name': d.label_zh or d.class_name,
                        'confidence': self.json_number(round(float(d.confidence), 2)),
                        'bbox': [d.bbox_x_min, d.bbox_y_min,
                                 d.bbox_x_max, d.bbox_y_max],
                        'distance': self.json_number(d.distance) if d.has_depth else None,
                        'map_position': [self.json_number(d.map_position.point.x),
                                         self.json_number(d.map_position.point.y)]
                        if d.has_map_position and all(math.isfinite(value) for
                            value in (d.map_position.point.x,
                                      d.map_position.point.y)) else None}
                        for d in detection_msg.detections]
                    result['image_size'] = [detection_msg.image_width,
                                            detection_msg.image_height]
            return result

    @staticmethod
    def system_status():
        result = {'cpu_load': round(os.getloadavg()[0], 2)}
        try:
            with open('/proc/meminfo', encoding='utf-8') as stream:
                mem = dict((line.split(':')[0], int(line.split(':')[1].split()[0]))
                           for line in stream if ':' in line)
            result['memory_percent'] = round(100 * (1 - mem['MemAvailable'] /
                                                     mem['MemTotal']))
            total, used, _ = shutil.disk_usage('/workspace')
            result['disk_percent'] = round(100 * used / total)
            temperatures = list(Path('/sys/class/thermal').glob('thermal_zone*/temp'))
            values = [int(path.read_text().strip()) / 1000 for path in temperatures]
            result['temperature_c'] = round(max(values), 1) if values else None
        except (OSError, ValueError, KeyError, ZeroDivisionError):
            pass
        return result

    def parse_model(self, text, provider):
        if not 1 <= len(text.strip()) <= 500:
            raise ValueError('任务文字长度需为 1～500 字')
        with self.data_lock:
            if self.detections and time.monotonic() - self.detections[1] < 2:
                scene = [{'id': d.id, 'label_zh': d.label_zh,
                          'class_name': d.class_name} for d in
                         self.detections[0].detections]
            else:
                scene = []
        if provider == 'local':
            proposal = QwenClient(str(self.settings['local_model_url']),
                                  timeout=180).chat(
                uuid.uuid4().hex, text, detections=scene)
        elif provider == 'external':
            endpoint = str(self.settings['external_model_url']).rstrip('/')
            if not endpoint:
                raise RuntimeError('外部大模型未配置')
            token = os.getenv(str(self.settings['external_model_token_env']), '')
            target = endpoint if endpoint.endswith('/chat/completions') else (
                endpoint + '/v1/chat/completions')
            request = UrlRequest(target, data=json.dumps({
                'model': self.settings['external_model_name'],
                'messages': [{'role': 'system', 'content': SYSTEM_INSTRUCTION},
                             {'role': 'user', 'content': json.dumps({
                                 'scene': scene, 'current_request': text},
                                 ensure_ascii=False)}], 'stream': False,
            }).encode(), headers={'Content-Type': 'application/json',
                                 'Authorization': 'Bearer ' + token})
            with urlopen(request, timeout=90) as response:
                payload = json.load(response)
            proposal = parse_model_response(
                payload['choices'][0]['message']['content'])
        else:
            raise ValueError('未知大模型来源')
        decision = CommandPolicy.authorize(text, proposal, scene)
        identifier = uuid.uuid4().hex
        if decision.action:
            self.model_proposals[identifier] = {
                'action': decision.action, 'created': time.monotonic(),
                'scene': scene}
        return {'answer': decision.answer, 'proposal_id': identifier
                if decision.action else None,
                'action': decision.action.name if decision.action else None}

    def confirm_model(self, identifier):
        proposal = self.model_proposals.pop(identifier, None)
        if not proposal or time.monotonic() - proposal['created'] > 60:
            raise ValueError('模型任务提案已过期')
        if self.control.stopped:
            raise ValueError('软件停止锁尚未解除')
        action = proposal['action']
        if action.name == 'follow_person':
            person = next((d for d in self.status()['detections']
                           if re.search(r'person|人', d['name'], re.I)), None)
            if not person:
                raise ValueError('当前未检测到人员')
            self.control.set_mode('follow')
            return self.start_task('follow', person['id'])
        if action.name == 'explore':
            self.control.set_mode('explore')
            return self.start_task('explore')
        label = action.arguments['label']
        detection = next((d for d in self.status()['detections']
                          if d['name'] == label and d['map_position'] and
                          d['distance']), None)
        if not detection:
            raise ValueError('目标不在当前可靠检测结果中')
        pose = self.robot_pose()
        goal = standoff_pose(pose, detection['map_position'], 0.5, 0.25)
        result = self.plan(goal['x'], goal['y'], goal['yaw'])
        self.control.set_mode('navigation')
        return self.navigate(result['id'])

    def map_data(self):
        with self.data_lock:
            # 页面可以继续显示最近一次地图；导航规划仍在任务入口检查地图新鲜度。
            if not self.map:
                raise RuntimeError('地图数据尚未收到')
            grid = self.map[0]
            return {'width': grid.info.width, 'height': grid.info.height,
                    'resolution': grid.info.resolution,
                    'origin': {'x': grid.info.origin.position.x,
                               'y': grid.info.origin.position.y,
                               'z': grid.info.origin.orientation.z,
                               'w': grid.info.origin.orientation.w},
                    'data': list(grid.data)}


def create_app(robot, web_dir=None):
    """通过 Unix Socket 交给宿主机 9100 网关转发。"""
    web_dir = Path(web_dir or Path(get_package_share_directory(
        'robot_web')) / 'web')
    app = FastAPI(docs_url=None, redoc_url=None)

    @app.get('/')
    def home():
        return FileResponse(web_dir / 'index.html')

    @app.get('/status')
    @app.get('/console')
    def page():
        return FileResponse(web_dir / 'index.html')

    @app.get('/app.js')
    def script():
        return FileResponse(web_dir / 'app.js', media_type='text/javascript')

    @app.get('/style.css')
    def style():
        return FileResponse(web_dir / 'style.css', media_type='text/css')

    @app.get('/api/status')
    def status():
        return robot.status()

    @app.get('/api/navigation-status')
    def navigation_status():
        """使用方法：导航控制区每 0.5 秒读取最新 Nav2 Action 反馈。"""
        return robot.navigation_status.snapshot()

    @app.get('/api/map')
    def map_data():
        try:
            return robot.map_data()
        except RuntimeError as exc:
            raise HTTPException(503, str(exc)) from exc

    @app.get('/api/frame/{camera}')
    def frame(camera: str):
        if camera not in ('left', 'right', 'depth'):
            raise HTTPException(404, '未知相机')
        with robot.data_lock:
            robot.frame_requested[camera] = time.monotonic()
            item = robot.images.get(camera)
            if not item or time.monotonic() - item[1] > 2:
                raise HTTPException(503, '图像断流')
            return Response(item[0], media_type='image/jpeg',
                            headers={'Cache-Control': 'no-store'})

    @app.get('/api/logs')
    def logs(after: int = 0, level: int = 0):
        with robot.data_lock:
            return {'items': [item for item in robot.logs
                              if item['id'] > after and item['level'] >= level]}

    @app.post('/api/heartbeat')
    def heartbeat():
        robot.control.heartbeat()
        return {'ok': True}

    @app.post('/api/stop')
    def stop():
        robot.stop_all()
        return {'stopped': True}

    @app.post('/api/reset-stop')
    def reset_stop():
        try:
            robot.control.reset_stop()
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        return {'stopped': False}

    @app.post('/api/mode')
    def mode(body: ModeBody):
        if robot.control.stopped:
            raise HTTPException(423, '软件停止锁尚未解除')
        robot.cancel_motion()
        try:
            robot.control.set_mode(body.mode)
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        return robot.control.snapshot()

    @app.post('/api/drive')
    def drive(body: DriveBody):
        try:
            robot.control.command(body.linear, body.angular,
                                  body.client_id, body.sequence)
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        return {'ok': True}

    @app.post('/api/plan')
    def plan(body: GoalBody):
        if robot.control.stopped:
            raise HTTPException(423, '软件停止锁尚未解除')
        try:
            return robot.plan(body.x, body.y, body.yaw)
        except (ValueError, RuntimeError, TimeoutError) as exc:
            raise HTTPException(422, str(exc)) from exc

    @app.post('/api/navigate/{preview_id}')
    def navigate(preview_id: str):
        if robot.control.stopped:
            raise HTTPException(423, '软件停止锁尚未解除')
        robot.control.set_mode('navigation')
        try:
            return robot.navigate(preview_id)
        except (ValueError, RuntimeError, TimeoutError) as exc:
            raise HTTPException(422, str(exc)) from exc

    @app.post('/api/cancel')
    def cancel():
        robot.cancel_motion()
        if not robot.control.stopped:
            robot.control.set_mode('manual')
        return {'canceled': True}

    @app.post('/api/save-map')
    def save_map():
        if not robot.snapshot_client.service_is_ready():
            raise HTTPException(503, '地图快照服务不可用')
        try:
            result = robot._wait(robot.snapshot_client.call_async(Trigger.Request()), 15)
        except TimeoutError as exc:
            raise HTTPException(504, str(exc)) from exc
        return {'success': result.success, 'message': result.message}

    @app.post('/api/task')
    def task(body: TaskBody):
        try:
            if body.task in ('explore-start', 'follow-start'):
                mode = 'explore' if body.task.startswith('explore') else 'follow'
                if robot.control.stopped:
                    raise ValueError('软件停止锁尚未解除')
                robot.control.set_mode(mode)
                return robot.start_task(mode, body.target_id,
                                        body.follow_distance, body.max_speed)
            if body.task == 'pause':
                robot.task_paused = True
                robot.output.publish(Twist())
                with robot.data_lock:
                    robot.nav_stamp = 0.0
                if robot.active_goal:
                    robot.active_goal.cancel_goal_async()
                    robot.active_goal = None
                return {'task': robot.task, 'paused': True}
            if body.task == 'resume':
                if robot.control.stopped:
                    raise ValueError('软件停止锁尚未解除')
                if not robot.task or not robot.task_paused:
                    raise ValueError('没有可继续的任务')
                robot.task_paused = False
                with robot.data_lock:
                    robot.nav_stamp = 0.0
                if robot.task == 'explore':
                    threading.Thread(target=robot.advance_explore,
                                     daemon=True).start()
                return {'task': robot.task, 'paused': False}
            if body.task == 'stop':
                robot.cancel_motion()
                if not robot.control.stopped:
                    robot.control.set_mode('manual')
                return {'task': ''}
            raise ValueError('不支持的任务操作')
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc

    @app.post('/api/model/parse')
    def parse_model(body: ModelBody):
        try:
            return robot.parse_model(body.text, body.provider)
        except (ValueError, RuntimeError, TimeoutError, KeyError,
                IndexError, OSError) as exc:
            raise HTTPException(422, str(exc)) from exc

    @app.post('/api/model/confirm')
    def confirm_model(body: ConfirmModelBody):
        try:
            return robot.confirm_model(body.proposal_id)
        except (ValueError, RuntimeError, TimeoutError) as exc:
            raise HTTPException(422, str(exc)) from exc

    return app


def main():
    rclpy.init()
    robot = RobotConsole()
    socket = Path(str(robot.settings['socket_path']))
    socket.parent.mkdir(parents=True, exist_ok=True)
    socket.unlink(missing_ok=True)
    app = create_app(robot)
    server = uvicorn.Server(uvicorn.Config(app, uds=str(socket),
                                           log_level='warning'))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    for _ in range(100):
        if socket.exists():
            # 宿主机网关以普通用户身份连接容器创建的 Socket。
            socket.chmod(0o666)
            break
        time.sleep(0.02)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(robot)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        robot.stop_all()
        server.should_exit = True
        thread.join(timeout=2)
        socket.unlink(missing_ok=True)
        executor.shutdown()
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
