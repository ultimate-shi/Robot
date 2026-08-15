#!/usr/bin/env python3
"""把探索、跟随和前往物体统一转换为 Nav2 路径预演."""

import json
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot.mission.frontier import frontier_world_candidates, standoff_pose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class BrainMission(Node):
    """任务层只生成和验证导航目标，默认绝不驱动底盘."""

    def __init__(self):
        super().__init__('brain_mission')
        defaults = {
            'motion_enabled': False,
            'request_topic': '/brain/mission_request',
            'detections_topic': '/perception/semantic_detections',
            'map_topic': '/map',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'planner_service': '/compute_path_to_pose',
            'surface_clearance': 0.5,
            'robot_radius': 0.25,
            'follow_distance': 1.2,
            'follow_update_distance': 0.4,
            'person_lost_timeout': 2.0,
            'frontier_min_cells': 8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.motion_enabled = bool(
            self.get_parameter('motion_enabled').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.surface_clearance = float(
            self.get_parameter('surface_clearance').value)
        self.robot_radius = float(
            self.get_parameter('robot_radius').value)
        self.follow_distance = float(
            self.get_parameter('follow_distance').value)
        self.follow_update_distance = float(
            self.get_parameter('follow_update_distance').value)
        self.person_lost_timeout = float(
            self.get_parameter('person_lost_timeout').value)
        self.frontier_min_cells = int(
            self.get_parameter('frontier_min_cells').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None
        self.detections = {}
        self.last_detection_received = 0.0
        self.active_task = ''
        self.active_target_id = ''
        self.last_follow_target = None
        self.plan_generation = 0

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._map_callback, map_qos)
        self.create_subscription(
            String, str(self.get_parameter('detections_topic').value),
            self._detections_callback, 10)
        self.create_subscription(
            String, str(self.get_parameter('request_topic').value),
            self._request_callback, 20)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/mission/preview_goal', 10)
        self.path_pub = self.create_publisher(
            Path, '/mission/preview_path', 10)
        self.status_pub = self.create_publisher(
            String, '/mission/status', 20)
        self.planner = self.create_client(
            ComputePathToPose,
            str(self.get_parameter('planner_service').value))
        self.create_timer(0.25, self._follow_timer)
        self._publish_status(
            'stopped', '大脑已启动：运动输出被强制关闭',
            motion_enabled=self.motion_enabled)

    def _map_callback(self, msg):
        self.latest_map = msg

    def _detections_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get('detections', [])
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.detections = {
            str(item.get('id')): item for item in detections
            if item.get('id')
        }
        self.last_detection_received = time.monotonic()

    def _request_callback(self, msg):
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._publish_status('error', f'任务请求不是有效 JSON: {exc}')
            return
        request_type = str(request.get('type', 'preview'))
        if request_type in ('stop', 'cancel', 'release'):
            self._clear_task('stopped', '任务和路径预演已清除')
            return
        task = str(request.get('task', ''))
        if task not in ('goto_object', 'follow_person', 'explore'):
            self._publish_status('rejected', f'不支持的任务类型: {task}')
            return
        self.active_task = task
        self.active_target_id = str(request.get('target_id', ''))
        self.last_follow_target = None
        if task == 'explore':
            self._preview_frontier(request)
        else:
            self._preview_detection(request)

    def _preview_detection(self, request):
        target = self._select_detection(
            self.active_target_id,
            person_only=self.active_task == 'follow_person')
        if target is None:
            self._publish_status(
                'rejected', '没有找到带可靠地图坐标的目标',
                mission_id=request.get('mission_id', ''))
            return
        self.active_target_id = str(target['id'])
        target_xy = self._target_xy(target)
        robot_xy = self._robot_xy()
        if target_xy is None or robot_xy is None:
            self._publish_status('waiting_tf', '目标或机器人 map 坐标不可用')
            return
        clearance = (self.follow_distance if self.active_task == 'follow_person'
                     else self.surface_clearance)
        pose = standoff_pose(
            robot_xy, target_xy, clearance, self.robot_radius)
        self.last_follow_target = target_xy
        self._request_path(pose, request.get('mission_id', ''), target)

    def _preview_frontier(self, request):
        robot_xy = self._robot_xy()
        if self.latest_map is None or robot_xy is None:
            self._publish_status('waiting_map', '地图或机器人 map 位姿尚未就绪')
            return
        try:
            candidates = frontier_world_candidates(
                self.latest_map, robot_xy, self.frontier_min_cells)
        except ValueError as exc:
            self._publish_status('error', f'前沿计算失败: {exc}')
            return
        if not candidates:
            self._publish_status('exploration_complete', '没有剩余可达前沿')
            return
        target = candidates[0]
        yaw = math.atan2(target['y'] - robot_xy[1],
                         target['x'] - robot_xy[0])
        self._request_path(
            {'x': target['x'], 'y': target['y'], 'yaw': yaw},
            request.get('mission_id', ''),
            {'id': 'frontier', 'label_zh': '探索前沿', **target})

    def _request_path(self, target, mission_id, semantic_target):
        goal = self._pose(target['x'], target['y'], target['yaw'])
        self.goal_pub.publish(goal)
        self.plan_generation += 1
        generation = self.plan_generation
        if not self.planner.service_is_ready():
            self._publish_status(
                'preview_goal', 'Nav2 规划服务未就绪，已显示候选目标',
                mission_id=mission_id, target=semantic_target,
                goal=self._pose_dict(goal))
            return
        request = ComputePathToPose.Request()
        request.goal = goal
        request.planner_id = 'GridBased'
        request.use_start = False
        future = self.planner.call_async(request)
        future.add_done_callback(lambda result: self._path_ready(
            result, generation, mission_id, semantic_target, goal))
        self._publish_status(
            'planning', '正在通过 Nav2 验证候选目标',
            mission_id=mission_id, target=semantic_target,
            goal=self._pose_dict(goal))

    def _path_ready(self, future, generation, mission_id, target, goal):
        if generation != self.plan_generation:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status('planning_failed', f'规划服务失败: {exc}')
            return
        if response is None or not response.path.poses:
            self._publish_status(
                'unreachable', 'Nav2 未找到可达路径',
                mission_id=mission_id, target=target,
                goal=self._pose_dict(goal))
            return
        self.path_pub.publish(response.path)
        self._publish_status(
            'preview_ready', '目标和路径预演已就绪，未输出运动命令',
            mission_id=mission_id, target=target,
            goal=self._pose_dict(goal), path_points=len(response.path.poses),
            motion_enabled=self.motion_enabled)

    def _follow_timer(self):
        if self.active_task != 'follow_person' or not self.active_target_id:
            return
        if time.monotonic() - self.last_detection_received > self.person_lost_timeout:
            self._clear_task('lost_target', '人员丢失超过两秒，已清除跟随目标')
            return
        target = self.detections.get(self.active_target_id)
        target_xy = self._target_xy(target) if target else None
        if target_xy is None:
            return
        if (self.last_follow_target is None or math.hypot(
                target_xy[0] - self.last_follow_target[0],
                target_xy[1] - self.last_follow_target[1])
                >= self.follow_update_distance):
            self._preview_detection({'mission_id': 'follow-update'})

    def _select_detection(self, target_id, person_only=False):
        if target_id:
            target = self.detections.get(target_id)
            if target and (not person_only or self._is_person(target)):
                return target
            return None
        candidates = [item for item in self.detections.values()
                      if self._target_xy(item) is not None]
        if person_only:
            candidates = [item for item in candidates if self._is_person(item)]
        if not candidates:
            return None
        return min(candidates, key=lambda item: float(
            item.get('distance', float('inf'))))

    @staticmethod
    def _is_person(target):
        return str(target.get('class_name', '')).lower() == 'person'

    @staticmethod
    def _target_xy(target):
        if not isinstance(target, dict):
            return None
        point = target.get('map_position')
        if not isinstance(point, dict):
            return None
        try:
            x, y = float(point['x']), float(point['y'])
        except (KeyError, TypeError, ValueError):
            return None
        return (x, y) if math.isfinite(x) and math.isfinite(y) else None

    def _robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    @staticmethod
    def _pose_dict(pose):
        return {
            'frame_id': pose.header.frame_id,
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'qz': pose.pose.orientation.z,
            'qw': pose.pose.orientation.w,
        }

    def _clear_task(self, state, message):
        self.active_task = ''
        self.active_target_id = ''
        self.last_follow_target = None
        self.plan_generation += 1
        self.path_pub.publish(Path())
        self._publish_status(state, message)

    def _publish_status(self, state, message, **extra):
        payload = {
            'state': state,
            'message': message,
            'task': self.active_task,
            'target_id': self.active_target_id,
            **extra,
        }
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = BrainMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
