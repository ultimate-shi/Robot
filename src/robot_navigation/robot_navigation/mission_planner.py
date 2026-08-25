#!/usr/bin/env python3
"""使用方法：由 mission_preview.launch.py 把探索、跟随和物体任务转换为 Nav2 路径预演。"""

import json
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_navigation.frontier import frontier_world_candidates, standoff_pose
from robot_interfaces.action import PlanMission
from robot_interfaces.msg import MissionState, SemanticDetectionArray
from robot_interfaces.srv import ConfirmMission, SetDetectionMode
from std_msgs.msg import String
from std_srvs.srv import Trigger
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
            'goal_boundary_margin': 0.3,
            'idle_detection_mode': 'on_demand',
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
        self.goal_boundary_margin = float(
            self.get_parameter('goal_boundary_margin').value)
        self.idle_detection_mode = str(
            self.get_parameter('idle_detection_mode').value)
        if self.idle_detection_mode not in ('on_demand', 'continuous'):
            self.get_logger().warning(
                'idle_detection_mode 无效，已回退为 on_demand')
            self.idle_detection_mode = 'on_demand'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None
        self.detections = {}
        self.last_detection_received = 0.0
        self.active_task = ''
        self.active_target_id = ''
        self.last_follow_target = None
        self.plan_generation = 0
        self.mission_confirmed = False
        self.preview_cache = {}
        self.callback_group = ReentrantCallbackGroup()

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._map_callback, map_qos)
        self.create_subscription(
            SemanticDetectionArray,
            str(self.get_parameter('detections_topic').value),
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
        self.typed_status_pub = self.create_publisher(
            MissionState, '/mission/state', 20)
        self.navigation_goal_pub = self.create_publisher(
            PoseStamped, '/mission/navigation_goal', 10)
        self.planner = ActionClient(
            self, ComputePathToPose,
            str(self.get_parameter('planner_service').value),
            callback_group=self.callback_group)
        self.detection_mode = self.create_client(
            SetDetectionMode, '/perception/set_detection_mode',
            callback_group=self.callback_group)
        self.plan_action = ActionServer(
            self, PlanMission, '/mission/plan', self._execute_plan,
            callback_group=self.callback_group)
        self.create_service(
            ConfirmMission, '/mission/confirm', self._confirm_service,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/mission/cancel', self._cancel_service,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/mission/stop', self._stop_service,
            callback_group=self.callback_group)
        self.create_timer(0.25, self._follow_timer)
        self._publish_status(
            'stopped', '大脑已启动：运动输出被强制关闭',
            motion_enabled=self.motion_enabled)

    def _map_callback(self, msg):
        self.latest_map = msg

    def _detections_callback(self, msg):
        detections = []
        for item in msg.detections:
            detections.append({
                'id': item.id, 'class_name': item.class_name,
                'label_zh': item.label_zh, 'confidence': item.confidence,
                'distance': item.distance if item.has_depth else None,
                'map_position': ({
                    'x': item.map_position.point.x,
                    'y': item.map_position.point.y,
                    'z': item.map_position.point.z,
                    'frame_id': item.map_position.header.frame_id,
                } if item.has_map_position else None),
            })
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
        target = self._bound_target_to_map(target)
        goal = self._pose(target['x'], target['y'], target['yaw'])
        self.goal_pub.publish(goal)
        self.plan_generation += 1
        generation = self.plan_generation
        if not self.planner.server_is_ready():
            self._publish_status(
                'preview_goal', 'Nav2 规划服务未就绪，已显示候选目标',
                mission_id=mission_id, target=semantic_target,
                goal=self._pose_dict(goal))
            return
        request = self._planner_goal(goal)
        future = self.planner.send_goal_async(request)
        future.add_done_callback(lambda result: self._planner_goal_ready(
            result, generation, mission_id, semantic_target, goal))
        self._publish_status(
            'planning', '正在通过 Nav2 验证候选目标',
            mission_id=mission_id, target=semantic_target,
            goal=self._pose_dict(goal))

    def _planner_goal_ready(
            self, future, generation, mission_id, target, goal):
        """等待 Nav2 接受路径计算 Action，再读取最终 Path。"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status('planning_failed', f'规划请求失败: {exc}')
            return
        if not goal_handle.accepted:
            self._publish_status('planning_failed', 'Nav2 拒绝路径规划请求')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._path_ready(
            result, generation, mission_id, target, goal))

    def _path_ready(self, future, generation, mission_id, target, goal):
        if generation != self.plan_generation:
            return
        try:
            response = future.result().result
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
        if (not self.mission_confirmed
                or self.active_task != 'follow_person'
                or not self.active_target_id):
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

    def _bound_target_to_map(self, target):
        """将贴近 OccupancyGrid 外缘的目标收进安全边界内。"""
        grid = self.latest_map
        if grid is None:
            return dict(target)
        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return dict(target)
        margin = max(resolution, self.goal_boundary_margin)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        lower_x = origin_x + margin
        lower_y = origin_y + margin
        upper_x = origin_x + width * resolution - margin
        upper_y = origin_y + height * resolution - margin
        # 极小地图还容不下两侧边界时，只保留一个像素的内缩量。
        if lower_x > upper_x:
            lower_x = upper_x = origin_x + width * resolution / 2.0
        if lower_y > upper_y:
            lower_y = upper_y = origin_y + height * resolution / 2.0
        bounded = dict(target)
        bounded['x'] = min(max(float(target['x']), lower_x), upper_x)
        bounded['y'] = min(max(float(target['y']), lower_y), upper_y)
        if (bounded['x'] != float(target['x'])
                or bounded['y'] != float(target['y'])):
            self.get_logger().warning(
                '预演目标贴近地图边界，已由 '
                f"({float(target['x']):.3f}, {float(target['y']):.3f}) "
                f"调整为 ({bounded['x']:.3f}, {bounded['y']:.3f})")
        return bounded

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
        self.mission_confirmed = False
        self.plan_generation += 1
        self.path_pub.publish(Path())
        self._set_detection_mode(
            self.idle_detection_mode == 'continuous')
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
        typed = MissionState()
        typed.header.stamp = self.get_clock().now().to_msg()
        typed.header.frame_id = self.map_frame
        typed.mission_id = str(extra.get('mission_id', ''))
        typed.mission_type = self._mission_type(self.active_task)
        typed.phase = str(state)
        typed.target_id = self.active_target_id
        typed.message = str(message)
        typed.motion_enabled = self.motion_enabled
        self.typed_status_pub.publish(typed)

    @staticmethod
    def _mission_type(task):
        return {
            'goto_object': MissionState.TYPE_GOTO_OBJECT,
            'follow_person': MissionState.TYPE_FOLLOW_PERSON,
            'explore': MissionState.TYPE_EXPLORE,
        }.get(task, MissionState.TYPE_NONE)

    def _execute_plan(self, goal_handle):
        """同步生成一次预览结果；多线程执行器负责处理 Nav2 Service 响应。"""
        request = goal_handle.request
        result = PlanMission.Result()
        task = {
            PlanMission.Goal.GOTO_OBJECT: 'goto_object',
            PlanMission.Goal.FOLLOW_PERSON: 'follow_person',
            PlanMission.Goal.EXPLORE: 'explore',
        }.get(request.mission_type, '')
        if not task:
            goal_handle.abort()
            result.error_code = 'INVALID_TASK'
            result.message = '不支持的任务类型'
            return result
        feedback = PlanMission.Feedback()
        feedback.phase = 'preparing'
        feedback.message = '正在生成候选目标'
        goal_handle.publish_feedback(feedback)
        self.active_task = task
        self.active_target_id = request.target_id
        self.mission_confirmed = False
        semantic_target = None
        if task == 'explore':
            robot_xy = self._robot_xy()
            if self.latest_map is None or robot_xy is None:
                return self._abort_plan(goal_handle, result, 'WAITING_MAP',
                                        '地图或机器人位姿尚未就绪')
            candidates = frontier_world_candidates(
                self.latest_map, robot_xy, self.frontier_min_cells)
            if not candidates:
                return self._abort_plan(goal_handle, result, 'NO_FRONTIER',
                                        '没有剩余可达前沿')
            semantic_target = {'id': 'frontier', **candidates[0]}
            target = {'x': candidates[0]['x'], 'y': candidates[0]['y'],
                      'yaw': math.atan2(candidates[0]['y'] - robot_xy[1],
                                        candidates[0]['x'] - robot_xy[0])}
        else:
            semantic_target = self._select_detection(
                request.target_id, person_only=task == 'follow_person')
            robot_xy = self._robot_xy()
            target_xy = self._target_xy(semantic_target)
            if semantic_target is None or robot_xy is None or target_xy is None:
                return self._abort_plan(goal_handle, result, 'NO_TARGET',
                                        '没有找到带可靠地图坐标的目标')
            self.active_target_id = str(semantic_target['id'])
            clearance = self.follow_distance if task == 'follow_person' else self.surface_clearance
            target = standoff_pose(robot_xy, target_xy, clearance, self.robot_radius)
        target = self._bound_target_to_map(target)
        pose = self._pose(target['x'], target['y'], target['yaw'])
        self.goal_pub.publish(pose)
        feedback.phase = 'planning'
        feedback.message = '正在通过 Nav2 验证路径'
        goal_handle.publish_feedback(feedback)
        if not self.planner.wait_for_server(timeout_sec=1.0):
            return self._abort_plan(goal_handle, result, 'PLANNER_UNAVAILABLE',
                                    'Nav2 规划服务未就绪', pose)
        path_request = self._planner_goal(pose)
        try:
            send_future = self.planner.send_goal_async(path_request)
            planner_handle = self._wait_future(send_future, 3.0)
            if not planner_handle.accepted:
                return self._abort_plan(
                    goal_handle, result, 'PLANNING_REJECTED',
                    'Nav2 拒绝路径规划请求', pose)
            wrapped = self._wait_future(
                planner_handle.get_result_async(), 10.0)
            response = wrapped.result
        except Exception as exc:
            return self._abort_plan(goal_handle, result, 'PLANNING_FAILED', str(exc), pose)
        if response is None or not response.path.poses:
            return self._abort_plan(goal_handle, result, 'UNREACHABLE', 'Nav2 未找到可达路径', pose)
        self.path_pub.publish(response.path)
        self.preview_cache[request.mission_id] = {
            'goal': pose, 'path': response.path, 'task': task,
            'target_id': self.active_target_id, 'created_at': time.monotonic(),
        }
        result.success = True
        result.message = '目标和路径预演已就绪，未输出运动命令'
        result.goal = pose
        result.path = response.path
        goal_handle.succeed()
        self._publish_status('preview_ready', result.message,
                             mission_id=request.mission_id)
        return result

    @staticmethod
    def _planner_goal(pose):
        """构造 Jazzy Nav2 ComputePathToPose Action Goal。"""
        request = ComputePathToPose.Goal()
        request.goal = pose
        request.planner_id = 'GridBased'
        request.use_start = False
        return request

    @staticmethod
    def _wait_future(future, timeout):
        """由 MultiThreadedExecutor 的其他线程推进 Nav2 Action Future。"""
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('等待 Nav2 路径规划超时')
            time.sleep(0.01)
        value = future.result()
        if value is None:
            raise RuntimeError('Nav2 路径规划没有返回结果')
        return value

    @staticmethod
    def _abort_plan(goal_handle, result, code, message, pose=None):
        result.success = False
        result.error_code = code
        result.message = message
        if pose is not None:
            result.goal = pose
        goal_handle.abort()
        return result

    def _confirm_service(self, request, response):
        """确认只发布未来导航输入，当前绝不调用 NavigateToPose。"""
        preview = self.preview_cache.get(request.mission_id)
        if preview is None or time.monotonic() - preview['created_at'] > 300.0:
            response.accepted = False
            response.error_code = 'PREVIEW_EXPIRED'
            response.message = '任务预览不存在或已过期'
            return response
        self.active_task = preview['task']
        self.active_target_id = preview['target_id']
        self.mission_confirmed = True
        response.accepted = True
        response.message = '任务已确认；当前仅发布导航目标接口，不驱动底盘'
        response.navigation_goal = preview['goal']
        self.navigation_goal_pub.publish(preview['goal'])
        if self.active_task == 'follow_person':
            self._set_detection_mode(True)
        self._publish_status('confirmed_preview_only', response.message,
                             mission_id=request.mission_id)
        return response

    def _cancel_service(self, request, response):
        del request
        self.preview_cache.clear()
        self._clear_task('canceled', '任务已取消')
        response.success = True
        response.message = '任务已取消'
        return response

    def _stop_service(self, request, response):
        del request
        self.preview_cache.clear()
        self._clear_task('stopped', '机器人任务已立即停止')
        response.success = True
        response.message = '机器人任务已立即停止'
        return response

    def _set_detection_mode(self, continuous):
        if not self.detection_mode.service_is_ready():
            return
        request = SetDetectionMode.Request()
        request.mode = (SetDetectionMode.Request.CONTINUOUS if continuous
                        else SetDetectionMode.Request.ON_DEMAND)
        request.target_id = self.active_target_id if continuous else ''
        self.detection_mode.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = BrainMission()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.plan_action.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
