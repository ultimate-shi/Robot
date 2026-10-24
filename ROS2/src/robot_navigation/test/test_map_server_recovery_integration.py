"""作用：用真实地图服务复现未激活并验证自动恢复；使用方法：独立 ROS_DOMAIN_ID 下设 RUN_MAP_RECOVERY_INTEGRATION=1 后 pytest 本文件。"""

import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.srv import GetMap
from rclpy.executors import SingleThreadedExecutor
from robot_navigation.map_server_recovery import MapServerRecovery


@pytest.mark.skipif(os.environ.get('RUN_MAP_RECOVERY_INTEGRATION') != '1', reason='需独立 DDS 域显式启用')
@pytest.mark.parametrize('stalled_manager', [False, True])
def test_real_map_server_recovers_configured_inactive_and_later_pause(tmp_path, stalled_manager):
    assert os.environ.get('ROS_DOMAIN_ID', '0') != '0', '禁止接入实车 DDS 域'
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([254]*4))
    (tmp_path / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    rclpy.init(args=['--ros-args', '-p', 'startup_grace:=0.1', '-p', 'check_interval:=0.1',
                     '-p', 'retry_interval:=0.1', '-p',
                     'request_timeout:=0.4' if stalled_manager else 'request_timeout:=4.0'])
    probe = rclpy.create_node('map_recovery_integration_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    processes = []
    recovery = None
    log = open(tmp_path / 'map_recovery.log', 'w')

    def start(args):
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def client(kind, name):
        value = probe.create_client(kind, name)
        assert value.wait_for_service(timeout_sec=15), name
        return value

    def call(value, request):
        future = value.call_async(request)
        deadline = time.monotonic()+12
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert future.done(), value.srv_name
        return future.result()

    def ready(min_attempts):
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if recovery.last_status and recovery.last_status[0] == 'active' and recovery.attempts >= min_attempts:
                return
        log.flush()
        pytest.fail((tmp_path / 'map_recovery.log').read_text()[-3000:])

    try:
        start(['/opt/ros/jazzy/lib/nav2_map_server/map_server', '--ros-args',
               '-p', f'yaml_filename:={tmp_path / "map.yaml"}'])
        if stalled_manager:
            # 独立进程的服务故意延迟答复，不阻塞被测恢复节点的定时器。
            stalled = '''import time
import rclpy
from std_srvs.srv import Trigger
rclpy.init()
node=rclpy.create_node("stalled_map_manager")
def slow(request,response):
    time.sleep(2)
    response.success=False
    return response
node.create_service(Trigger,"/lifecycle_manager_map/is_active",slow)
rclpy.spin(node)
'''
            start([sys.executable, '-c', stalled])
        else:
            start(['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager', '--ros-args',
                   '-r', '__node:=lifecycle_manager_map', '-p', 'autostart:=false',
                   '-p', 'node_names:=[map_server]'])
        get_state = client(GetState, '/map_server/get_state')
        change = client(ChangeState, '/map_server/change_state')
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        assert call(change, request).success
        assert call(get_state, GetState.Request()).current_state.id == State.PRIMARY_STATE_INACTIVE
        # 复现原故障：配置已执行成功，但管理器没有继续 activate。
        recovery = MapServerRecovery()
        executor.add_node(recovery)
        ready(1)
        assert call(get_state, GetState.Request()).current_state.id == State.PRIMARY_STATE_ACTIVE
        get_map = client(GetMap, '/map_server/map')
        assert call(get_map, GetMap.Request()).map.info.width == 2
        if stalled_manager:
            assert recovery.manager_bypass_until > time.monotonic()
            assert not recovery.manager_active
        else:
            # 复用已建立连接的客户端，避免故障注入本身受首次 DDS 发现影响。
            manager = recovery.recovery_clients['recover']
            assert call(manager, ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.PAUSE)).success
            ready(2)
            assert call(get_state, GetState.Request()).current_state.id == State.PRIMARY_STATE_ACTIVE
            assert call(get_map, GetMap.Request()).map.info.width == 2
    finally:
        executor.shutdown()
        if recovery is not None:
            recovery.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        log.close()
