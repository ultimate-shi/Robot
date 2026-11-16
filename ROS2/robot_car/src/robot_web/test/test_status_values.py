"""作用：防止无效传感器量程破坏状态 API；使用方法：pytest -q src/robot_web/test。"""
import math

from robot_web.console import RobotConsole


def test_invalid_sensor_numbers_are_json_safe():
    assert RobotConsole.json_number(float('inf')) is None
    assert RobotConsole.json_number(float('nan')) is None
    assert RobotConsole.json_number(1.25) == 1.25
    assert math.isfinite(RobotConsole.json_number(0.0))


def test_real_imu_message_status_endpoint_is_json_serializable():
    """真实 Imu 协方差索引是 NumPy 标量，须经过整个 HTTP 序列化链验证。"""
    import rclpy
    import asyncio
    import json
    from robot_web.console import create_app
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = None
    try:
        node = RobotConsole()
        message = Imu()
        message.orientation.w = 1.0
        app = create_app(node)
        async def get_status():
            messages = []
            async def receive():
                return {'type': 'http.request', 'body': b'', 'more_body': False}
            async def send(message):
                messages.append(message)
            await app({'type': 'http', 'asgi': {'version': '3.0'}, 'http_version': '1.1',
                       'method': 'GET', 'scheme': 'http', 'path': '/api/status',
                       'raw_path': b'/api/status', 'query_string': b'', 'headers': [],
                       'client': ('127.0.0.1', 1), 'server': ('localhost', 9100)}, receive, send)
            return messages
        for covariance, expected in [(0.0, True), (-1.0, False)]:
            message.orientation_covariance[0] = covariance
            node.save('imu', message)
            assert type(node.status()['imu_valid']) is bool
            response = asyncio.run(get_status())
            assert response[0]['status'] == 200
            payload = json.loads(b''.join(item.get('body', b'') for item in response))
            assert payload['imu_valid'] is expected
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
