"""作用：验证无效初始估计不会转发给 AMCL 或撤销有效锚点；使用方法：在 Jazzy 环境执行 pytest 本文件。"""

from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_navigation.global_localization import GlobalLocalization


def test_invalid_standard_initial_pose_is_not_forwarded():
    node = object.__new__(GlobalLocalization)
    node.internal_initial = None
    node.map_frame = 'map'
    node.floor = [.005, .005, .01]
    node.get_logger = lambda: SimpleNamespace(warning=lambda *args: None)
    forwarded, resets = [], []
    node.amcl_initial_pub = SimpleNamespace(publish=forwarded.append)
    node.reset = lambda: resets.append(True)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.pose.pose.orientation.w = 0.
    # 显式构造零四元数，不能传播给真实定位器。
    node.on_external_initial(message)
    assert not forwarded and not resets
