"""使用方法：Web层用 MissionManager 保存私有预览并通过租约原子确认任务。"""

import time
import uuid

from robot_brain.multi_user import MultiUserMissionState


class MissionManager:
    """只管理网页预览和租约，路径与坐标由 ROS navigation 提供。"""

    def __init__(self, grace_seconds=10.0):
        self.lease = MultiUserMissionState(grace_seconds=grace_seconds)
        self.previews = {}

    def create_preview(self, client_id, task, label='', target_id='',
                       snapshot_id='', scene_stamp_ns=None):
        mission_id = str(uuid.uuid4())
        preview = {
            'mission_id': mission_id, 'client_id': client_id,
            'task': task, 'label': label, 'selected_target_id': target_id,
            'candidates': [], 'snapshot_id': str(snapshot_id),
            'scene_stamp_ns': scene_stamp_ns, 'created_at': time.time(),
        }
        self.previews[mission_id] = preview
        cutoff = time.time() - 300.0
        for key in list(self.previews):
            if self.previews[key]['created_at'] < cutoff:
                self.previews.pop(key, None)
        return preview

    def owned_preview(self, mission_id, client_id):
        preview = self.previews.get(mission_id)
        if preview is None or preview['client_id'] != client_id:
            return None
        return preview
