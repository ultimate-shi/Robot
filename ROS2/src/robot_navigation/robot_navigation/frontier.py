"""使用方法：MissionPlanner 调用本模块提取前沿并计算语义目标停靠位姿。"""

from collections import deque
import math

import numpy as np


def find_frontiers(data, width, height, min_cells=8):
    """返回自由栅格与未知栅格交界处的连通前沿中心."""
    grid = np.asarray(data, dtype=np.int16)
    if width <= 0 or height <= 0 or grid.size != width * height:
        raise ValueError('占据栅格尺寸或数据长度无效')
    grid = grid.reshape((height, width))
    free = (grid >= 0) & (grid <= 25)
    unknown = grid < 0
    adjacent_unknown = np.zeros_like(unknown)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    frontier_mask = free & adjacent_unknown

    visited = np.zeros_like(frontier_mask)
    groups = []
    for row, col in np.argwhere(frontier_mask):
        if visited[row, col]:
            continue
        queue = deque([(int(row), int(col))])
        visited[row, col] = True
        cells = []
        while queue:
            current_row, current_col = queue.popleft()
            cells.append((current_row, current_col))
            for next_row, next_col in (
                    (current_row - 1, current_col),
                    (current_row + 1, current_col),
                    (current_row, current_col - 1),
                    (current_row, current_col + 1)):
                if (0 <= next_row < height and 0 <= next_col < width
                        and frontier_mask[next_row, next_col]
                        and not visited[next_row, next_col]):
                    visited[next_row, next_col] = True
                    queue.append((next_row, next_col))
        if len(cells) >= int(min_cells):
            rows = np.asarray([cell[0] for cell in cells])
            cols = np.asarray([cell[1] for cell in cells])
            groups.append({
                'row': int(round(float(np.mean(rows)))),
                'col': int(round(float(np.mean(cols)))),
                'size': len(cells),
            })
    return groups


def frontier_world_candidates(grid, robot_xy, min_cells=8):
    """把前沿中心转换到 map 坐标并按信息增益与距离排序."""
    groups = find_frontiers(
        grid.data, int(grid.info.width), int(grid.info.height), min_cells)
    resolution = float(grid.info.resolution)
    origin_x = float(grid.info.origin.position.x)
    origin_y = float(grid.info.origin.position.y)
    candidates = []
    for group in groups:
        x = origin_x + (group['col'] + 0.5) * resolution
        y = origin_y + (group['row'] + 0.5) * resolution
        distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
        score = group['size'] * resolution - 0.35 * distance
        candidates.append({
            **group, 'x': x, 'y': y,
            'distance': distance, 'score': score,
        })
    return sorted(candidates, key=lambda item: item['score'], reverse=True)


def standoff_pose(robot_xy, target_xy, surface_clearance=0.5,
                  robot_radius=0.25):
    """计算车体外轮廓距目标表面指定距离、且正面朝向目标的位姿."""
    delta_x = float(robot_xy[0]) - float(target_xy[0])
    delta_y = float(robot_xy[1]) - float(target_xy[1])
    length = math.hypot(delta_x, delta_y)
    if length < 1e-6:
        delta_x, delta_y, length = -1.0, 0.0, 1.0
    center_distance = float(surface_clearance) + float(robot_radius)
    x = float(target_xy[0]) + delta_x / length * center_distance
    y = float(target_xy[1]) + delta_y / length * center_distance
    yaw = math.atan2(float(target_xy[1]) - y, float(target_xy[0]) - x)
    return {'x': x, 'y': y, 'yaw': yaw}
