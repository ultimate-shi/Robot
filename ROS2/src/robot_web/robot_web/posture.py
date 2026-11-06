"""作用：计算手动车身高度与头部运动；使用方法：由关节桥接器调用，也可独立测试。"""
import math


def heights(angles, length):
    """四腿顺序为左前、右前、左后、右后；正高度表示车身上移。"""
    return [sign * length * math.sin(math.radians(angle))
            for sign, angle in zip((1, 1, -1, -1), angles)]


def angles(values, length):
    return [sign * math.degrees(math.asin(max(-1, min(1, value / length))))
            for sign, value in zip((1, 1, -1, -1), values)]


def bounded_move(current, delta, lower, upper):
    """整组同比例限幅，保留各腿位移比例；返回限位标记。"""
    scale = 1.0
    for value, step, low, high in zip(current, delta, lower, upper):
        if step > 0:
            scale = min(scale, max(0.0, (high - value) / step))
        elif step < 0:
            scale = min(scale, max(0.0, (low - value) / step))
    return [value + step * scale for value, step in zip(current, delta)], scale < 1.0


def body_step(current, intent, distance, lower, upper):
    """倾斜只伸展对侧腿；整体位移与倾斜分别限位，避免某动作锁死其余动作。"""
    pitch = intent.get('pitch', 0)
    roll = intent.get('roll', 0)
    lift = intent.get('lift', 0)
    # 正 pitch 表示前倾，正 roll 表示左倾。
    tilt = [max(-pitch, 0) + max(-roll, 0),
            max(-pitch, 0) + max(roll, 0),
            max(pitch, 0) + max(-roll, 0),
            max(pitch, 0) + max(roll, 0)]
    tilted, tilt_limit = bounded_move(
        current, [step * distance for step in tilt], lower, upper)
    moved, lift_limit = bounded_move(
        tilted, [lift * distance] * 4, lower, upper)
    return moved, tilt_limit or lift_limit


def body_angle_step(current, action, step_deg, length, minimum, maximum):
    """每步参与腿最大改变指定角度；等高度位移保持另一方向的倾斜及升降姿态。"""
    masks = {'w': (0, 0, 1, 1), 's': (1, 1, 0, 0),
             'a': (0, 1, 0, 1), 'd': (1, 0, 1, 0),
             'q': (1, 1, 1, 1), 'e': (1, 1, 1, 1)}
    if action not in masks:
        raise ValueError('腿部步进方向无效')
    mask = masks[action]
    direction = -1 if action == 'q' else 1
    actual_h = heights(current, length)
    candidates = list(current)
    limited = False
    for i, sign in enumerate((1, 1, -1, -1)):
        if not mask[i]:
            continue
        requested = current[i] + direction*sign*step_deg
        candidates[i] = max(minimum[i], min(maximum[i], requested))
        limited |= abs(candidates[i]-requested) > 1e-8
    candidate_h = heights(candidates, length)
    distance = min(abs(candidate_h[i]-actual_h[i]) for i in range(4) if mask[i])
    target_h = [value + direction*distance*mask[i] for i, value in enumerate(actual_h)]
    return angles(target_h, length), limited
