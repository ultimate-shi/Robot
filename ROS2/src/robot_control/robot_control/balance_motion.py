"""作用：计算四腿平面到位目标及静态档位轨迹；用法：由 body_balance 调用，可离线测试。"""

import math


def position_plane(heights, center, pitch, roll, half_length, half_width, lower, upper):
    """实际腿高叠加 IMU 几何修正，去除不可由 IMU 观测的对角扭曲；整体限幅保留姿态比例。"""
    front_left, front_right, rear_left, rear_right = heights
    measured_pitch = (rear_left+rear_right-front_left-front_right)/4
    measured_roll = (front_right+rear_right-front_left-rear_left)/4
    pitch_height = measured_pitch+half_length*math.tan(pitch)
    roll_height = measured_roll+half_width*math.tan(roll)
    offsets = [-pitch_height-roll_height, -pitch_height+roll_height,
               pitch_height-roll_height, pitch_height+roll_height]
    def center_interval(scale):
        return (max(low-scale*offset for low, offset in zip(lower, offsets)),
                min(high-scale*offset for high, offset in zip(upper, offsets)))
    scale = 1.0
    low, high = center_interval(scale)
    if low > high:
        minimum, maximum = 0.0, 1.0
        base_low, base_high = center_interval(0.0)
        if base_low > base_high:
            raise ValueError('四腿高度限位没有共同的基准高度')
        for _ in range(40):
            candidate = (minimum+maximum)/2
            left, right = center_interval(candidate)
            if left <= right:
                minimum = candidate
            else:
                maximum = candidate
        scale = minimum
        low, high = center_interval(scale)
    # 先平移公共高度利用剩余行程；完整姿态确实不可达时才整体缩小修正量。
    center = min(high, max(low, center))
    return [center+scale*offset for offset in offsets]



def limit_motion(targets, previous, velocities, measured, lower, upper,
                 dt, max_speed, acceleration, max_lead):
    """硬限位及反馈窗具有最高优先级，普通反向动作先减速再换向。"""
    result, rates = [], []
    for goal, start, speed, actual, low, high in zip(
            targets, previous, velocities, measured, lower, upper):
        low = max(low, actual-max_lead)
        high = min(high, actual+max_lead)
        # 越界反馈只允许回到机械边界，不生成反向的空区间。
        if low > high:
            low = high = min(upper[len(result)], max(lower[len(result)], actual))
        goal = min(high, max(low, goal))
        distance = goal-start
        # 给下一周期的制动留余量，避免接近目标时突然切断速度。
        brake_speed = math.sqrt(2*acceleration*abs(distance) +
                                (acceleration*dt/2)**2) - acceleration*dt/2
        desired = math.copysign(min(max_speed, brake_speed), distance)
        speed = min(speed+acceleration*dt, max(speed-acceleration*dt, desired))
        speed = min(max_speed, max(-max_speed, speed))
        value = min(high, max(low, start+speed*dt))
        rates.append((value-start)/dt)
        result.append(value)
    return result, rates
