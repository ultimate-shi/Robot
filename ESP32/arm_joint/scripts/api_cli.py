#!/usr/bin/env python3
# 作用：验证0.5编码器/角度/弧度状态并等待到位；用法：--command R1查询，--interactive交互，需pyserial。
import argparse
import math
import time
import serial

ARRIVAL_DEG = 0.5
ARRIVAL_DPS = 1.0
ARRIVAL_SECONDS = 0.2
SPEED_FILTER_SECONDS = 0.1


def exchange(port, command):
    port.write((command + "\n").encode("ascii"))
    port.flush()
    lines = []
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        if "abort()" in line or "Rebooting" in line:
            raise RuntimeError("设备启动异常或重启，请检查固件")
        name = command.split()[0]
        if command.strip() == "R":
            if not (line.startswith("R1 ") or line.startswith("R2 ") or line == "R END" or line.startswith("!R ")):
                continue
        elif not (line.startswith(name + " ") or line.startswith("!" + name + " ")):
            continue
        lines.append(line)
        if command.strip() != "R" or line == "R END" or line.startswith("!"):
            return lines
    raise TimeoutError(f"{command!r}: {lines!r}")


def checked_exchange(port, command):
    lines = exchange(port, command)
    for line in lines:
        print("<", line)
        if line.startswith("!"):
            raise RuntimeError(line)
    return lines


def motion_axes(command):
    """仅识别运动/停止命令；设置和查询不替换跟踪中的目标。"""
    words = command.split()
    if not words:
        return None, {}
    name = words[0]
    if name[0] not in "RUX" or (name[0] != "X" and len(words) == 1):
        return None, {}
    ids = [int(name[1:])] if name[1:] else [1, 2]
    values = [float(v) for v in words[1:]] if name[0] != "X" else [0] * len(ids)
    return name[0], dict(zip(ids, values))


def wait_positions(port, targets, timeout):
    """targets为id -> (角度, 保持参数, 下发时刻)，只查状态、不续发目标。"""
    deadline = time.monotonic() + timeout
    stable, previous = {}, {}
    pending = dict(targets)
    while pending:
        if time.monotonic() >= deadline:
            raise TimeoutError("等待关节到位超时")
        for joint_id, (target, hold, _sent_at) in list(pending.items()):
            if time.monotonic() >= deadline:
                raise TimeoutError("等待关节到位超时")
            line = checked_exchange(port, f"R{joint_id}")[0]
            fields = line.split()
            if len(fields) != 9 or fields[0] != f"R{joint_id}":
                raise RuntimeError(f"状态格式异常：{line}")
            raw = int(fields[1])
            if not 0 <= raw <= 4095:
                raise RuntimeError(f"关节{joint_id}编码器反馈无效：{line}")
            state, fault, mode = int(fields[6]), int(fields[7]), int(fields[8])
            if fault or state == 2:
                raise RuntimeError(f"关节{joint_id}故障：{line}")
            angle, radians = float(fields[2]), float(fields[3])
            if not math.isfinite(angle) or not math.isfinite(radians) or not math.isclose(radians, math.radians(angle), abs_tol=2e-6):
                raise RuntimeError(f"关节{joint_id}反馈无效：{line}")
            now = time.monotonic()
            # 0.5不返回速度；保持模式按连续角度采样估算速度，STOP由板端确认到位。
            speed = math.inf
            if joint_id in previous:
                last_angle, last_time, last_speed = previous[joint_id]
                dt = now - last_time
                if dt > 0:
                    measured = math.remainder(angle - last_angle, 360) / dt
                    speed = measured if not math.isfinite(last_speed) else last_speed + dt / (dt + SPEED_FILTER_SECONDS) * (measured - last_speed)
            previous[joint_id] = (angle, now, speed)
            near = abs(angle - target) <= ARRIVAL_DEG
            arrived = near and abs(speed) <= ARRIVAL_DPS
            if not hold:
                if state == 0 and mode == 0:
                    if not near:
                        raise RuntimeError(f"关节{joint_id}未到目标便停止：{line}")
                    del pending[joint_id]
                elif state != 1 or mode != 1:
                    raise RuntimeError(f"关节{joint_id}已被其他指令覆盖：{line}")
            else:
                if state != 1 or mode != 1:
                    raise RuntimeError(f"关节{joint_id}保持控制已停止：{line}")
                if not arrived:
                    stable.pop(joint_id, None)
                else:
                    stable.setdefault(joint_id, now)
                    if now - stable[joint_id] >= ARRIVAL_SECONDS:
                        del pending[joint_id]
        if pending:
            time.sleep(0.05)


def run_batch(port, commands, repeat, hz, wait_timeout):
    pending, pwm = {}, set()
    motion_attempted = False
    try:
        next_tick = time.monotonic()
        for _ in range(repeat):
            for command in commands:
                op, axes = motion_axes(command)
                modes = {}
                if op == "R":
                    for joint_id in axes:
                        response = checked_exchange(port, f"H{joint_id}")[0].split()
                        if len(response) != 2 or response[1] not in ("0", "1"):
                            raise RuntimeError("到位保持参数响应异常")
                        modes[joint_id] = int(response[1])
                motion_attempted |= op in ("R", "U")
                print(">", command)
                checked_exchange(port, command)
                sent_at = time.monotonic()
                if op:
                    for joint_id, target in axes.items():
                        pending.pop(joint_id, None)
                        pwm.discard(joint_id)
                        if op == "R":
                            pending[joint_id] = (target, modes[joint_id], sent_at)
                        elif op == "U":
                            pwm.add(joint_id)
            next_tick += 1 / hz
            time.sleep(max(0, next_tick - time.monotonic()))
        # 先停止仍在PWM模式的轴，不能用全局X取消另一轴的位置保持。
        for joint_id in sorted(pwm):
            checked_exchange(port, f"X{joint_id}")
        wait_positions(port, pending, wait_timeout)
    except BaseException:
        if motion_attempted:
            try:
                exchange(port, "X")
            except (OSError, TimeoutError, RuntimeError, serial.SerialException):
                pass  # 总线失联由板间看门狗停止；主板USB失联不会取消位置保持。
        raise


def main():
    parser = argparse.ArgumentParser(description="机械臂USB文本API 0.5终端")
    parser.add_argument("--port", required=True)
    parser.add_argument("--command", action="append", help="可重复；角度和PID直接填写实际小数")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--hz", type=float, default=20)
    parser.add_argument("--wait-timeout", type=float, default=60, help="批处理角度到位最长等待秒数，默认60")
    parser.add_argument("--interactive", action="store_true", help="保持连接，手动发送新目标覆盖运动；退出发送X")
    args = parser.parse_args()
    if args.repeat < 1 or not math.isfinite(args.hz) or not 0 < args.hz <= 20:
        parser.error("repeat须>=1，hz须在(0,20]内")
    if not math.isfinite(args.wait_timeout) or args.wait_timeout <= 0:
        parser.error("wait-timeout须为有限正数")
    commands = args.command or ["$info", "$version", "R", "$raw 1", "A1", "P1", "H1"]
    port = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=1)
    port.dtr = False
    port.rts = False
    port.port = args.port
    port.open()
    interactive_motion = False
    try:
        port.write(b"\n")
        time.sleep(0.05)
        port.reset_input_buffer()
        if checked_exchange(port, "$version") != ["$version 0.5"]:
            raise RuntimeError("工具需要0.5固件（raw、角度、弧度的新状态格式）；请配套升级主从板")
        if args.interactive:
            print("整行发送；:quit退出并停止。角度目标无需刷新；X输出配置中立值。")
            while True:
                try:
                    command = input("> ").strip()
                except EOFError:
                    break
                if command == ":quit":
                    break
                if not command:
                    continue
                op, _ = motion_axes(command)
                interactive_motion |= op in ("R", "U")
                try:
                    checked_exchange(port, command)
                except (TimeoutError, UnicodeError, RuntimeError) as exc:
                    if interactive_motion:
                        exchange(port, "X")
                    print("请求未完成：", exc)
        else:
            run_batch(port, commands, args.repeat, args.hz, args.wait_timeout)
    finally:
        if interactive_motion:
            try:
                exchange(port, "X")
            except (OSError, TimeoutError, RuntimeError, serial.SerialException):
                pass
        port.close()


if __name__ == "__main__":
    main()
