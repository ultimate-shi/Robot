#!/usr/bin/env python3
# 作用：独占USB串口验证API，支持整行交互及20Hz刷新；用法：加--interactive保持同一连接进行标定，需pyserial。
import argparse
import time
import serial


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
            continue  # 启动ROM输出不能被当作API成功响应。
        lines.append(line)
        if command.strip() != "R" or line == "R END" or line.startswith("!"):
            return lines
    raise TimeoutError(f"{command!r}: {lines!r}")


def main():
    parser = argparse.ArgumentParser(description="机械臂USB文本API终端")
    parser.add_argument("--port", required=True)
    parser.add_argument("--command", action="append", help="可重复；默认仅查询身份、状态及原始传感器")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--hz", type=float, default=20)
    parser.add_argument("--interactive", action="store_true", help="按回车发送完整命令，保持连接和RAM标定")
    args = parser.parse_args()
    if args.repeat < 1 or not 0 < args.hz <= 20:
        parser.error("repeat须>=1，hz须在(0,20]内")
    commands = args.command or ["$info", "$version", "R", "$raw 1", "A1", "J1", "P1"]
    motion = any(c.startswith("U") or (c.startswith("R") and len(c.split()) > 1) for c in commands)
    port = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=1)
    port.dtr = False
    port.rts = False
    port.port = args.port
    port.open()
    try:
        port.write(b"\n")
        time.sleep(0.05)
        port.reset_input_buffer()
        if args.interactive:
            print("整行发送；输入 :quit 退出。X使用配置中立值，不能代替舵机动力断电。")
            while True:
                try:
                    command = input("> ").strip()
                except EOFError:
                    break
                if command == ":quit":
                    break
                if not command:
                    continue
                if command.startswith("U") or (command.startswith("R") and len(command.split()) > 1):
                    motion = True
                try:
                    for line in exchange(port, command):
                        print("<", line)
                except (TimeoutError, UnicodeError) as exc:
                    print("请求未完成：", exc)
            return
        next_tick = time.monotonic()
        for _ in range(args.repeat):
            for command in commands:
                print(">", command)
                for line in exchange(port, command):
                    print("<", line)
                    if line.startswith("!"):
                        raise RuntimeError(line)
            next_tick += 1 / args.hz
            time.sleep(max(0, next_tick - time.monotonic()))
    finally:
        if motion:
            try:
                exchange(port, "X")
            except (OSError, TimeoutError, serial.SerialException):
                pass  # 线路失效由关节自身100ms超时兜底。
        port.close()


if __name__ == "__main__":
    main()
