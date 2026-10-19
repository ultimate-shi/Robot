#!/usr/bin/env python3
"""用途：本地编辑完整串口命令，按回车后整行发送并显示本轮响应。

使用方法：
  python3 scripts/serial/line_serial_terminal.py /dev/ttyACM0
  python3 scripts/serial/line_serial_terminal.py /dev/ttyUSB0 --baud 9600 --eol CRLF

输入 ``:quit`` 或按 Ctrl-D 退出。运行前须退出 minicom，并停止 ROS 硬件驱动和
vctrl.service，确保本工具独占串口。
"""

from __future__ import annotations

import argparse
import fcntl
import os
import select
import struct
import sys
import termios
import time
from typing import Sequence


BAUD_RATES = {
    1200: termios.B1200,
    2400: termios.B2400,
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}

LINE_ENDINGS = {
    "LF": b"\n",
    "CR": b"\r",
    "CRLF": b"\r\n",
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="本地编辑完整命令，按回车后一次写入串口并显示响应。",
    )
    parser.add_argument("device", help="串口设备，例如 /dev/ttyACM0")
    parser.add_argument(
        "--baud",
        type=int,
        choices=sorted(BAUD_RATES),
        default=115200,
        help="波特率（默认：115200）",
    )
    parser.add_argument(
        "--eol",
        choices=LINE_ENDINGS,
        default="LF",
        help="命令结束符（默认：LF）",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=1.0,
        help="等待首个响应字节的秒数（默认：1.0）",
    )
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=0.15,
        help="收到数据后，以多久无新数据判定响应结束（默认：0.15 秒）",
    )
    parser.add_argument(
        "--encoding",
        default="utf-8",
        help="命令和响应文本编码（默认：utf-8）",
    )
    parser.add_argument(
        "--no-control-lines",
        action="store_true",
        help="不主动拉高 DTR/RTS；本项目控制板通常不应使用此选项",
    )
    args = parser.parse_args(argv)
    if args.response_timeout < 0 or args.idle_timeout < 0:
        parser.error("超时时间不能为负数")
    return args


def configure_serial(fd: int, baud: int, set_control_lines: bool = True) -> None:
    """把已打开的文件描述符配置为指定波特率、8N1、无流控。"""
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNBRK
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = BAUD_RATES[baud]
    attrs[5] = BAUD_RATES[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    if set_control_lines:
        bits = termios.TIOCM_DTR | termios.TIOCM_RTS
        fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", bits))


def open_serial(device: str, baud: int, set_control_lines: bool = True) -> int:
    """打开并配置串口，返回文件描述符。"""
    flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    fd = os.open(device, flags)
    try:
        configure_serial(fd, baud, set_control_lines)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def write_all(fd: int, payload: bytes) -> None:
    """等待串口可写，并保证完整写入一条命令。"""
    view = memoryview(payload)
    while view:
        try:
            written = os.write(fd, view)
        except BlockingIOError:
            select.select([], [fd], [])
            continue
        if written == 0:
            raise OSError("串口写入返回 0 字节")
        view = view[written:]


def read_response(fd: int, response_timeout: float, idle_timeout: float) -> bytes:
    """读取一轮响应；首字节使用响应超时，之后使用空闲超时分帧。"""
    response = bytearray()
    deadline = time.monotonic() + response_timeout

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            break
        response.extend(chunk)
        deadline = time.monotonic() + idle_timeout

    return bytes(response)


def send_command(
    fd: int,
    command: str,
    line_ending: bytes,
    encoding: str,
    response_timeout: float,
    idle_timeout: float,
) -> bytes:
    """清除旧输入、整行写入命令并返回本轮设备响应。"""
    payload = command.encode(encoding) + line_ending
    termios.tcflush(fd, termios.TCIFLUSH)
    write_all(fd, payload)
    termios.tcdrain(fd)
    return read_response(fd, response_timeout, idle_timeout)


def run_terminal(args: argparse.Namespace) -> int:
    """运行按行交互循环。"""
    fd = open_serial(args.device, args.baud, not args.no_control_lines)
    try:
        print(
            f"已打开 {args.device}：{args.baud} 8N1，结束符 {args.eol}。"
            " 输入 :quit 或按 Ctrl-D 退出。"
        )
        while True:
            try:
                command = input("serial> ")
            except EOFError:
                print()
                break
            except KeyboardInterrupt:
                print("\n已取消当前输入；输入 :quit 退出。")
                continue

            if command.strip() in {":q", ":quit", ":exit"}:
                break
            if not command:
                continue

            response = send_command(
                fd,
                command,
                LINE_ENDINGS[args.eol],
                args.encoding,
                args.response_timeout,
                args.idle_timeout,
            )
            if not response:
                print(f"[等待 {args.response_timeout:g} 秒后无响应]")
                continue
            text = response.decode(args.encoding, errors="replace")
            sys.stdout.write(text)
            if not text.endswith(("\n", "\r")):
                sys.stdout.write("\n")
            sys.stdout.flush()
    finally:
        os.close(fd)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """程序入口，并把常见错误转换为简洁提示。"""
    args = parse_args(argv)
    try:
        return run_terminal(args)
    except LookupError as exc:
        print(f"编码无效：{exc}", file=sys.stderr)
    except PermissionError:
        print(
            f"没有权限访问 {args.device}；请检查 dialout 组或设备权限。",
            file=sys.stderr,
        )
    except FileNotFoundError:
        print(f"串口设备不存在：{args.device}", file=sys.stderr)
    except OSError as exc:
        print(f"串口操作失败：{exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
