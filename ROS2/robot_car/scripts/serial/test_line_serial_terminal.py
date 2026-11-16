#!/usr/bin/env python3
"""用途：用伪终端验证按行串口工具；直接运行本文件，无需连接真实串口。"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import pty
import threading
import time
import unittest


MODULE_PATH = Path(__file__).with_name("line_serial_terminal.py")
SPEC = importlib.util.spec_from_file_location("line_serial_terminal", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
terminal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(terminal)


class LineSerialTerminalTest(unittest.TestCase):
    """使用伪终端验证命令边界和响应读取。"""

    def setUp(self) -> None:
        self.master_fd, slave_fd = pty.openpty()
        self.slave_path = os.ttyname(slave_fd)
        os.close(slave_fd)
        self.serial_fd = terminal.open_serial(
            self.slave_path,
            115200,
            set_control_lines=False,
        )

    def tearDown(self) -> None:
        os.close(self.serial_fd)
        os.close(self.master_fd)

    def test_send_complete_line_then_return_response(self) -> None:
        received = bytearray()

        def emulate_device() -> None:
            while not received.endswith(b"\n"):
                received.extend(os.read(self.master_fd, 64))
            os.write(self.master_fd, b"OK 123\n")

        device = threading.Thread(target=emulate_device)
        device.start()
        response = terminal.send_command(
            self.serial_fd,
            "S 1 2 3 4 500",
            b"\n",
            "utf-8",
            response_timeout=0.5,
            idle_timeout=0.02,
        )
        device.join(timeout=1.0)

        self.assertFalse(device.is_alive())
        self.assertEqual(bytes(received), b"S 1 2 3 4 500\n")
        self.assertEqual(response, b"OK 123\n")

    def test_response_stops_after_idle_timeout(self) -> None:
        os.write(self.master_fd, b"first")
        started = time.monotonic()
        response = terminal.read_response(
            self.serial_fd,
            response_timeout=0.5,
            idle_timeout=0.03,
        )

        self.assertEqual(response, b"first")
        self.assertLess(time.monotonic() - started, 0.3)

    def test_reject_negative_timeout(self) -> None:
        with self.assertRaises(SystemExit):
            terminal.parse_args(["/dev/null", "--response-timeout", "-1"])


if __name__ == "__main__":
    unittest.main()
