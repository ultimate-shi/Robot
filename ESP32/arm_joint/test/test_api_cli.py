#!/usr/bin/env python3
# 作用：无设备验证串口工具的到位等待、保持退出和失败停止；用法：python -m unittest discover -s ESP32/arm_joint/test -p 'test_*.py'。
import contextlib
import importlib.util
import io
import pathlib
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location("api_cli", pathlib.Path(__file__).parents[1] / "scripts/api_cli.py")
cli = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cli)


class Clock:
    def __init__(self):
        self.now = 1.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Board:
    def __init__(self, clock):
        self.clock = clock
        self.hold = {1: 0, 2: 0}
        self.targets = {}
        self.commands = []
        self.fault = False
        self.stuck = False
        self.premature_stop = False
        self.stale_once = False
        self.invalid_feedback = None

    def exchange(self, port, command):
        self.commands.append(command)
        name = command.split()[0]
        op, axes = cli.motion_axes(command)
        if op:
            for joint_id, target in axes.items():
                self.targets[joint_id] = (target, self.clock.now, op)
            return [name + " OK"]
        if name.startswith("H"):
            return [f"{name} {self.hold[int(name[1:])]}"]
        if name.startswith("R"):
            joint_id = int(name[1:])
            target, sent_at, _ = self.targets[joint_id]
            if self.fault:
                return [f"{name} NA NA NA 1500 2 5 0 NA 0"]
            if self.stale_once:
                self.stale_once = False
                return [f"{name} 0 0 NA 1500 0 0 0 50 0"]
            if self.invalid_feedback:
                return [f"{name} {self.invalid_feedback} 0 NA 1500 1 0 0 0 1"]
            ready = self.clock.now - sent_at >= .15 and not self.stuck
            angle = target if ready else target - 2
            state = 0 if self.premature_stop or (ready and not self.hold[joint_id]) else 1
            mode = 0 if state == 0 else 1
            return [f"{name} {angle} 0 NA 1500 {state} 0 0 0 {mode}"]
        return [name + " OK"]


class CliTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.board = Board(self.clock)
        self.stack = contextlib.ExitStack()
        self.stack.enter_context(mock.patch.object(cli.time, "monotonic", self.clock.monotonic))
        self.stack.enter_context(mock.patch.object(cli.time, "sleep", self.clock.sleep))
        self.stack.enter_context(mock.patch.object(cli, "exchange", self.board.exchange))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.addCleanup(self.stack.close)

    def batch(self, commands, timeout=1, repeat=1):
        cli.run_batch(None, commands, repeat, 20, timeout)

    def test_single_angle_waits_without_refresh(self):
        self.batch(["R1 2.5"])
        self.assertEqual(self.board.commands.count("R1 2.5"), 1)
        self.assertGreaterEqual(self.clock.now, 1.15)
        self.assertNotIn("X", self.board.commands)

    def test_hold_survives_successful_exit(self):
        self.board.hold[1] = 1
        self.batch(["R1 2.5"])
        self.assertGreaterEqual(self.clock.now, 1.35)
        self.assertFalse(any(c.startswith("X") for c in self.board.commands))

    def test_mixed_hold_and_stop_axes(self):
        self.board.hold[1] = 1
        self.batch(["R 2.5 -3.25"])
        self.assertIn("H1", self.board.commands)
        self.assertIn("H2", self.board.commands)
        self.assertFalse(any(c.startswith("X") for c in self.board.commands))

    def test_pwm_cleanup_does_not_cancel_other_axis_hold(self):
        self.board.hold[1] = 1
        self.batch(["R1 2.5", "U2 1510"])
        self.assertIn("X2", self.board.commands)
        self.assertNotIn("X", self.board.commands)
        self.assertNotIn("X1", self.board.commands)

    def test_new_target_replaces_previous_target(self):
        self.batch(["R1 2.5", "R1 -3.25"])
        self.assertEqual(self.board.targets[1][0], -3.25)

    def test_pwm_replaces_position_without_waiting(self):
        self.batch(["R1 2.5", "U1 1510"])
        self.assertNotIn("R1", self.board.commands)
        self.assertIn("X1", self.board.commands)

    def test_stale_stop_is_ignored(self):
        self.board.stale_once = True
        self.batch(["R1 2.5"])
        self.assertNotIn("X", self.board.commands)

    def test_timeout_stops(self):
        self.board.stuck = True
        with self.assertRaises(TimeoutError):
            self.batch(["R1 2.5"], timeout=.2)
        self.assertEqual(self.board.commands[-1], "X")

    def test_fault_stops(self):
        self.board.fault = True
        with self.assertRaises(RuntimeError):
            self.batch(["R1 2.5"])
        self.assertEqual(self.board.commands[-1], "X")

    def test_premature_stop_is_failure(self):
        self.board.premature_stop = True
        with self.assertRaises(RuntimeError):
            self.batch(["R1 2.5"])
        self.assertEqual(self.board.commands[-1], "X")

    def test_nonfinite_feedback_rejected(self):
        for value in ("nan", "inf"):
            self.board.invalid_feedback = value
            with self.assertRaises(RuntimeError):
                self.batch(["R1 2.5"])
            self.assertEqual(self.board.commands[-1], "X")

    def test_interrupt_stops(self):
        with mock.patch.object(cli, "wait_positions", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                self.batch(["R1 2.5"])
        self.assertEqual(self.board.commands[-1], "X")


if __name__ == "__main__":
    unittest.main()
