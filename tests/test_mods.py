"""Mod FSMs against a fake driver; no hardware.

From ALOHA/:  uv run pytest
"""
import threading
import time
from types import SimpleNamespace

import pytest
import trossen_arm

from myaloha.mods import Instruction, Reset, Teleop
from myaloha.robot import Aloha

N = 7
UNREACHABLE: set[str] = set()


class FakeDriver:
    def __init__(self):
        self.ip, self.configured = None, False
        self.mode = trossen_arm.Mode.idle
        self.pos = [0.1 * (i + 1) for i in range(N)]

    def configure(self, model, end_effector, ip, clear_error, timeout=20.0):
        if ip in UNREACHABLE:
            raise RuntimeError(f"timed out connecting to {ip}")
        self.ip, self.configured = ip, True

    def get_is_configured(self):
        return self.configured

    def get_num_joints(self):
        return N

    def set_all_modes(self, mode):
        self.mode = mode

    def get_modes(self):
        return [self.mode] * N

    def set_all_positions(self, goal_positions, goal_time=2.0, blocking=True):
        assert self.mode == trossen_arm.Mode.position
        self.pos = list(goal_positions)

    def set_all_external_efforts(self, efforts, goal_time=0.0, blocking=True):
        assert self.mode == trossen_arm.Mode.external_effort

    def get_all_positions(self):
        return list(self.pos)

    def get_robot_output(self):
        z = [0.0] * N
        return SimpleNamespace(joint=SimpleNamespace(all=SimpleNamespace(
            positions=list(self.pos), velocities=z, efforts=z, external_efforts=z)))

    def cleanup(self):
        self.configured = False


@pytest.fixture
def make_robot(monkeypatch):
    monkeypatch.setattr(trossen_arm, "TrossenArmDriver", FakeDriver)

    def make(unreachable=()):
        UNREACHABLE.clear()
        UNREACHABLE.update(unreachable)
        robot = Aloha()
        robot.connect()
        return robot
    return make


LEADERS = ("192.168.10.2", "192.168.10.3")


def path(mod):
    return [s for s, _ in mod.history]


def test_reset_zeros_connected_arms_and_skips_absent_leaders(make_robot):
    robot = make_robot(unreachable=LEADERS)
    mod = Reset(robot, Reset.Params(goal_time=0.01))
    assert mod.run() == "done"
    assert path(mod) == ["idle", "checking", "staging", "zeroing", "done"]
    assert set(mod.skipped) == {"leader_left", "leader_right"}
    for arm in robot.connected():
        assert arm.positions() == [0.0] * N


def test_reset_fails_with_nothing_connected(make_robot):
    robot = make_robot(unreachable=LEADERS + ("192.168.10.4", "192.168.10.5"))
    mod = Reset(robot)
    assert mod.run() == "failed"
    assert "no arm connected" in mod.error


def test_teleop_without_leaders_fails_before_moving(make_robot):
    robot = make_robot(unreachable=LEADERS)
    mod = Teleop(robot)
    assert mod.run() == "failed"
    assert path(mod) == ["idle", "pairing", "failed"]
    assert "leader_left not connected" in mod.error


def test_teleop_runs_the_complete_side_only(make_robot):
    robot = make_robot(unreachable=(LEADERS[1],))  # right leader missing
    leader = robot.arms["leader_left"]
    leader.driver.pos = [0.5] * N
    mod = Teleop(robot, Teleop.Params(duration_s=0.1, align_time=0.01))
    assert mod.run() == "done"
    assert path(mod) == ["idle", "pairing", "aligning", "following", "releasing", "done"]
    assert mod.skipped == {"right": "leader_right not connected"}
    assert robot.arms["follower_left"].positions() == [0.5] * N
    assert robot.arms["follower_right"].positions() != [0.5] * N


def test_instruction_floats_then_holds_on_stop(make_robot):
    robot = make_robot(unreachable=LEADERS)
    mod = Instruction(robot, Instruction.Params(sides=("left",)))
    worker = threading.Thread(target=mod.run)
    worker.start()
    time.sleep(0.1)
    assert mod.state == "free"
    assert robot.arms["follower_left"].driver.mode == trossen_arm.Mode.external_effort
    assert robot.status()["state"] == "running" and robot.status()["mods"][0]["state"] == "free"
    mod.stop()
    worker.join(1)
    assert path(mod) == ["idle", "entering", "free", "exiting", "stopped"]
    assert robot.arms["follower_left"].driver.mode == trossen_arm.Mode.position
    assert "follower_left" in mod.readings and len(mod.ranges["follower_left"]) == N


def test_illegal_transition_raises(make_robot):
    mod = Reset(make_robot())
    with pytest.raises(RuntimeError, match="illegal transition"):
        mod.goto("zeroing")


def test_second_mod_is_refused_while_one_runs(make_robot):
    robot = make_robot()
    first = Instruction(robot)
    worker = threading.Thread(target=first.run)
    worker.start()
    time.sleep(0.05)
    with pytest.raises(RuntimeError, match="busy"):
        Reset(robot).run()
    first.stop()
    worker.join(1)


def test_child_mod_runs_inside_parent(make_robot):
    robot = make_robot()
    parent = Instruction(robot)
    robot.claim(parent)
    child = Reset(robot, Reset.Params(goal_time=0.01), parent=parent)
    assert child.run() == "done"
    robot.release(parent)
