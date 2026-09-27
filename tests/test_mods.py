"""Mod FSMs against a fake driver; no hardware.

From ALOHA/:  uv run pytest
"""
import threading
import time
from types import SimpleNamespace

import pytest
import trossen_arm

from myaloha.mods import Instruction, Reset, Teleop
from myaloha.mods.base import RequirementError
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
        return SimpleNamespace(header=SimpleNamespace(timestamp=time.monotonic(), id=0),
                               joint=SimpleNamespace(all=SimpleNamespace(
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
RIGHT_LEADER = "192.168.10.2"
FOLLOWERS = ("192.168.10.4", "192.168.10.5")


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


def test_reset_is_refused_with_nothing_connected(make_robot):
    robot = make_robot(unreachable=LEADERS + FOLLOWERS)
    mod = Reset(robot)
    with pytest.raises(RequirementError, match="at least one arm"):
        mod.run()
    assert path(mod) == ["idle"] and robot.state == "offline"


def test_without_leaders_teleop_is_refused_and_the_rest_runs(make_robot):
    robot = make_robot(unreachable=LEADERS)
    mod = Teleop(robot)
    with pytest.raises(RequirementError, match="leader\\+follower pair.*left pair: no leader_left.*right pair: no leader_right"):
        mod.run()
    assert path(mod) == ["idle"] and not robot.mods
    assert Teleop.requirements(robot)["met"] is False
    assert Reset.requirements(robot)["met"] and Instruction.requirements(robot)["met"]
    assert Reset(robot, Reset.Params(goal_time=0.01)).run() == "done"


def test_requirements_follow_the_requested_sides(make_robot):
    robot = make_robot(unreachable=(RIGHT_LEADER,))
    assert Teleop.requirements(robot, Teleop.Params(sides=("left",)))["met"]
    with pytest.raises(RequirementError, match="right pair: no leader_right"):
        Teleop(robot, Teleop.Params(sides=("right",))).run()
    robot = make_robot(unreachable=("192.168.10.5",))  # follower_left
    with pytest.raises(RequirementError, match="follower L: no follower_left"):
        Instruction(robot, Instruction.Params(sides=("left",))).run()


def test_teleop_stages_then_mirrors_the_complete_side_only(make_robot):
    robot = make_robot(unreachable=(RIGHT_LEADER,))
    mod = Teleop(robot, Teleop.Params(duration_s=0.1, stage_time=0.01))
    assert mod.run() == "done"
    assert path(mod) == ["idle", "pairing", "staging", "following", "releasing", "done"]
    assert mod.skipped == {"right": "leader_right not connected"}
    staged = robot.arms["leader_left"].staged_positions
    assert robot.arms["leader_left"].positions() == staged  # released clear of the limits
    assert robot.arms["follower_left"].positions() == staged  # and followed from there
    assert robot.arms["follower_right"].positions() != staged


def test_gravity_comp_is_refused_on_the_joint_limits(make_robot):
    robot = make_robot()
    arm = robot.arms["leader_left"]
    arm.move_to([0.0] * N, goal_time=0.01)
    with pytest.raises(RuntimeError, match="stage the arm"):
        arm.free()
    assert arm.driver.mode == trossen_arm.Mode.position


def test_instruction_floats_then_holds_on_stop(make_robot):
    robot = make_robot(unreachable=LEADERS)
    mod = Instruction(robot, Instruction.Params(sides=("left",), stage_time=0.01))
    worker = threading.Thread(target=mod.run)
    worker.start()
    time.sleep(0.1)
    assert mod.state == "free"
    assert robot.arms["follower_left"].driver.mode == trossen_arm.Mode.external_effort
    assert robot.status()["state"] == "running" and robot.status()["mods"][0]["state"] == "free"
    assert robot.arms["follower_left"].status()["mode"] == ["external_effort"]
    mod.stop()
    worker.join(1)
    assert path(mod) == ["idle", "staging", "free", "exiting", "stopped"]
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


def test_timeline_records_each_run_and_its_result(make_robot):
    robot = make_robot()
    Reset(robot, Reset.Params(goal_time=0.01)).run()
    assert [(s["state"], s.get("mod"), s.get("result")) for s in robot.timeline] == [
        ("idle", None, None), ("running", "reset", "done"), ("idle", None, None)]


def test_params_from_strings_and_json():
    assert Teleop.params_from({"duration_s": "5", "sides": "left"}) == Teleop.Params(duration_s=5.0, sides=("left",))
    assert Reset.params_from({"via_staged": False, "goal_time": 2}) == Reset.Params(goal_time=2.0, via_staged=False)
    with pytest.raises(ValueError, match="no param"):
        Reset.params_from({"speed": 1})
