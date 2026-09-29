"""Arms come back on their own: one that drops (power, cable) is reconnected as soon as it answers
again, and one absent at start is connected when it appears. Nobody presses connect."""
import time

import pytest
from simrig import connect, release, simulated_rig

from myaloha import robot as robot_mod


def wait(cond, timeout=10.0):
    end = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < end
        time.sleep(0.05)


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setattr(robot_mod, "RECONNECT_S", 0.3)
    with simulated_rig(unplugged=("leader_left", "leader_right")) as sim:
        robot = connect(sim.address)
        yield robot, sim
        release(robot)


def live(arm):
    s = arm.status()
    return s["connected"] and not s["lost"] and s["pos"] is not None


def test_an_arm_that_drops_is_reconnected_when_it_answers_again(rig):
    robot, sim = rig
    arm = robot.arms["follower_left"]
    assert live(arm)
    sim.unplug("follower_left")                             # power off, cable out
    wait(lambda: not live(arm))
    time.sleep(1.0)                                         # tried meanwhile, quietly: it isn't there
    assert not live(arm)
    sim.plug("follower_left")
    wait(lambda: live(arm))
    assert robot.state == "idle"


def test_an_arm_absent_at_start_is_connected_when_it_appears(rig):
    robot, sim = rig
    assert not robot.arms["leader_left"].connected
    sim.plug("leader_left")
    wait(lambda: live(robot.arms["leader_left"]))
