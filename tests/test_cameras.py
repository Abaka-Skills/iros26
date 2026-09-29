"""The camera rig follows its cameras through whoever serves them (the relay, here the simulator): a
camera that drops is asked for again every RESUME_S and resumes as soon as it is served again, with
nobody doing anything; asking is cheap (the server answers at once whether it has the camera)."""
import time

import pytest
from simrig import simulated_rig

from myaloha import cameras
from myaloha.cameras import CameraRig


@pytest.fixture
def rig(monkeypatch):
    """A rig on the simulator whose opens are recorded in `rig.tries`."""
    monkeypatch.setattr(cameras, "RESUME_S", 0.2)
    with simulated_rig() as sim:
        r = CameraRig(hardware=sim.address)
        r.sim, r.tries = sim, []
        opened = r._open

        def counting(keys, quiet=False):
            r.tries.append(list(keys))
            opened(keys, quiet)
        monkeypatch.setattr(r, "_open", counting)
        r.connect()
        for key in r.specs:
            wait_ok(r, key)
        r.tries.clear()
        yield r
        r.disconnect()


def wait_ok(rig, key, ok=True, timeout=5.0):
    end = time.monotonic() + timeout
    while rig.status()[key]["ok"] != ok:
        assert time.monotonic() < end, rig.status()[key]
        time.sleep(0.05)


def test_a_dropped_camera_is_reopened(rig):
    rig.sim.unplug("low")                                          # a glitch: the stream drops ...
    rig.sim.plug("low")                                            # ... and the camera is back at once
    wait_ok(rig, "low", ok=False)
    wait_ok(rig, "low")
    assert rig.tries and all(t == ["low"] for t in rig.tries)      # only the camera that dropped


def test_a_camera_away_for_a_while_resumes_when_it_is_back(rig):
    rig.sim.unplug("low")
    wait_ok(rig, "low", ok=False)
    time.sleep(1.5)
    assert rig.status()["low"]["error"]                            # says why: the server doesn't have it
    assert all(rig.status()[k]["ok"] for k in ("high", "wrist_left", "wrist_right"))
    rig.sim.plug("low")                                            # plugged back in: nothing else to do
    wait_ok(rig, "low")
