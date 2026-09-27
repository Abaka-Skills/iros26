"""The camera rig reopens a camera that goes down (a dropped cable) without a restart, but never
in a loop: a failed RealSense open stalls the whole process."""
import threading
import time

import pytest

from myaloha import cameras, sim
from myaloha.cameras import CameraRig


@pytest.fixture
def rig(monkeypatch):
    """A simulated rig under sudo whose opens are recorded in `rig.tries` and succeed while
    `rig.plugged`; USB shows `rig.usb`. Its resume thread is running."""
    monkeypatch.setattr(cameras, "RESUME_S", 0.2)
    monkeypatch.setattr(cameras.os, "geteuid", lambda: 0)
    r = CameraRig()
    sim.add_cameras(r)
    r.tries, r.plugged, r.usb = [], True, frozenset({(1, 1)})

    def reopen(keys, quiet=False):                                 # stands in for the RealSense open
        r.tries.append(list(keys))
        if r.plugged:
            for key in keys:
                r.add(key, sim.SimCamera(key, r.specs[key].label), depth_mm_per_unit=1.0)
    monkeypatch.setattr(r, "_open", reopen)
    monkeypatch.setattr("myaloha.realsense_macos.usb_addresses", lambda: r.usb)
    threading.Thread(target=r._resume, daemon=True).start()
    time.sleep(0.6)
    yield r
    r.disconnect()


def wait_ok(rig, key, ok=True, timeout=3.0):
    end = time.monotonic() + timeout
    while rig.status()[key]["ok"] != ok:
        assert time.monotonic() < end, rig.status()[key]
        time.sleep(0.05)


def test_a_dropped_camera_is_reopened(rig):
    assert all(c["ok"] for c in rig.status().values()) and not rig.tries
    rig.cams["low"].disconnect()                                   # a glitch: frames stop
    wait_ok(rig, "low", ok=False)
    wait_ok(rig, "low")
    assert rig.tries == [["low"]]


def test_a_camera_that_stays_down_is_retried_only_after_a_replug(rig):
    rig.plugged = False
    rig.cams["low"].disconnect()
    time.sleep(2.0)
    assert rig.tries == [["low"]]                                  # one try, no loop
    rig.plugged, rig.usb = True, frozenset({(1, 7)})               # replugged: a new USB address
    wait_ok(rig, "low")
    assert rig.tries == [["low"], ["low"]]
