"""The relay follows the cameras with nobody's help: frames flow, an unplugged camera's stream ends,
a replugged one is served again, a wedged one is reopened, one that won't open is retried. A fake
source stands in for USB."""
import socket
import threading
import time

import numpy as np
import pytest

from camrelay import relay as relay_mod
from camrelay.protocol import close, recv, send
from camrelay.relay import Relay


class FakeCamera:
    def __init__(self, serial):
        self.serial, self.alive, self.latest = serial, True, None
        self.optics = {"K_color": [[400.0, 0, 320], [0, 400.0, 240], [0, 0, 1]], "aligned": True}
        self.depth_mm_per_unit = 0.1
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        seq = 0
        while self.alive:
            seq += 1
            with self.cond:
                self.latest = (seq, time.perf_counter(), np.full((48, 64, 3), seq % 255, np.uint8),
                               np.full((48, 64), 1000, np.uint16))
                self.cond.notify_all()
            time.sleep(0.02)

    def frame(self, after, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: not self.alive or self.latest[0] > after, timeout)
            return self.latest if self.alive and self.latest[0] > after else None

    def close(self):
        self.alive = False
        with self.cond:
            self.cond.notify_all()


class FakeSource:
    def __init__(self, *serials):
        self.attached, self.opened, self.broken = set(serials), [], set()

    def serials(self):
        return set(self.attached)

    def open(self, serial):
        if serial in self.broken:
            raise RuntimeError("failed to set power state")
        cam = FakeCamera(serial)
        self.opened.append(cam)
        return cam


def hello(address, **head):
    host, port = address.rsplit(":", 1)
    sock = socket.create_connection((host, int(port)), timeout=3)
    send(sock, head)
    return sock, recv(sock)[0]


def frames(sock, n=3):
    got = [recv(sock) for _ in range(n)]
    return [h["t"] for h, _ in got], got[-1][1]


def wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < end
        time.sleep(0.02)


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setattr(relay_mod, "POLL_S", 0.05)
    monkeypatch.setattr(relay_mod, "RETRY_S", 0.3)
    source = FakeSource("A", "B")
    r = Relay(source, "127.0.0.1:0").start()
    wait(lambda: set(r.cams) == {"A", "B"})
    yield r, source
    r.stop()


def test_frames_flow_with_their_capture_times(rig):
    r, _ = rig
    sock, head = hello(r.address, open="camera", serial="A")
    assert head["depth_mm_per_unit"] == 0.1 and head["optics"]["aligned"]
    ts, (color, depth) = frames(sock)
    assert ts == sorted(ts) and len(color) == 48 * 64 * 3 and len(depth) == 48 * 64 * 2
    close(sock)


def test_an_unplugged_camera_ends_its_stream_and_is_served_again_when_replugged(rig):
    r, source = rig
    sock, _ = hello(r.address, open="camera", serial="A")
    frames(sock)
    source.attached.discard("A")                       # unplugged
    with pytest.raises((ConnectionError, OSError)):
        for _ in range(200):
            recv(sock)
    _, head = hello(r.address, open="camera", serial="A")
    assert head["error"] == "camera A: not attached"
    source.attached.add("A")                           # plugged back in
    wait(lambda: "A" in r.cams)
    sock, head = hello(r.address, open="camera", serial="A")
    assert "optics" in head
    frames(sock)
    close(sock)


def test_a_camera_whose_frames_stop_is_reopened(rig):
    r, source = rig
    first = r.cams["B"]
    first.alive = False                                # wedged: frames stopped, still attached
    wait(lambda: "B" in r.cams and r.cams["B"] is not first)
    sock, _ = hello(r.address, open="camera", serial="B")
    frames(sock)
    close(sock)


def test_a_camera_that_wont_open_is_retried_and_reported(rig):
    r, source = rig
    source.broken.add("C")
    source.attached.add("C")
    wait(lambda: "C" in r.failed)
    _, head = hello(r.address, open="camera", serial="C")
    assert head["error"] == "camera C: attached, not open: failed to set power state"
    _, status = hello(r.address, open="status")
    assert status["attached"] == ["A", "B", "C"] and status["streaming"] == ["A", "B"]
    source.broken.discard("C")                         # it opens on a later try
    wait(lambda: "C" in r.cams)


def test_a_relay_whose_camera_listing_keeps_failing_stops_to_be_started_afresh(rig):
    """After a USB reset librealsense can fail to list the cameras for good in a running process; a
    fresh one lists them. So after LIST_FAILS failures in a row the relay stops, marked stuck (the
    daemon then exits and launchd starts it again). A single failure is only a hiccup."""
    r, source = rig
    broken = source.serials
    source.serials = lambda: (_ for _ in ()).throw(RuntimeError("failed to set power state"))
    time.sleep(relay_mod.POLL_S * 1.5)
    source.serials = broken                                        # one failure, then fine again
    time.sleep(relay_mod.POLL_S * (relay_mod.LIST_FAILS + 2))
    assert not r.stopped.is_set() and not r.stuck
    source.serials = lambda: (_ for _ in ()).throw(RuntimeError("failed to set power state"))
    wait(lambda: r.stopped.is_set())
    assert r.stuck
