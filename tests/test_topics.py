"""Topics as real child processes of a real (simulated-hardware) backend: dependencies on start
and stop, results over each topic's own port, a crash, and cleanup."""
import logging
import os
import signal
import threading
import time

import httpx
import numpy as np
import pytest
import trossen_arm
import uvicorn

from myaloha import sim
from myaloha.cameras import CameraRig
from myaloha.coords import look_at
from myaloha.robot import Aloha
from myaloha.server import LogBuffer, create_app
from myaloha.topics.manager import TopicError, TopicManager, free_port


def coordinates() -> dict:
    """A calibration as localize would save it: bases 0.93 m apart, facing, an overhead camera."""
    left = np.eye(4)
    left[0, 3] = -0.465
    right = np.diag([-1.0, -1.0, 1.0, 1.0])
    right[0, 3] = 0.465
    high = {"T_W_cam": look_at([0, 0, 0], [0, -1], 1.0, np.radians(85)).tolist(),
            "K": [[380.0, 0, 320], [0, 380.0, 240], [0, 0, 1]], "dist": [0.0] * 5, "model": "brown_conrady",
            "size": [640, 480], "report": {}}
    return {"id": "test", "T_W_base": {"left": left.tolist(), "right": right.tolist()},
            "T_W_mount": {"left": left.tolist(), "right": right.tolist()}, "cameras": {"high": high}}


@pytest.fixture(scope="module")
def backend():
    mp = pytest.MonkeyPatch()
    mp.setattr(trossen_arm, "TrossenArmDriver", sim.SimDriver)
    logs = LogBuffer()
    logging.getLogger("myaloha").setLevel(logging.INFO)
    logging.getLogger("myaloha").addHandler(logs)
    rig = CameraRig()
    robot = Aloha(cameras=rig)
    robot.connect(timeout=0.1)
    sim.add_cameras(rig)
    robot.start_monitor(hz=100)
    robot.coordinates = coordinates()
    port = free_port()
    topics = TopicManager(robot, f"http://127.0.0.1:{port}")
    server = uvicorn.Server(uvicorn.Config(create_app(robot, logs, sim=True, topics=topics), host="127.0.0.1",
                                           port=port, log_level="warning"))
    threading.Thread(target=server.run, daemon=True).start()
    while not server.started:
        time.sleep(0.05)
    yield robot, topics, logs
    topics.stop_all()
    server.should_exit = True
    robot.stop()
    robot.disconnect()
    logging.getLogger("myaloha").removeHandler(logs)
    mp.undo()


def wait_for(cond, timeout=15.0):
    end = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < end, "timed out"
        time.sleep(0.1)


def test_start_needs_dependencies_then_serves_results(backend):
    robot, topics, logs = backend
    with pytest.raises(TopicError) as e:
        topics.start("locate_cubes")
    assert e.value.detail["missing"] == ["cam_params"]
    topics.start("locate_cubes", with_deps=True)
    wait_for(lambda: topics.status()["cam_params"]["state"] == "running")
    st = topics.status()
    assert st["locate_cubes"]["state"] in ("running", "waiting", "starting")
    m = httpx.get(st["cam_params"]["url"] + "/latest").json()
    cams = m["data"]["cameras"]
    assert cams["high"]["source"] == "localize" and cams["high"]["T_W_cam"] is not None
    assert cams["low"]["T_W_cam"] is None and "not registered" in cams["low"]["reason"]
    wl = np.array(cams["wrist_left"]["T_W_cam"])          # follows the left follower's joints
    assert wl[0, 3] < 0 and 0.1 < wl[2, 3] < 0.4
    assert any(r["src"] == "cam_params" and "serving" in r["msg"] for r in logs.since(0))   # its log reached ours


def test_stop_asks_about_dependents_and_cascades(backend):
    robot, topics, _ = backend
    topics.start("locate_cubes", with_deps=True)
    with pytest.raises(TopicError) as e:
        topics.stop("cam_params")
    assert e.value.detail["dependents"] == ["locate_cubes"]
    assert topics.stop("cam_params", cascade=True) == ["locate_cubes", "cam_params"]
    assert {t["state"] for t in topics.status().values()} == {"stopped"}


def test_a_crash_is_reported_not_restarted(backend):
    robot, topics, _ = backend
    topics.start("cam_params")
    wait_for(lambda: topics.status()["cam_params"]["state"] == "running")
    os.kill(topics.status()["cam_params"]["pid"], signal.SIGKILL)
    wait_for(lambda: topics.status()["cam_params"]["state"] == "crashed")
    time.sleep(1.0)
    assert topics.status()["cam_params"]["state"] == "crashed"   # still: nothing restarts on its own
    topics.start("cam_params")
    wait_for(lambda: topics.status()["cam_params"]["state"] == "running")
    topics.stop_all()
    assert not topics.procs


def test_requirements_block_the_start(backend):
    robot, topics, _ = backend
    doc, robot.coordinates = robot.coordinates, None
    try:
        with pytest.raises(TopicError) as e:
            topics.start("cam_params")
        assert "coordinate calibration" in str(e.value)
    finally:
        robot.coordinates = doc
