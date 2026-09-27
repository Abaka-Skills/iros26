"""The API against simulated hardware (myaloha.sim); no rig needed."""
import logging
import time

import pytest
import trossen_arm
from fastapi.testclient import TestClient

from myaloha import sim
from myaloha.cameras import CameraRig
from myaloha.robot import Aloha
from myaloha.server import LogBuffer, create_app


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(trossen_arm, "TrossenArmDriver", sim.SimDriver)
    monkeypatch.setattr(sim.SimDriver, "ABSENT", {"192.168.10.2"})  # leader_right unplugged
    logs = LogBuffer()
    root = logging.getLogger("myaloha")
    root.setLevel(logging.INFO)
    root.addHandler(logs)
    rig = CameraRig()
    robot = Aloha(cameras=rig)
    robot.connect(timeout=0.1)
    sim.add_cameras(rig, absent={"wrist_right"})
    robot.start_monitor(hz=100)
    time.sleep(1.2)
    with TestClient(create_app(robot, logs, sim=True)) as c:
        yield c
    robot.stop()
    time.sleep(0.3)
    robot.disconnect()
    root.removeHandler(logs)


def wait_idle(c, timeout=10):
    end = time.monotonic() + timeout
    while c.get("/api/status").json()["state"] == "running":
        assert time.monotonic() < end, "mod did not finish"
        time.sleep(0.05)


def test_status_reports_live_arms_and_cameras(client):
    s = client.get("/api/status").json()
    assert s["state"] == "idle"
    assert s["arms"]["leader_right"]["connected"] is False
    assert s["arms"]["follower_left"]["connected"] and len(s["arms"]["follower_left"]["pos"]) == 7
    assert s["cameras"]["high"]["ok"] and s["cameras"]["high"]["fps"] > 10
    assert s["cameras"]["wrist_right"]["ok"] is False
    teleop = s["requirements"]["teleop"]
    assert teleop["met"] and [(i["label"], i["ok"]) for i in teleop["needs"][0]["items"]] == [
        ("left pair", True), ("right pair", False)]


def test_catalog_lists_params_states_transitions(client):
    teleop = client.get("/api/mods/teleop").json()
    assert {p["name"] for p in teleop["params"]} == {"duration_s", "stage_time", "sides"}
    assert teleop["transitions"]["pairing"] == ["staging"]
    assert "failed" in teleop["states"]


def test_run_is_refused_below_the_minimal_hardware(client):
    r = client.post("/api/mods/teleop/run", json={"params": {"sides": ["right"]}})
    assert r.status_code == 412 and "right pair: no leader_right" in r.json()["detail"]
    assert client.get("/api/status").json()["state"] == "idle"
    assert client.get("/api/mods/teleop").json()["requires"][0]["text"] == "a leader+follower pair on a requested side"


def test_run_rejects_bad_params_and_second_run(client):
    assert client.post("/api/mods/reset/run", json={"params": {"speed": 3}}).status_code == 422
    assert client.post("/api/mods/nope/run", json={}).status_code == 404
    r = client.post("/api/mods/reset/run", json={"params": {"goal_time": 0.3}})
    assert r.status_code == 202 and r.json()["name"] == "reset"
    assert client.post("/api/mods/instruction/run", json={}).status_code == 409
    wait_idle(client)
    tl = client.get("/api/timeline").json()
    assert tl[-2]["mod"] == "reset" and tl[-2]["result"] == "done"


def test_teleop_runs_left_pair_and_stops_on_request(client):
    r = client.post("/api/mods/teleop/run", json={"params": {"duration_s": 30, "stage_time": 0.2}})
    assert r.status_code == 202
    time.sleep(0.8)
    s = client.get("/api/status").json()
    assert s["mods"][0]["state"] == "following"
    assert s["arms"]["leader_left"]["mode"] == ["external_effort"]
    client.post("/api/stop")
    wait_idle(client)
    msgs = [r["msg"] for r in client.get("/api/log").json()["records"] if r["src"] == "teleop"]
    assert "skip right: leader_right not connected" in msgs and "releasing → stopped" in msgs


def test_stream_pushes_state_log_and_timeline(client):
    with client.websocket_connect("/api/stream?hz=30") as ws:
        kinds = {ws.receive_json()["type"] for _ in range(6)}
    assert {"state", "timeline", "log"} <= kinds


def test_camera_frames(client):
    r = client.get("/api/cameras/high/frame?kind=depth")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg" and r.content[:2] == b"\xff\xd8"
    assert client.get("/api/cameras/wrist_right/frame").status_code == 503


def test_emergency_stop_brakes_arms_and_latches(client):
    client.post("/api/mods/teleop/run", json={"params": {"duration_s": 30, "stage_time": 0.2}})
    time.sleep(0.8)
    t0 = time.monotonic()
    assert client.post("/api/estop").json() == {"estopped": True}
    assert time.monotonic() - t0 < 0.5
    wait_idle_or_estop = time.monotonic() + 5
    while client.get("/api/mods/running").json():
        assert time.monotonic() < wait_idle_or_estop
        time.sleep(0.05)
    s = client.get("/api/status").json()
    assert s["state"] == "estop"
    assert all(a["mode"] == ["idle"] and a["halted"] for a in s["arms"].values() if a["connected"])
    assert client.post("/api/mods/reset/run", json={}).status_code == 409
    log = [r["msg"] for r in client.get("/api/log").json()["records"]]
    assert "EMERGENCY STOP" in log and any("halted by emergency stop" in m for m in log)
    assert client.post("/api/estop/reset").json() == {"estopped": False}
    assert client.get("/api/status").json()["state"] == "idle"
    assert client.post("/api/mods/reset/run", json={"params": {"goal_time": 0.1}}).status_code == 202


def test_emergency_stop_over_the_stream(client):
    with client.websocket_connect("/api/stream") as ws:
        ws.send_json({"type": "estop"})
        for _ in range(60):
            m = ws.receive_json()
            if m["type"] == "state" and m["state"] == "estop":
                break
        else:
            raise AssertionError("no estop state on the stream")
    client.post("/api/estop/reset")


def test_camera_socket_sends_every_camera_and_switches_kind(client):
    with client.websocket_connect("/api/cameras/ws?fps=30") as ws:
        ws.send_json({"key": "high", "kind": "depth"})
        seen = {}
        for _ in range(40):
            b = ws.receive_bytes()
            n = b[0]
            key, kind, jpeg = b[1:1 + n].decode(), b[1 + n], b[2 + n:]
            assert jpeg[:2] == b"\xff\xd8"
            seen[key] = kind
            if seen.get("high") == 1 and {"low", "wrist_left"} <= seen.keys():
                break
    assert seen["high"] == 1 and seen["low"] == 0 and "wrist_right" not in seen  # wrist_right is unplugged
