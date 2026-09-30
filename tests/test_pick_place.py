"""pick_place end to end on the simulated rig, with nothing from its ground truth: the backend
calibrates itself (localize), finds the cube itself (the locate_cubes topic), then picks, moves and
places it. The simulator's truth only judges the outcome. Covers the full run, a refused plan, a
dry run, and the stop-and-reset when the cube's track is lost. Slow (~3 min): localize surveys first."""
import logging
import threading
import time

import httpx
import numpy as np
import pytest
import uvicorn
from simrig import connect, release, simulated_rig

from myaloha.mods import localize, pick_place
from myaloha.mods.localize import Localize
from myaloha.mods.pick_place import PickPlace
from myaloha.server import LogBuffer, create_app
from myaloha.topics.manager import TopicManager, free_port

CUBE = {"color": "red", "x": -0.20, "y": 0.05, "yaw": 20.0, "level": 0}
TARGET = (-0.15, -0.10)


@pytest.fixture(scope="module")
def rig(tmp_path_factory):
    """(robot, Sim) at full noise, calibrated by localize, with locate_cubes running."""
    mp = pytest.MonkeyPatch()
    data = tmp_path_factory.mktemp("rig")                  # this rig's calibration and logs, not the lab's
    mp.setattr(localize, "CALIB", data / "calib")
    mp.setattr(localize, "FRAMES", data / "frames")
    mp.setattr(pick_place, "LOGS", data / "pick_place")
    logs = LogBuffer()
    logging.getLogger("myaloha").setLevel(logging.INFO)
    logging.getLogger("myaloha").addHandler(logs)
    with simulated_rig(unplugged=("leader_left", "leader_right"), cubes=[CUBE], noise=1.0) as sim:
        robot = connect(sim.address)
        port = free_port()
        topics = TopicManager(robot, f"http://127.0.0.1:{port}")
        server = uvicorn.Server(uvicorn.Config(create_app(robot, logs, topics=topics), host="127.0.0.1",
                                               port=port, log_level="warning"))
        threading.Thread(target=server.run, daemon=True).start()
        while not server.started:
            time.sleep(0.05)
        m = Localize(robot, Localize.Params(move_time=2.0))
        assert m.run() == "done", m.error
        topics.start("locate_cubes", with_deps=True)
        yield robot, sim
        topics.stop_all()
        server.should_exit = True
        release(robot)
    logging.getLogger("myaloha").removeHandler(logs)
    mp.undo()


def the_cube(robot, near=(CUBE["x"], CUBE["y"]), timeout=30.0) -> dict:
    """The red cube as locate_cubes reports it near `near` (where it was put), once it does."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            m = robot.topics.latest("locate_cubes")
        except httpx.HTTPError:                          # the topic is still starting
            m = None
        cube = next((c for c in ((m or {}).get("data") or {}).get("cubes", []) if c["color"] == "red" and c["visible"]
                     and np.hypot(c["x"] - near[0], c["y"] - near[1]) < 0.03), None)
        if cube:
            return cube
        time.sleep(0.5)
    status = {n: {k: t.get(k) for k in ("state", "waiting", "hz")} for n, t in robot.topics.status().items()}
    try:
        cam = robot.topics.latest("cam_params")
        cam = cam and {"seq": cam.get("seq"), "has_data": cam.get("data") is not None}
    except httpx.HTTPError as e:
        cam = repr(e)
    seen = [(c["track"], c["color"], round(c["x"], 3), round(c["y"], 3), c["level"], c["visible"])
            for c in ((m or {}).get("data") or {}).get("cubes", [])]
    raise AssertionError(f"locate_cubes never reported the red cube near {near}; it has {seen}; topics {status}; "
                         f"cam_params latest {cam}")


def test_localize_found_the_bases(rig):
    robot, sim = rig
    left, right = (np.array(robot.coordinates["T_W_mount"][s])[:3, 3] for s in ("left", "right"))
    assert abs(np.linalg.norm(left - right) - 0.932) < 0.02      # the simulated rig's bases are 932 mm apart


def test_a_target_out_of_reach_is_refused_before_any_motion(rig):
    robot, _ = rig
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=0.0, target_y=0.30))
    assert m.run() == "failed" and "no follower can do it" in m.error, m.error
    assert [s for s, _ in m.history] == ["idle", "planning", "failed"]


def test_a_target_outside_the_safety_boundary_is_refused_before_any_motion(rig):
    robot, _ = rig
    robot.boundary.set(down=0.30)                          # the near edge 30 cm in: the target is beyond it
    try:
        m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1]))
        assert m.run() == "failed" and "the target at" in m.error and "outside the safety boundary" in m.error
        assert [s for s, _ in m.history] == ["idle", "planning", "failed"]
    finally:
        robot.boundary.set(down=0.03)


def test_dry_run_plans_and_moves_nothing(rig):
    robot, _ = rig
    before = robot.arms["follower_left"].positions()
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1],
                                          dry_run=True))
    assert m.run() == "done" and [s for s, _ in m.history] == ["idle", "planning", "done"]
    assert np.allclose(robot.arms["follower_left"].positions()[:6], before[:6], atol=0.01)   # reading jitter only


def test_picks_moves_and_places(rig):
    robot, sim = rig
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1],
                                          speed=0.1))
    assert m.run() == "done", m.error
    assert [s for s, _ in m.history] == ["idle", "planning", "staging", "looking", "approaching", "descending",
                                         "grasping", "lifting", "moving", "lowering", "releasing", "retreating",
                                         "returning", "done"]
    cube = next(c for c in sim.truth()["cubes"] if c["color"] == "red")
    assert cube["held_by"] is None and cube["level"] == 0
    assert np.hypot(cube["x"] - TARGET[0], cube["y"] - TARGET[1]) < 0.02, cube    # really on the target
    assert np.max(np.abs(robot.arms["follower_left"].positions()[:6])) < 0.03       # back at rest


def test_a_cube_placed_on_another_is_set_down_on_it_not_pressed_through(rig):
    """The arm lowers until it feels the cube rest on something (here a cube already on the target),
    lets go there, and the cube below stays where it was."""
    robot, sim = rig
    sim.spawn([CUBE, {"color": "blue", "x": TARGET[0], "y": TARGET[1], "yaw": 0.0, "level": 0}])
    time.sleep(3.0)
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1],
                                          speed=0.1))
    assert m.run() == "done", m.error
    red, blue = (next(c for c in sim.truth()["cubes"] if c["color"] == k) for k in ("red", "blue"))
    assert red["held_by"] is None and red["level"] == 1, red                       # on the blue one
    assert np.hypot(blue["x"] - TARGET[0], blue["y"] - TARGET[1]) < 0.003, blue    # not pushed away


FAR = (0.22, -0.05)          # the right arm's side: the left can't reach it, the right can't reach CUBE


def test_a_handover_carries_a_cube_neither_arm_can_move_alone(rig):
    robot, sim = rig
    sim.spawn([CUBE])
    time.sleep(3.0)
    track = the_cube(robot)["track"]
    alone = PickPlace(robot, PickPlace.Params(track=track, target_x=FAR[0], target_y=FAR[1], handover=False))
    assert alone.run() == "failed" and "handover=true" in alone.error
    m = PickPlace(robot, PickPlace.Params(track=track, target_x=FAR[0], target_y=FAR[1], speed=0.1))    # handover by default
    worker, carried = m.start(), []
    while worker.is_alive():                             # the cube's height while the taker carries it over
        if m.state == "moving":
            carried.append(next(c for c in sim.truth()["cubes"] if c["color"] == "red")["z"])
        time.sleep(0.05)
    assert m.state == "done", m.error
    path = [s for s, _ in m.history]
    assert path[path.index("lifting"):] == ["lifting", "handing_over", "receiving", "exchanging", "moving", "lowering",
                                            "releasing", "retreating", "returning", "done"]
    assert carried and min(carried) > 0.04, min(carried)   # never swung down near the table (it was 25 mm)
    cube = next(c for c in sim.truth()["cubes"] if c["color"] == "red")
    assert cube["held_by"] is None and cube["level"] == 0
    assert np.hypot(cube["x"] - FAR[0], cube["y"] - FAR[1]) < 0.02, cube
    for side in ("left", "right"):
        assert np.max(np.abs(robot.arms[f"follower_{side}"].positions()[:6])) < 0.03   # both back at rest


def test_a_lost_cube_stops_the_task_and_resets_the_arms(rig):
    robot, sim = rig
    sim.spawn([CUBE])
    time.sleep(3.0)                                     # the moved cube's old track fades, the new one confirms
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1],
                                          lost_s=1.0))
    worker = m.start()
    end = time.monotonic() + 30
    while m.state != "staging" and worker.is_alive():     # gone before any grasp: only the tracker can tell
        assert time.monotonic() < end
        time.sleep(0.1)
    sim.spawn([])                                       # the cube is gone
    worker.join(60)
    assert m.state == "failed" and "stopped and reset the arms" in m.error, m.error   # the tracker or the wrist saw it go
    assert "resetting" in [s for s, _ in m.history]
    assert np.max(np.abs(robot.arms["follower_left"].positions()[:6])) < 0.03       # reset to zero


def test_a_cube_dropped_on_the_way_is_found_again_and_placed(rig):
    """A failed try is recovered from: the cube leaves the gripper in the carry (it lands on the table
    partway, as a slip would), the check above the target sees an empty gripper, the arm backs off, the
    cameras find the cube on the way, and the next try places it."""
    robot, sim = rig
    sim.spawn([CUBE])
    time.sleep(3.0)
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1],
                                          speed=0.1))
    worker = m.start()
    end = time.monotonic() + 60
    while m.state != "moving" and worker.is_alive():
        assert time.monotonic() < end
        time.sleep(0.05)
    mid = {**CUBE, "x": (CUBE["x"] + TARGET[0]) / 2, "y": (CUBE["y"] + TARGET[1]) / 2, "yaw": 5.0}
    sim.spawn([mid])                                    # out of the gripper, onto the table on the way
    worker.join(240)
    assert m.state == "done", m.error
    path = [s for s, _ in m.history]
    assert "recovering" in path and path.count("grasping") == 2, path
    cube = next(c for c in sim.truth()["cubes"] if c["color"] == "red")
    assert cube["held_by"] is None and cube["level"] == 0
    assert np.hypot(cube["x"] - TARGET[0], cube["y"] - TARGET[1]) < 0.02, cube


def test_auto_target_keeps_moving_random_cubes_to_free_places_until_stopped(rig):
    """With auto_target, every round picks a cube and a free place by itself: the places it chooses are
    clear of every cube (by the cameras, so checked here with some slack against the truth), each cube
    lands where it was sent, and a stop ends it with the cubes apart on the table."""
    robot, sim = rig
    sim.spawn([CUBE, {"color": "blue", "x": -0.12, "y": -0.02, "yaw": 0.0, "level": 0}])
    time.sleep(3.0)
    the_cube(robot)
    m = PickPlace(robot, PickPlace.Params(speed=0.1, auto_target=True))
    chosen, choose = [], m._choose

    def recording():
        cube, plans = choose()
        truth = sim.truth()["cubes"]
        chosen.append((cube["color"], (m.params.target_x, m.params.target_y),
                       [(c["x"], c["y"]) for c in truth if c["color"] in ("red", "blue")]))
        return cube, plans
    m._choose = recording
    worker = m.start()
    end = time.monotonic() + 400
    while len(chosen) < 3 and worker.is_alive():         # two rounds done, the third chosen
        assert time.monotonic() < end, m.history[-5:]
        time.sleep(0.2)
    m.stop()
    worker.join(120)
    assert m.state == "stopped", m.error
    path = [s for s, _ in m.history]
    assert path.count("releasing") >= 2 and path.count("planning") >= 3, path
    for color, target, cubes in chosen:
        assert min(np.hypot(x - target[0], y - target[1]) for x, y in cubes) > pick_place.AUTO_GAP - 0.015, (target, cubes)
    truth = [c for c in sim.truth()["cubes"] if c["color"] in ("red", "blue")]
    assert all(c["level"] == 0 and c["held_by"] is None for c in truth), truth
    assert np.hypot(truth[0]["x"] - truth[1]["x"], truth[0]["y"] - truth[1]["y"]) > 0.03, truth


def test_iros26_serves_its_page_and_carries_out_a_choice_from_it(rig):
    """iros26: pickandplay.html is served only while it runs; a choice sent as the page sends it (the
    cube's track, then the target) is carried out by pick_place, and the mod waits for the next one."""
    robot, sim = rig
    sim.spawn([CUBE])
    time.sleep(3.0)
    api = httpx.Client(base_url=robot.topics.backend, timeout=10)
    choice = {"track": the_cube(robot)["track"], "target_x": TARGET[0], "target_y": TARGET[1]}
    assert api.get("/pickandplay.html").status_code == 404
    assert api.post("/api/mods/iros26/input", json=choice).status_code == 409          # not running

    def iros26():
        return next((m for m in api.get("/api/mods/running").json() if m["name"] == "iros26"), None)

    def wait(ok, timeout):
        end = time.monotonic() + timeout
        while not ok():
            assert time.monotonic() < end, iros26()
            time.sleep(0.2)

    assert api.post("/api/mods/iros26/run", json={"params": {"speed": 0.1}}).status_code == 202
    try:
        assert api.get("/pickandplay.html").status_code == 200
        wait(lambda: iros26()["state"] == "waiting", 5)
        assert api.post("/api/mods/iros26/input", json={"track": "red"}).status_code == 422
        assert api.post("/api/mods/iros26/input", json=choice).status_code == 200
        assert api.post("/api/mods/iros26/input", json=choice).status_code == 409      # one at a time
        wait(lambda: iros26()["rounds"], 240)
        (done,) = iros26()["rounds"]
        assert done["result"] == "done", done["error"]
        assert iros26()["state"] == "waiting"                                           # for the next one
    finally:
        api.post("/api/stop")
        wait(lambda: not robot.mods, 30)
    cube = next(c for c in sim.truth()["cubes"] if c["color"] == "red")
    assert cube["held_by"] is None and np.hypot(cube["x"] - TARGET[0], cube["y"] - TARGET[1]) < 0.02, cube
    assert api.get("/pickandplay.html").status_code == 404


def test_iros26_auto_hands_a_random_cube_over_to_the_other_arms_side(rig):
    """An auto round, as the page asks for one when no one has used it for a while: pick_place chooses
    the cube and a free place only the other arm reaches, so the cube is handed over. Refused when
    iros26 runs without handover."""
    robot, sim = rig
    sim.spawn([CUBE])                                    # on the left arm's side
    time.sleep(3.0)
    the_cube(robot)
    api = httpx.Client(base_url=robot.topics.backend, timeout=10)

    def running(name):
        return next((m for m in api.get("/api/mods/running").json() if m["name"] == name), None)

    def wait(ok, timeout):
        end = time.monotonic() + timeout
        while not ok():
            assert time.monotonic() < end, running("iros26")
            time.sleep(0.2)

    assert api.post("/api/mods/iros26/run", json={"params": {"handover": False}}).status_code == 202
    try:
        wait(lambda: running("iros26")["state"] == "waiting", 5)
        assert api.post("/api/mods/iros26/input", json={"auto": True}).status_code == 422
    finally:
        api.post("/api/stop")
        wait(lambda: not robot.mods, 30)
    assert api.post("/api/mods/iros26/run", json={"params": {"speed": 0.1}}).status_code == 202
    states = set()
    try:
        wait(lambda: running("iros26")["state"] == "waiting", 5)
        assert api.post("/api/mods/iros26/input", json={"auto": True}).status_code == 200
        end = time.monotonic() + 300
        while not running("iros26")["rounds"]:
            assert time.monotonic() < end, running("iros26")
            states.add((running("pick_place") or {}).get("state"))
            time.sleep(0.2)
        (r,) = running("iros26")["rounds"]
        assert r["result"] == "done" and r["auto"] and r["track"], r
    finally:
        api.post("/api/stop")
        wait(lambda: not robot.mods, 30)
    assert "handing_over" in states, states
    cube = next(c for c in sim.truth()["cubes"] if c["color"] == "red")
    assert cube["held_by"] is None and cube["level"] == 0 and cube["x"] > 0, cube   # now on the right arm's side
    assert np.hypot(cube["x"] - r["target_x"], cube["y"] - r["target_y"]) < 0.02, (cube, r)


def test_a_placed_cube_is_set_down_not_dropped(rig):
    """The fingers let go only once the cube rests on the table (its bottom on it, not above it), and
    open only a little: the cube stays where it was set down, square and flat."""
    robot, sim = rig
    sim.spawn([CUBE])
    time.sleep(3.0)
    samples, done = [], threading.Event()

    def watch():                                        # the red cube as it really is, ~30 times a second
        while not done.is_set():
            samples.append(next(c for c in sim.truth()["cubes"] if c["color"] == "red"))
            time.sleep(0.03)
    watcher = threading.Thread(target=watch, daemon=True)
    watcher.start()
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1],
                                          speed=0.1))
    try:
        assert m.run() == "done", m.error
    finally:
        done.set()
        watcher.join()
    held = [i for i, c in enumerate(samples) if c["held_by"]]
    let_go, end = samples[held[-1]], samples[-1]        # the last moment in the fingers; at rest after
    assert let_go["z"] - 0.0125 < 0.0015, let_go        # its bottom on the table when they opened
    assert np.hypot(end["x"] - let_go["x"], end["y"] - let_go["y"]) < 0.003, (let_go, end)   # not pushed
    assert abs(end["z"] - 0.0125) < 0.001 and end["tilt"] < 2.0, end                        # flat on the table


def grasp_axis(m, track):
    """The planned grasp's closing axis (the tool's y) in W xy, planning only."""
    plans = m._plan_task(m._cube(track))
    return next(iter(plans.values()))["R"]["grasp"][:2, 1]


def test_the_fingers_go_down_on_the_faces_the_neighbours_leave_free(rig):
    """Cubes 1 cm off the two faces the fingers would go down on: the grasp turns 90 deg to the free
    pair of faces, and the neighbours stay where they are."""
    robot, sim = rig
    sim.spawn([CUBE])
    time.sleep(3.0)
    track = the_cube(robot)["track"]
    axis = grasp_axis(PickPlace(robot, PickPlace.Params(track=track, target_x=TARGET[0], target_y=TARGET[1])), track)
    normals = [np.array([np.cos(a), np.sin(a)]) for a in np.radians(CUBE["yaw"] + np.arange(4) * 90)]
    axis = max(normals, key=lambda n: n @ axis)                  # the face the fingers would go down on
    around = [{"color": color, "x": CUBE["x"] + s * 0.035 * axis[0], "y": CUBE["y"] + s * 0.035 * axis[1],
               "yaw": CUBE["yaw"], "level": 0} for color, s in (("blue", 1), ("green", -1))]
    sim.spawn([CUBE, *around])
    time.sleep(3.0)
    track = the_cube(robot)["track"]
    m = PickPlace(robot, PickPlace.Params(track=track, target_x=TARGET[0], target_y=TARGET[1], speed=0.1))
    assert abs(grasp_axis(m, track) @ axis) < 0.5                 # closing across the free faces (the yaw the
                                                                  # overhead camera reads is coarse)
    assert m.run() == "done", m.error
    truth = {c["color"]: c for c in sim.truth()["cubes"]}
    for a in around:
        assert np.hypot(truth[a["color"]]["x"] - a["x"], truth[a["color"]]["y"] - a["y"]) < 0.002, truth[a["color"]]
    assert np.hypot(truth["red"]["x"] - TARGET[0], truth["red"]["y"] - TARGET[1]) < 0.02, truth["red"]


def test_a_boxed_in_cube_or_a_target_half_on_a_cube_is_refused_before_any_motion(rig):
    robot, sim = rig
    ring = [{"color": color, "x": CUBE["x"] + dx, "y": CUBE["y"] + dy, "yaw": 0.0, "level": 0}
            for color, dx, dy in (("blue", 0.03, 0), ("green", -0.03, 0), ("yellow", 0, 0.03), ("purple", 0, -0.03))]
    sim.spawn([{**CUBE, "yaw": 0.0}, *ring])
    time.sleep(3.0)
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0], target_y=TARGET[1]))
    assert m.run() == "failed" and "boxed in" in m.error, m.error
    assert [s for s, _ in m.history] == ["idle", "planning", "failed"]
    sim.spawn([CUBE, {"color": "blue", "x": TARGET[0], "y": TARGET[1], "yaw": 0.0, "level": 0}])
    time.sleep(3.0)
    m = PickPlace(robot, PickPlace.Params(track=the_cube(robot)["track"], target_x=TARGET[0] + 0.012, target_y=TARGET[1]))
    assert m.run() == "failed" and "half on cube" in m.error, m.error
