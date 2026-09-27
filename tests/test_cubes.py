"""locate_cubes' algorithm on a synthetic table: coloured cubes ray-cast into an overhead camera
(far: noisy depth) and a wrist-like camera (near), with W known exactly."""
import cv2
import numpy as np
import pytest

from myaloha import cubes
from myaloha.coords import look_at

K = np.array([[430.0, 0, 320], [0, 430.0, 240], [0, 0, 1]])
H, W_ = 480, 640
S = cubes.SIZE


def cube(color, x, y, level=0, yaw=0.0, visible=True, confidence=1.0):
    return {"color": color, "x": x, "y": y, "z": (level + 0.5) * S, "level": level, "level_measured": True,
            "yaw": yaw, "visible": visible, "confidence": confidence, "sigma": 0.003, "seen_by": ["high"]}


def lab_to_bgr(L, a, b):
    return cv2.cvtColor(np.uint8([[[L, a + 128, b + 128]]]), cv2.COLOR_Lab2BGR)[0, 0].astype(float)


def render(T_W_cam, scene, noise, seed=0):
    """Colour and depth of the table (z = 0) and `scene` cubes [(color, x, y, level, yaw_deg)]."""
    v, u = np.mgrid[0:H, 0:W_].astype(float)
    d = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)
    dW = d @ T_W_cam[:3, :3].T
    o = T_W_cam[:3, 3]
    t_best = np.where(dW[..., 2] < 0, -o[2] / dW[..., 2], np.inf)       # the table
    color = np.zeros((H, W_, 3)) + lab_to_bgr(*cubes.TABLE_REF)
    for name, x, y, lv, yaw in scene:
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        centre = np.array([x, y, (lv + 0.5) * S])
        ol, dl = R.T @ (o - centre), dW @ R                              # into the cube's frame
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (-S / 2 - ol) / dl, (S / 2 - ol) / dl
        tn, tf = np.minimum(t1, t2).max(-1), np.maximum(t1, t2).min(-1)
        hit = (tn < tf) & (tn > 0) & (tn < t_best)
        t_best = np.where(hit, tn, t_best)
        color[hit] = lab_to_bgr(*cubes.PROTO[name])
    ray_len = np.linalg.norm(d, axis=-1)
    depth = t_best * np.linalg.norm(dW, axis=-1) / ray_len               # along the optical axis
    depth = depth + np.random.default_rng(seed).normal(0, noise, depth.shape)
    depth[~np.isfinite(t_best)] = 0
    return np.clip(color, 0, 255).astype(np.uint8), depth.astype(np.float32)


SCENE = [("red", 0.10, 0.05, 0, 20), ("green", -0.12, -0.04, 0, 0), ("blue", 0.0, 0.12, 0, 0),
         ("yellow", 0.0, 0.12, 1, 0), ("purple", 0.15, -0.10, 0, 40)]
HIGH = look_at([0.0, 0.0, 0.0], [0.0, -1.0], 1.0, np.radians(85))
NEAR = look_at([0.0, 0.05, 0.0], [-1.0, -0.6], 0.45, np.radians(35))


def view(key, T, noise, scene=SCENE):
    return cubes.CameraView(key, *render(T, scene, noise), K, np.zeros(5), "brown_conrady", T)


def visible(found):
    return {(c["color"], c["level"]): c for c in found if c["visible"]}


def test_overhead_alone_finds_every_top_but_not_levels():
    found, _ = cubes.locate([view("high", HIGH, noise=0.015)])
    got = visible(found)
    assert {k[0] for k in got} == {"red", "green", "yellow", "purple"}             # blue is under yellow
    for name, x, y, lv, _ in SCENE:
        if name != "blue":
            c = next(c for c in found if c["color"] == name)
            assert not c["level_measured"]
            if lv == 0:
                assert np.hypot(c["x"] - x, c["y"] - y) < 0.004, c


def test_near_camera_measures_the_stack_and_hidden_cube_colour():
    found, _ = cubes.locate([view("high", HIGH, noise=0.015), view("near", NEAR, noise=0.001)])
    got = visible(found)
    assert ("yellow", 1) in got and got[("yellow", 1)]["level_measured"], found
    assert ("blue", 0) in got, found                                              # seen from the side
    y = got[("yellow", 1)]
    assert np.hypot(y["x"], y["y"] - 0.12) < 0.005 and abs(y["z"] - 1.5 * S) < 1e-9
    assert sum(1 for c in found if c["visible"]) == len(SCENE)
    assert got[("red", 0)]["yaw"] is not None and abs(got[("red", 0)]["yaw"] - 20) < 4


def test_hidden_cube_is_reported_without_colour():
    """The near camera sees the stack's height from a side where the lower cube is occluded."""
    g = S + 0.0005                                                # the bottom cube is boxed in
    scene = [("blue", 0.0, 0.0, 0, 0), ("yellow", 0.0, 0.0, 1, 0), ("red", -g, 0.0, 0, 0), ("red", g, 0.0, 0, 0),
             ("red", 0.0, -g, 0, 0), ("red", 0.0, g, 0, 0)]
    near = look_at([0.0, 0.0, 0.0], [-1.0, -0.6], 0.45, np.radians(35))
    found, _ = cubes.locate([view("high", HIGH, 0.015, scene), view("near", near, 0.001, scene)])
    col = [c for c in found if np.hypot(c["x"], c["y"]) < 0.01]
    assert [(c["level"], c["color"], c["visible"]) for c in sorted(col, key=lambda c: c["level"])] == [
        (0, None, False), (1, "yellow", True)], col


def test_wrist_pose_is_corrected_by_the_table():
    """A wrist pose tilted 1.5 deg and 1 cm high (arm sag) is levelled onto the table before
    detection. The plane fixes tilt and height; the small sideways shift a tilt causes stays."""
    T = NEAR.copy()
    color, depth = render(T, SCENE, 0.001)
    T_bad = T.copy()
    T_bad[:3, :3] = cv2.Rodrigues(np.array([np.radians(1.5), 0, 0]))[0] @ T[:3, :3]
    T_bad[2, 3] += 0.01
    v = cubes.CameraView("wrist_left", color, depth, K, np.zeros(5), "brown_conrady", T_bad, fixed=False)
    found, info = cubes.locate([v])
    fix = info["wrist_left"]["table_fix"]
    assert abs(fix["tilt_deg"] - 1.5) < 0.2, fix
    red = next(c for c in found if c["color"] == "red")
    assert red["level"] == 0 and red["level_measured"] and np.hypot(red["x"] - 0.10, red["y"] - 0.05) < 0.008, red


def test_tracker_bridges_misses_and_ignores_one_frame_hits():
    tr = cubes.Tracker()
    red = cube("red", 0.1, 0.05)
    ghost = cube("green", -0.2, 0.05, confidence=0.3)
    assert tr.update([red, ghost], 0.0) == []                          # not confirmed yet
    out = tr.update([{**red, "x": 0.104}], 0.33)
    assert [c["color"] for c in out] == ["red"] and abs(out[0]["x"] - 0.102) < 1e-9   # smoothed
    held = tr.update([], 0.66)                                         # a missed frame: still there
    assert len(held) == 1 and held[0]["held"] and held[0]["track"] == out[0]["track"]
    flipped = tr.update([{**red, "color": "orange", "confidence": 0.4}], 1.0)
    assert flipped[0]["color"] == "red"                                # one odd colour doesn't flip it
    assert tr.update([], 2.2) == []                                    # gone for > 1 s: dropped


def test_tracker_keeps_hidden_cubes():
    tr = cubes.Tracker()
    hidden = cube(None, 0.0, 0.0, visible=False, confidence=0.0)
    tr.update([hidden], 0.0)
    out = tr.update([hidden], 0.33)
    assert len(out) == 1 and out[0]["color"] is None and not out[0]["visible"]


def test_table_level_camera_reads_a_stack_beside_a_touching_cube():
    """Overhead + a camera 3 cm above the table (like the low one): the overhead finds the columns,
    the low camera gives the stack's levels and colours, and the cube touching the stack's base
    (between it and the camera, off to one side) is not taken for the base."""
    scene = [("blue", 0.0, 0.12, 0, 0), ("purple", 0.0, 0.12, 1, 0), ("green", 0.0, 0.12, 2, 0),
             ("purple", 0.026, 0.10, 0, 0), ("red", -0.15, 0.05, 0, 0)]
    low = look_at([0.0, 0.10, 0.03], [0.0, -1.0], 0.40, np.radians(0.5))
    found, _ = cubes.locate([view("high", HIGH, 0.015, scene), view("low", low, 0.001, scene)])
    stack = sorted([c for c in found if np.hypot(c["x"], c["y"] - 0.12) < 0.012], key=lambda c: c["level"])
    assert [(c["level"], c["color"]) for c in stack] == [(0, "blue"), (1, "purple"), (2, "green")], found
    assert all(c["level_measured"] for c in stack)
    beside = [c for c in found if np.hypot(c["x"] - 0.026, c["y"] - 0.10) < 0.01]
    assert [(c["level"], c["color"]) for c in beside] == [(0, "purple")], found


def test_no_two_cubes_overlap():
    kept = cubes.resolve_overlaps([cube("red", 0.0, 0.0), cube("red", 0.008, 0.0, confidence=0.6)])
    assert len(kept) == 1 and kept[0]["confidence"] == 1.0                         # a duplicate
    touching = cubes.resolve_overlaps([cube("red", 0.0, 0.0), cube("blue", S, 0.0), cube("green", 0.0, S, yaw=45)])
    assert len(touching) == 3                                                       # neighbours stay
    stacked = cubes.resolve_overlaps([cube("red", 0.0, 0.0), cube("red", 0.0, 0.0, level=1)])
    assert len(stacked) == 2                                                        # a stack is fine
    # a stacked cube's side seen from above as a cube beside it: one cube, at the stack's place
    merged = cubes.resolve_overlaps([cube(None, 0.0, 0.0, visible=False, confidence=0.0), cube("blue", 0.009, 0.0, confidence=0.8)])
    assert [(c["color"], c["x"]) for c in merged] == [("blue", 0.0)]


def test_iou_of_rotated_footprints():
    assert cubes.iou(cube("red", 0, 0), cube("red", 0, 0, yaw=45)) > 0.6
    assert cubes.iou(cube("red", 0, 0), cube("red", S / 2, 0)) == pytest.approx(1 / 3, abs=0.01)
    assert cubes.iou(cube("red", 0, 0), cube("red", S, 0)) == 0.0
