"""localize's solver on synthetic scenes: a textured table ray-cast from each wrist
camera. Different textures and base layouts stand in for new sites; the solver only gets the
seeds and each camera's pose in its own base frame, and must recover the true layout."""
import cv2
import numpy as np
import pytest

from myaloha import coords
from myaloha.coords import CoordinatesError, inv_T, make_T

K = np.array([[380.0, 0, 320], [0, 380.0, 240], [0, 0, 1]])
H, W_ = 480, 640


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def texture(kind: str, size_px=2400, seed=0) -> np.ndarray:
    """A 1.2 m x 1.2 m table top, 2 mm per pixel."""
    rng = np.random.default_rng(seed)
    img = np.full((size_px, size_px), 170, np.uint8)
    if kind == "desk":        # grain + scattered clutter
        img = cv2.GaussianBlur(rng.integers(90, 200, (size_px, size_px)).astype(np.uint8), (0, 0), 3)
        for _ in range(900):
            c = tuple(int(v) for v in rng.integers(0, size_px, 2))
            col = int(rng.integers(0, 255))
            if rng.random() < 0.5:
                cv2.circle(img, c, int(rng.integers(4, 30)), col, -1)
            else:
                cv2.rectangle(img, c, (c[0] + int(rng.integers(6, 50)), c[1] + int(rng.integers(6, 50))), col, -1)
    elif kind == "tags":      # a plain desk with a sheet of random binary tags
        for i in range(12):
            for j in range(12):
                x0, y0 = 480 + i * 120, 480 + j * 120
                cells = rng.integers(0, 2, (6, 6)) * 255
                tag = np.zeros((8, 8), np.uint8)
                tag[1:7, 1:7] = cells
                img[y0:y0 + 80, x0:x0 + 80] = cv2.resize(tag, (80, 80), interpolation=cv2.INTER_NEAREST)
    return img


def render(tex, T_W_cam, noise=0.0005, seed=0):
    """Colour (BGR) and depth (m) of the table z = 0 in W; texture centred on W's origin."""
    v, u = np.mgrid[0:H, 0:W_].astype(float)
    d = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)
    dW = d @ T_W_cam[:3, :3].T
    c = T_W_cam[:3, 3]
    s = -c[2] / dW[..., 2]
    P = c + s[..., None] * dW
    px = (P[..., 0] / 0.002 + tex.shape[1] / 2).astype(np.float32)
    py = (-P[..., 1] / 0.002 + tex.shape[0] / 2).astype(np.float32)
    gray = cv2.remap(tex, px, py, cv2.INTER_LINEAR, borderValue=0)
    depth = (s + np.random.default_rng(seed).normal(0, noise, s.shape)).astype(np.float32)
    depth[(s <= 0) | (gray == 0)] = 0
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), depth


def site(distance, left_yaw_deg, lateral, h_left, h_right):
    """True base poses in W (W on the table, x from left base to right base)."""
    L = make_T(rot_z(np.radians(left_yaw_deg)), [-distance / 2, lateral / 2, h_left])
    R = make_T(rot_z(np.pi), [distance / 2, -lateral / 2, h_right])
    return {"left": L, "right": R}


def views_for(truth, tex, seed_distance):
    out = []
    for arm in ("left", "right"):
        table_z = -truth[arm][2, 3]
        for k, cands in enumerate(coords.camera_targets(arm, seed_distance, table_z)):
            T_B_cam = cands[0]
            color, depth = render(tex, truth[arm] @ T_B_cam, seed=k)
            out.append(coords.View(arm, color, depth, K, T_B_cam))
    return out


@pytest.mark.parametrize("kind,layout,seed_distance", [
    ("desk", (0.93, -0.5, 0.009, 0.022, 0.028), 0.91),   # like the lab bench, tape seed 2 cm off
    ("tags", (0.86, 2.0, -0.03, 0.020, 0.020), 0.91),    # a different site: closer, twisted, offset
])
def test_recovers_the_base_layout(kind, layout, seed_distance):
    truth = site(*layout)
    res = coords.solve(views_for(truth, texture(kind), seed_distance), seed_distance, table_z_seed=-0.025,
                       parallel=False)
    got = {a: np.array(T) for a, T in res["T_W_base"].items()}
    T_true, T_got = inv_T(truth["right"]) @ truth["left"], inv_T(got["right"]) @ got["left"]
    err = inv_T(T_true) @ T_got
    assert np.linalg.norm(err[:3, 3]) < 0.002, res["report"]                      # < 2 mm
    assert coords.angle_deg(err[:3, :3]) < 0.2, res["report"]                     # < 0.2 deg
    for a in ("left", "right"):     # W: on the table, x through both bases (so both have y = 0)
        assert abs(got[a][2, 3] - truth[a][2, 3]) < 0.002 and abs(got[a][1, 3]) < 1e-6
    g = res["report"]["geometry"]
    assert abs(g["base_distance_mm"] - np.linalg.norm(truth["left"][:3, 3] - truth["right"][:3, 3]) * 1000) < 2
    assert res["report"]["inliers"] >= 30 and res["report"]["split_half"]["translation_mm"] < 2


def test_blank_table_is_reported_not_guessed():
    truth = site(0.93, 0, 0, 0.025, 0.025)
    blank = np.full((2400, 2400), 170, np.uint8)
    with pytest.raises(CoordinatesError, match="SIFT found only"):
        coords.solve(views_for(truth, blank, 0.93), 0.93)


def test_registers_a_fixed_overhead_camera():
    truth = site(0.93, -0.5, 0.009, 0.022, 0.028)
    tex = texture("desk")
    views = views_for(truth, tex, 0.91)
    res = coords.solve(views, 0.91)
    got = {a: np.array(T) for a, T in res["T_W_base"].items()}
    # overhead camera: 0.9 m up, a little in front, looking down (true W), rendered from that pose
    T_Wt_cam = coords.look_at([0.0, 0.0, 0.0], [0.0, -1.0], 0.92, np.radians(80))
    color, depth = render(tex, T_Wt_cam, noise=0.006)
    # the solver's W vs the true W: both hold the right base, so compare through it
    T_Wg_Wt = got["right"] @ inv_T(truth["right"])
    cam = coords.register_fixed(views, res["T_W_base"], color, K, 0.91)
    err = inv_T(T_Wg_Wt @ T_Wt_cam) @ np.array(cam["T_W_cam"])
    assert np.linalg.norm(err[:3, 3]) < 0.01 and coords.angle_deg(err[:3, :3]) < 0.5, cam["report"]
    assert cam["report"]["reprojection_px"]["rms"] < 1.5
    # with the camera's own depth (noisy, 2 % scale bias) the table fixes the tilt and height
    cam = coords.register_fixed(views, res["T_W_base"], color, K, 0.91, depth=depth * 1.02)
    err = inv_T(T_Wg_Wt @ T_Wt_cam) @ np.array(cam["T_W_cam"])
    assert cam["report"]["method"] == "depth plane + 2-D fit"
    assert abs(cam["report"]["depth_scale"] - 1.02) < 0.005
    assert np.linalg.norm(err[:3, 3]) < 0.005 and coords.angle_deg(err[:3, :3]) < 0.15, cam["report"]


def test_parallel_rails_prior_solves_only_the_offsets():
    """Bases level on parallel rails (no twist), one slid 12 mm along its rail: the constrained
    solve returns exactly facing, level mounts and the true offsets."""
    truth = site(0.93, 0.0, 0.012, 0.022, 0.028)
    res = coords.solve(views_for(truth, texture("desk"), 0.91), 0.91, parallel=True)
    g = res["report"]["geometry"]
    assert abs(g["base_distance_mm"] - np.hypot(930, 12)) < 2 and abs(g["left_yaw_vs_facing_deg"]) < 1e-6
    assert abs(g["left_lateral_in_right_mm"] + 12) < 2      # right base is turned 180 deg: -12 in its frame
    M = {a: np.array(T) for a, T in res["T_W_mount"].items()}
    assert all(abs(M[a][2, 2] - 1) < 1e-9 for a in M)                       # level
    assert abs(M["left"][2, 3] - 0.022) < 0.002 and abs(M["right"][2, 3] - 0.028) < 0.002
    assert res["report"]["parallel_fit"]["rms_mm"] < 2


@pytest.mark.parametrize("model", ["brown_conrady", "inverse_brown_conrady"])
def test_lens_models_round_trip(model):
    c = np.array([-0.055, 0.065, -0.0006, 0.0008, -0.021])   # D405-like magnitudes
    x, y = np.meshgrid(np.linspace(-0.7, 0.7, 15), np.linspace(-0.5, 0.5, 11))
    xd, yd = coords.distort_norm(x, y, c, model)
    assert np.abs(xd - x).max() > 1e-3                       # it does something
    xu, yu = coords.undistort_norm(xd, yd, c, model)
    assert np.abs(xu - x).max() < 1e-9 and np.abs(yu - y).max() < 1e-9


def test_registers_a_grazing_camera_through_a_rectified_view():
    """A camera 5 cm above the table (like the low one): too oblique for PnP on SIFT matches, so
    its view of the table is resampled top-down from its own depth, then fitted in the plane."""
    truth = site(0.93, -0.5, 0.009, 0.022, 0.028)
    tex = texture("desk")
    views = views_for(truth, tex, 0.91)
    res = coords.solve(views, 0.91)
    got = {a: np.array(T) for a, T in res["T_W_base"].items()}
    T_Wt_cam = coords.look_at([0.0, 0.05, 0.0], [0.0, -1.0], 0.40, np.radians(7))
    color, depth = render(tex, T_Wt_cam, noise=0.001)
    T_Wg_Wt = got["right"] @ inv_T(truth["right"])
    cam = coords.register_rectified(views, res["T_W_base"], color, depth * 1.02, K, 0.91)
    err = inv_T(T_Wg_Wt @ T_Wt_cam) @ np.array(cam["T_W_cam"])
    assert cam["report"]["method"] == "rectified table view + 2-D fit"
    assert abs(cam["report"]["depth_scale"] - 1.02) < 0.01, cam["report"]
    assert np.linalg.norm(err[:3, 3]) < 0.006 and coords.angle_deg(err[:3, :3]) < 0.5, cam["report"]
