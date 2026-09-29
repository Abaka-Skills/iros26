"""Where a follower's own gripper shows in its wrist camera. The camera and the fingers both ride on
link_6: the fingers' pose in the camera is fixed by the URDF (the carriage joints) and the hand-eye
calibration, and moves only with the aperture. So the pixels they cover are known without looking:
a mask per aperture, projected from the carriage and finger meshes (the robot's own, as the panel
draws them) and cached."""
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np

from myaloha.coords import apply_T, distort_norm, inv_T

ROOT = Path(__file__).resolve().parent.parent
MESHES = ROOT / "panel" / "assets" / "meshes" / "wxai"
EE_IN_LINK6 = 0.156062            # the tool point (ee_gripper_link) on link_6's x (URDF joint ee_gripper)
CARRIAGE = {"left": (0.023, 1.0), "right": (-0.023, -1.0)}   # carriage joint origin y on link_6, its axis y
CARRIAGE_X = 0.0865
MARGIN_PX = 4                     # grown this much: the hand-eye pose and the meshes are not exact


@lru_cache(maxsize=None)
def _mesh(name: str) -> np.ndarray:
    """A binary STL's vertices (n, 3), deduplicated."""
    d = (MESHES / name).read_bytes()
    n = int.from_bytes(d[80:84], "little")
    tris = np.frombuffer(d[84:84 + n * 50], np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"]
    return np.unique(tris.reshape(-1, 3).round(5), axis=0).astype(float)


def finger_points(aperture: float) -> list[np.ndarray]:
    """The carriage and finger meshes' vertices, each part on its own (each nearly convex), in the
    tool (ee) frame at `aperture` (m)."""
    out = []
    for finger, (y0, axis) in CARRIAGE.items():
        y = y0 + axis * aperture / 2
        out += [_mesh(f"{part}_{finger}.stl") + [CARRIAGE_X - EE_IN_LINK6, y, 0.0] for part in ("carriage", "gripper")]
    return out


def _mask(T_ee_cam: tuple, K: tuple, dist: tuple, model: str, size: tuple, aperture: float) -> np.ndarray:
    T_cam_ee, K = inv_T(np.array(T_ee_cam)), np.array(K)
    w, h = size
    mask = np.zeros((h, w), np.uint8)
    for V in finger_points(aperture):
        P = apply_T(T_cam_ee, V)
        P = P[P[:, 2] > 0.01]
        if len(P) < 3:
            continue
        xd, yd = distort_norm(P[:, 0] / P[:, 2], P[:, 1] / P[:, 2], np.array(dist), model)
        uv = np.stack([xd * K[0, 0] + K[0, 2], yd * K[1, 1] + K[1, 2]], -1)
        uv = np.clip(uv, -4 * w, 5 * w).astype(np.float32)
        cv2.fillConvexPoly(mask, cv2.convexHull(uv).astype(np.int32), 1)
    if MARGIN_PX:
        mask = cv2.dilate(mask, np.ones((2 * MARGIN_PX + 1,) * 2, np.uint8))
    return mask.astype(bool)


_cached = lru_cache(maxsize=64)(_mask)


def finger_mask(T_ee_cam: np.ndarray, K: np.ndarray, dist: np.ndarray, model: str, size: tuple,
                aperture: float) -> np.ndarray:
    """(h, w) bool: the wrist camera's pixels the gripper's own carriages and fingers cover at
    `aperture` (m; rounded to 2 mm for the cache)."""
    key = lambda a: tuple(map(tuple, np.round(np.asarray(a, float), 6)))   # noqa: E731
    return _cached(key(T_ee_cam), key(K), tuple(np.round(np.asarray(dist, float), 6)), str(model),
                   tuple(size), round(float(aperture) / 0.002) * 0.002)
