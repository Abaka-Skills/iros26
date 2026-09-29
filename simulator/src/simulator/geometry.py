"""Rigid transforms (4x4) and rotations."""
import math

import numpy as np


def rodrigues(rvec) -> np.ndarray:
    """Rotation vector (axis * angle, rad) -> 3x3 rotation."""
    rvec = np.asarray(rvec, float)
    th = float(np.linalg.norm(rvec))
    if th < 1e-12:
        return np.eye(3)
    k = rvec / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def rot(axis, deg: float) -> np.ndarray:
    return rodrigues(np.asarray(axis, float) * math.radians(deg))


def rpy(r: float, p: float, y: float) -> np.ndarray:
    """URDF roll-pitch-yaw (fixed axes x, y, z) -> 3x3 rotation."""
    return rot([0, 0, 1], math.degrees(y)) @ rot([0, 1, 0], math.degrees(p)) @ rot([1, 0, 0], math.degrees(r))


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    return make_T(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])
