"""Where a follower's tool point can be, and pointing which way: the table jobs/reach_map.py measured by IK
on the Ray cluster (config/reach/wxai_follower.npz), in an arm's base frame by the tool point's radius and
height and the tool's approach direction relative to the radial one. A quick filter for planners (IK
still has the last word on a pose)."""
from functools import cache
from pathlib import Path

import numpy as np

from myaloha.coords import apply_T, inv_T

MAP = Path(__file__).resolve().parent.parent / "config" / "reach" / "wxai_follower.npz"
JOINT0 = np.radians(167)           # joint 0's limit less the planner's margin: bearings beyond are behind it


class ReachMap:
    def __init__(self, path: Path = MAP):
        d = np.load(path)
        self.r, self.z, self.margin, self.low = d["r"], d["z"], d["margin"], d["low"]
        self.q = d["q"] if "q" in d else None     # the solution found per cell (bearing 0, tool frame _frame)
        el, az = np.radians(d["dirs"]).T
        self.units = np.c_[np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]   # radial frame

    def _cell(self, Pb):
        r, bear = np.hypot(Pb[:, 0], Pb[:, 1]), np.arctan2(Pb[:, 1], Pb[:, 0])
        i = np.clip(np.round(r / (self.r[1] - self.r[0])).astype(int), 0, len(self.r) - 1)
        j = np.clip(np.round((Pb[:, 2] - self.z[0]) / (self.z[1] - self.z[0])).astype(int), 0, len(self.z) - 1)
        return r, bear, i, j

    def seed(self, T_W_B: np.ndarray, p, R) -> np.ndarray | None:
        """Joints near a solution for the tool at (p, R) (W), from the nearest cell's stored solution:
        joint 0 turned to the bearing, joint 5 to the roll; a start from which IK converges in a few
        steps. None where the map has no solution nearby."""
        if self.q is None:
            return None
        Pb = apply_T(inv_T(T_W_B), np.atleast_2d(p))
        Rb = T_W_B[:3, :3].T @ R
        _, bear, i, j = self._cell(Pb)
        c, s = np.cos(-bear[0]), np.sin(-bear[0])
        d = np.array([c * Rb[0, 0] - s * Rb[1, 0], s * Rb[0, 0] + c * Rb[1, 0], Rb[2, 0]])   # approach, radial frame
        ok = np.isfinite(self.margin[i[0], j[0]])
        if not ok.any():
            return None
        k = int(np.argmax(np.where(ok, self.units @ d, -np.inf)))
        q = self.q[i[0], j[0], k].astype(float).copy()
        Rz = np.array([[np.cos(bear[0]), -np.sin(bear[0]), 0], [np.sin(bear[0]), np.cos(bear[0]), 0], [0, 0, 1.0]])
        rel = (Rz @ _frame(self.units[k])).T @ Rb                       # the rest: about the approach axis
        q[0] = (q[0] + bear[0] + np.pi) % (2 * np.pi) - np.pi
        q[5] = (q[5] + np.arctan2(rel[2, 1], rel[1, 1]) + np.pi) % (2 * np.pi) - np.pi
        return q

    def reaches(self, T_W_B: np.ndarray, P, d, clear: float = 0.03, tol: float = np.radians(20),
                margin: float = 0.0) -> np.ndarray:
        """(n,) bool: the arm on base T_W_B can have its tool point at P (n, 3; W) with its approach axis
        within `tol` of d (n, 3; W unit vectors), every joint past the shoulder `clear` above the table
        and at least `margin` (rad) inside its limits. Cells are 2 cm and 30 deg apart: at a cell's edge
        the pose between can be a few degrees tighter, so ask for more margin than the planner needs."""
        P, d = np.atleast_2d(P).astype(float), np.atleast_2d(d).astype(float)
        Pb, db = apply_T(inv_T(T_W_B), P), d @ T_W_B[:3, :3]
        r, bear, i, j = self._cell(Pb)
        c, s = np.cos(-bear), np.sin(-bear)
        dr = np.c_[c * db[:, 0] - s * db[:, 1], s * db[:, 0] + c * db[:, 1], db[:, 2]]
        step_r, step_z = self.r[1] - self.r[0], self.z[1] - self.z[0]
        inside = (r <= self.r[-1] + step_r / 2) & (np.abs(Pb[:, 2] - self.z.clip(self.z[0], self.z[-1])[j]) <= step_z)
        inside &= np.abs(bear) < JOINT0
        cell = (self.margin[i, j] >= margin) & (self.low[i, j] + T_W_B[2, 3] > clear)   # NaN: no solution
        return inside & (cell & (dr @ self.units.T > np.cos(tol))).any(1)


def table_regions(T_W_B: np.ndarray, extent=(-0.6, -0.45, 0.6, 0.45), step: float = 0.01,
                  grasp_z: float = 0.008, hover: float = 0.04) -> list[dict]:
    """Where on the table this arm can pick or place straight down, as pick_place does (its tool point
    at grasp_z, and `hover` above it): [{outer: [[x, y], ...], holes: [[[x, y], ...], ...]}] in W,
    traced on a `step` grid over `extent` (x0, y0, x1, y1)."""
    import cv2
    m = reach_map()
    xs, ys = np.arange(extent[0], extent[2] + 1e-9, step), np.arange(extent[1], extent[3] + 1e-9, step)
    X, Y = np.meshgrid(xs, ys)
    P = np.c_[X.ravel(), Y.ravel(), np.full(X.size, grasp_z)]
    down = np.broadcast_to([0.0, 0.0, -1.0], P.shape)
    ok = m.reaches(T_W_B, P, down) & m.reaches(T_W_B, P + [0, 0, hover], down)
    mask = ok.reshape(X.shape).astype(np.uint8)
    contours, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    def to_w(c):
        return [[round(float(xs[0] + u * step), 4), round(float(ys[0] + v * step), 4)] for u, v in c[:, 0]]
    out = []
    for i, c in enumerate(contours):
        if hier[0][i][3] < 0 and len(c) >= 3:                 # an outer boundary, with its holes
            holes = [to_w(contours[k]) for k in range(len(contours)) if hier[0][k][3] == i and len(contours[k]) >= 3]
            out.append({"outer": to_w(c), "holes": holes})
    return out


def _frame(x: np.ndarray) -> np.ndarray:
    """The tool rotation jobs/reach_map.py solved each cell with: approach x, y level where it can be."""
    up = np.array([0, 0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0, 0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    return np.c_[x, y, np.cross(x, y)]


@cache
def reach_map() -> ReachMap:
    return ReachMap()
