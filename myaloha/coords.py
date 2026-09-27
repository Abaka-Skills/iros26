"""Establish coordinates: where the two follower bases are relative to each other, and the world
frame W between them, from SIFT features that both wrist cameras see on the table.

    1. views     each follower's wrist camera looks at the table between the arms from several
                 poses; camera pose in that arm's base frame = FK(q) @ T_ee_cam
    2. features  SIFT keypoints on each colour image, lifted to 3-D with the aligned depth, in
                 that arm's base frame; only points in a box around the expected midpoint
    3. table     RANSAC plane through each arm's table points (base height + tilt)
    4. match     SIFT descriptors matched across the arms (ratio test, mutual best), giving
                 3-D/3-D pairs (left-base point, right-base point)
    5. fit       RANSAC + Kabsch: the left base's pose in the right base frame
    6. W         z = table normal, origin = midpoint of the two base origins projected onto the
                 table, x from the left base toward the right base, y = z x x (away from the
                 operator)

Evaluation: fit residuals, per-arm view-to-view consistency (hand-eye + FK), table agreement
across arms, split-half stability, and the change from a reference registration when given.
Adapted from the reference-cube registration in _tmp/aloha_grid_demo (grid_demo/register.py).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

ARMS = ("left", "right")


# ---- rigid-transform helpers ------------------------------------------------------------------

def inv_T(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def apply_T(T: np.ndarray, p) -> np.ndarray:
    p = np.asarray(p, float)
    return p @ T[:3, :3].T + T[:3, 3]


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def angle_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def kabsch(A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(R, t) minimising |R a + t - b| over the rows of A and B (no reflection)."""
    ma, mb = A.mean(axis=0), B.mean(axis=0)
    U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mb - R @ ma


def ransac_rigid(A: np.ndarray, B: np.ndarray, thr: float, iters: int = 3000, seed: int = 0):
    """Robust (R, t) with B ~ R A + t; returns (R, t, inlier mask)."""
    np.seterr(over="ignore", invalid="ignore")  # macOS Accelerate matmul raises spurious FP warnings
    rng = np.random.default_rng(seed)
    n, best = len(A), None
    if n < 3:
        raise ValueError("fewer than 3 correspondences")
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        a = A[idx]
        if np.linalg.norm(np.cross(a[1] - a[0], a[2] - a[0])) < 1e-4:  # (nearly) collinear
            continue
        R, t = kabsch(a, B[idx])
        inl = np.linalg.norm(A @ R.T + t - B, axis=1) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    for _ in range(3):  # refine on the inliers
        R, t = kabsch(A[best], B[best])
        best = np.linalg.norm(A @ R.T + t - B, axis=1) < thr
    R, t = kabsch(A[best], B[best])
    return R, t, best


def fit_plane(P: np.ndarray, tol: float, iters: int = 400, seed: int = 0):
    """RANSAC + least squares plane n.p + d = 0 with n pointing +z; returns (n, d, inliers)."""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    Q = P[best]
    mu = Q.mean(axis=0)
    n = np.linalg.svd(Q - mu, full_matrices=False)[2][2]
    if n[2] < 0:
        n = -n
    d = -float(n @ mu)
    return n, d, np.abs(P @ n + d) < tol


def look_at(target, from_dir_xy, distance: float, elevation: float, roll: float = 0.0) -> np.ndarray:
    """Camera pose (optical frame: z forward, y down in the image) `distance` from `target`,
    raised `elevation` rad above the horizontal on the side `from_dir_xy` points to."""
    u = np.asarray(from_dir_xy, float)[:2]
    u /= np.linalg.norm(u)
    d = np.array([np.cos(elevation) * u[0], np.cos(elevation) * u[1], np.sin(elevation)])
    z = -d
    y = np.array([0.0, 0.0, -1.0]) - z * (-z[2])
    y /= np.linalg.norm(y)
    R = np.column_stack([np.cross(y, z), y, z])
    c, s = np.cos(roll), np.sin(roll)
    R = R @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return make_T(R, np.asarray(target, float) + distance * d)


# ---- where to look from ------------------------------------------------------------------------

# Viewpoints per arm, from the reference demo (proven reachable on this rig): spot = midpoint +
# offset [toward the other arm, away from the operator], camera `distance` from the spot, steep
# (D405 depth is poor at shallow angles), from the arm's own side, azimuth turned about the spot.
VIEWS = (
    {"offset": (0.0, 0.0), "elevation_deg": 82, "azimuth_deg": 0},
    {"offset": (0.0, -0.08), "elevation_deg": 70, "azimuth_deg": -20},
    {"offset": (0.0, 0.08), "elevation_deg": 70, "azimuth_deg": 20},
    {"offset": (0.07, 0.0), "elevation_deg": 72, "azimuth_deg": 0},
    {"offset": (-0.07, 0.0), "elevation_deg": 78, "azimuth_deg": 0},
)


def camera_targets(arm: str, base_distance: float, table_z: float, distance: float = 0.25,
                   aim_beyond: float = 0.04, rolls_deg=(0, 20, -20, 40, -40)) -> list[list[np.ndarray]]:
    """Per view, candidate camera poses (base frame, several rolls) looking at the table near the
    expected midpoint. Base x points at the other arm; "away from the operator" is +y for the
    left arm and -y for the right arm, so both arms look at the same spots."""
    side = 1.0 if arm == "left" else -1.0
    mid = np.array([base_distance / 2, 0.0, table_z])
    out = []
    for v in VIEWS:
        spot = mid + np.array([v["offset"][0], side * v["offset"][1], 0.0])
        az = np.radians(side * v["azimuth_deg"])
        back = -spot[:2] / np.linalg.norm(spot[:2])            # from the spot toward the own base
        from_dir = np.array([[np.cos(az), -np.sin(az)], [np.sin(az), np.cos(az)]]) @ back
        aim = spot - np.r_[from_dir * aim_beyond, 0.0]         # spot lands above the fingers
        out.append([look_at(aim, from_dir, distance, np.radians(v["elevation_deg"]), np.radians(r))
                    for r in rolls_deg])
    return out


# ---- views and features ------------------------------------------------------------------------

@dataclass
class View:
    arm: str                 # "left" | "right"
    color: np.ndarray        # HxWx3 BGR
    depth: np.ndarray        # HxW metres, aligned to colour; 0 = no reading
    K: np.ndarray            # 3x3 colour intrinsics
    T_B_cam: np.ndarray      # camera optical frame in the arm's base frame
    q: list = field(default_factory=list)

    def save(self, path: Path) -> None:
        np.savez_compressed(path, arm=self.arm, color=self.color, depth=self.depth.astype(np.float32),
                            K=self.K, T_B_cam=self.T_B_cam, q=np.asarray(self.q, float))

    @classmethod
    def load_dir(cls, folder: Path) -> list["View"]:
        """A survey's wrist views (left_NN.npz, right_NN.npz); other files there are ignored."""
        return [cls.load(p) for p in sorted(Path(folder).glob("*.npz")) if p.name.startswith(("left_", "right_"))]

    @classmethod
    def load(cls, path: Path) -> "View":
        d = np.load(path)
        return cls(str(d["arm"]), d["color"], d["depth"], d["K"], d["T_B_cam"], d["q"].tolist())


@dataclass
class Box:
    """Region around the expected midpoint, in an arm's base frame (x toward the other arm)."""
    center: np.ndarray       # (x, y, table z)
    half_x: float = 0.30
    half_y: float = 0.35
    below: float = 0.04
    above: float = 0.15

    def contains(self, P: np.ndarray) -> np.ndarray:
        c = self.center
        return ((np.abs(P[:, 0] - c[0]) < self.half_x) & (np.abs(P[:, 1] - c[1]) < self.half_y)
                & (P[:, 2] > c[2] - self.below) & (P[:, 2] < c[2] + self.above))


def lift(view: View, uv: np.ndarray, win: int = 2) -> tuple[np.ndarray, np.ndarray]:
    """3-D points in the base frame for pixel coordinates uv (N,2); invalid depth -> mask False."""
    h, w = view.depth.shape
    K = view.K
    P, ok = np.zeros((len(uv), 3)), np.zeros(len(uv), bool)
    for i, (u, v) in enumerate(np.round(uv).astype(int)):
        patch = view.depth[max(v - win, 0):v + win + 1, max(u - win, 0):u + win + 1]
        patch = patch[patch > 0]
        if patch.size < 3:
            continue
        z = float(np.median(patch))
        P[i] = [(uv[i, 0] - K[0, 2]) * z / K[0, 0], (uv[i, 1] - K[1, 2]) * z / K[1, 1], z]
        ok[i] = True
    return apply_T(view.T_B_cam, P), ok


@dataclass
class Features:
    uv: np.ndarray           # (N,2) pixels
    P: np.ndarray            # (N,3) base frame
    desc: np.ndarray         # (N,128) SIFT


def features(view: View, box: Box, n_features: int = 4000, near: float = 0.12, far: float = 0.8) -> Features:
    """SIFT keypoints with a depth reading inside `box`. Pixels nearer than `near` (the gripper
    fingers) or farther than `far` are masked out before detection."""
    gray = cv2.cvtColor(view.color, cv2.COLOR_BGR2GRAY)
    mask = ((view.depth > near) & (view.depth < far)).astype(np.uint8) * 255
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
    kps, desc = cv2.SIFT_create(nfeatures=n_features, contrastThreshold=0.03).detectAndCompute(gray, mask)
    if desc is None:
        return Features(np.zeros((0, 2)), np.zeros((0, 3)), np.zeros((0, 128), np.float32))
    uv = np.array([k.pt for k in kps])
    P, ok = lift(view, uv)
    keep = ok & box.contains(P)
    return Features(uv[keep], P[keep], desc[keep])


def table_plane(view: View, box: Box, tol: float = 0.004, step: int = 4):
    """(n, d, n_points) of the table in the base frame, from this view's depth."""
    h, w = view.depth.shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    z = view.depth[::step, ::step]
    ok = (z > 0.12) & (z < 0.8)
    K = view.K
    Pc = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], axis=-1)[ok]
    P = apply_T(view.T_B_cam, Pc)
    band = box.contains(P) & (np.abs(P[:, 2] - box.center[2]) < box.below)
    P = P[band]
    if len(P) < 300:
        return None
    n, d, inl = fit_plane(P, tol)
    return n, d, int(inl.sum())


def match(fa: Features, fb: Features, ratio: float = 0.75) -> np.ndarray:
    """Mutual-best matches passing Lowe's ratio test, as (M,2) index pairs into fa, fb."""
    if len(fa.desc) < 2 or len(fb.desc) < 2:
        return np.zeros((0, 2), int)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    ab = bf.knnMatch(fa.desc, fb.desc, k=2)
    ba = {m[0].queryIdx: m[0].trainIdx for m in bf.knnMatch(fb.desc, fa.desc, k=1) if m}
    out = [(m.queryIdx, m.trainIdx) for m, n in (p for p in ab if len(p) == 2)
           if m.distance < ratio * n.distance and ba.get(m.trainIdx) == m.queryIdx]
    return np.array(out, int).reshape(-1, 2)


# ---- solving ------------------------------------------------------------------------------------

def plane_in(T_A_B: np.ndarray, n: np.ndarray, d: float) -> tuple[np.ndarray, float]:
    """A plane given in frame B (n.p + d = 0), expressed in frame A."""
    n2 = T_A_B[:3, :3] @ n
    return n2, float(d - n2 @ T_A_B[:3, 3])


def table_frame(n: np.ndarray, d: float) -> np.ndarray:
    """A base's level frame F in its base frame B: origin = foot of the base origin on the table,
    z = table normal, x = base x projected onto the table (from the reference demo)."""
    x = np.array([1.0, 0.0, 0.0]) - n * n[0]
    x /= np.linalg.norm(x)
    return make_T(np.column_stack([x, np.cross(n, x), n]), -d * n)


def base_line_frame(T_R_L: np.ndarray, n: np.ndarray, d: float) -> np.ndarray:
    """W in the right base frame: z = n, origin = midpoint of the base origins projected on the
    table, x from the left base toward the right base, y = z x x."""
    proj = lambda p: p - (n @ p + d) * n  # noqa: E731
    pL, pR = proj(T_R_L[:3, 3]), proj(np.zeros(3))
    x = pR - pL
    x -= (x @ n) * n
    x /= np.linalg.norm(x)
    return make_T(np.column_stack([x, np.cross(n, x), n]), (pL + pR) / 2)


def summarize(T_W_base: dict) -> dict:
    """Geometry numbers that are easy to check against a tape measure or another registration."""
    L, R = np.asarray(T_W_base["left"]), np.asarray(T_W_base["right"])
    T_R_L = inv_T(R) @ L
    yaw_left = np.degrees(np.arctan2(L[1, 0], L[0, 0]))
    return {
        "base_distance_mm": round(float(np.linalg.norm(L[:3, 3] - R[:3, 3])) * 1000, 1),
        "left_yaw_vs_facing_deg": round(float(yaw_left), 2),
        "left_lateral_in_right_mm": round(float(T_R_L[1, 3]) * 1000, 1),
        "base_height_mm": {"left": round(float(L[2, 3]) * 1000, 1), "right": round(float(R[2, 3]) * 1000, 1)},
        "base_tilt_deg": {a: round(float(np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))), 2)
                          for a, T in (("left", L), ("right", R))},
    }


FEW_INLIERS = 20   # below `min_inliers`, down to this, a fit is accepted only if its halves agree


class CoordinatesError(RuntimeError):
    pass


def solve(views: list[View], base_distance_seed: float, table_z_seed: float = -0.025,
          min_inliers: int = 30, inlier_mm: float = 8.0, reference: dict | None = None,
          parallel: bool = False) -> dict:
    """Registration from views of both arms. Raises CoordinatesError with the reason if SIFT
    does not find enough shared, geometrically consistent features.

    parallel: the bases are known to sit level on parallel rails, facing each other. Then only
    their positions are solved (distance, offset along the rails); the free fit's yaw is still
    reported, so the prior can be checked. Each base's apparent tilt (gravity sag seen through
    FK) stays in T_W_base, so FK points land on the table; T_W_mount is the level, physical pose."""
    box = {a: Box(np.array([base_distance_seed / 2, 0.0, table_z_seed])) for a in ARMS}
    feats = {a: [(v, features(v, box[a])) for v in views if v.arm == a] for a in ARMS}
    for a in ARMS:
        if not feats[a]:
            raise CoordinatesError(f"no views from the {a} arm")

    # table plane per arm (base frame), from every view's depth
    planes, report = {}, {"features_per_view": {a: [len(f.P) for _, f in feats[a]] for a in ARMS}}
    for a in ARMS:
        fits = [p for p in (table_plane(v, box[a]) for v, _ in feats[a]) if p]
        if not fits:
            raise CoordinatesError(f"{a}: no table surface in the depth near the midpoint")
        n = np.mean([f[0] for f in fits], axis=0)
        n /= np.linalg.norm(n)
        planes[a] = (n, float(np.mean([f[1] for f in fits])))

    # per-arm consistency: the same feature seen from two views of one arm, in its base frame
    consistency = {}
    for a in ARMS:
        dist = []
        for i in range(len(feats[a])):
            for j in range(i + 1, len(feats[a])):
                fi, fj = feats[a][i][1], feats[a][j][1]
                m = match(fi, fj)
                if len(m):
                    dist.extend(np.linalg.norm(fi.P[m[:, 0]] - fj.P[m[:, 1]], axis=1))
        dist = np.array(dist)
        close = dist[dist < 0.03]
        consistency[a] = {"matches": int(len(dist)), "median_mm": round(float(np.median(close)) * 1000, 1) if len(close) else None,
                          "p90_mm": round(float(np.percentile(close, 90)) * 1000, 1) if len(close) else None}
    report["view_consistency"] = consistency

    # cross-arm correspondences, tagged by view pair for the split-half check
    A, B, UA, UB, pair = [], [], [], [], []
    for i, (_, fl) in enumerate(feats["left"]):
        for j, (_, fr) in enumerate(feats["right"]):
            m = match(fl, fr)
            A.append(fl.P[m[:, 0]])
            B.append(fr.P[m[:, 1]])
            UA.append(fl.uv[m[:, 0]])
            UB.append(fr.uv[m[:, 1]])
            pair.extend([i * len(feats["right"]) + j] * len(m))
    A, B, pair = np.concatenate(A), np.concatenate(B), np.array(pair, int)
    UA, UB = np.concatenate(UA), np.concatenate(UB)
    report["cross_matches"] = int(len(A))
    if len(A) < min_inliers:
        raise CoordinatesError(f"SIFT found only {len(A)} feature matches between the two wrist cameras "
                               f"(need {min_inliers}); the shared table area may lack texture")
    R, t, inl = ransac_rigid(A, B, inlier_mm / 1000)
    resid = np.linalg.norm(A[inl] @ R.T + t - B[inl], axis=1)
    report["inliers"] = int(inl.sum())
    report["inlier_ratio"] = round(float(inl.mean()), 3)
    report["fit_residual_mm"] = {"rms": round(float(np.sqrt(np.mean(resid ** 2))) * 1000, 2),
                                 "p90": round(float(np.percentile(resid, 90)) * 1000, 2)}
    if inl.sum() < min(min_inliers, FEW_INLIERS):
        raise CoordinatesError(f"only {int(inl.sum())} of {len(A)} SIFT matches agree on one rigid "
                               f"transform (need {min(min_inliers, FEW_INLIERS)}); the matches are mostly wrong")
    T_R_L = make_T(R, t)

    # split-half stability: solve from even and odd view pairs separately
    halves = []
    for k in (0, 1):
        sel = inl & (pair % 2 == k)
        if sel.sum() >= 6:
            halves.append(make_T(*kabsch(A[sel], B[sel])))
    if len(halves) == 2:
        D = inv_T(halves[0]) @ halves[1]
        report["split_half"] = {"translation_mm": round(float(np.linalg.norm(D[:3, 3])) * 1000, 2),
                                "rotation_deg": round(angle_deg(D[:3, :3]), 3)}
    if inl.sum() < min_inliers:   # few matches: accept only a fit whose halves agree
        sh, rms = report.get("split_half"), report["fit_residual_mm"]["rms"]
        if not sh or sh["translation_mm"] > 5 or sh["rotation_deg"] > 1 or rms > 5:
            raise CoordinatesError(f"only {int(inl.sum())} of {len(A)} SIFT matches agree on one rigid transform "
                                   f"and they are not self-consistent (split-half {sh}, rms {rms} mm); "
                                   "add texture between the arms (a printed sheet, a magazine)")
        report["few_inliers"] = f"{int(inl.sum())} < {min_inliers}, accepted: split-half and rms within 5 mm / 1 deg"

    # table: both arms' planes in the right base frame should coincide
    nL, dL = plane_in(T_R_L, *planes["left"])
    nR, dR = planes["right"]
    n = nL + nR
    n /= np.linalg.norm(n)
    W_R = base_line_frame(T_R_L, n, (dL + dR) / 2)
    o = W_R[:3, 3]
    report["table_agreement"] = {"normal_angle_deg": round(float(np.degrees(np.arccos(np.clip(nL @ nR, -1, 1)))), 2),
                                 "height_gap_at_origin_mm": round(float((nL @ o + dL) - (nR @ o + dR)) * 1000, 2)}
    free_yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    report["free_fit_left_yaw_vs_facing_deg"] = round(float((free_yaw % 360) - 180), 2)

    T_B_F = {a: table_frame(*planes[a]) for a in ARMS}
    if parallel:
        # level frames, facing exactly: right-F point = Rz(pi) left-F point + t, t from the inliers
        AF = apply_T(inv_T(T_B_F["left"]), A[inl])
        BF = apply_T(inv_T(T_B_F["right"]), B[inl])
        Rf = np.diag([-1.0, -1.0, 1.0])
        diff = BF - AF @ Rf.T
        t_f = np.median(diff, axis=0)
        res = np.linalg.norm(diff[:, :2] - t_f[:2], axis=1)
        keep = res < inlier_mm / 1000
        t_f = diff[keep].mean(axis=0)
        res = np.linalg.norm(diff[keep] - t_f, axis=1)
        report["parallel_fit"] = {"rms_mm": round(float(np.sqrt(np.mean(res ** 2))) * 1000, 2),
                                  "table_height_mismatch_mm": round(float(t_f[2]) * 1000, 2)}
        T_FR_FL = make_T(Rf, [t_f[0], t_f[1], 0.0])
        # W in F_right: x along the bases' facing axis (perpendicular to the rails), origin midway
        W_FR = make_T(np.column_stack([[-1.0, 0, 0], [0, -1.0, 0], [0, 0, 1.0]]), T_FR_FL[:3, 3] / 2)
        T_W_F = {"right": inv_T(W_FR), "left": inv_T(W_FR) @ T_FR_FL}
        T_W_base = {a: T_W_F[a] @ inv_T(T_B_F[a]) for a in ARMS}
    else:
        T_W_base = {"right": inv_T(W_R), "left": inv_T(W_R) @ T_R_L}
        T_W_F = {a: T_W_base[a] @ T_B_F[a] for a in ARMS}
    T_W_mount = {a: T_W_F[a] @ make_T(np.eye(3), [0, 0, planes[a][1]]) for a in ARMS}
    report["constraint"] = "parallel rails: bases level and facing" if parallel else "none (free rigid fit)"
    report["geometry"] = summarize(T_W_base)
    report["geometry"].update({k: v for k, v in summarize(T_W_mount).items() if k in (
        "base_distance_mm", "left_yaw_vs_facing_deg", "left_lateral_in_right_mm")})
    report["geometry"]["apparent_tilt_deg"] = report["geometry"].pop("base_tilt_deg")
    if reference:
        ref = summarize(reference)
        g = report["geometry"]
        report["vs_reference"] = {
            "base_distance_mm": round(g["base_distance_mm"] - ref["base_distance_mm"], 1),
            "left_yaw_deg": round(g["left_yaw_vs_facing_deg"] - ref["left_yaw_vs_facing_deg"], 2),
            "left_lateral_mm": round(g["left_lateral_in_right_mm"] - ref["left_lateral_in_right_mm"], 1),
        }
    return {"T_W_base": {a: np.round(T, 6).tolist() for a, T in T_W_base.items()},
            "T_W_mount": {a: np.round(T, 6).tolist() for a, T in T_W_mount.items()},
            "T_right_left": np.round(T_R_L, 6).tolist(), "report": report,
            "_matches": {"inliers": inl, "pair": pair, "uv_left": UA, "uv_right": UB,
                         "n_right": len(feats["right"])}}


def draw_matches(views: list[View], res: dict, path: Path, max_lines: int = 80) -> None:
    """The left/right view pair with the most consistent matches, side by side, inliers drawn."""
    m = res["_matches"]
    inl, pair = m["inliers"], m["pair"]
    best = np.bincount(pair[inl]).argmax()
    i, j = divmod(int(best), m["n_right"])
    left = [v for v in views if v.arm == "left"][i].color
    right = [v for v in views if v.arm == "right"][j].color
    img = np.hstack([left, right])
    sel = np.flatnonzero(inl & (pair == best))
    sel = sel[np.random.default_rng(0).permutation(len(sel))[:max_lines]]
    for k in sel:
        a = tuple(int(x) for x in m["uv_left"][k])
        b = (int(m["uv_right"][k][0]) + left.shape[1], int(m["uv_right"][k][1]))
        cv2.line(img, a, b, (60, 200, 90), 1, cv2.LINE_AA)
        cv2.circle(img, a, 3, (60, 200, 90), -1)
        cv2.circle(img, b, 3, (60, 200, 90), -1)
    cv2.putText(img, f"left view {i + 1}  |  right view {j + 1}  |  {int((inl & (pair == best)).sum())} consistent matches",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


# ---- fixed cameras (e.g. the overhead one) ------------------------------------------------------

# ---- lens distortion (RealSense models; coefficients k1, k2, p1, p2, k3 as in OpenCV) -----------

def _bc(x, y, c):
    """Brown-Conrady: applied to undistorted normalised coordinates it distorts them (the
    "inverse" RealSense model applies the same formula the other way round)."""
    k1, k2, p1, p2, k3 = c
    r2 = x * x + y * y
    f = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    return x * f + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * f + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y


def _bc_inv(x, y, c, iters: int = 20):
    xu, yu = x, y
    for _ in range(iters):
        fx, fy = _bc(xu, yu, c)
        xu, yu = xu + (x - fx), yu + (y - fy)
    return xu, yu


def undistort_norm(xd, yd, dist, model: str = "brown_conrady"):
    """Distorted normalised image coordinates -> undistorted."""
    if dist is None or not np.any(dist):
        return xd, yd
    return _bc(xd, yd, dist) if model == "inverse_brown_conrady" else _bc_inv(xd, yd, dist)


def distort_norm(xu, yu, dist, model: str = "brown_conrady"):
    """Undistorted normalised image coordinates -> distorted."""
    if dist is None or not np.any(dist):
        return xu, yu
    return _bc_inv(xu, yu, dist) if model == "inverse_brown_conrady" else _bc(xu, yu, dist)


def ideal_px(uv: np.ndarray, K: np.ndarray, dist, model: str = "brown_conrady") -> np.ndarray:
    """Raw pixel coordinates -> where they would be with the same K and no lens distortion."""
    x, y = undistort_norm((uv[:, 0] - K[0, 2]) / K[0, 0], (uv[:, 1] - K[1, 2]) / K[1, 1], dist, model)
    return np.column_stack([x * K[0, 0] + K[0, 2], y * K[1, 1] + K[1, 2]])


def umeyama_2d(A: np.ndarray, B: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """(s, R, t) minimising |s R a + t - b| over rows (2-D, no reflection)."""
    ma, mb = A.mean(axis=0), B.mean(axis=0)
    Ac, Bc = A - ma, B - mb
    U, S, Vt = np.linalg.svd(Bc.T @ Ac / len(A))
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (Ac ** 2).sum() * len(A))
    return s, R, mb - s * R @ ma


def depth_plane(depth: np.ndarray, K: np.ndarray, near: float = 0.3, far: float = 2.0, tol: float = 0.02,
                prior: tuple | None = None, prior_band: float = 0.05, dist=None, model: str = "brown_conrady",
                normal_hint=None, hint_deg: float = 35.0):
    """The dominant plane in a depth image, in the camera frame: (n, d, inlier fraction) with
    n.p + d = 0 and d > 0 (n points from the plane toward the camera). With `prior` (n, d), only
    points within `prior_band` of that plane are considered, so the floor or a wall can't win, and
    the fraction is of those points. `tol` 2 cm: the D405's depth noise at ~1 m is ~6 mm."""
    h, w = depth.shape
    v, u = np.mgrid[0:h:3, 0:w:3]
    z = depth[::3, ::3]
    ok = (z > near) & (z < far)
    if ok.sum() < 2000:
        return None
    xn, yn = undistort_norm((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], dist, model)
    P = np.stack([xn * z, yn * z, z], -1)[ok]
    if prior is not None:
        P = P[np.abs(P @ prior[0] + prior[1]) < prior_band]
        if len(P) < 2000:
            return None
    n_all = len(P)
    rng = np.random.default_rng(0)
    best = None
    for _ in range(300):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if normal_hint is not None and abs(n @ normal_hint) < np.cos(np.radians(hint_deg)):
            continue                                   # e.g. a wall, when the table is wanted
        inl = np.abs((P - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None:
        return None
    Q = P[best]
    mu = Q.mean(axis=0)
    n = np.linalg.svd(Q - mu, full_matrices=False)[2][2]
    d = -float(n @ mu)
    if d < 0:
        n, d = -n, -d
    return n, d, float(best.sum() / n_all)


def register_fixed(views: list[View], T_W_base: dict, color: np.ndarray, K: np.ndarray,
                   base_distance_seed: float, table_z_seed: float = -0.025, reproj_px: float = 3.0,
                   min_inliers: int = 30, depth: np.ndarray | None = None, dist=None,
                   model: str = "brown_conrady") -> dict:
    """Pose of a fixed camera in W from one colour frame: its SIFT keypoints matched to the wrist
    views' features (3-D in W via each arm's base pose).

    PnP + RANSAC alone pins the position well but the tilt poorly when the matches cover only a
    small patch of the table (the error then grows toward the table edges). With `depth`, the
    table plane measured by this camera fixes its tilt and height instead, and the matches only
    solve the in-plane yaw and offset (a 2-D similarity, whose scale absorbs depth scale bias).
    `dist`/`model`: the camera's lens distortion; everything below works on undistorted pixels."""
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    kps, desc = cv2.SIFT_create(nfeatures=8000, contrastThreshold=0.02).detectAndCompute(gray, None)
    if desc is None:
        raise CoordinatesError("no SIFT features in the fixed camera's image")
    fixed = Features(np.array([k.pt for k in kps]), np.zeros((len(kps), 3)), desc)
    obj, img = [], []
    box = Box(np.array([base_distance_seed / 2, 0.0, table_z_seed]))
    for v in views:
        f = features(v, box)
        m = match(f, fixed, ratio=0.8)
        obj.append(apply_T(np.asarray(T_W_base[v.arm]), f.P[m[:, 0]]))
        img.append(fixed.uv[m[:, 1]])
    obj, img = np.concatenate(obj), np.concatenate(img)
    if len(obj) < min_inliers:
        raise CoordinatesError(f"only {len(obj)} SIFT matches between the fixed camera and the wrist views")
    img = ideal_px(img, K, dist, model)
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj.astype(np.float64), img.astype(np.float64), K, None,
                                             reprojectionError=reproj_px, iterationsCount=5000, confidence=0.999)
    if not ok or inl is None or len(inl) < min_inliers:
        why = f"PnP found only {0 if inl is None else len(inl)} consistent matches of {len(obj)}"
        if depth is None:
            raise CoordinatesError(why)
        return register_rectified(views, T_W_base, color, depth, K, base_distance_seed, table_z_seed,
                                  dist=dist, model=model, pnp_failed=why)
    inl = inl[:, 0]
    rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], K, None, rvec, tvec)
    T_W_pnp = inv_T(make_T(cv2.Rodrigues(rvec)[0], tvec[:, 0]))
    report = {"matches": int(len(obj)), "inliers": int(len(inl)), "method": "pnp"}

    T_W_cam = T_W_pnp
    # the table (W z = 0) in the camera frame by PnP: preselects the table in the depth
    prior = (T_W_pnp[2, :3], float(T_W_pnp[2, 3]))
    plane = None
    if depth is not None:
        plane = depth_plane(depth, K, prior=prior, dist=dist, model=model)
        report["depth_valid"] = round(float(((depth > 0.3) & (depth < 2.0)).mean()), 3)
        report["depth_plane"] = "none" if plane is None else f"{plane[2]:.2f} of samples on it"
    if plane and plane[2] > 0.5:
        n, d, frac = plane
        on_table = np.abs(obj[inl, 2]) < 0.01                  # matched points lying on the table
        uv = img[inl][on_table]
        rays = np.column_stack([(uv[:, 0] - K[0, 2]) / K[0, 0], (uv[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(uv))])
        q = rays * (-d / (rays @ n))[:, None]                  # ray / table-plane hits, camera frame
        T_cam_P = table_frame(n, d)                            # table frame P in the camera frame
        Rp, o = T_cam_P[:3, :3], T_cam_P[:3, 3]
        qp = (q - o) @ Rp                                      # hits in P (z ~ 0)
        s, R2, t2 = umeyama_2d(qp[:, :2], obj[inl][on_table][:, :2])
        T_W_cam = _plane_fit_to_W(T_cam_P, s, R2, t2)
        resid = np.linalg.norm(s * qp[:, :2] @ R2.T + t2 - obj[inl][on_table][:, :2], axis=1)
        report.update({"method": "depth plane + 2-D fit", "table_plane_inliers": round(frac, 2),
                       "depth_scale": round(1 / s, 4), "in_plane_rms_mm": round(float(np.sqrt(np.mean(resid ** 2))) * 1000, 2),
                       "pnp_tilt_disagreement_deg": round(angle_deg(T_W_pnp[:3, :3].T @ T_W_cam[:3, :3]), 2)})

    T_cam_W = inv_T(T_W_cam)
    proj, _ = cv2.projectPoints(obj[inl], cv2.Rodrigues(T_cam_W[:3, :3])[0], T_cam_W[:3, 3], K, None)
    err = np.linalg.norm(proj[:, 0] - img[inl], axis=1)
    report.update({"reprojection_px": {"rms": round(float(np.sqrt(np.mean(err ** 2))), 2),
                                       "p90": round(float(np.percentile(err, 90)), 2)},
                   "height_above_table_mm": round(float(T_W_cam[2, 3]) * 1000, 1)})
    return _fixed_camera(T_W_cam, K, dist, model, color, report)


def _plane_fit_to_W(T_cam_P: np.ndarray, s: float, R2: np.ndarray, t2) -> np.ndarray:
    """A fixed camera in W from its table frame P (from its own depth) and the in-plane similarity
    W = s R2 P + t2 fitted to matches. The scale s is the depth scale error; it corrects the
    camera's height above the table."""
    T_W_P = make_T(np.block([[R2, np.zeros((2, 1))], [np.zeros((1, 2)), np.ones((1, 1))]]), [*t2, 0.0])
    return T_W_P @ inv_T(make_T(T_cam_P[:3, :3], T_cam_P[:3, 3] * s))


def _fixed_camera(T_W_cam, K, dist, model, color, report) -> dict:
    dist = [0.0] * 5 if dist is None else [float(v) for v in dist]
    return {"T_W_cam": np.round(T_W_cam, 6).tolist(), "K": np.round(K, 4).tolist(),
            "dist": dist, "model": model, "size": [int(color.shape[1]), int(color.shape[0])], "report": report}


def ransac_similarity_2d(A: np.ndarray, B: np.ndarray, thr: float, iters: int = 3000, seed: int = 0):
    """(s, R, t, inliers) of b = s R a + t (2-D) robust to outliers."""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i, j = rng.choice(len(A), 2, replace=False)
        if np.linalg.norm(A[i] - A[j]) < 0.02:
            continue
        s, R, t = umeyama_2d(A[[i, j]], B[[i, j]])
        if not 0.85 < s < 1.15:
            continue
        inl = np.linalg.norm(s * A @ R.T + t - B, axis=1) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 3:
        return None
    for _ in range(2):   # refine on the inliers, then re-select
        s, R, t = umeyama_2d(A[best], B[best])
        best = np.linalg.norm(s * A @ R.T + t - B, axis=1) < thr
    return (*umeyama_2d(A[best], B[best]), best)


def register_rectified(views: list[View], T_W_base: dict, color: np.ndarray, depth: np.ndarray, K: np.ndarray,
                       base_distance_seed: float, table_z_seed: float = -0.025, dist=None,
                       model: str = "brown_conrady", mm_per_px: float = 1.5, min_inliers: int = 15,
                       pnp_failed: str | None = None) -> dict:
    """Pose in W of a fixed camera that sees the table at a grazing angle (e.g. one at table level),
    where SIFT can't match its view to the wrist views directly.

    Its own depth gives the table plane (tilt and height); its image of the table is resampled into
    a straight-down view of that plane, whose SIFT keypoints then match the wrist views' table
    features; a 2-D similarity (RANSAC) on the plane gives yaw and position, its scale the depth
    scale error."""
    why = pnp_failed or "PnP not tried"
    plane = depth_plane(depth, K, near=0.1, far=1.5, tol=0.006, dist=dist, model=model,
                        normal_hint=np.array([0.0, -1.0, 0.0]))       # up in the image
    if plane is None:
        raise CoordinatesError(f"{why}; and no table plane in this camera's depth")
    n, d, frac = plane
    T_cam_P = table_frame(n, d)                                       # plane frame in the camera frame
    Rp, o = T_cam_P[:3, :3], T_cam_P[:3, 3]
    ex, ey = Rp[:, 0], Rp[:, 1]

    # the table as seen by this camera, in plane coordinates (a, b)
    h, w = depth.shape
    v, u = np.mgrid[0:h:4, 0:w:4]
    z = depth[::4, ::4]
    ok = (z > 0.1) & (z < 1.5)
    xn, yn = undistort_norm((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], dist, model)
    Pc = np.stack([xn * z, yn * z, z], -1)[ok]
    ab = ((Pc[np.abs(Pc @ n + d) < 0.01] - o) @ Rp)[:, :2]
    if len(ab) < 200:
        raise CoordinatesError(f"{why}; too little table in view")
    lo, hi = np.percentile(ab, 1, axis=0), np.percentile(ab, 99, axis=0)
    res = mm_per_px / 1000
    cols, rows = int((hi[0] - lo[0]) / res), int((hi[1] - lo[1]) / res)
    A, Bm = np.meshgrid(lo[0] + np.arange(cols) * res, hi[1] - np.arange(rows) * res)   # seen from above
    X = o + A[..., None] * ex + Bm[..., None] * ey
    xd, yd = distort_norm(X[..., 0] / X[..., 2], X[..., 1] / X[..., 2], dist, model)
    mapx = (xd * K[0, 0] + K[0, 2]).astype(np.float32)
    mapy = (yd * K[1, 1] + K[1, 2]).astype(np.float32)
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    top = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR, borderValue=0)
    valid = ((mapx > 2) & (mapx < w - 3) & (mapy > 2) & (mapy < h - 3) & (X[..., 2] > 0)).astype(np.uint8) * 255
    kps, desc = cv2.SIFT_create(nfeatures=8000, contrastThreshold=0.02).detectAndCompute(top, cv2.erode(valid, np.ones((7, 7), np.uint8)))
    if desc is None:
        raise CoordinatesError(f"{why}; no SIFT features in the rectified table view")
    px = np.array([k.pt for k in kps])
    mine = Features(px, np.zeros((len(px), 3)), desc)
    src, dst = [], []
    box = Box(np.array([base_distance_seed / 2, 0.0, table_z_seed]))
    for view in views:
        f = features(view, box)
        PW = apply_T(np.asarray(T_W_base[view.arm]), f.P)
        on = np.abs(PW[:, 2]) < 0.01                                  # table features only
        m = match(Features(f.uv[on], PW[on], f.desc[on]), mine, ratio=0.8)
        src.append(px[m[:, 1]])
        dst.append(PW[on][m[:, 0], :2])
    src, dst = np.concatenate(src), np.concatenate(dst)
    if len(src) < min_inliers:
        raise CoordinatesError(f"{why}; only {len(src)} matches in the rectified view")
    q = np.column_stack([lo[0] + src[:, 0] * res, hi[1] - src[:, 1] * res])   # plane coordinates (a, b)
    fit = ransac_similarity_2d(q, dst, 0.006)
    if fit is None or fit[3].sum() < min_inliers:
        raise CoordinatesError(f"{why}; rectified: only {0 if fit is None else int(fit[3].sum())} "
                               f"consistent matches of {len(src)}")
    s, R2, t2, inl = fit
    T_W_cam = _plane_fit_to_W(T_cam_P, s, R2, t2)
    resid = np.linalg.norm(s * q[inl] @ R2.T + t2 - dst[inl], axis=1)
    report = {"method": "rectified table view + 2-D fit", "pnp": pnp_failed, "matches": int(len(src)),
              "inliers": int(inl.sum()), "table_plane_inliers": round(frac, 2), "depth_scale": round(1 / s, 4),
              "in_plane_rms_mm": round(float(np.sqrt(np.mean(resid ** 2))) * 1000, 2),
              "height_above_table_mm": round(float(T_W_cam[2, 3]) * 1000, 1)}
    # reprojection of the inlier table points, for comparison with the other method
    T_cam_W = inv_T(T_W_cam)
    Pw = np.column_stack([dst[inl], np.zeros(int(inl.sum()))])
    Pc2 = apply_T(T_cam_W, Pw)
    uvp = Pc2[:, :2] / Pc2[:, 2:] * [K[0, 0], K[1, 1]] + K[:2, 2]
    Xs = apply_T(T_cam_P, np.column_stack([q[inl], np.zeros(int(inl.sum()))]))
    uvo = Xs[:, :2] / Xs[:, 2:] * [K[0, 0], K[1, 1]] + K[:2, 2]
    err = np.linalg.norm(uvp - uvo, axis=1)
    report["reprojection_px"] = {"rms": round(float(np.sqrt(np.mean(err ** 2))), 2), "p90": round(float(np.percentile(err, 90)), 2)}
    return _fixed_camera(T_W_cam, K, dist, model, color, report)
