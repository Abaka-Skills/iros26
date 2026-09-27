"""Cube detection and fusion across cameras (the locate_cubes topic's algorithm; no I/O here).

Each camera: colour labels per pixel after white-balancing the frame on the table (every camera
sees the same table, so this cancels their white-balance differences); then each labelled blob
becomes observations in W:

- near (depth noise a few mm: wrists, low): blob pixels are lifted with depth; the blob's
  highest points (its top edge) give the level. A top face gives the cube's centre directly; a
  side face alone is pushed back half a cube, away from the camera.
- far (the overhead camera at ~1 m, depth noise ~2 cm): the blob is a top face at an unknown
  level; its centre is where its rays meet that level's top plane, once fusion knows the level.

Wrist cameras first correct their pose by the table plane in their own depth (the table is W's
z = 0): the arm's sag, which changes with the pose, is not in the kinematic model.

No two cubes overlap: at one level, footprints overlapping by more than OVERLAP_IOU are one cube.

Fusion: observations group into columns by position. Near observations set the levels; far ones
belong to the column's top (level 0 when no near camera sees the column: `level_measured`
false). Each (column, level) is one cube: colour by weighted vote, centre from top-face views
weighted by their expected error. Levels under a column's top that no camera sees are reported
as hidden cubes with no colour.

Table-level cameras (low) see only cube sides, with ~1 cm position error: too coarse to tell a
stack's base from the cube touching it. So they don't find cubes; they answer questions: for each
column the other cameras found and each level, project the face that cube would turn to the
camera, and compare the depth there with the face's: a match is a cube (colour read off it),
seeing past it is an empty level, something nearer hides it (no answer).
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

from myaloha.coords import apply_T, distort_norm, fit_plane, inv_T, make_T, undistort_norm

SIZE = 0.025                      # cube edge (m)
MAX_LEVEL = 3                     # stacks up to four cubes
NAMES = ("red", "orange", "yellow", "green", "blue", "purple")
# OpenCV Lab (L 0-255, a/b centred on 0), sampled on the overhead camera, 2026-09-26, whose table
# then read TABLE_REF. Keys starting with "_" are not cubes.
PROTO = {"red": (120, 61, 36), "orange": (147, 45, 28), "yellow": (205, 11, 60), "green": (105, -21, -14),
         "blue": (82, 15, -51), "purple": (62, 4, -26),
         "_wood": (205, 0, -2), "_wood2": (154, 2, 7), "_shadow": (155, 1, -12), "_black": (9, -4, -1),
         "_dark": (40, 0, 0), "_grey": (100, 0, 0),
         "_ink": (85, -4, -19)}           # blue-grey ink and blurred black on the tag sheet (not purple)
TABLE_REF = (163.0, 1.0, 0.0)
TABLE_REF_BGR = cv2.cvtColor(np.uint8([[[TABLE_REF[0], TABLE_REF[1] + 128, TABLE_REF[2] + 128]]]),
                             cv2.COLOR_Lab2BGR)[0, 0].astype(np.float32)
L_WEIGHT = 0.4                    # lightness matters less than hue (shading)
MAX_COLOR_DIST = 28.0
MIN_CHROMA = 15.0                 # greyish pixels (rails, shadows, the arms) are never cubes
POSE_SIGMA = {True: 0.003, False: 0.008}   # position error of a registered camera / an arm-borne one
MERGE_RADIUS = 0.6 * SIZE         # observations this close are one column, whatever their colour
OVERLAP_IOU = 0.2                 # two cubes at one level overlapping more than this are one cube
RANGE_BIAS = 0.05                 # D405 range error beyond ~0.5 m, as a fraction of the distance (measured
                                  # 2026-09-26: wrist positions 25-40 mm long at 0.5-0.6 m vs the overhead)


# the prototypes as arrays, lightness down-weighted, and each one's cube index (-1: not a cube)
_W = np.float32([L_WEIGHT, 1, 1])
_P = np.array(list(PROTO.values()), np.float32) * _W
_P2 = (_P * _P).sum(1)
_IDS = np.array([NAMES.index(k) if not k.startswith("_") else -1 for k in PROTO], np.int8)


def depth_noise(d):
    """D405 depth noise (1 sigma, m) at distance d, roughly quadratic: ~4 mm at 0.5 m."""
    return 0.004 * (d / 0.5) ** 2


@dataclass
class CameraView:
    key: str
    color: np.ndarray
    depth: np.ndarray             # metres, aligned to colour
    K: np.ndarray
    dist: np.ndarray
    model: str
    T_W_cam: np.ndarray
    fixed: bool = True            # False: a wrist, whose pose comes from the arm
    info: dict = field(default_factory=dict)


@dataclass
class Obs:
    cam: str
    label: int
    xy: np.ndarray
    level: int | None             # None: a far view, level unknown
    top: bool                     # the top face was seen: the centre is direct
    n: int                        # pixels
    dist: float                   # camera to cube (m)
    sigma: float                  # expected position error (m)
    yaw: float | None = None      # deg, mod 90
    rays: np.ndarray | None = None      # far views: pixel rays in W, to place the face at its level
    origin: np.ndarray | None = None
    spread: float = 0.0

    @property
    def vote(self) -> float:
        return min(self.n, 400) / self.dist

    def place(self, lv: int) -> None:
        """Far view: put the face on level `lv`'s top plane (xy, yaw, spread; not the level)."""
        s = ((lv + 1) * SIZE - self.origin[2]) / self.rays[:, 2]
        X = self.origin[:2] + s[:, None] * self.rays[:, :2]
        self.xy, self.yaw = X.mean(axis=0), _yaw(X)
        self.spread = float(np.ptp(X, axis=0).max())


def _rays(K, dist, model, u, v):
    x, y = undistort_norm((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], dist, model)
    return np.stack([x, y, np.ones_like(x)], -1)


def _lift(view: CameraView, step: int):
    """(W points, their pixel coordinates u, v, depths) for every valid depth pixel on a grid."""
    h, w = view.depth.shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    d = view.depth[::step, ::step]
    ok = d > 0.05
    r = _rays(view.K, view.dist, view.model, u[ok].astype(float), v[ok].astype(float)) * d[ok][:, None]
    return apply_T(view.T_W_cam, r), u[ok], v[ok], d[ok]


def _in_ws(P, workspace):
    return (np.abs(P[..., 0]) < workspace[0]) & (np.abs(P[..., 1]) < workspace[1])


def table_fix(view: CameraView, lifted, workspace) -> None:
    """Correct the camera's tilt and height so the table in its own depth lies on W's z = 0."""
    P, _, _, d = lifted
    m = (d < 0.8) & (np.abs(P[:, 2]) < 0.03) & _in_ws(P, workspace)
    if m.sum() < 800:
        view.info["table_fix"] = "too little table in view"
        return
    n, _, inl = fit_plane(P[m], 0.003, iters=150)
    if inl.sum() < 500:
        view.info["table_fix"] = "no table plane"
        return
    tilt = float(np.degrees(np.arccos(np.clip(n[2], -1, 1))))
    c = P[m][inl].mean(axis=0)
    if tilt > 8 or abs(c[2]) > 0.05:
        view.info["table_fix"] = f"rejected: {tilt:.1f} deg, {c[2] * 1000:.0f} mm"
        return
    axis = np.cross(n, [0, 0, 1.0])
    R = cv2.Rodrigues(axis / (np.linalg.norm(axis) + 1e-12) * np.radians(tilt))[0]
    view.T_W_cam = make_T(R, [c[0], c[1], 0.0] - R @ c) @ view.T_W_cam
    view.info["table_fix"] = {"tilt_deg": round(tilt, 2), "height_mm": round(float(c[2]) * 1000, 1)}


def classify(bgr: np.ndarray, table_bgr) -> np.ndarray:
    """Per-pixel cube colour index (-1: not a cube). White balance first: per-channel gains that
    give this frame's table TABLE_REF's tint (multiplicative, so dark pixels stay dark and grey).
    Tint only, not brightness: an overexposed table would otherwise darken every cube."""
    if table_bgr is None:
        gains = np.ones(3, np.float32)
    else:
        table = np.maximum(np.asarray(table_bgr, np.float32), 1.0)
        gains = (TABLE_REF_BGR / TABLE_REF_BGR.mean()) / (table / table.mean())
    balanced = np.clip(cv2.GaussianBlur(bgr, (3, 3), 0).astype(np.float32) * gains, 0, 255).astype(np.uint8)
    lab = cv2.cvtColor(balanced, cv2.COLOR_BGR2Lab).astype(np.float32) - np.float32([0, 128, 128])
    out = np.full(lab.shape[:2], -1, np.int8)
    cand = np.hypot(lab[..., 1], lab[..., 2]) >= MIN_CHROMA
    x = lab[cand] * _W
    d = _P2 - 2 * x @ _P.T                        # squared distance to each prototype, less |x|^2
    k = d.argmin(-1)
    ok = d[np.arange(len(k)), k] + (x * x).sum(1) < MAX_COLOR_DIST ** 2
    out[cand] = np.where(ok, _IDS[k], -1)
    return out


def _yaw(xy: np.ndarray) -> float | None:
    if len(xy) < 8:
        return None
    return float(cv2.minAreaRect((xy * 1000).astype(np.float32))[2] % 90.0)


def _labels(view: CameraView, workspace):
    """Colour labels for the view (None if it sees no table), after correcting a wrist's pose by the
    table and white-balancing on it."""
    lifted = _lift(view, step=3)
    if not view.fixed:
        table_fix(view, lifted, workspace)
        lifted = _lift(view, step=3)
    P, u, v, d = lifted
    on_table = (np.abs(P[:, 2]) < np.maximum(0.004, 2 * depth_noise(d))) & _in_ws(P, workspace)
    if on_table.sum() < 300:
        view.info["skipped"] = "no table in view"
        return None
    table = np.median(view.color[v[on_table], u[on_table]].astype(np.float32), axis=0)
    # at a glancing angle the table mirrors the room light: its colour says nothing about the camera's
    grazing = np.degrees(np.arcsin(np.clip(view.T_W_cam[2, 3] / np.median(d[on_table]), 0, 1))) < 15
    if grazing:
        view.info["white_balance"] = "off: the table is seen at a glancing angle"
    return classify(view.color, None if grazing else table)


def observe(view: CameraView, workspace) -> list[Obs]:
    labels = _labels(view, workspace)
    if labels is None:
        return []
    T, origin = view.T_W_cam, view.T_W_cam[:3, 3]
    fpx, out = float(view.K[0, 0]), []
    kernel, bridge = np.ones((3, 3), np.uint8), np.ones((5, 5), np.uint8)
    for li in range(len(NAMES)):
        mask = cv2.morphologyEx((labels == li).astype(np.uint8), cv2.MORPH_CLOSE, bridge)   # glare splits faces
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        n, comp, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for ci in np.nonzero(stats[1:, cv2.CC_STAT_AREA] >= 12)[0] + 1:
            x0, y0, w, h = stats[ci, :4]
            vv, uu = np.nonzero(comp[y0:y0 + h, x0:x0 + w] == ci)
            uu, vv = uu + x0, vv + y0
            dd = view.depth[vv, uu]
            ok = dd > 0.05
            if ok.sum() < 8:
                continue
            rays = _rays(view.K, view.dist, view.model, uu.astype(float), vv.astype(float)) @ T[:3, :3].T
            dmed = float(np.median(dd[ok]))
            expect = (fpx * SIZE / dmed) ** 2          # pixels of one face at this distance
            P3 = rays[ok] * dd[ok][:, None] + origin
            if depth_noise(dmed) <= SIZE / 3:
                out += _near(view, li, P3, dmed, expect, workspace)
            else:
                out += _far(view, li, P3, rays, dmed, expect, workspace)
    view.info["observations"] = len(out)
    return out


def _near(view, li, P, dist, expect, workspace) -> list[Obs]:
    """A blob's highest points are its cube's top edge, whether the top face or only a side is
    seen: they give the level. Points a cube or more below belong to another cube of the same
    colour under it (a same-colour stack), handled the same way."""
    origin, noise = view.T_W_cam[:3, 3], depth_noise(dist)
    P = P[(P[:, 2] > -0.004) & (P[:, 2] < (MAX_LEVEL + 1) * SIZE + 0.008)]
    out = []
    while len(P) >= max(8, 0.15 * expect):
        z_top = float(np.percentile(P[:, 2], 95))
        lv = int(np.clip(np.round(z_top / SIZE) - 1, 0, MAX_LEVEL))
        Q, P = P[P[:, 2] > lv * SIZE - 0.004], P[P[:, 2] <= lv * SIZE - 0.004]
        if abs(z_top - (lv + 1) * SIZE) > 0.01 or len(Q) < max(8, 0.15 * expect) \
                or np.ptp(Q[:, :2], axis=0).max() > 2.2 * SIZE:
            continue
        top = Q[np.abs(Q[:, 2] - (lv + 1) * SIZE) < max(0.004, 1.5 * noise)]
        sigma = POSE_SIGMA[view.fixed] + noise + RANGE_BIAS * max(0.0, dist - 0.4)
        above = origin[2] - (lv + 1) * SIZE                          # a camera below the top can't see it
        slant = max(0.2, above / dist)                               # a top face seen at a low angle is thin
        if above > 0.01 and len(top) >= max(8, 0.3 * expect * slant):
            xy, is_top, yaw = top[:, :2].mean(axis=0), True, _yaw(top[:, :2])
        else:
            c = Q[:, :2].mean(axis=0)
            away = c - origin[:2]
            xy, is_top, yaw = c + away / (np.linalg.norm(away) + 1e-9) * SIZE / 2, False, None
            sigma += SIZE / 4
        if _in_ws(xy, workspace):
            out.append(Obs(view.key, li, xy, lv, is_top, len(Q), dist, sigma, yaw))
    return out


def _far(view, li, P, rays, dist, expect, workspace) -> list[Obs]:
    zmed = float(np.median(P[:, 2]))
    if not -0.15 < zmed < 0.25 or len(rays) < 0.25 * expect:
        return []   # far from the table (the floor, ~0.8 m down), or a speck. On a cube's textureless top
                    # one overhead depth frame reads anywhere within about +-0.1 m (well beyond
                    # depth_noise), so the window is wide.
    o = Obs(view.key, li, np.zeros(2), None, True, len(rays), dist, POSE_SIGMA[view.fixed], rays=rays,
            origin=view.T_W_cam[:3, 3])
    o.place(0)
    if o.spread > 2.2 * SIZE or not _in_ws(o.xy, workspace):
        return []
    return [o]


def probe(view: CameraView, labels, T_cam_W, xy, yaw) -> list[tuple]:
    """What a table-level (side) camera sees, level by level, where the cubes of column `xy` would
    show their faces: per level 'cube' (depth there matches the face; with the colour label read
    off it, or -1), 'empty' (it sees past the face), or None (something nearer hides it, or no
    depth). All levels are projected at once."""
    to_cam = view.T_W_cam[:2, 3] - np.asarray(xy)
    a = np.radians(yaw or 0.0)
    nrm = max((np.array([np.cos(a + k * np.pi / 2), np.sin(a + k * np.pi / 2)]) for k in range(4)),
              key=lambda n: n @ to_cam)                               # the face turned toward the camera
    tan = np.array([-nrm[1], nrm[0], 0.0])
    g = np.linspace(-0.25, 0.25, 5) * SIZE
    face = np.array([[*(np.asarray(xy) + nrm * SIZE / 2), 0.0] + tan * s + [0, 0, t] for s in g for t in g])
    X = np.concatenate([face + [0, 0, (lv + 0.5) * SIZE] for lv in range(MAX_LEVEL + 1)])
    Pc = apply_T(T_cam_W, X)
    front = Pc[:, 2] > 0.05
    Pc[~front, 2] = 1.0                                               # masked below; keeps the division finite
    xd, yd = distort_norm(Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2], view.dist, view.model)
    u = np.round(xd * view.K[0, 0] + view.K[0, 2]).astype(int)
    v = np.round(yd * view.K[1, 1] + view.K[1, 2]).astype(int)
    h, w = view.depth.shape
    inside = front & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    out, n = [], len(face)
    for lv in range(MAX_LEVEL + 1):
        s = slice(lv * n, (lv + 1) * n)
        ok = inside[s]
        uu, vv, z = u[s][ok], v[s][ok], Pc[s, 2][ok]
        d = view.depth[vv, uu]
        known = d > 0.05
        if not front[s].all() or ok.sum() < 10 or known.sum() < 8:
            out.append((None, -1))
            continue
        uu, vv, z, d = uu[known], vv[known], z[known], d[known]
        tol = max(0.012, 2 * depth_noise(float(z.mean())))          # plus registration error (~1 cm)
        hit = np.abs(d - z) < tol
        if hit.mean() >= 0.4:
            lab = labels[vv[hit], uu[hit]]
            lab = lab[lab >= 0]
            out.append(("cube", int(np.bincount(lab, minlength=len(NAMES)).argmax()) if len(lab) >= 4 else -1))
        elif (d > z + tol).mean() >= 0.6:
            out.append(("empty", -1))
        else:
            out.append((None, -1))
    return out


def fuse(obs: list[Obs], side: list = ()) -> list[dict]:
    """Cubes from every camera's observations; see the module docstring. An observation joins the
    nearest column within MERGE_RADIUS, or within a gate sized by both expected errors: the same
    cube, or another in its stack, seen less precisely by another camera. The gate is capped at 1.6
    cubes for the column's colour and 0.8 (less than touching neighbours) for another colour. A
    camera sees at most one cube per level of a column, so it never joins a column where it already
    has that level. `side`: (view, labels) of table-level cameras, probed at each column (see probe)."""
    cols: list[list[Obs]] = []
    for o in sorted(obs, key=lambda o: o.sigma):
        best, best_d = None, np.inf
        for c in cols:
            if any(p.cam == o.cam and p.level == o.level for p in c):
                continue
            d = np.linalg.norm(_xy(c) - o.xy)
            gate = min((1.6 if o.label == c[0].label else 0.8) * SIZE, 2 * np.hypot(o.sigma, c[0].sigma))
            if d < max(MERGE_RADIUS, gate) and d < best_d:
                best, best_d = c, d
        (best.append(o) if best is not None else cols.append([o]))
    inverse = [inv_T(view.T_W_cam) for view, _ in side]
    cubes = []
    for col in cols:
        cxy = _xy(col)
        near = [o for o in col if o.level is not None]
        top_lv = max((o.level for o in near), default=0)
        yaw = next((o.yaw for o in sorted(col, key=lambda o: o.sigma) if o.yaw is not None), None)
        # side cameras: which levels hold a cube, and their colours (positions stay the column's)
        seen = {}
        for (view, labels), T_cam_W in zip(side, inverse):
            for lv, (verdict, label) in enumerate(probe(view, labels, T_cam_W, cxy, yaw)):
                if verdict == "cube":
                    seen.setdefault(lv, []).append((view.key, label))
                elif verdict == "empty" and lv > top_lv:
                    break                                          # nothing above an empty level
        measured = bool(near) or bool(seen)
        top_lv = max([top_lv, *seen])
        for o in col:
            if o.level is None:
                o.place(top_lv)
                o.level = top_lv
        cxy = _xy(col)
        for lv in range(top_lv + 1):
            group = [o for o in col if o.level == lv]
            votes = np.zeros(len(NAMES))
            for o in group:
                votes[o.label] += o.vote
            for cam, label in seen.get(lv, []):
                if label >= 0:
                    votes[label] += 200.0                          # a clear side view of this very cube
            cams = sorted({o.cam for o in group} | {cam for cam, label in seen.get(lv, []) if label >= 0})
            base = {"x": round(float(cxy[0]), 4), "y": round(float(cxy[1]), 4), "z": round((lv + 0.5) * SIZE, 4),
                    "level": lv, "level_measured": measured, "seen_by": cams}
            if not votes.any():
                cubes.append({"color": None, **base, "yaw": None, "visible": False, "confidence": 0.0,
                              "sigma": round(float(min(o.sigma for o in col)), 4)})
                continue
            label = int(votes.argmax())
            src = [o for o in group if o.top] or group or col
            xy = _xy(src)
            yaws = [o.yaw for o in sorted(group, key=lambda o: o.sigma) if o.top and o.yaw is not None]
            n = sum(o.n for o in group) + 60 * len(seen.get(lv, []))
            conf = votes[label] / votes.sum() * min(1.0, n / 60) * (1.0 if measured else 0.8)
            sig = 1 / np.sqrt(sum(1 / o.sigma ** 2 for o in src))
            cubes.append({"color": NAMES[label], **base, "x": round(float(xy[0]), 4), "y": round(float(xy[1]), 4),
                          "yaw": round(yaws[0], 1) if yaws else None, "visible": True,
                          "confidence": round(float(conf), 2), "sigma": round(float(sig), 4)})
    return resolve_overlaps(cubes)


def _footprint(c: dict):
    return (c["x"] * 1000, c["y"] * 1000), (SIZE * 1000, SIZE * 1000), c["yaw"] or 0.0


def iou(a: dict, b: dict) -> float:
    """Overlap of two cubes' footprints (rotated squares), as intersection over union."""
    if a["level"] != b["level"] or np.hypot(a["x"] - b["x"], a["y"] - b["y"]) >= SIZE * np.sqrt(2):
        return 0.0
    ret, pts = cv2.rotatedRectangleIntersection(_footprint(a), _footprint(b))
    if ret == cv2.INTERSECT_NONE or pts is None or len(pts) < 3:
        return 0.0
    inter = cv2.contourArea(cv2.convexHull(pts))
    area = (SIZE * 1000) ** 2
    return float(inter / (2 * area - inter))


def resolve_overlaps(cubes: list[dict]) -> list[dict]:
    """No two cubes in one place: of cubes at one level overlapping by more than OVERLAP_IOU, keep the
    most reliable. A hidden cube (under a stack) that overlaps a visible one takes its colour and
    keeps its own position: from above, a stacked cube's side can look like a cube beside it."""
    rank = lambda c: (c["visible"], not c.get("held", False), c["confidence"], -c["sigma"])
    kept: list[dict] = []
    for c in sorted(cubes, key=rank, reverse=True):
        clash = next((k for k in kept if iou(k, c) > OVERLAP_IOU), None)
        if clash is None:
            kept.append(c)
        elif not c["visible"] and clash["visible"] and clash["color"]:
            clash.update({"x": c["x"], "y": c["y"], "level_measured": c["level_measured"] or clash["level_measured"]})
    return sorted(kept, key=lambda c: (c["x"], c["y"], c["level"]))


def _xy(group: list[Obs]) -> np.ndarray:
    w = np.array([1 / o.sigma ** 2 for o in group])
    return (np.array([o.xy for o in group]) * w[:, None]).sum(axis=0) / w.sum()


def is_side(view: CameraView) -> bool:
    """A camera at table level (below the tallest stack plus 2 cm): it sees cube sides only."""
    return view.T_W_cam[2, 3] < (MAX_LEVEL + 1) * SIZE + 0.02


def locate(views: list[CameraView], workspace=(0.5, 0.5), pool=None) -> tuple[list[dict], dict]:
    """(cubes, per-camera info). `workspace`: half extents (x, y) in W where cubes can be. Cameras
    above the table find the cubes; table-level ones then answer, per found column and level,
    whether a cube is there and its colour. With an executor `pool`, cameras are processed in
    parallel (OpenCV and numpy release the GIL)."""
    def one(v):
        return (v, _labels(v, workspace)) if is_side(v) else observe(v, workspace)
    obs, side = [], []
    for r in (pool.map(one, views) if pool else map(one, views)):
        if isinstance(r, tuple):
            side += [r] if r[1] is not None else []
        else:
            obs += r
    return fuse(obs, side), {v.key: v.info for v in views}


class Tracker:
    """Cubes over time, so single-frame misses and false hits don't blink. A detection updates the
    nearest track at the same level or one off, within MERGE_RADIUS widened for vaguer cubes
    (2.5 sigma, at most 1.6 cubes); positions are smoothed with `alpha`. A track is reported once
    seen `confirm` times (one more if vague: sigma > 1 cm) and kept `hold_s` after it was last seen
    (`held`). Colour is voted over the detections of the last `hold_s`: memory never outlasts that."""

    KEEP = ("z", "level", "level_measured", "yaw", "visible", "confidence", "sigma", "seen_by")

    def __init__(self, alpha: float = 0.5, confirm: int = 2, hold_s: float = 1.0):
        self.alpha, self.confirm, self.hold_s = alpha, confirm, hold_s
        self.tracks: list[dict] = []
        self._next = 1

    def update(self, found: list[dict], t: float) -> list[dict]:
        dist = lambda tr, c: float(np.hypot(tr["x"] - c["x"], tr["y"] - c["y"]))
        gate = lambda c: min(1.6 * SIZE, max(MERGE_RADIUS, 2.5 * c["sigma"]))    # wider for vaguer cubes
        # a level may flip by one between frames (a stack read from a new angle): same cube, same track
        pairs = sorted((dist(tr, c) + SIZE * abs(tr["level"] - c["level"]), i, j)
                       for i, tr in enumerate(self.tracks) for j, c in enumerate(found)
                       if abs(tr["level"] - c["level"]) <= 1 and dist(tr, c) < gate(c))
        match, used = {}, set()
        for _, i, j in pairs:                        # nearest pairs first
            if j not in match and i not in used:
                match[j] = self.tracks[i]
                used.add(i)
        for j, c in enumerate(found):
            tr = match.get(j)
            if tr is None:
                if (c["visible"] and c["confidence"] < 0.5) or any(tr2["level"] == c["level"] and dist(tr2, c) < gate(c)
                                                                   for tr2 in match.values()):
                    continue                         # weak, or a duplicate of a cube already matched
                vague = c["sigma"] > 0.01                    # e.g. seen by one wrist only: prove it longer
                tr = {"track": self._next, "hits": 0, "votes": [], "x": c["x"], "y": c["y"],
                      "confirm": self.confirm + (1 if vague else 0)}
                self._next += 1
                self.tracks.append(tr)
                match[j] = tr
            else:
                tr["x"] += self.alpha * (c["x"] - tr["x"])
                tr["y"] += self.alpha * (c["y"] - tr["y"])
            tr.update({k: c[k] for k in self.KEEP})
            tr["hits"] += 1
            tr["seen"] = t
            if c["color"]:
                tr["votes"].append((t, c["color"], c["confidence"]))
        self.tracks = [tr for tr in self.tracks if t - tr["seen"] <= self.hold_s]
        out = []
        for tr in self.tracks:
            if tr["hits"] < tr["confirm"]:
                continue
            tr["votes"] = [v for v in tr["votes"] if t - v[0] <= self.hold_s]
            score: dict = {}
            for _, name, conf in tr["votes"]:
                score[name] = score.get(name, 0.0) + max(conf, 0.05)
            color = max(score, key=score.get) if score else None
            out.append({**{k: tr[k] for k in self.KEEP}, "color": color, "x": round(tr["x"], 4),
                        "y": round(tr["y"], 4), "visible": tr["visible"] or color is not None,
                        "track": tr["track"], "held": t > tr["seen"]})
        return resolve_overlaps(out)       # a held track can overlap one seen now
