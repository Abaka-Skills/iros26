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

from myaloha import colors
from myaloha.coords import apply_T, distort_norm, fit_plane, inv_T, make_T, undistort_norm

SIZE = 0.025                      # cube edge (m)
MAX_LEVEL = 3                     # stacks up to four cubes
MAX_Z = 0.45                      # cube centres this high are considered (held ones: in the air)
REST_TOL = 0.008                  # a centre within this of a level's height rests at that level; else in the air
NAMES = ("red", "orange", "yellow", "green", "blue", "purple")
POSE_SIGMA = {True: 0.003, False: 0.008}   # position error of a registered camera / an arm-borne one
MERGE_RADIUS = 0.6 * SIZE         # observations this close are one column, whatever their colour
OVERLAP_IOU = 0.2                 # two cubes at one level overlapping more than this are one cube
# a gripper whose fingers stopped this far apart (m, as the driver reads it) may hold a cube. Measured
# on the right follower 2026-09-28: squeezing a 25 mm cube reads 23.4 mm (28.6 mm on 2026-09-26 was the
# fingers sprung back open, holding nothing); fingers shut on nothing read 1.2 mm.
HELD_APERTURE = (0.018, 0.034)
FACE_SEEN = 0.6                   # share of `expect` a whole face fills at least, seen from far (the blur before
                                  # classifying costs a small blob its edge ring: 0.65 at 1 m, 11 px across)
VAGUE_SIGMA = SIZE / 2            # an observation expected off by half a cube or more places no cube by itself (see fuse)
SIDE_DOWN = 30.0                  # deg: a camera at table level looking down less than this sees cube sides only
RANGE_BIAS = 0.05                # D405 range error beyond ~0.5 m, as a fraction of the distance (measured
                                  # 2026-09-26: wrist positions 25-40 mm long at 0.5-0.6 m vs the overhead)


class Palette:
    """A colour profile (myaloha.colors) ready to classify with: the prototypes as arrays, lightness
    down-weighted, and each one's cube index (-1: not a cube)."""

    def __init__(self, doc: dict):
        self.doc, self.name, self.balance = doc, doc.get("name", "?"), doc["balance"]
        L, a, b = doc["table"]
        self.table_bgr = cv2.cvtColor(np.uint8([[[L, a + 128, b + 128]]]), cv2.COLOR_Lab2BGR)[0, 0].astype(np.float32)
        self.max_dist, self.min_chroma = float(doc["max_dist"]), float(doc["min_chroma"])
        self.w = np.float32([doc["l_weight"], 1, 1])
        protos = [(NAMES.index(k), p) for k, ps in doc["colors"].items() for p in ps] + \
                 [(-1, p) for ps in doc["others"].values() for p in ps]
        self.p = np.array([p for _, p in protos], np.float32) * self.w
        self.p2 = (self.p * self.p).sum(1)
        self.ids = np.array([i for i, _ in protos], np.int8)


PALETTE = Palette(colors.load("default"))


def use_profile(doc: dict) -> None:
    """Classify with this colour profile from now on (a doc from myaloha.colors.load)."""
    global PALETTE
    PALETTE = Palette(doc)


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
    labels: np.ndarray | None = None   # colour labels, once classified
    mask: np.ndarray | None = None     # pixels of the robot itself (a wrist camera's own fingers): no colour, no depth

    def __post_init__(self):
        if self.mask is not None:
            self.depth = np.where(self.mask, 0, self.depth).astype(self.depth.dtype)


@dataclass
class Obs:
    cam: str
    label: int
    xy: np.ndarray
    level: int | None             # the level it rests at; None: in the air, or (z None) a far view not placed yet
    top: bool                     # the top face was seen: the centre is direct
    n: int                        # pixels
    dist: float                   # camera to cube (m)
    sigma: float                  # expected position error (m)
    yaw: float | None = None      # deg, mod 90
    rays: np.ndarray | None = None      # far views: pixel rays in W, to place the face at its level
    origin: np.ndarray | None = None
    spread: float = 0.0
    z: float | None = None        # the centre's height (W); None: a far view not placed yet

    @property
    def vote(self) -> float:
        return min(self.n, 400) / self.dist

    def place(self, top: float) -> None:
        """Far view: put the face on the horizontal plane at height `top` (xy, yaw, spread, z)."""
        s = (top - self.origin[2]) / self.rays[:, 2]
        X = self.origin[:2] + s[:, None] * self.rays[:, :2]
        self.xy, self.yaw, self.z = X.mean(axis=0), _yaw(X), top - SIZE / 2
        self.spread = float(np.ptp(X, axis=0).max())


def near_sigma(view: CameraView, dist: float) -> float:
    """A near camera's expected position error for a cube at `dist` (m): its pose, depth noise, and the
    D405 reading long beyond 0.4 m."""
    return POSE_SIGMA[view.fixed] + depth_noise(dist) + RANGE_BIAS * max(0.0, dist - 0.4)


def rest_level(z: float) -> int | None:
    """The level a cube centred at height z rests at, or None if it is in the air (between levels,
    or above the tallest stack)."""
    lv = int(round(z / SIZE - 0.5))
    return lv if 0 <= lv <= MAX_LEVEL and abs(z - (lv + 0.5) * SIZE) <= REST_TOL else None


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


def balanced_lab(bgr: np.ndarray, table_bgr, palette: Palette | None = None) -> np.ndarray:
    """The frame in OpenCV Lab (a, b centred on 0) after white balance on its table: per-channel
    gains (multiplicative, so dark pixels stay dark and grey) that give the table the profile's
    colour, its tint only (`balance: tint`) or its brightness too (`full`: evens out exposure
    between cameras)."""
    pal = palette or PALETTE
    if table_bgr is None:
        gains = np.ones(3, np.float32)
    else:
        table = np.maximum(np.asarray(table_bgr, np.float32), 1.0)
        gains = (pal.table_bgr / pal.table_bgr.mean()) / (table / table.mean()) if pal.balance == "tint" \
            else pal.table_bgr / table
    balanced = np.clip(cv2.GaussianBlur(bgr, (3, 3), 0).astype(np.float32) * gains, 0, 255).astype(np.uint8)
    return cv2.cvtColor(balanced, cv2.COLOR_BGR2Lab).astype(np.float32) - np.float32([0, 128, 128])


def classify(bgr: np.ndarray, table_bgr) -> np.ndarray:
    """Per-pixel cube colour index (-1: not a cube), by the nearest prototype of the colour profile."""
    pal = PALETTE
    lab = balanced_lab(bgr, table_bgr, pal)
    out = np.full(lab.shape[:2], -1, np.int8)
    cand = np.hypot(lab[..., 1], lab[..., 2]) >= pal.min_chroma
    x = lab[cand] * pal.w
    d = pal.p2 - 2 * x @ pal.p.T                  # squared distance to each prototype, less |x|^2
    k = d.argmin(-1)
    ok = d[np.arange(len(k)), k] + (x * x).sum(1) < pal.max_dist ** 2
    out[cand] = np.where(ok, pal.ids[k], -1)
    return out


def _yaw(xy: np.ndarray) -> float | None:
    if len(xy) < 8:
        return None
    return float(cv2.minAreaRect((xy * 1000).astype(np.float32))[2] % 90.0)


def table_color(view: CameraView, workspace):
    """(the table's colour in this view (BGR) to white-balance on, or None; whether the view sees a
    table at all). Corrects a wrist's pose by the table first."""
    lifted = _lift(view, step=3)
    if not view.fixed:
        table_fix(view, lifted, workspace)
        lifted = _lift(view, step=3)
    P, u, v, d = lifted
    on_table = (np.abs(P[:, 2]) < np.maximum(0.004, 2 * depth_noise(d))) & _in_ws(P, workspace)
    if on_table.sum() < 300:
        view.info["skipped"] = "no table in view"
        return None, False
    # at a glancing angle the table mirrors the room light: its colour says nothing about the camera's
    if np.degrees(np.arcsin(np.clip(view.T_W_cam[2, 3] / np.median(d[on_table]), 0, 1))) < 15:
        view.info["white_balance"] = "off: the table is seen at a glancing angle"
        return None, True
    return np.median(view.color[v[on_table], u[on_table]].astype(np.float32), axis=0), True


def _labels(view: CameraView, workspace):
    """Colour labels for the view (None if it sees no table), after correcting a wrist's pose by the
    table and white-balancing on it."""
    table, seen = table_color(view, workspace)
    if not seen:
        return None
    view.labels = classify(view.color, table)
    if view.mask is not None:
        view.labels[view.mask] = -1
    return view.labels


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
    seen: they give the centre's height (continuous: a cube in a gripper is anywhere in the air;
    one within REST_TOL of a level's height rests there). Points a cube or more below belong to
    another cube of the same colour under it (a same-colour stack), handled the same way."""
    origin, noise = view.T_W_cam[:3, 3], depth_noise(dist)
    P = P[(P[:, 2] > -0.004) & (P[:, 2] < MAX_Z + SIZE / 2)]
    out = []
    while len(P) >= max(8, 0.15 * expect):
        z_top = float(np.percentile(P[:, 2], 95))
        Q, P = P[P[:, 2] > z_top - SIZE - 0.004], P[P[:, 2] <= z_top - SIZE - 0.004]
        if z_top < SIZE - 0.01 or len(Q) < max(8, 0.15 * expect) or np.ptp(Q[:, :2], axis=0).max() > 2.2 * SIZE:
            continue
        z = z_top - SIZE / 2
        lv = rest_level(z)
        c = Q[:, :2].mean(axis=0)
        if lv and evidence(view, np.array([*c, (lv - 0.5) * SIZE])) == "empty":
            lv = None                     # this camera sees past where a cube under it would be: in the air
        if lv is not None:
            z = (lv + 0.5) * SIZE
        top = Q[np.abs(Q[:, 2] - z_top) < max(0.004, 1.5 * noise)]
        sigma = near_sigma(view, dist)
        # a top face: the camera above it, and its points spread both ways in xy (a side face's top
        # edge is a line); seen, it gives the centre directly
        spread = np.sqrt(np.linalg.eigvalsh(np.cov(top[:, :2].T))[0]) if len(top) >= 8 else 0.0
        if origin[2] - z_top > 0.01 and spread > 0.15 * SIZE:
            xy, is_top, yaw = top[:, :2].mean(axis=0), True, _yaw(top[:, :2])
        else:
            away = c - origin[:2]
            xy, is_top, yaw = c + away / (np.linalg.norm(away) + 1e-9) * SIZE / 2, False, None
            sigma += SIZE / 4
        if _in_ws(xy, workspace):
            out.append(Obs(view.key, li, xy, lv, is_top, len(Q), dist, sigma, yaw, z=z))
    return out


def _far(view, li, P, rays, dist, expect, workspace) -> list[Obs]:
    zmed = float(np.median(P[:, 2]))
    if not -0.15 < zmed < 0.25 or len(rays) < 0.25 * expect:
        return []   # far from the table (the floor, ~0.8 m down), or a speck. On a cube's textureless top
                    # one overhead depth frame reads anywhere within about +-0.1 m (well beyond
                    # depth_noise), so the window is wide.
    # a face partly hidden (a gripper coming down over it) or shaded: the centre of what shows is off by
    # up to half the hidden share of a cube
    sigma = POSE_SIGMA[view.fixed] + max(0.0, 1.0 - len(rays) / (FACE_SEEN * expect)) * SIZE / 2
    o = Obs(view.key, li, np.zeros(2), None, True, len(rays), dist, sigma, rays=rays, origin=view.T_W_cam[:3, 3])
    o.place(SIZE)
    o.z = None                                     # its height is not known until fusion places it
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
    has that level. `side`: (view, labels) of table-level cameras, probed at each column (see probe).

    Cubes in the air (in a gripper) are not part of any column: their near observations group by
    position in 3-D, and a far view whose rays, placed at such a cube's top, land on it belongs to
    it (it is the highest thing there). No hidden cubes are assumed under them.

    A vague observation (sigma over VAGUE_SIGMA: a wrist camera half a metre away, reading long, or a
    side face alone, its centre guessed half a cube behind) joins a cube a precise camera sees, but
    alone makes none: placed by it, a resting cube would jump and a ghost appear beside it. Where only
    such a camera sees one, the tracker keeps the track on the depth evidence, where it was last seen
    well."""
    air, obs = _in_air([o for o in obs if o.level is None and o.z is not None],
                       [o for o in obs if o.level is not None or o.z is None])
    cols = [c for c in _cluster(obs, lambda c, o: any(p.cam == o.cam and p.level == o.level for p in c))
            if min(o.sigma for o in c) <= VAGUE_SIGMA]
    inverse = [inv_T(view.T_W_cam) for view, _ in side]
    cubes = []
    for col in cols:
        cxy = _xy(col)
        near = [o for o in col if o.level is not None]
        top_lv = max((o.level for o in near), default=0)
        yaw = next((o.yaw for o in sorted(col, key=lambda o: o.sigma) if o.yaw is not None), None)
        # side cameras: which levels hold a cube, and their colours (positions stay the column's)
        seen, empty = {}, set()
        for (view, labels), T_cam_W in zip(side, inverse):
            for lv, (verdict, label) in enumerate(probe(view, labels, T_cam_W, cxy, yaw)):
                if verdict == "cube" and (label >= 0 or lv <= top_lv):
                    seen.setdefault(lv, []).append((view.key, label))   # no cube colour above the top: a gripper's finger
                elif verdict == "empty":
                    empty.add(lv)
                    if lv > top_lv:
                        break                                      # nothing above an empty level
        measured = bool(near) or bool(seen)
        top_lv = max([top_lv, *seen])
        for o in col:
            if o.level is None:
                o.place((top_lv + 1) * SIZE)
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
            if lv and lv - 1 in empty and not seen.get(lv - 1):
                base["level"] = None                               # a camera sees nothing under it: in the air
            if not votes.any():
                if lv not in empty:                                # a hidden cube, unless a camera sees past it
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
    return resolve_overlaps(cubes + air)


def _in_air(near: list[Obs], rest: list[Obs]) -> tuple[list[dict], list[Obs]]:
    """(cubes in the air, the observations left for the columns). Near observations of cubes in the
    air group by 3-D position (one per camera per cube); a far view joins one when its rays, placed
    at that cube's top, land within the gate."""
    groups = [g for g in _cluster(near, lambda g, o: any(p.cam == o.cam for p in g) or abs(_z(g) - o.z) > SIZE / 2)
              if min(o.sigma for o in g) <= VAGUE_SIGMA]
    left = []
    for o in rest:
        if o.z is None and groups:                  # a far view: the cube in the air above, if it is that
            for g in sorted(groups, key=lambda g: -_z(g)):
                o.place(_z(g) + SIZE / 2)
                if np.linalg.norm(_xy(g) - o.xy) < max(MERGE_RADIUS, 2 * np.hypot(o.sigma, g[0].sigma)):
                    g.append(o)
                    break
            else:
                o.z = None
                left.append(o)
        else:
            left.append(o)
    out = []
    for g in groups:
        votes = np.zeros(len(NAMES))
        for o in g:
            votes[o.label] += o.vote
        label, src = int(votes.argmax()), [o for o in g if o.top] or g
        xy, z = _xy(src), _z([o for o in g if o.rays is None])
        yaws = [o.yaw for o in sorted(g, key=lambda o: o.sigma) if o.top and o.yaw is not None]
        conf = votes[label] / votes.sum() * min(1.0, sum(o.n for o in g) / 60)
        out.append({"color": NAMES[label], "x": round(float(xy[0]), 4), "y": round(float(xy[1]), 4),
                    "z": round(float(z), 4), "level": None, "level_measured": True,
                    "seen_by": sorted({o.cam for o in g}), "yaw": round(yaws[0], 1) if yaws else None,
                    "visible": True, "confidence": round(float(conf), 2),
                    "sigma": round(float(1 / np.sqrt(sum(1 / o.sigma ** 2 for o in src))), 4)})
    return out, left


def _cluster(obs: list[Obs], clash) -> list[list[Obs]]:
    """Observations grouped, most precise first: each joins the nearest group within MERGE_RADIUS or
    the gate sized by both expected errors (capped as in fuse), unless `clash(group, o)`."""
    groups: list[list[Obs]] = []
    for o in sorted(obs, key=lambda o: o.sigma):
        best, best_d = None, np.inf
        for g in groups:
            if clash(g, o):
                continue
            d = np.linalg.norm(_xy(g) - o.xy)
            gate = min((1.6 if o.label == g[0].label else 0.8) * SIZE, 2 * np.hypot(o.sigma, g[0].sigma))
            if d < max(MERGE_RADIUS, gate) and d < best_d:
                best, best_d = g, d
        (best.append(o) if best is not None else groups.append([o]))
    return groups


def _footprint(c: dict):
    return (c["x"] * 1000, c["y"] * 1000), (SIZE * 1000, SIZE * 1000), c["yaw"] or 0.0


def iou(a: dict, b: dict) -> float:
    """Overlap of two cubes' footprints (rotated squares), as intersection over union."""
    if abs(a["z"] - b["z"]) >= SIZE / 2 or np.hypot(a["x"] - b["x"], a["y"] - b["y"]) >= SIZE * np.sqrt(2):
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
    return sorted(kept, key=lambda c: (c["x"], c["y"], c["z"]))


def _xy(group: list[Obs]) -> np.ndarray:
    w = np.array([1 / o.sigma ** 2 for o in group])
    return (np.array([o.xy for o in group]) * w[:, None]).sum(axis=0) / w.sum()


def _z(group: list[Obs]) -> float:
    w = np.array([1 / o.sigma ** 2 for o in group])
    return float((np.array([o.z for o in group]) * w).sum() / w.sum())


def is_side(view: CameraView) -> bool:
    """A camera at table level (below the tallest stack plus 2 cm) looking across it (less than
    SIDE_DOWN below the horizontal): it sees cube sides only. A wrist camera as low, over a cube it
    is about to grasp, looks down on its top."""
    return view.T_W_cam[2, 3] < (MAX_LEVEL + 1) * SIZE + 0.02 and -view.T_W_cam[2, 2] < np.sin(np.radians(SIDE_DOWN))


def locate(views: list[CameraView], workspace=(0.5, 0.5), pool=None) -> tuple[list[dict], dict]:
    """(cubes, per-camera info). `workspace`: half extents (x, y) in W where cubes can be. Cameras
    above the table find the cubes; table-level ones find cubes in the air, and answer, per found
    column and level, whether a cube is there and its colour. Only what the cameras see: nothing is inferred from the
    arms (a cube is not assumed to be in a gripper because its fingers stopped apart). With an
    executor `pool`, cameras are processed in parallel (OpenCV and numpy release the GIL)."""
    def one(v):
        found = observe(v, workspace)
        if is_side(v):              # a cube in the air it may find; the stacks, it only answers about
            return v, [o for o in found if o.level is None]
        return None, found
    obs, side = [], []
    for v, found in (pool.map(one, views) if pool else map(one, views)):
        obs += found
        if v is not None and v.labels is not None:
            side.append((v, v.labels))
    return fuse(obs, side), {v.key: v.info for v in views}


def evidence(view: CameraView, P: np.ndarray) -> str | None:
    """What a camera's depth says about a cube centred at P (W) that it did not detect: 'hidden'
    (something nearer covers where it would be: a gripper), 'there' (the depth is a cube's surface
    there: the detection missed it, its colour say), 'empty' (the camera sees past where the cube
    would be: it is not there), or None (out of view, no depth there, or too far to tell: the overhead
    camera's depth can't tell a cube's top from the table, only a gripper well above both). A camera
    too vague to place a cube there (see fuse) can't say it is gone either: off by more than the
    window it looks through, it would see the table beside the cube."""
    Pc = apply_T(inv_T(view.T_W_cam), P[None])[0]
    if Pc[2] < 0.05:
        return None
    xd, yd = distort_norm(np.array([Pc[0] / Pc[2]]), np.array([Pc[1] / Pc[2]]), view.dist, view.model)
    u, v = int(round(xd[0] * view.K[0, 0] + view.K[0, 2])), int(round(yd[0] * view.K[1, 1] + view.K[1, 2]))
    r = max(1, int(0.25 * view.K[0, 0] * SIZE / Pc[2]))             # inside the cube's outline
    h, w = view.depth.shape
    if not (r <= u < w - r and r <= v < h - r):
        return None
    d = view.depth[v - r:v + r + 1, u - r:u + r + 1]
    d = d[d > 0.05]
    if len(d) < 0.3 * (2 * r + 1) ** 2:
        return None
    dm, n = float(np.median(d)), depth_noise(Pc[2])
    if n > SIZE / 3:                                                  # far: only something well in front counts
        return "hidden" if dm < Pc[2] - 0.1 - 3 * n else None
    if dm < Pc[2] - SIZE - 3 * n:
        return "hidden"
    if Pc[2] - SIZE / 2 - 3 * n <= dm <= Pc[2] + 2 * n:              # a visible face: at most half a cube nearer
        return "there"
    if dm > Pc[2] + SIZE / 4 + 3 * n and near_sigma(view, Pc[2]) <= VAGUE_SIGMA:
        return "empty"
    return None


class Tracker:
    """Cubes over time, visual evidence only. Each track is a constant-velocity Kalman filter on the
    cube's centre (x, y, z and their rates; ACCEL: how hard a gripper can move it): at each frame
    every track is predicted to the frame's time, detections are associated with the nearest
    prediction in 3-D (within a gate sized by the prediction's and the detection's uncertainty, at
    most GATE_MAX; another colour than the track's counts half a cube further; a resting cube's
    level may read one off from a new angle: its height then jumps to the new level rather than
    blending), and each associated track is updated with its
    detection. A track is reported once seen `confirm` times (one more if vague: sigma > 1 cm). A
    confirmed track no camera detects now is asked about of every camera (`evidence`): one that sees
    past where it was drops it at once (gone); one whose depth shows it there, or something in front
    of it, keeps it (`unseen`: 'undetected' or 'occluded') for as long as that holds; with neither,
    it is kept `hold_s` after it was last seen (`held`), then dropped. An undetected track is
    reported where it was last seen. Colour is voted over the detections of the `hold_s` before it
    was last seen."""

    KEEP = ("level", "level_measured", "yaw", "visible", "confidence", "sigma", "seen_by")
    ACCEL = 0.15                    # m/s^2, the process noise: how hard a gripper accelerates a cube (discrete white-noise acceleration)
    GATE_MAX = 2 * SIZE
    V0 = 0.05                       # m/s, a new track's velocity uncertainty (most cubes rest)

    def __init__(self, confirm: int = 2, hold_s: float = 1.0):
        self.confirm, self.hold_s = confirm, hold_s
        self.tracks: list[dict] = []
        self._next = 1

    @staticmethod
    def _noise(c: dict) -> np.ndarray:
        """A detection's variance per axis: its sigma; in height, a cube a camera saw only from far
        away was put on the table (level_measured false): its height is a guess."""
        sz = c["sigma"] if c["level_measured"] else SIZE
        return np.array([c["sigma"] ** 2, c["sigma"] ** 2, sz ** 2])

    def _color(self, tr: dict) -> str | None:
        """The track's colour: its detections' votes over the `hold_s` before it was last seen."""
        score: dict = {}
        for t, name, conf in tr["votes"]:
            if tr["seen"] - t <= self.hold_s:
                score[name] = score.get(name, 0.0) + max(conf, 0.05)
        return max(score, key=score.get) if score else None

    def _predict(self, tr: dict, t: float) -> tuple[np.ndarray, np.ndarray]:
        dt = t - tr["seen"]
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        q = self.ACCEL ** 2
        Q = np.block([[np.eye(3) * q * dt ** 4 / 4, np.eye(3) * q * dt ** 3 / 2],
                      [np.eye(3) * q * dt ** 3 / 2, np.eye(3) * q * dt ** 2]])
        return F @ tr["x"], F @ tr["P"] @ F.T + Q

    def _update(self, tr: dict, c: dict, t: float) -> None:
        x, P = self._predict(tr, t)
        m, R = np.array([c["x"], c["y"], c["z"]]), np.diag(self._noise(c))
        if c["level"] is not None and tr["level"] is not None and c["level"] != tr["level"]:
            x[2], x[5] = m[2], 0.0                       # a resting cube read a level off: its height jumps
            P[2, :], P[:, 2], P[5, :], P[:, 5] = 0, 0, 0, 0
            P[2, 2], P[5, 5] = R[2, 2], self.V0 ** 2
        S = P[:3, :3] + R
        K = P[:, :3] @ np.linalg.inv(S)
        tr["x"], tr["P"] = x + K @ (m - x[:3]), (np.eye(6) - K[:, :3] @ np.eye(3, 6)) @ P

    def update(self, found: list[dict], t: float, views=()) -> list[dict]:
        pred = [self._predict(tr, t) for tr in self.tracks]
        hue = [self._color(tr) for tr in self.tracks]

        def near(i, c):
            """(cost, allowed) of detection c for track i: the distance to its prediction, half a cube
            more for another colour than the track's (a misread still matches when nothing closer does)."""
            (x, P), m = pred[i], np.array([c["x"], c["y"], c["z"]])
            dxy, dz = float(np.hypot(*(m[:2] - x[:2]))), abs(m[2] - x[2])
            gate = min(self.GATE_MAX, max(MERGE_RADIUS, 3 * np.sqrt(P[0, 0] + P[1, 1] + 2 * c["sigma"] ** 2)))
            other = c["color"] and hue[i] and c["color"] != hue[i]
            return dxy + dz + (SIZE / 2 if other else 0.0), dxy < gate and dz <= 1.2 * SIZE
        pairs = sorted((cost, i, j) for i in range(len(self.tracks)) for j, c in enumerate(found)
                       for cost, ok in [near(i, c)] if ok)
        match, used = {}, set()
        for _, i, j in pairs:                        # nearest pairs first
            if j not in match and i not in used:
                match[j] = self.tracks[i]
                used.add(i)
        for j, c in enumerate(found):
            tr = match.get(j)
            if tr is None:
                dup = any(abs(tr2["x"][2] - c["z"]) < SIZE / 2 and np.hypot(tr2["x"][0] - c["x"], tr2["x"][1] - c["y"])
                          < max(MERGE_RADIUS, 2.5 * c["sigma"]) for tr2 in match.values())
                if (c["visible"] and c["confidence"] < 0.5) or dup:
                    continue                         # weak, or a duplicate of a cube already matched
                vague = c["sigma"] > 0.01                    # e.g. seen by one wrist only: prove it longer
                tr = {"track": self._next, "hits": 0, "votes": [], "level": c["level"],
                      "x": np.array([c["x"], c["y"], c["z"], 0.0, 0.0, 0.0]),
                      "P": np.diag([*self._noise(c), *[self.V0 ** 2] * 3]),
                      "confirm": self.confirm + (1 if vague else 0)}
                self._next += 1
                self.tracks.append(tr)
                match[j] = tr
            else:
                self._update(tr, c, t)
            tr.update({k: c[k] for k in self.KEEP})
            tr["hits"] += 1
            tr["seen"] = t
            if c["color"]:
                tr["votes"].append((t, c["color"], c["confidence"]))
        kept = []
        for tr in self.tracks:                       # not detected now: ask each camera's depth why
            tr["unseen"] = None
            if tr["seen"] != t and tr["hits"] >= tr["confirm"]:
                said = {evidence(v, tr["x"][:3]) for v in views}
                if "empty" in said:
                    continue                         # a camera sees past it: gone
                if said & {"hidden", "there"}:
                    tr["unseen"] = "occluded" if "hidden" in said else "undetected"
            if t - tr["seen"] <= self.hold_s or tr["unseen"]:
                kept.append(tr)
        self.tracks = kept
        out = []
        for tr in self.tracks:
            if tr["hits"] < tr["confirm"]:
                continue
            tr["votes"] = [v for v in tr["votes"] if tr["seen"] - v[0] <= self.hold_s]
            color = self._color(tr)
            x = tr["x"]
            out.append({**{k: tr[k] for k in self.KEEP}, "color": color,
                        "x": round(float(x[0]), 4), "y": round(float(x[1]), 4), "z": round(float(x[2]), 4),
                        "vx": round(float(x[3]), 4), "vy": round(float(x[4]), 4), "vz": round(float(x[5]), 4),
                        "t": tr["seen"], "visible": tr["visible"] or color is not None,
                        "track": tr["track"], "held": t > tr["seen"], "unseen": tr["unseen"],
                        "seen_ago": round(t - tr["seen"], 2)})
        return resolve_overlaps(out)       # a held track can overlap one seen now
