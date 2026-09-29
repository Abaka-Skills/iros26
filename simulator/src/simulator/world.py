"""The simulated world: the table, its cubes, the followers, and what each camera sees.

Everything here is truth. What leaves the simulator is only what the real devices report
(controller readings, camera frames and their factory intrinsics), each off by a little (see the
`errors` in rig.yaml): joints have zero offsets and droop the encoders don't see, readings jitter,
wrist cameras sit off their nominal mount, intrinsics differ from the reported ones, and frames
carry pixel, exposure, tint and depth noise.

Physics is MuJoCo's (scene.py), stepped in real time by a thread: the cubes are rigid bodies that
fall, slide, topple and stack; the arms push them and each other; the fingers grip them by friction,
stopping where a cube is. Each follower's servos are driven to where its links really are (its motor
targets plus zero offsets and sag); its readings are the motor targets plus how far the links are
from them (a blocked arm reads where it is), and its fingers read where they are.
"""
from __future__ import annotations

import math
import threading
import time

import cv2
import mujoco
import numpy as np

from simulator.geometry import inv_T, make_T, rodrigues, rot
from simulator.scene import MAX_CUBES, Scene

NEAR, FAR = 0.07, 3.0        # D405 depth range (m)
BOOT = [0.0, 0.05, 0.02, 0.0, 0.0, 0.0, 0.0]    # a follower at power-up: resting on its stops, fingers shut


def _pose(x, y, z, yaw_deg=0.0) -> np.ndarray:
    return make_T(rot([0, 0, 1], yaw_deg), (x, y, z))


def _jolt(T: np.ndarray, rng, mm: float, deg: float) -> np.ndarray:
    """T moved by a random translation (mm per axis) and rotation (deg) of those sizes."""
    return make_T(rodrigues(rng.normal(0, math.radians(deg), 3)), rng.normal(0, mm / 1000, 3)) @ T


def _depth_K(K: np.ndarray, f: float) -> np.ndarray:
    """K with its focal lengths scaled by f about the same principal point."""
    out = K.copy()
    out[0, 0] *= f
    out[1, 1] *= f
    return out


def _K(fx, fy, cx, cy) -> np.ndarray:
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])


class Cube:
    def __init__(self, color: str, albedo: np.ndarray):
        self.color, self.albedo = color, albedo


class World:
    def __init__(self, cfg: dict, realtime: bool = True):
        """`realtime`: a thread steps the physics with the clock; without it, step() does (tests)."""
        seed = cfg["seed"] if cfg["seed"] is not None else int(np.random.SeedSequence().entropy % 1_000_000)
        self.seed, self.cfg = seed, cfg
        self.rng = rng = np.random.default_rng(seed)
        e = {k: v * cfg["noise"] for k, v in cfg["errors"].items()}
        self.err = e
        self.S = cfg["cubes"]["size"]
        self.size = tuple(cfg["size"])
        self.lock = threading.RLock()            # the physics state; never held while taking a controller's lock

        # ---- followers: mount, joint zero offsets; where the shoulder sags ----
        self.mount, self.offset = {}, {}
        for ip, a in cfg["arms"].items():
            if a["role"] == "follower":
                self.mount[ip] = _pose(*a["mount"])
                self.offset[ip] = rng.normal(0, math.radians(e["joint_offset"]), 6)

        # ---- cameras: true K and pose; what each reports; its sensor quirks ----
        self.cams = {}
        for serial, c in cfg["cameras"].items():
            cam = {"K": _K(*c["K"])}
            if "arm" in c:
                cam["arm"] = c["arm"]
                cam["T_ee_cam"] = _jolt(np.array(c["T_ee_cam"], float), rng, e["handeye_mm"], e["handeye_deg"])
            else:
                cam["T_W_cam"] = np.array(c["T_W_cam"], float)
            cam["reported_K"] = cam["K"] + np.diag([1, 1, 0]) * cam["K"][0, 0] * rng.normal(0, e["focal"])
            cam["reported_K"][:2, 2] += rng.normal(0, e["centre_px"], 2)
            # float32 throughout: per-frame work stays single precision
            cam["gain"] = ((1 + rng.normal(0, e["exposure"])) * (1 + rng.normal(0, e["tint"], 3))).astype(np.float32)
            cam["depth_scale"] = np.float32(1 + rng.normal(0, e["depth_scale"]))
            cam["range_bias"] = np.float32(abs(rng.normal(0, e["range_bias"])) if "arm" in c else 0.0)
            cam["bf"] = np.float32(cfg["sensor"]["baseline"] * cam["K"][0, 0])   # baseline x focal (px m)
            f = cfg["sensor"]["depth_focal"]                       # the depth stream: same centre, own focal length
            cam["K_depth"], cam["reported_K_depth"] = _depth_K(cam["K"], f), _depth_K(cam["reported_K"], f)
            u, v = np.meshgrid(np.arange(self.size[0], dtype=np.float32), np.arange(self.size[1], dtype=np.float32))
            Kc, Kd = cam["K"], cam["K_depth"]                      # colour pixel under each depth-stream pixel
            cam["depth_map"] = (((u - Kd[0, 2]) / Kd[0, 0] * Kc[0, 0] + Kc[0, 2]).astype(np.float32),
                                ((v - Kd[1, 2]) / Kd[1, 1] * Kc[1, 1] + Kc[1, 2]).astype(np.float32))
            self.cams[serial] = cam

        self.controllers: dict = {}              # follower IP -> ArmController (motor targets)
        self.cubes: list[Cube] = []
        self.scene = sc = Scene(cfg, table_texture(cfg["table"], cfg["_dir"], rng))
        ip = next(iter(self.mount))                     # the shoulder's sag direction: the tool goes down
        q = np.zeros(6)
        q[1] = 1e-3
        self.droop_sign = -np.sign(sc.tool_pose(ip, q)[2, 3] - sc.tool_pose(ip, np.zeros(6))[2, 3])
        w, h = self.size
        self._noise_bank = rng.normal(0, 1, (h + 64, w + 64)).astype(np.float32)
        self._grain = self._noise_bank[..., None] * np.array([1, -0.7, 0.4], np.float32) * np.float32(e["pixel"])
        blotch = cv2.GaussianBlur(rng.normal(0, 1, (h + 64, w + 64)).astype(np.float32), (0, 0), cfg["sensor"]["correlation_px"])
        self._smooth_bank = blotch / blotch.std()                     # unit sigma, correlated over ~correlation_px

        # ---- physics: the followers at power-up, then the cubes ----
        self.phys = mujoco.MjData(sc.model)
        self._target: dict[str, np.ndarray] = {}         # follower IP -> the link angles its servos hold
        for ip in sc.followers:
            self._drive(ip, BOOT)
            self.phys.qpos[sc.qadr[ip]] = self._target[ip]
        self.spawn(cfg["cubes"]["scene"] or cfg["cubes"]["count"])
        self._stop = threading.Event()
        if realtime:
            threading.Thread(target=self._run, name="physics", daemon=True).start()

    # ---- physics ----
    def _drive(self, ip: str, motor) -> None:
        """Aim follower ip's servos: its links where motor targets put them, its fingers at motor[6]."""
        q = self.link_q(ip, motor)
        a = self.scene.act[ip]
        with self.lock:
            self._target[ip] = q
            self.phys.ctrl[a[:6]] = q
            self.phys.ctrl[a[6:]] = motor[6]

    def step(self, seconds: float) -> None:
        """Advance the physics `seconds`, the servos aimed at the controllers' targets now."""
        for ip, ctl in list(self.controllers.items()):
            self._drive(ip, ctl.motor())                  # a controller's lock, then the world's: never the other way
        with self.lock:
            for _ in range(max(1, round(seconds / self.scene.model.opt.timestep))):
                mujoco.mj_step(self.scene.model, self.phys)

    def _run(self) -> None:
        """Keep the physics with the clock (a backlog of more than 0.1 s is dropped, not caught up)."""
        dt, t_sim = self.scene.model.opt.timestep, time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            behind = min(now - t_sim, 0.1)
            if behind >= dt:
                self.step(behind)
            t_sim = now
            time.sleep(0.002)

    def stop(self) -> None:
        self._stop.set()

    # ---- scene ----
    def spawn(self, scene, seed: int | None = None) -> None:
        """Replace the cubes: a count scatters that many over the reachable table (at least one of
        each colour while there are enough, one two-cube stack, sometimes a three-cube one); a list
        places them as given ([{color, x, y, yaw (deg), level, z (optional: its centre's height; a
        cube above what is under it falls)}]). Cubes beyond MAX_CUBES are left out."""
        if seed is not None:
            self.seed, self.rng = seed, np.random.default_rng(seed)
        rng, S, sc = self.rng, self.S, self.scene
        names = list(self.cfg["cubes"]["colors"])
        if isinstance(scene, int):
            scene = self._scatter(scene, names)
        scene = list(scene)[:MAX_CUBES]
        cubes = []
        for c in scene:
            rgb = np.array(self.cfg["cubes"]["colors"][c["color"]], np.uint8)
            lab = cv2.cvtColor(rgb[None, None, ::-1].copy(), cv2.COLOR_BGR2Lab)[0, 0].astype(float)
            lab += rng.normal(0, self.err["albedo"], 3)
            albedo = cv2.cvtColor(np.clip(lab, 0, 255).astype(np.uint8)[None, None], cv2.COLOR_Lab2BGR)[0, 0]
            cubes.append(Cube(c["color"], albedo.astype(np.float32)))
        with self.lock:
            d, m = self.phys, sc.model
            for i in range(MAX_CUBES):
                a, v = sc.cube_qadr[i], sc.cube_dadr[i]
                if i < len(scene):
                    c = scene[i]
                    z = c.get("z", (c.get("level", 0) + 0.5) * S)
                    d.qpos[a:a + 3] = c["x"], c["y"], z
                    d.qpos[a + 3:a + 7] = [math.cos(math.radians(c.get("yaw", 0.0)) / 2), 0.0, 0.0,
                                           math.sin(math.radians(c.get("yaw", 0.0)) / 2)]
                    m.geom_rgba[sc.cube_geom[i]] = [*(cubes[i].albedo[::-1] / 255.0), 1.0]
                else:
                    d.qpos[a:a + 3], d.qpos[a + 3:a + 7] = sc.parking[i], [1.0, 0.0, 0.0, 0.0]
                d.qvel[v:v + 6] = 0.0
            mujoco.mj_forward(m, d)
            self.cubes = cubes

    def _scatter(self, n: int, names: list) -> list[dict]:
        rng = self.rng
        colors = [str(c) for c in (*rng.permutation(names)[:n], *rng.choice(names, max(0, n - len(names))))]
        heights = [1] * n
        if n >= 3:
            heights[0] = 2
        if n >= 6 and rng.random() < 0.5:
            heights[1] = 3
        out, spots = [], []
        for h in heights:
            if len(colors) < h:
                break
            for _ in range(500):
                x, y = rng.uniform(-0.28, 0.28), rng.uniform(-0.2, 0.2)
                r = min(math.hypot(x - T[0, 3], y - T[1, 3]) for T in self.mount.values())
                if 0.19 < r < 0.37 and all(math.hypot(x - a, y - b) > 0.055 for a, b in spots):
                    break
            else:
                continue
            spots.append((x, y))
            yaw = rng.uniform(0, 90)
            for lv in range(h):
                out.append({"color": colors.pop(), "x": x + rng.normal(0, 0.001), "y": y + rng.normal(0, 0.001),
                            "yaw": yaw + rng.normal(0, 1.5), "level": lv})
        return out

    def truth(self) -> dict:
        """Every cube and each follower's tool point as they really are. A cube touched by both of a
        gripper's pads is held by it (level -1); `tilt` is how far its nearest axis is off vertical."""
        sc, names = self.scene, {ip: self.cfg["arms"][ip]["name"] for ip in self.scene.followers}
        with self.lock:
            d = self.phys
            touch: dict[int, dict[str, set]] = {}
            for con in d.contact[:d.ncon]:
                for g, other in ((con.geom1, con.geom2), (con.geom2, con.geom1)):
                    for ip, pads in sc.pad_geoms.items():
                        if other in pads:
                            touch.setdefault(g, {}).setdefault(ip, set()).add(other)
            cubes = []
            for i, c in enumerate(self.cubes):
                a = sc.cube_qadr[i]
                p, R = d.qpos[a:a + 3], np.zeros(9)
                mujoco.mju_quat2Mat(R, d.qpos[a + 3:a + 7])
                R = R.reshape(3, 3)
                up = int(np.argmax(np.abs(R[2])))                            # the axis nearest vertical
                side = R[:, (up + 1) % 3]
                held = next((names[ip] for ip, pads in touch.get(sc.cube_geom[i], {}).items() if pads == sc.pad_geoms[ip]), None)
                cubes.append({"color": c.color, "x": round(float(p[0]), 4), "y": round(float(p[1]), 4),
                              "z": round(float(p[2]), 4), "yaw": round(math.degrees(math.atan2(side[1], side[0])) % 90, 1),
                              "tilt": round(math.degrees(math.acos(min(1.0, abs(R[2, up])))), 1),
                              "level": -1 if held else int(round(p[2] / self.S - 0.5)), "held_by": held})
            tools = {names[ip]: np.round(self._tool(ip)[:3, 3], 4).tolist() for ip in sc.followers}
        return {"seed": self.seed, "cubes": cubes, "tools": tools}

    # ---- arms ----
    def link_q(self, ip: str, motor) -> np.ndarray:
        """Where motor angles put the links: plus each joint's zero offset and gravity sag, growing
        with reach."""
        q = np.array(motor[:6], float) + self.offset[ip]
        reach = np.hypot(*(inv_T(self.mount[ip]) @ self.scene.tool_pose(ip, q))[:2, 3])
        q[1] += self.droop_sign * self.err["droop"] * float(reach)
        return q

    def _tool(self, ip: str) -> np.ndarray:
        """Follower ip's tool frame in W as the physics has it now (call with the lock held)."""
        b = self.scene.tool_body[ip]
        return make_T(self.phys.xmat[b].reshape(3, 3).copy(), self.phys.xpos[b].copy())

    def measured(self, ip: str, motor: list[float]) -> list[float]:
        """What follower ip's encoders see with motor targets `motor`: the targets plus how far its links
        are from where they should be (0 unless something blocks them), and where its fingers are."""
        with self.lock:
            q = self.phys.qpos[self.scene.qadr[ip]]
            off = q - self._target.get(ip, q)
            return [*(np.asarray(motor[:6], float) + off).tolist(), float(self.phys.qpos[self.scene.fingers[ip][0]])]

    def reading(self, pos: list[float]) -> list[float]:
        """What a controller reports for positions `pos`: with its jitter."""
        e = self.err
        jitter = np.append(self.rng.normal(0, math.radians(e["joint_jitter"]), 6), self.rng.normal(0, e["gripper_jitter"]))
        return (np.array(pos) + jitter).tolist()

    def efforts(self, ip: str) -> tuple[list[float], list[float]]:
        """What follower ip's motors exert, as its driver reports it: the total (N m; the fingers N) and the
        external part, beyond the driver's model. Gravity is compensated, so what the servos add on top of
        it is what contacts (and accelerations) take: a cube set down on something shows here."""
        with self.lock:
            ext = self.phys.actuator_force[self.scene.act[ip][:7]].copy()
            grav = np.append(self.phys.qfrc_gravcomp[self.scene.dadr[ip]], 0.0)
        ext += self.rng.normal(0, self.err["effort_jitter"], 7)
        return (ext + grav).tolist(), ext.tolist()

    # ---- cameras ----
    def optics(self, serial: str) -> dict:
        """What the camera reports about itself (factory intrinsics)."""
        c, aligned = self.cams[serial], not self.cfg["sensor"]["artifacts"]
        return {"K_color": c["reported_K"], "K_depth": c["reported_K"] if aligned else c["reported_K_depth"],
                "T_color_depth": np.eye(4), "aligned": aligned,
                "dist_color": np.zeros(5), "model_color": "brown_conrady"}

    def frame(self, serial: str) -> tuple[np.ndarray, np.ndarray] | None:
        """(BGR uint8, depth in 0.1 mm units (uint16, H x W x 1)) as the camera sees the world now;
        None for a camera on an arm that is unplugged."""
        c = self.cams[serial]
        if "arm" in c and not getattr(self.controllers.get(c["arm"]), "plugged", False):
            return None
        with self.lock:
            T_W_cam = self._tool(c["arm"]) @ c["T_ee_cam"] if "arm" in c else c["T_W_cam"]
            qpos = self.phys.qpos.copy()
        return self._render(serial, T_W_cam, qpos)

    def _render(self, serial, T_W_cam, qpos):
        c, rng = self.cams[serial], self.rng
        w, h = self.size
        color, depth = self.scene.render(serial, T_W_cam, qpos)
        # the sensor: blur, exposure and tint, pixel noise; depth noise, scale, range bias, dropouts
        img = cv2.GaussianBlur(color, (3, 3), 0.7).astype(np.float32)
        img *= c["gain"]
        dx, dy = rng.integers(0, 64, 2)
        img += self._grain[dy:dy + h, dx:dx + w]
        color = np.clip(img, 0, 255, out=img).astype(np.uint8)
        return color, self._stereo(c, depth, color)

    def _stereo(self, c: dict, z: np.ndarray, color: np.ndarray) -> np.ndarray:
        """The D405's depth for true depth `z` (m, colour camera): passive stereo from the left imager,
        so its errors are in disparity. Blotchy disparity noise (error grows with z^2); no depth where
        the right imager can't see (the left band, shadows left of near objects); holes on dark,
        textureless surfaces; lost edges and random dropouts; per-camera scale and range bias. Sent in
        the depth stream's own intrinsics, which the receiver re-aligns to colour. Depth in 0.1 mm."""
        e, rng, artifacts = self.err, self.rng, self.cfg["sensor"]["artifacts"]
        h, w = z.shape
        bf = c["bf"]
        with np.errstate(divide="ignore"):
            d = np.where(z > 0, bf / z, 0).astype(np.float32)               # disparity (px)
        dy, dx = rng.integers(0, 64, 2)
        blotch = self._smooth_bank[dy:dy + h, dx:dx + w]
        dy, dx = rng.integers(0, 64, 2)
        d_noisy = d + np.float32(e["depth_subpixel"]) * (0.9 * blotch + 0.45 * self._noise_bank[dy:dy + h, dx:dx + w])
        with np.errstate(divide="ignore", invalid="ignore"):
            out = np.where(d_noisy > 0.5, bf / d_noisy, 0).astype(np.float32)
        out *= c["depth_scale"] + c["range_bias"] * np.clip((out - 0.4) / 0.2, 0, 1)
        drop = (z < NEAR) | (z > FAR) | (rng.random((h, w), dtype=np.float32) < e["dropout"])
        edges = cv2.morphologyEx(z, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0.01
        drop[edges] |= rng.random(int(edges.sum()), dtype=np.float32) < e["edge_drop"]
        if artifacts:
            drop |= self._unmatched(d, color, blotch)
        out[drop] = 0
        if artifacts:                        # sent in the depth stream's own intrinsics: the receiver re-aligns it
            out = cv2.remap(out, *c["depth_map"], cv2.INTER_NEAREST, borderValue=0)
        return np.clip(out * 1e4, 0, 65535).astype(np.uint16)[..., None]

    def _unmatched(self, d: np.ndarray, color: np.ndarray, blotch: np.ndarray) -> np.ndarray:
        """Where stereo finds no match: pixels the right imager can't see (it sees pixel u at u - d:
        off its image when u < d, hidden where a nearer surface to the right covers it), and dark,
        flat patches, in blobs."""
        h, w = d.shape
        excess = d - np.arange(w, dtype=np.float32)
        ahead = np.maximum.accumulate(excess[:, ::-1], axis=1)[:, ::-1]
        ahead = np.concatenate([ahead[:, 1:], np.full((h, 1), -np.inf, np.float32)], 1)
        grey = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY).astype(np.float32)
        texture = cv2.GaussianBlur(np.abs(cv2.Laplacian(grey, cv2.CV_32F)), (0, 0), 3)
        flat_dark = np.clip(1 - grey / 50, 0, 1) * np.clip(1 - texture / 6, 0, 1)
        return (excess < ahead - 0.5) | (excess > 0) | (flat_dark * self.err["texture_holes"] > 0.5 + 0.25 * blotch)


def table_texture(cfg: dict, root, rng) -> np.ndarray:
    """The table top as an image (BGR, 1 mm per pixel, row 0 at the front edge): procedural wood or
    a photo, with the printed sheet at the origin."""
    x0, y0, x1, y1 = cfg["extent"]
    res = 0.001
    size = (int(round((x1 - x0) / res)), int(round((y1 - y0) / res)))
    if cfg["texture"] == "procedural":
        tex = _wood(size, rng)
    else:
        tex = cv2.resize(cv2.imread(str(root / cfg["texture"])), size, interpolation=cv2.INTER_AREA)
    if cfg.get("sheet"):
        _sheet(tex, cfg["sheet"], (-x0 / res, -y0 / res), res, rng)
    return tex


def _wood(size, rng) -> np.ndarray:
    """Light wood: grain along x, blotches at several scales, knots, and scattered marks and specks
    (a used table top: plenty for SIFT at the wrist cameras' ~1 mm/px)."""
    w, h = size

    def smooth(n_x, n_y, amp):
        return amp * cv2.resize(rng.normal(0, 1, (n_y, n_x)), (w, h), interpolation=cv2.INTER_CUBIC)

    yy = np.arange(h, dtype=np.float32)[:, None]
    warp = smooth(12, 8, 25) + smooth(60, 30, 4)
    L = 158 + smooth(16, 9, 5) + smooth(80, 45, 3) + smooth(400, 225, 2)
    L += 4 * np.sin((yy + warp) * 0.35) + 2 * np.sin((yy + warp) * 1.3)
    L = L.astype(np.float32)
    for _ in range(40):                                         # knots
        cx, cy, r = rng.integers(0, w), rng.integers(0, h), rng.uniform(4, 14)
        cv2.ellipse(L, (int(cx), int(cy)), (int(r * 2.5), int(r)), 0, 0, 360, float(L[cy, cx] - rng.uniform(15, 35)), -1)
    for _ in range(900):                                        # marks: scratches, dots, blots
        cx, cy = int(rng.integers(0, w)), int(rng.integers(0, h))
        v = float(np.clip(L[cy, cx] + rng.choice([-1, 1]) * rng.uniform(20, 60), 30, 250))
        kind = rng.integers(3)
        if kind == 0:
            a = rng.uniform(0, np.pi)
            dx, dy = np.cos(a) * rng.uniform(5, 40), np.sin(a) * rng.uniform(5, 40)
            cv2.line(L, (cx, cy), (int(cx + dx), int(cy + dy)), v, int(rng.integers(1, 3)))
        elif kind == 1:
            cv2.circle(L, (cx, cy), int(rng.integers(2, 7)), v, -1)
        else:
            pts = (np.array([cx, cy]) + rng.normal(0, 6, (5, 2))).astype(np.int32)
            cv2.fillConvexPoly(L, cv2.convexHull(pts), v)
    L = cv2.GaussianBlur(L, (3, 3), 0.8)
    lab = np.stack([L, np.full_like(L, 131), np.full_like(L, 136)], -1)
    return cv2.cvtColor(np.clip(lab, 0, 255).astype(np.uint8), cv2.COLOR_Lab2BGR)


def _sheet(tex, size, centre, res, rng) -> None:
    """A white printed sheet with black square codes (tag-like) at `centre` (texel coordinates)."""
    sw, sh = int(size[0] / res), int(size[1] / res)
    x0, y0 = int(centre[0] - sw / 2), int(centre[1] - sh / 2)
    sheet = np.full((sh, sw, 3), 235, np.uint8)
    cell = int(0.05 / res)
    for i in range(3):
        for j in range(2 if sh < sw else 4):
            code = rng.random((6, 6)) < 0.5
            code[0, :] = code[-1, :] = code[:, 0] = code[:, -1] = True
            tag = np.kron(code, np.ones((cell // 6, cell // 6), bool))
            top, left = 20 + j * (cell + 20), 15 + i * (cell + 15)
            if top + tag.shape[0] <= sh and left + tag.shape[1] <= sw:
                sheet[top:top + tag.shape[0], left:left + tag.shape[1]][tag] = 25
    tex[y0:y0 + sh, x0:x0 + sw] = sheet
