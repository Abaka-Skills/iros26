from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

from myaloha import cubes
from myaloha.coords import make_T
from myaloha.gripper import finger_mask
from myaloha.mods.base import Need
from myaloha.mods.localize import load_arm_model
from myaloha.topics.base import Topic

MAX_POSE_GAP = 0.05    # s between a wrist frame and the joint readings its pose is interpolated from


class LocateCubes(Topic):
    """Every cube on the table (2.5 cm; red, orange, yellow, green, blue, purple), in W, fused
    from every registered camera: the overhead camera finds the tops, the wrist and low cameras
    measure stack levels and see the sides of stacked cubes. A cube under a stack that no camera
    sees is reported as hidden (no colour). Every camera counts, a moving wrist too (its pose at the
    frame's capture time, from the joint readings around it; the D405's colour sensor has a global
    shutter). A cube in a gripper is in the air: reported with its height and no level. Tracked over
    time with a Kalman filter per cube (position and velocity, at the frames' time `t`)."""

    name = "locate_cubes"
    DEPENDS = ("cam_params",)
    HZ = 3.0
    REQUIRES = (Need("the high camera", (("high", "", ("high",)),)),
                Need("a coordinate calibration (run localize)", (("coordinates", "", ("coordinates",)),)))
    OUTPUT = ("cubes: [{color (null: hidden under a stack), x, y, z (cube centre in W, m; filtered), vx, vy, vz "
              "(its velocity, m/s), t (the robot time of that state), level (0 = on the table; null: in the air, "
              "e.g. in a gripper), level_measured, yaw (deg mod 90), visible, confidence (0-1), sigma (expected "
              "position error of the detection, m), seen_by, track (id, "
              "stable while the cube stays), held (not detected this frame), unseen (null, or why it is kept undetected: "
              "'occluded' (a camera's depth shows something in front of it) or 'undetected' (its depth shows the "
              "cube there)), seen_ago (s since last detected)}]: only what the cameras see, nothing inferred from "
              "the arms; a camera that sees past where a cube was drops it at once; "
              "count, hidden; "
              "cameras: per camera, what it contributed or why not; coordinates: calibration id; "
              "t: robot clock of the frames")

    def setup(self):
        self.cam_params = self.subscribe("cam_params")
        self.pool = ThreadPoolExecutor(4)
        self.workspace, self.coord_id = None, None
        self.tracker = cubes.Tracker()
        self.T_ee_cam = load_arm_model()[1]
        cubes.use_profile(self.get("/colors"))           # the backend's colour profile

    def _workspace(self, coord_id):
        """Half extents where cubes can be: between the follower bases, 37 cm either side of their line
        (the lab table's front edge is at about -0.39 m; the rails beyond flicker as cubes)."""
        if coord_id != self.coord_id:
            doc = self.get("/coordinates")
            x = min(abs(np.array(T)[0, 3]) for T in doc["T_W_mount"].values())
            self.workspace, self.coord_id = (x - 0.02, 0.37), coord_id
        return self.workspace

    def step(self):
        cp = self.cam_params.latest
        if cp is None:
            self.wait("waiting on cam_params")
            return None
        cams = {k: c for k, c in cp["data"]["cameras"].items() if c["T_W_cam"] is not None}
        info = {}
        frames = dict(zip(cams, self.pool.map(self._frame, cams)))
        views = []
        for key, c in cams.items():
            f = frames[key]
            if isinstance(f, Exception):
                info[key] = {"skipped": f"no frame: {f}"}
                continue
            T, mask = np.array(c["T_W_cam"]), None
            if c["source"] == "arm":       # the pose at the frame's capture time, and its own fingers masked
                T = self._pose_at(key, f.t)
                if T is None:
                    info[key] = {"skipped": "no joint readings around the frame time"}
                    continue
                side = key.removeprefix("wrist_")
                m = self.cam_params.at(f.t)
                g = m and m["data"].get("grippers", {}).get(side)
                if g:
                    mask = finger_mask(self.T_ee_cam[side], f.K, f.dist, f.model, f.color.shape[1::-1], g["aperture"])
            views.append(cubes.CameraView(key, f.color, f.depth, f.K, f.dist, f.model, T,
                                          fixed=c["source"] == "localize", mask=mask))
        if not any(v.key == "high" for v in views):
            self.wait("no frame from the high camera")
            return None
        found, used = cubes.locate(views, self._workspace(cp["data"]["coordinates"]), self.pool)
        t = max(frames[v.key].t for v in views)
        found = self.tracker.update(found, t, views)
        return {"t": t, "cubes": found,
                "count": sum(c["visible"] for c in found), "hidden": sum(not c["visible"] for c in found),
                "cameras": {**info, **used}, "coordinates": cp["data"]["coordinates"]}

    def _pose_at(self, key: str, t: float) -> np.ndarray | None:
        """A wrist camera's pose at robot time t: interpolated between the joint readings (cam_params
        results) just before and after it, or the nearest one within MAX_POSE_GAP."""
        a, b = self.cam_params.around(t)
        near = [m for m in (a, b) if m and abs(m["t"] - t) <= MAX_POSE_GAP
                and m["data"]["cameras"].get(key, {}).get("T_W_cam") is not None]
        if not near:
            return None
        if len(near) == 1:
            return np.array(near[0]["data"]["cameras"][key]["T_W_cam"])
        Ta, Tb = (np.array(m["data"]["cameras"][key]["T_W_cam"]) for m in near)
        s = (t - a["t"]) / max(b["t"] - a["t"], 1e-9)
        dR = cv2.Rodrigues(Ta[:3, :3].T @ Tb[:3, :3])[0]
        return make_T(Ta[:3, :3] @ cv2.Rodrigues(dR * s)[0], Ta[:3, 3] + s * (Tb[:3, 3] - Ta[:3, 3]))

    def _frame(self, key):
        try:
            return self.frame(key)
        except Exception as e:
            return e
