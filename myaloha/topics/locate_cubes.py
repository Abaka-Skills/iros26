from concurrent.futures import ThreadPoolExecutor

import numpy as np

from myaloha import cubes
from myaloha.mods.base import Need
from myaloha.topics.base import Topic

MAX_SPEED = 0.1        # rad/s: a wrist view taken faster than this is blurred and its pose uncertain
MAX_POSE_GAP = 0.05    # s between a wrist frame and the joint reading paired with it


class LocateCubes(Topic):
    """Every cube on the table (2.5 cm; red, orange, yellow, green, blue, purple), in W, fused
    from every registered camera: the overhead camera finds the tops, the wrist and low cameras
    measure stack levels and see the sides of stacked cubes. A cube under a stack that no camera
    sees is reported as hidden (no colour). Wrist views are used only while their arm is still.
    Tracked over time: a cube shows after two detections and stays up to 1 s after its last."""

    name = "locate_cubes"
    DEPENDS = ("cam_params",)
    HZ = 3.0
    REQUIRES = (Need("the high camera", (("high", "", ("high",)),)),
                Need("a coordinate calibration (run localize)", (("coordinates", "", ("coordinates",)),)))
    OUTPUT = ("cubes: [{color (null: hidden under a stack), x, y, z (cube centre in W, m), level (0 = on "
              "the table), level_measured, yaw (deg mod 90), visible, confidence (0-1), sigma (expected position "
              "error, m), seen_by, track (id, "
              "stable while the cube stays), held (not seen this frame; kept up to 1 s)}]; count, hidden; "
              "cameras: per camera, what it contributed or why not; coordinates: calibration id; "
              "t: robot clock of the frames")

    def setup(self):
        self.cam_params = self.subscribe("cam_params")
        self.pool = ThreadPoolExecutor(4)
        self.workspace, self.coord_id = None, None
        self.tracker = cubes.Tracker()

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
        cams, info = {}, {}
        for k, c in cp["data"]["cameras"].items():
            if c["T_W_cam"] is None:
                continue
            if c["source"] == "arm" and c["speed"] > MAX_SPEED:        # don't fetch a blurred view
                info[k] = {"skipped": f"arm moving ({c['speed']:.2f} rad/s)"}
            else:
                cams[k] = c
        frames = dict(zip(cams, self.pool.map(self._frame, cams)))
        views = []
        for key, c in cams.items():
            f = frames[key]
            if isinstance(f, Exception):
                info[key] = {"skipped": f"no frame: {f}"}
                continue
            if c["source"] == "arm":       # the pose at the frame's capture time
                m = self.cam_params.at(f.t)
                c = m and m["data"]["cameras"][key]
                if not c or c["T_W_cam"] is None or abs(m["t"] - f.t) > MAX_POSE_GAP:
                    info[key] = {"skipped": "no joint reading near the frame time"}
                    continue
                if c["speed"] > MAX_SPEED:
                    info[key] = {"skipped": f"arm moving ({c['speed']:.2f} rad/s)"}
                    continue
            views.append(cubes.CameraView(key, f.color, f.depth, f.K, f.dist, f.model, np.array(c["T_W_cam"]),
                                          fixed=c["source"] == "localize"))
        if not any(v.key == "high" for v in views):
            self.wait("no frame from the high camera")
            return None
        found, used = cubes.locate(views, self._workspace(cp["data"]["coordinates"]), self.pool)
        t = max(frames[v.key].t for v in views)
        found = self.tracker.update(found, t)
        return {"t": t, "cubes": found,
                "count": sum(c["visible"] for c in found), "hidden": sum(not c["visible"] for c in found),
                "cameras": {**info, **used}, "coordinates": cp["data"]["coordinates"]}

    def _frame(self, key):
        try:
            return self.frame(key)
        except Exception as e:
            return e
