"""cam_params: every registered camera's intrinsics and its pose in W, live.

Each result is stamped with the arms' read time, so a consumer can pair a frame with the pose
nearest its capture time (see Subscription.at).
"""
import time

import numpy as np

from myaloha.mods.base import Need
from myaloha.mods.localize import FIXED_CAMERAS, load_arm_model
from myaloha.topics.base import Topic


class CamParams(Topic):
    """Every registered camera's intrinsics and pose in W, live. Fixed cameras (high, low) keep the
    pose the last localize run registered. A wrist camera follows its follower: T_W_cam = T_W_mount
    (localize) · FK(joints now) · T_ee_cam (hand-eye).

    T_W_mount, the level physical mount, not T_W_base: localize's T_W_base also holds the arm's
    sag at the survey poses (3-4 deg), and the sag changes with the pose. At rest, T_W_mount puts
    the table seen by each wrist within 0.6 deg / 7 mm of level (T_W_base: 4 deg / 30 mm).
    Consumers that see the table can correct the rest (myaloha.cubes.table_fix)."""

    name = "cam_params"
    HZ = 30.0
    REQUIRES = (Need("a coordinate calibration (run localize)", (("coordinates", "", ("coordinates",)),)),)
    OUTPUT = ("cameras: {key: {K (3x3), dist (k1 k2 p1 p2 k3), model, size [w, h], T_W_cam (4x4, metres) "
              "or null with reason, source: 'localize' (fixed) | 'arm' (wrist, from joints), speed (max joint "
              "rad/s, wrists)}}; grippers: {side: {T_W_ee (tool point, 4x4), aperture (finger gap, m)}}; "
              "coordinates: calibration id; t: robot clock of the joint reading")

    def setup(self):
        self.kin, self.T_ee_cam = load_arm_model()
        self.doc, self.doc_checked, self.optics = None, 0.0, {}

    def _coordinates(self):
        if time.monotonic() - self.doc_checked > 2.0:     # a new localize run replaces it
            self.doc_checked = time.monotonic()
            r = self.http.get("/api/coordinates")
            self.doc = r.json() if r.status_code == 200 else None
        return self.doc

    def _optics(self, key):
        if key not in self.optics:
            r = self.http.get(f"/api/cameras/{key}/optics")
            if r.status_code != 200:
                return None
            self.optics[key] = r.json()
        return self.optics[key]

    def step(self):
        doc = self._coordinates()
        if not doc:
            self.wait("no coordinate calibration; run localize")
            return None
        arms = self.get("/arms")
        cams, grippers, t_arms = {}, {}, []
        for key in FIXED_CAMERAS:
            reg = (doc.get("cameras") or {}).get(key)
            if reg:
                cams[key] = {"K": reg["K"], "dist": reg["dist"], "model": reg["model"], "size": reg["size"],
                             "T_W_cam": reg["T_W_cam"], "source": "localize"}
            else:
                o = self._optics(key) or {}
                cams[key] = {**o, "T_W_cam": None, "source": "localize",
                             "reason": "not registered by the last localize run"}
        for side in self.T_ee_cam:
            key, arm_name = f"wrist_{side}", f"follower_{side}"
            o = self._optics(key)
            a = arms.get(arm_name, {})
            entry = {**(o or {}), "source": "arm", "T_W_cam": None}
            if o is None:
                entry["reason"] = f"{key} camera not connected"
            elif not a.get("connected") or a.get("lost") or a.get("pos") is None:
                entry["reason"] = f"{arm_name} not connected"
            else:
                q = np.array(a["pos"][:6])
                fk = self.kin.fk(q)
                T = np.array(doc["T_W_mount"][side]) @ fk @ self.T_ee_cam[side]
                # the tool point in the frame mods command in (localize's T_W_base: sag at work poses)
                grippers[side] = {"T_W_ee": np.round(np.array(doc["T_W_base"][side]) @ fk, 5).tolist(),
                                  "aperture": round(2 * a["pos"][6], 4)}
                entry.update({"T_W_cam": np.round(T, 6).tolist(), "t": a["t"],
                              "speed": round(float(np.max(np.abs(a["vel"][:6]))), 4)})
                t_arms.append(a["t"])
            cams[key] = entry
        return {"t": max(t_arms) if t_arms else time.perf_counter(), "coordinates": doc["id"], "cameras": cams,
                "grippers": grippers}
