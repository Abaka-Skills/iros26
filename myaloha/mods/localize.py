import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from myaloha import coords
from myaloha.coords import inv_T
from myaloha.mods.base import Mod, Need
from myaloha.vendor.heron_kinematics import Kinematics, rotvec

ROOT = Path(__file__).resolve().parents[2]
CALIB = ROOT / "calib" / "coordinates"
FRAMES = ROOT / "logs" / "coordinates"
TABLE_Z = -0.025          # table below the base-frame origin (the lab measured 22-28 mm)
JOINT_MARGIN = np.radians(8)
CLEARANCE = 0.05          # every joint origin past the shoulder stays this high above the table
GRIPPER_OPEN = 0.04       # carriage position (m): fingers apart, so they hide less of the view


def load_arm_model():
    """(follower kinematics, {side: T_ee_cam}): the URDF and each wrist camera's hand-eye pose."""
    kin = Kinematics(ROOT / "myaloha" / "vendor" / "wxai_follower.urdf")
    T_ee_cam = {a: np.array(yaml.safe_load((ROOT / "config" / "handeye" / f"{a}.yaml").read_text())["T_ee_cam"])
                for a in coords.ARMS}
    return kin, T_ee_cam


def load_latest() -> dict | None:
    p = CALIB / "latest.yaml"
    return yaml.safe_load(p.read_text()) if p.exists() else None


def plain(x):
    if isinstance(x, dict):
        return {k: plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return plain(x.tolist())
    if isinstance(x, np.generic):
        return x.item()
    return x


FIXED_CAMERAS = ("high", "low")   # registered in W at the end of each run, from its wrist views


def save_doc(robot, doc: dict) -> None:
    text = yaml.safe_dump(plain(doc), default_flow_style=None, sort_keys=False)
    CALIB.mkdir(parents=True, exist_ok=True)
    (CALIB / f"{doc['id']}.yaml").write_text(text)
    (CALIB / "latest.yaml").write_text(text)
    robot.coordinates = doc


def register_fixed_camera(robot, key: str, doc: dict, views) -> dict:
    """Pose in W of a fixed camera, from one fresh frame matched against this run's wrist views.
    Moves nothing. Features are never kept past the run that captured them: the table changes."""
    color, depth, K, _ = robot.cameras.capture(key, n=15)   # depth median: noisy at ~1 m
    dist, model = robot.cameras.distortion(key)
    np.savez_compressed(ROOT / doc["frames"] / f"fixed_{key}.npz", color=color, depth=depth, K=K,
                        dist=dist, model=model)            # for debugging only
    cam = coords.register_fixed(views, doc["T_W_base"], color, K, doc.get("base_distance", 0.91), TABLE_Z,
                                depth=depth, dist=dist, model=model)
    doc.setdefault("cameras", {})[key] = cam
    save_doc(robot, doc)
    return cam


class Localize(Mod):
    """Locate both follower bases and the world frame between them, from SIFT features seen by
    both wrist cameras on whatever the table shows (no markers needed).

    W: on the table, origin midway between the two follower bases, x from the left base to the
    right base, z up. Each follower in turn surveys the table between the arms from several
    viewpoints while the other rests folded.
    """

    name = "localize"
    category = "calibration"

    @dataclass
    class Params:
        base_distance: float = field(default=0.91, metadata={
            "help": "Rough distance between the two follower bases, in metres: from the turret axis "
                    "(joint 0) of one base to the turret axis of the other, along the table. Or measure "
                    "the gap between the facing front edges of the bases and add 0.074. It only aims the "
                    "cameras; ±5 cm is fine.",
            "figure": "base_distance.svg"})
        parallel_bases: bool = field(default=True, metadata={
            "help": "The bases sit level on parallel rails, facing each other: solve only their positions "
                    "(distance, offset along the rails). The free fit's yaw is still reported."})
        move_time: float = field(default=3.0, metadata={"help": "Seconds per arm move."})
        dry_run: bool = field(default=False, metadata={"help": "Plan and check every viewpoint; move nothing."})

    STATES = {
        "idle": "not started",
        "planning": "solving the camera viewpoints (IK) and checking every path above the table",
        "parking": "both followers go to rest (folded), so neither blocks the other's view",
        "surveying_left": "left follower: staged pose, then each viewpoint (wrist camera, gripper open), then rest",
        "surveying_right": "right follower: staged pose, then each viewpoint, then rest",
        "returning": "the moving follower goes back to rest after an interruption",
        "solving": "SIFT matching across the two arms, rigid fit, table plane, world frame",
    }
    TRANSITIONS = {
        "idle": {"planning"},
        "planning": {"parking", "done"},
        "parking": {"surveying_left", "returning"},
        "surveying_left": {"surveying_right", "returning"},
        "surveying_right": {"solving", "returning"},
        "solving": set(),
        "returning": set(),
    }

    REQUIRES = (
        Need("both followers", (("follower L", "", ("follower_left",)), ("follower R", "", ("follower_right",))), least=2),
        Need("both wrist cameras", (("wrist L", "", ("wrist_left",)), ("wrist R", "", ("wrist_right",))), least=2),
    )

    result: dict | None = None

    def execute(self) -> None:
        kin, T_ee_cam = load_arm_model()
        arms = {a: self.robot.arms[f"follower_{a}"] for a in coords.ARMS}
        staged = arms["left"].staged_positions

        self.goto("planning")
        plans = {a: self._plan(kin, T_ee_cam[a], a, np.array(staged[:6])) for a in coords.ARMS}
        self.log.info("planned %s views", {a: len(p) for a, p in plans.items()})
        if self.params.dry_run:
            return

        self.goto("parking")
        moving = None
        try:
            for arm in arms.values():
                if np.max(np.abs(arm.positions()[:6])) > 0.05:
                    if not (self._go(arm, staged) and self._go(arm, [0.0] * 7)):
                        return
            views = []
            for a in coords.ARMS:
                self.goto(f"surveying_{a}")
                moving = arms[a]
                if not self._go(moving, staged):
                    return
                for i, q in enumerate(plans[a]):
                    if not self._go(moving, [*q, GRIPPER_OPEN]) or not self.sleep(0.8):
                        return
                    color, depth, K, _ = self.robot.cameras.capture(f"wrist_{a}", n=5, after=time.perf_counter())
                    q_now = np.array(moving.positions()[:6])
                    views.append(coords.View(a, color, depth, K, kin.fk(q_now) @ T_ee_cam[a], q_now.tolist()))
                    self.log.info("%s view %d/%d captured", a, i + 1, len(plans[a]))
                if not (self._go(moving, staged) and self._go(moving, [0.0] * 7)):
                    return
                moving = None
        finally:
            if moving is not None and not self.robot.estopped:
                self.goto("returning")
                moving.move_to(staged, self.params.move_time, blocking=True)
                moving.move_to([0.0] * 7, self.params.move_time, blocking=True)
        if self.stopping:
            return

        self.goto("solving")
        self.result = self._solve_and_save(views)

    # ---- motion ----
    def _go(self, arm, q) -> bool:
        """Move and wait; False if stopped on the way."""
        arm.move_to(list(q), self.params.move_time)
        if not self.sleep(self.params.move_time + 0.3):
            return False
        err = np.max(np.abs(np.array(arm.positions()[:6]) - np.array(q[:6])))
        if err > 0.03:
            raise RuntimeError(f"{arm.name} did not reach its target (off by {np.degrees(err):.1f} deg)")
        return True

    def _plan(self, kin, T_ee_cam, arm, q_staged) -> list[np.ndarray]:
        """One joint configuration per viewpoint, each reachable within the joint margins with every
        joint (past the shoulder) above the table, including along the path from the previous one."""
        plan, prev = [], q_staged
        lo, hi = kin.lo + JOINT_MARGIN, kin.hi - JOINT_MARGIN
        for k, candidates in enumerate(coords.camera_targets(arm, self.params.base_distance, TABLE_Z)):
            for T_B_cam in candidates:
                T_B_ee = T_B_cam @ inv_T(T_ee_cam)
                q = kin.ik(T_B_ee[:3, 3], rotvec(T_B_ee[:3, :3]), q_seed=prev, seeds=12)
                if q is None or np.any(q < lo) or np.any(q > hi):
                    continue
                if all(self._clear(kin, prev + s * (q - prev)) for s in np.linspace(0, 1, 12)):
                    break
            else:
                raise RuntimeError(f"{arm}: no safe reachable pose for viewpoint {k + 1}; "
                                   f"is base_distance ({self.params.base_distance} m) about right?")
            plan.append(q)
            prev = q
        return plan

    @staticmethod
    def _clear(kin, q) -> bool:
        return bool(np.all(kin.link_points(q)[2:, 2] > TABLE_Z + CLEARANCE))

    # ---- result ----
    def _solve_and_save(self, views) -> dict:
        ref_path = ROOT / "config" / "reference_registration.yaml"
        reference = yaml.safe_load(ref_path.read_text())["T_W_base"] if ref_path.exists() else None
        ts = time.strftime("%Y%m%d_%H%M%S")
        frames = FRAMES / ts
        frames.mkdir(parents=True, exist_ok=True)
        for i, v in enumerate(views):
            v.save(frames / f"{v.arm}_{i:02d}.npz")
        res = coords.solve(views, self.params.base_distance, TABLE_Z, reference=reference,
                           parallel=self.params.parallel_bases)
        coords.draw_matches(views, res, frames / "matches.jpg")
        doc = plain({"id": ts, "created": time.strftime("%Y-%m-%d %H:%M:%S"), "method": "sift_wrist_cameras",
                     "frames": str(frames.relative_to(ROOT) if frames.is_relative_to(ROOT) else frames), "base_distance": self.params.base_distance,
                     "parallel_bases": self.params.parallel_bases,
                     "T_W_base": res["T_W_base"], "T_W_mount": res["T_W_mount"],
                     "T_right_left": res["T_right_left"], "report": res["report"]})
        save_doc(self.robot, doc)
        for key in FIXED_CAMERAS:  # both arms are at rest now: the fixed views are clear
            try:
                cam = register_fixed_camera(self.robot, key, doc, views)
                self.log.info("%s camera in W: %d/%d matches, %.2f px rms", key, cam["report"]["inliers"],
                              cam["report"]["matches"], cam["report"]["reprojection_px"]["rms"])
            except Exception as e:
                self.log.warning("%s camera not registered: %s", key, e)
        g = res["report"]["geometry"]
        self.log.info("coordinates %s: bases %.1f mm apart, left yaw %+.2f deg, fit rms %.2f mm (%d inliers)",
                      ts, g["base_distance_mm"], g["left_yaw_vs_facing_deg"],
                      res["report"]["fit_residual_mm"]["rms"], res["report"]["inliers"])
        return doc

    def status(self) -> dict:
        return {**super().status(), "result": self.result and {"id": self.result["id"], "report": self.result["report"]}}
