"""pick_place: pick one cube that locate_cubes tracks and put it down at (x, y) on the table.

Every pose is planned (straight-down IK, joint margins, clearance above the table, the tool point
inside the safety boundary all the way) before the arm moves. Big moves go joint to joint; the approach, lift, carry and descents are straight tool
lines streamed at 20 Hz. A watcher follows the cube's track in locate_cubes the whole time: lost
for longer than `lost_s`, the task stops and every arm is reset.
"""
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from myaloha import cubes
from myaloha.coords import apply_T, inv_T, make_T
from myaloha.cubes import HELD_APERTURE, NAMES, SIZE
from myaloha.gripper import finger_points
from myaloha.mods.base import Mod, Need
from myaloha.mods.localize import JOINT_MARGIN, ROOT, load_arm_model
from myaloha.mods.reset import Reset
from myaloha.reach import reach_map
from myaloha.vendor.heron_kinematics import rodrigues, rotvec

HOVER = 0.04               # above the grasp height, before going down, and the least travel height above it
TRAVEL_CLEAR = 0.015       # the carried cube's bottom passes this far over any column seen near its path
PATH_WIDTH = 0.05          # columns this close (xy) to the path are under it
LOOK_Z = (0.10, 0.08, 0.06)   # tool heights tried for the look pose, highest first (the wrist camera
                              # ~0.2 m from the cube; lower ones reach further from the base)
OPEN = 0.045               # finger gap while approaching a 25 mm cube
OPEN_NARROW = 0.035        # ... where neighbours leave no room for that on any side (5 mm each side for the tracking)
FINGER_CLEAR = 0.004       # m kept between a finger (going down around a cube, or letting go of one) and a
                           # neighbouring cube, beyond the neighbour's own expected error (its sigma)
STACK_OFF = 0.008          # a target this far or less from a cube's centre stacks on it; further, up to a cube's
                           # width, it would sit half on it
SET_PRESS = 0.002          # placing: the cube's bottom is aimed this far below the table, so it always touches
                           # down; the arm stops where it feels that (CONTACT_N) and lets go there, not above it
RELEASE_OPEN = 0.006       # placing: the fingers open only this much wider than the cube, then the arm lifts
CONTACT_BAND = 0.015       # a touch counts within this of the surface the cameras see under the target (table
                           # or stack top); higher up (the arm's efforts jump by ~2 N there on the rig) only a
CONTACT_FAR = 3.0          # ... this many times CONTACT_N: a cube the cameras missed, before it is pressed flat
CONTACT_HOLD = 2           # readings in a row over the threshold: a touch lasts, noise doesn't
CONTACT_WARMUP = 8         # readings (DT apart) before any counts: the arm is speeding up
CONTACT_N = 1.5            # N: the vertical force at the tool (the joints' external efforts through J^T) jumping
                           # this far from its running median while lowering: the carried cube rests on something
TCP_FLOOR = 0.004          # the tool point is never commanded lower
GRASP_TIP = 0.008          # fingertips (the tool point) this far above the cube's bottom: the pads cover
                           # most of its faces (at its mid-height only the tapered tips pinched its top edge;
                           # at 5 mm the tips dragged on the table on the rig and stalled 30 mm apart)
HELD_FILL = 0.2            # share of the view between the fingertips at their depth: something is held
PLACED_WAIT = 4.0          # s for the cameras to see the placed cube on the target
PLACED_TOL = 0.015         # a placed cube further than this from the target is picked and placed again
REFIND_WAIT = 4.0          # s for the cameras to find the cube again after a failed try
OTHER_BASE = 0.25          # and at least this far from the other arm's base
CLEARANCE = 0.03           # joint origins past the shoulder stay this high above the table
DT = 0.05                  # setpoint period of a straight move (s)
LOGS = ROOT / "logs" / "pick_place"
JOINT_SPEED = 0.6          # rad/s for joint-to-joint moves
JOINT_CHECKS = 25          # tool points checked against the safety boundary along a joint move
IK_SEEDS = 1               # descents per start in _ik (its own starts: the seed, the reach map's, mid-range)
TURN_SPEED = 0.5           # rad/s at most for the tool turning along a straight line
HANDOVER_MIN_Z = 0.10      # the grippers meet at least this high (the giver's horizontal fingers clear the table)
MEET_TRIES = 8             # meeting points IK-checked, best first, before a handover is given up
HANDOVER_DEPTH = 0.004     # the taker's tool point goes this far past the cube's centre: a deep grip
APPROACH = 0.08            # the taker comes in (and the giver backs off) along its tool axis this far
AUTO_GAP = 0.06            # auto_target: a new place at least this far (centres) from every cube, its own included
AUTO_EDGE = 0.03           # ... and this far inside the table area the tracker takes cubes in
AUTO_PAIRS = 20            # random (cube, place) pairs tried per round before the round counts as failed
AUTO_FAILS = 3             # rounds failing in a row before the run gives up
RY90 = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # tool x (its approach axis) down


def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def down(yaw: float) -> np.ndarray:
    """A straight-down tool turned `yaw` (rad) about W's z."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ RY90


class PlanningError(RuntimeError):
    pass


class GaveUp(RuntimeError):
    """Every try of a task missed."""


class Missed(RuntimeError):
    """A try that failed but can be tried again: what happened, and where the cameras should look for
    the cube now (within `radius` of the segment a-b, W xy)."""

    def __init__(self, why: str, a, b=None, radius: float = 0.06):
        super().__init__(why)
        self.a, self.b, self.radius = np.asarray(a, float)[:2], np.asarray(a if b is None else b, float)[:2], radius


class PickPlace(Mod):
    """Pick a cube that locate_cubes tracks and put it down at (target_x, target_y) on the table,
    square to W. The follower that reaches both does it; the other stays at rest. Every step is
    checked by the cameras; a try that fails (a missed grasp, a cube dropped on the way or in a
    handover, a placement off the target) is recovered from: the gripper opens, the arm backs off,
    the cameras find the cube again near where it must be, and the task is planned and tried again,
    up to `tries` times. If the cube's track is lost before the wrist camera has looked at it, or the
    wrist camera sees no cube where it should be, the task stops and the arms are reset."""

    name = "pick_place"
    category = "manipulation"

    @dataclass
    class Params:
        track: int = field(default=0, metadata={"help": "The cube to move: its track id in locate_cubes "
                                                          "(ids start at 1; a wrong id lists the ones there are)."})
        target_x: float = field(default=0.0, metadata={"help": "Where to put it down: x in W (m)."})
        target_y: float = field(default=0.0, metadata={"help": "Where to put it down: y in W (m)."})
        arm: str = field(default="auto", metadata={"help": "left, right, or auto: the follower that reaches both, "
                                                           "nearest first."})
        speed: float = field(default=0.05, metadata={"help": "Tool speed (m/s); going down to the cube or the "
                                                             "table, a third of it."})
        lost_s: float = field(default=3.0, metadata={"help": "Stop and reset the arms when the cube is untracked "
                                                             "this long (s)."})
        tries: int = field(default=3, metadata={"help": "Tries before giving up: after a missed grasp, a drop or a "
                                                        "placement off the target, look again and retry."})
        handover: bool = field(default=True, metadata={
            "help": "Allow two arms: when no single follower can do it, the one that reaches the cube picks it "
                    "and hands it over to the other, which places it. The grippers meet facing each other, "
                    "turned 90 deg, each on its own pair of faces, where both reach. Off: one arm or nothing."})
        dry_run: bool = field(default=False, metadata={"help": "Find the cube and plan every pose; move nothing."})
        auto_target: bool = field(default=False, metadata={
            "help": "Keep going on its own until stopped: each round picks a random cube the cameras see (one "
                    "with nothing on it) and a random free place for it, on the table, in reach and at least "
                    "6 cm from every cube, and moves it there. track, target_x and target_y are ignored."})

    STATES = {
        "idle": "not started",
        "planning": "finding the cube, choosing the arm, solving every pose before anything moves",
        "staging": "the arm goes to its staged pose, clear of the table",
        "looking": "pick: the wrist camera looks at the cube from close by; the grasp is re-planned on what it sees",
        "approaching": "pick: gripper open (as wide as the cubes around leave room for), above the cube",
        "descending": "pick: straight down around the cube",
        "grasping": "pick: closing until the fingers stop on the cube, then a gentle hold",
        "lifting": "pick: straight up; the cube must still be held",
        "handing_over": "handover: the giver carries the cube to the handover point, its tool turned toward the other arm",
        "receiving": "handover: the taker comes in facing it, turned 90 deg, and closes on the cube's free faces",
        "exchanging": "handover: the giver opens, backs off and goes to rest; the taker has the cube",
        "moving": "move: carrying the cube at travel height to above the target (the taker, after a handover, "
                  "in a straight line from where it took it, turning its tool down on the way)",
        "lowering": "place: straight down until the arm feels the cube rest on something (the table, or a cube)",
        "releasing": "place: the fingers open a little, slowly, the cube resting where it was set down",
        "retreating": "place: straight up from where it let go, clear of the cube",
        "returning": "the arm goes back to rest",
        "recovering": "a try failed: the gripper opens, the arm backs off, and the cameras find the cube again",
        "resetting": "the cube's track was lost: every arm resets (staged, then zero)",
    }
    TRANSITIONS = {
        "idle": {"planning"},
        "planning": {"staging", "done"},
        "staging": {"looking", "resetting"},
        "looking": {"approaching", "resetting"},
        "approaching": {"descending", "resetting"},
        "descending": {"grasping", "resetting"},
        "grasping": {"lifting", "recovering", "resetting"},
        "lifting": {"moving", "handing_over", "recovering", "resetting"},
        "handing_over": {"receiving", "resetting"},
        "receiving": {"exchanging", "recovering", "resetting"},
        "exchanging": {"moving", "recovering", "resetting"},
        "moving": {"lowering", "recovering", "resetting"},
        "lowering": {"releasing", "resetting"},
        "releasing": {"retreating", "resetting"},
        "retreating": {"returning", "resetting"},
        "returning": {"recovering", "resetting", "planning"},
        "recovering": {"planning", "resetting"},
        "resetting": set(),
    }

    REQUIRES = (
        Need("a follower", (("follower L", "", ("follower_left",)), ("follower R", "", ("follower_right",)))),
        Need("the cube tracker (start locate_cubes)", (("locate_cubes", "", ("topic:locate_cubes",)),)),
        Need("a coordinate calibration (run localize)", (("coordinates", "", ("coordinates",)),)),
    )

    lost = ""                     # why the cube counts as gone (then the task stops and the arms reset)
    fenced = False                # a pose was refused for being outside the safety boundary (while planning)
    frames: dict = {}             # {side: (kinematics, T_W_B)} of the arms in the plan (for the boundary checks)

    @property
    def target(self) -> np.ndarray:
        """Where to put the cube, W (m)."""
        return np.array([self.params.target_x, self.params.target_y])

    @property
    def speed(self) -> float:
        """Tool speed (m/s)."""
        return self.params.speed
    look = ""                     # what came of the last wrist look

    def execute(self) -> None:
        p = self.params
        if p.auto_target:
            self._auto()
            return
        self.goto("planning")
        cube = self._cube(p.track)
        if cube is None:
            there = [f"{c['track']} ({c['color'] or 'hidden'} at {c['x']:+.2f}, {c['y']:+.2f}"
                     f"{', level ' + str(c['level']) if c['level'] else ''})" for c in self._cubes() if c["color"]]
            raise PlanningError(f"locate_cubes has no cube with track {p.track}; "
                                f"{'it has ' + ', '.join(there) if there else 'it sees no cubes'}")
        if cube["level"] is None:
            raise PlanningError(f"cube {p.track} is in the air (in a gripper?), not on the table")
        self._task(cube)

    def _auto(self) -> None:
        """auto_target: round after round, a random cube to a random free place, until stopped. A round
        that fails (every try missed, or nothing plannable) is left safe and followed by another."""
        fails = 0
        while not self._halted():
            if self.state != "planning":
                self.goto("planning")
            try:
                self._task(*self._choose())
                fails = 0
            except (GaveUp, PlanningError) as e:
                fails += 1
                if fails >= AUTO_FAILS:
                    raise RuntimeError(f"{fails} rounds in a row failed; the last: {e}") from None
                self.log.warning("round failed: %s; choosing again", e)
            if self.params.dry_run:
                return

    def _choose(self) -> tuple[dict, dict]:
        """A random cube (seen, on the table, nothing on it) and a random free place for it, the first
        pair the planner can do: (cube, its plans); track and target set to them."""
        p, rng = self.params, np.random.default_rng()
        there = [c for c in self._cubes() if c["level"] is not None and not c["held"]]
        movable = [c for c in there if c["color"] and c["visible"] and not any(
            o["level"] > c["level"] and np.hypot(o["x"] - c["x"], o["y"] - c["y"]) < SIZE for o in there)]
        if not movable:
            raise PlanningError("the cameras see no cube to move")
        places = self._free_places(there)
        if not len(places):
            raise PlanningError(f"no place in reach at least {AUTO_GAP * 100:.0f} cm from every cube")
        why = ""
        for _ in range(AUTO_PAIRS):
            cube, (x, y) = movable[rng.integers(len(movable))], places[rng.integers(len(places))]
            p.track, p.target_x, p.target_y = cube["track"], round(float(x), 3), round(float(y), 3)
            try:
                return cube, self._plan_task(cube)
            except PlanningError as e:
                why = str(e)
        raise PlanningError(f"none of {AUTO_PAIRS} random picks could be planned; the last: {why}")

    def _free_places(self, there: list[dict]) -> np.ndarray:
        """(n, 2) W points, 1 cm apart: on the table (inside the area the tracker takes cubes in, see
        locate_cubes' workspace, and the safety boundary), where either follower reaches straight down
        to put a cube, and AUTO_GAP from every cube."""
        doc, m = self.robot.coordinates, reach_map()
        hx = min(abs(np.array(T)[0, 3]) for T in doc["T_W_mount"].values()) - 0.02 - AUTO_EDGE
        hy = 0.37 - AUTO_EDGE
        X, Y = np.meshgrid(np.arange(-hx, hx, 0.01), np.arange(-hy, hy, 0.01))
        P = np.c_[X.ravel(), Y.ravel(), np.full(X.size, GRASP_TIP - SET_PRESS)]
        down = np.broadcast_to([0.0, 0.0, -1.0], P.shape)
        ok = self.robot.boundary.contains(P)
        reach = np.zeros(len(P), bool)
        for T in doc["T_W_base"].values():
            T = np.array(T)
            reach |= m.reaches(T, P, down) & m.reaches(T, P + [0, 0, HOVER], down)
        ok &= reach
        for c in there:
            ok &= np.hypot(P[:, 0] - c["x"], P[:, 1] - c["y"]) >= AUTO_GAP
        return P[ok, :2]

    def _task(self, cube: dict, plans: dict | None = None) -> None:
        """Move `cube` to the target: tries, each recovered from, until one places it. `plans`: already
        made for the first try."""
        p, failures = self.params, []
        for attempt in range(1, max(1, p.tries) + 1):
            plans = plans or self._plan_task(cube)
            if p.dry_run:
                for name, pl in plans.items():
                    for key in pl.get("points", {}):
                        self.log.info("  %-5s %-14s tool at %s, joints %s", name, key,
                                      np.round(pl["points"][key], 3).tolist(), np.round(pl[key], 2).tolist())
                return
            try:
                self._try(plans)
            except Missed as e:
                failures.append(str(e))
                if attempt == max(1, p.tries) or self._halted():
                    break
                self.log.warning("try %d of %d failed: %s; recovering, then looking again", attempt, p.tries, e)
                self.goto("recovering")
                if not self._recover(plans):
                    return
                cube = self._refind(cube["color"], e)
                if cube is None:                  # the arms are safe (recovered): for auto_target, another round
                    raise GaveUp(f"{e}; then the cameras found no {plans[self.side]['cube']['color']} cube "
                                 f"where it should be") from None
                self.log.info("found it again: cube %d at (%.3f, %.3f), level %d", cube["track"], cube["x"], cube["y"],
                              cube["level"])
                p.track, plans = cube["track"], None
                self.goto("planning")
                continue
            if self.lost:
                self.goto("resetting")
                Reset(self.robot, parent=self).run()
                raise RuntimeError(f"{self.lost}: stopped and reset the arms")
            return
        if p.auto_target and not self._halted():  # left safe for the next round
            self.goto("recovering")
            self._recover(plans)
        raise GaveUp(f"gave up after {len(failures)} tries: " + "; ".join(failures))

    def _plan_task(self, cube: dict) -> dict:
        """{side: plan} for the arms the task needs, the picking arm first; logged. Every tool point,
        and the path of every joint move, inside the safety boundary."""
        p, box = self.params, self.robot.boundary
        for what, xy in ((f"cube {cube['track']}", (cube["x"], cube["y"])), ("the target", self.target)):
            if not box.contains(xy):
                x0, y0, x1, y1 = box.box()
                raise PlanningError(f"{what} at ({xy[0]:.3f}, {xy[1]:.3f}) is outside the safety boundary "
                                    f"(x {x0:+.3f} to {x1:+.3f}, y {y0:+.3f} to {y1:+.3f})")
        self.fenced = False
        self.around = [c for c in self._cubes() if c["track"] != cube["track"] and c["level"] is not None]
        self._check_target()
        try:
            plans, side, taker = self._plan_arms(cube)
        except PlanningError as e:
            if self.fenced:
                raise PlanningError(f"{e} (some poses would leave the safety boundary)") from None
            raise
        self.frames = {s: (pl["kin"], pl["T_W_B"]) for s, pl in plans.items()}
        for s, pl in plans.items():
            if (out := self._fence(s, pl)) is not None:
                raise PlanningError(f"the {s} arm's {out[0]} would take the tool outside the safety boundary, "
                                    f"at ({out[1][0]:.3f}, {out[1][1]:.3f})")
        self.side, self.last = side, cube
        self.log.info("%s cube %d at (%.3f, %.3f) level %d -> (%.3f, %.3f), %s arm%s",
                      cube["color"] or "hidden", p.track, cube["x"], cube["y"], cube["level"],
                      p.target_x, p.target_y, side, f", handed over to the {taker}" if taker else "")
        return plans

    def _plan_arms(self, cube: dict) -> tuple[dict, str, str | None]:
        """({side: plan}, the picking side, the taking side or None)."""
        p = self.params
        self.travel = self._travel_z(cube, self.target)
        # what the cube will be set on, seen now: once it is carried there the overhead camera can take
        # it for a cube on the table under the gripper
        self.support = max([(c["level"] + 1) * SIZE for c in self.around if not c["held"]
                            and np.hypot(c["x"] - self.target[0], c["y"] - self.target[1]) < SIZE] + [0.0])
        try:
            side, plan = self._plan(cube)
            plans, taker = {side: plan}, None
        except PlanningError as alone:
            if not p.handover:
                raise
            try:                                  # no arm alone: two, handing it over
                (side, plan), (taker, taker_plan) = self._plan_handover(cube)
            except PlanningError as e:
                raise PlanningError(f"{alone}; {e}") from None
            plans = {side: plan, taker: taker_plan}
        return plans, side, taker

    def _check_target(self) -> None:
        """The target has room for the cube: not half on a cube (off a cube's centre by more than
        STACK_OFF but less than a cube: stack on it, or put it beside), nor overlapping one at the level
        it would rest at."""
        tx, ty = self.target
        under = [c for c in self.around if np.hypot(c["x"] - tx, c["y"] - ty) < SIZE]
        top = max(under, key=lambda c: c["level"], default=None)
        if top is not None and np.hypot(top["x"] - tx, top["y"] - ty) > STACK_OFF:
            raise PlanningError(f"the target ({tx:.3f}, {ty:.3f}) is half on cube {top['track']} (its centre "
                                f"{np.hypot(top['x'] - tx, top['y'] - ty) * 1000:.0f} mm away): aim at its centre to "
                                f"stack on it, or a cube's width away")
        level = 0 if top is None else top["level"] + 1
        placed = {"x": tx, "y": ty, "z": (level + 0.5) * SIZE, "yaw": 0.0}
        for c in self.around:
            if c["level"] == level and cubes.iou(placed, c) > 0.05:
                raise PlanningError(f"the target ({tx:.3f}, {ty:.3f}) overlaps cube {c['track']} "
                                    f"({c['color'] or 'hidden'} at {c['x']:.3f}, {c['y']:.3f})")

    def _crowded(self, xy, tip_z: float, g: float, aperture: float) -> bool:
        """Would a finger, the tool straight down at finger yaw g, `aperture` apart, its tips at tip_z
        over xy, touch a cube around (grown by FINGER_CLEAR and its sigma)? Only the part of each finger
        below a cube's top can: the fingers widen higher up."""
        R = down(g)[:2]
        for c in self.around:
            above = (c["level"] + 1) * SIZE - tip_z
            if above <= 0 or np.hypot(c["x"] - xy[0], c["y"] - xy[1]) > 0.1:
                continue
            corner = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * (SIZE / 2 + FINGER_CLEAR + c.get("sigma", 0.0))
            if c["yaw"] is None:
                corner = corner * np.sqrt(2)                  # its turn unknown: the square around any turn
            else:
                a = np.radians(c["yaw"])
                corner = corner @ np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
            corners = ((corner + [c["x"], c["y"]]) * 1000).astype(np.float32)   # mm: OpenCV's tolerances
            for V in finger_points(aperture):
                V = V[-V[:, 0] <= above]                      # tool x points down: -x is the height over the tips
                if len(V) < 3:
                    continue
                P = ((V @ R.T + xy) * 1000).astype(np.float32)
                area, _ = cv2.intersectConvexConvex(cv2.convexHull(P), corners)
                if area > 0:
                    return True
        return False

    def _release_yaws(self, yaws) -> list[float]:
        """Of these finger yaws for setting the cube down at the target, the ones where the fingers can
        open to let go without touching a cube around; PlanningError if none can."""
        tip = self.support + GRASP_TIP
        ok = [g for g in yaws if not self._crowded(self.target, tip, g, SIZE + RELEASE_OPEN)]
        if not ok:
            raise PlanningError(f"no room to let go at the target ({self.target[0]:.3f}, {self.target[1]:.3f}): cubes "
                                f"crowd both pairs of its sides")
        return ok

    def _fence(self, side: str, plan: dict) -> tuple[str, np.ndarray] | None:
        """(which move, where) of the first joint move in the plan that would take the tool outside the
        safety boundary, or None: from where the arm is to its staged pose, the ones the plan makes, and
        back to rest (from above the target: where the retreat ends)."""
        now = np.array(self.robot.arms[f"follower_{side}"].positions()[:6])
        moves = [("move to its staged pose", now, plan["staged"])]
        if "look" in plan:
            moves += [("move to the look pose", plan["staged"], plan["look"]),
                      ("move above the cube", plan["look"], plan["above_cube"])]
        if "handover" in plan and "look" in plan:
            moves += [("move to the handover", plan["above_cube"], plan["handover"]),
                      ("return from the handover", plan["handover_back"], plan["staged"])]
        if "pre_handover" in plan:
            moves += [("move to the handover", plan["staged"], plan["pre_handover"])]
        if "above_target" in plan:
            moves += [("return from the target", plan["above_target"], plan["staged"])]
        moves += [("move to rest", plan["staged"], np.zeros(6))]
        for what, q0, q1 in moves:
            if (at := self._leaves(side, q0, q1)) is not None:
                return what, at
        return None

    def _leaves(self, side: str, q0, q1) -> np.ndarray | None:
        """The first tool point outside the safety boundary on a joint move from q0 to q1 (every joint
        moves in step: a straight line in joint space), or None."""
        kin, T_W_B = self.frames[side]
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        for s in np.linspace(0.0, 1.0, JOINT_CHECKS):
            p = (T_W_B @ kin.fk(q0 + s * (q1 - q0)))[:3, 3]
            if not self.robot.boundary.contains(p):
                return p
        return None

    def _try(self, plans: dict) -> None:
        """One try of the planned task, the tracker watched until the wrist camera has looked."""
        (side, plan), *rest = plans.items()
        done = threading.Event()
        watcher = threading.Thread(target=self._watch, args=(done,), name="pick-place-watch", daemon=True)
        watcher.start()
        try:
            if rest:
                (taker, taker_plan), = rest
                self._run_handover(self.robot.arms[f"follower_{side}"], plan,
                                   self.robot.arms[f"follower_{taker}"], taker_plan)
            else:
                self._run_plan(self.robot.arms[f"follower_{side}"], plan)
        finally:
            done.set()
            watcher.join()

    def _recover(self, plans: dict) -> bool:
        """After a failed try: every arm of the task opens its gripper, lifts its tool straight up to
        clear the table (if it is down near it), and goes to its staged pose, out of the cameras' way."""
        for side, plan in plans.items():
            arm = self.robot.arms[f"follower_{side}"]
            kin, T_W_B = plan["kin"], plan["T_W_B"]
            T = T_W_B @ kin.fk(np.array(arm.positions()[:6]))
            up = {"kin": kin, "T_W_B": T_W_B, "points": {"up": np.array([*T[:2, 3], max(T[2, 3], self.travel + 0.03)])},
                  "R": {"up": T[:3, :3]}}
            if not (self._gripper(arm, plan.get("open", OPEN)) and self._line(arm, up, "up", self.speed / 3)
                    and self._joint(arm, plan["staged"])):
                return False
        return True

    def _refind(self, color: str, miss: Missed) -> dict | None:
        """The cube again, by the cameras: of its colour, not seen only from memory, nearest the segment
        where the miss says it must be (within its radius); waits REFIND_WAIT for the tracker."""
        def off(c):
            ab, pc = miss.b - miss.a, np.array([c["x"], c["y"]]) - miss.a
            t = float(np.clip(pc @ ab / max(ab @ ab, 1e-9), 0.0, 1.0))
            return float(np.linalg.norm(pc - t * ab))
        end = time.monotonic() + REFIND_WAIT
        while self._wait(0.3):
            there = [c for c in self._cubes() if c["color"] == color and not c["held"] and c["level"] is not None
                     and off(c) < miss.radius]
            if there:
                return min(there, key=off)
            if time.monotonic() > end:
                return None
        return None

    # ---- the task ----
    def _run_plan(self, arm, plan) -> None:
        if not self._run_pick(arm, plan):
            return
        self.goto("moving")
        if not self._line(arm, plan, "above_target", self.speed):
            return
        self._run_place(arm, plan)

    def _run_pick(self, arm, plan) -> bool:
        """Staging to lifting; False if stopped on the way."""
        p = self.params
        down_speed = self.speed / 3
        self.goto("staging")
        if not self._joint(arm, plan["staged"]):
            return False
        self.goto("looking")                      # fingers open: closed, their tips hide the cube
        if not (self._joint(arm, plan["look"]) and self._gripper(arm, OPEN)):
            return False
        seen = self._measure(arm, plan)
        if seen is None and self._wait(0.5):      # one more look, a fresh frame
            seen = self._measure(arm, plan)
        if seen is None and self.look.startswith("no cube"):
            # it looked right where the tracker has the cube and sees none: gone, whatever the tracker holds
            self.lost = f"the wrist camera sees no {plan['cube']['color']} cube where cube {p.track} should be"
            self.log.warning("%s: stopping", self.lost)
            return False
        if seen is not None:
            self.log.info("wrist view: cube centre %.1f mm across, %.1f mm off in height from the plan, yaw %.0f deg "
                          "(tracker %.0f); re-planning",
                          np.hypot(seen["x"] - plan["cube"]["x"], seen["y"] - plan["cube"]["y"]) * 1000,
                          seen["dz"] * 1000, seen["yaw"] or 0.0, plan["cube"]["yaw"] or 0.0)
            plan.update(self._replan(plan, seen))
        else:
            self.log.warning("the wrist camera did not find the cube; grasping where the tracker has it")
        self.goto("approaching")                  # only as wide as the cubes around leave room for
        if not (self._joint(arm, plan["above_cube"]) and self._gripper(arm, plan["open"])):
            return False
        self.goto("descending")
        if not self._line(arm, plan, "grasp", down_speed):
            return False
        self.goto("grasping")
        gap = arm.close_on_object(stopping=self._halted)
        if self._halted():
            return False
        here = (plan["cube"]["x"], plan["cube"]["y"])
        if not HELD_APERTURE[0] <= gap <= HELD_APERTURE[1]:
            raise Missed(f"missed the cube: the fingers closed to {gap * 1000:.1f} mm", here)
        self.log.info("fingers stopped %.1f mm apart", gap * 1000)
        self.goto("lifting")
        if not self._line(arm, plan, "above_cube", down_speed):
            return False
        why = self._check_held(arm, plan, gap, "after the lift")
        if why:
            raise Missed(f"missed the cube: {why}", here)
        return True

    def _check_held(self, arm, plan, gap: float, when: str) -> str:
        """'' if the arm holds the cube, else why not. Both have to say so: the gripper (its fingers
        still stopped a cube's width apart) and the wrist camera, whose depth has to show something
        filling the space between the fingertips at their depth (not the table behind them). Not its
        colour: in the fingers' shadow, against the bright table, the rig's cubes read too dark to
        classify (every colour failure logged on the rig was a cube held). Each check is saved in LOGS
        with its outcome."""
        now = arm.gripper_aperture()
        if not HELD_APERTURE[0] <= now <= HELD_APERTURE[1] or gap - now > 0.003:
            why = f"{arm.name} fingers at {now * 1000:.1f} mm {when} (closed on {gap * 1000:.1f} mm)"
        else:
            why = self._in_view(arm, plan)
            why = why and f"{why} {when}"
        if why:
            self.log.warning("not holding the cube: %s", why)
        else:
            self.log.info("holding the cube %s: fingers %.1f mm apart, the wrist camera sees it", when, now * 1000)
        return why

    def _in_view(self, arm, plan) -> str:
        """'' if the arm's wrist camera sees something between its fingertips, at their depth, else what
        it sees there."""
        side = arm.name.removeprefix("follower_")
        key = f"wrist_{side}"
        if not (self.robot.cameras and self.robot.cameras.status().get(key, {}).get("ok")):
            return f"the {key} camera is down, so the grasp can't be seen"
        color, depth, K, _ = self.robot.cameras.capture(key, n=3)
        dist, model = self.robot.cameras.distortion(key)
        T_ee_cam = plan["T_ee_cam"]
        # the held cube's centre: SIZE/2 - GRASP_TIP back from the tool point, along the fingers (ee x)
        c = apply_T(inv_T(T_ee_cam), np.array([[GRASP_TIP - SIZE / 2, 0.0, 0.0]]))[0]
        xd, yd = cubes.distort_norm(np.array([c[0] / c[2]]), np.array([c[1] / c[2]]), dist, model)
        u, v, r = int(xd[0] * K[0, 0] + K[0, 2]), int(yd[0] * K[1, 1] + K[1, 2]), int(0.3 * K[0, 0] * SIZE / c[2])
        box = np.s_[max(v - r, 0):v + r, max(u - r, 0):u + r]
        fill = (np.abs(depth[box] - c[2]) < SIZE).mean()           # at the fingertips, not the table behind
        # on the rig a miss fills 0% (the table is 8 cm and more behind); a cube held off-centre, deeper in
        # the grip than planned, fills 31% (the region catches its edge), one held as planned 50-97%
        why = "" if fill >= HELD_FILL else f"nothing between the fingertips ({fill:.0%} of the view there is at their depth)"
        LOGS.mkdir(parents=True, exist_ok=True)
        np.savez(LOGS / f"{time.strftime('%Y%m%d_%H%M%S')}_{side}_held.npz", color=color, depth=depth, K=K,
                 box=np.array([u, v, r]), depth_expected=c[2], why=why or "holding")
        return why

    def _check_placed(self, plan) -> None:
        """The cameras see a cube of its colour within PLACED_TOL of the target (the arm back at rest,
        out of their way); otherwise the try missed, and the cube is looked for around the target."""
        name, target = plan["cube"]["color"], self.target
        end = time.monotonic() + PLACED_WAIT
        while self._wait(0.3):
            there = sorted((float(np.hypot(c["x"] - target[0], c["y"] - target[1])), c) for c in self._cubes()
                           if c["color"] == name and not c["held"] and c["level"] is not None)
            if there and there[0][0] < PLACED_TOL:
                d, c = there[0]
                self.log.info("placed: the cameras see a %s cube at (%.3f, %.3f), %.1f mm from the target, level %d",
                              name, c["x"], c["y"], d * 1000, c["level"])
                return
            if time.monotonic() > end:
                near = f"the nearest {there[0][0] * 1000:.0f} mm off" if there else "none"
                raise Missed(f"placed, but the cameras see no {name} cube on the target ({target[0]:.3f}, "
                             f"{target[1]:.3f}): {near}", target, radius=0.08)

    def _run_place(self, arm, plan) -> None:
        """Lowering to returning, from above the target."""
        down_speed = self.speed / 3
        gap = arm.gripper_aperture()
        why = self._check_held(arm, plan, gap, "above the target")
        if why:                                   # it fell somewhere between where it was and here
            raise Missed(f"lost the cube on the way: {why}", (plan["cube"]["x"], plan["cube"]["y"]),
                         plan["points"]["above_target"], radius=0.08)
        self.goto("lowering")
        if not self._line(arm, plan, "place", down_speed, until=self._touched(arm, plan)):
            return
        self.goto("releasing")                    # a little and slowly: wide, the pads could flick it or a neighbour
        if not self._gripper(arm, gap + RELEASE_OPEN):
            return
        self.goto("retreating")                   # straight up from where it let go, clear of the cube's top
        kin, T_W_B = plan["kin"], plan["T_W_B"]
        T = T_W_B @ kin.fk(np.array(arm.positions()[:6]))
        z = max(T[2, 3] + HOVER, plan["points"]["above_target"][2])
        up = {"kin": kin, "T_W_B": T_W_B, "points": {"up": np.array([*T[:2, 3], z])}, "R": {"up": T[:3, :3]}}
        if not self._line(arm, up, "up", down_speed):
            return
        self.goto("returning")
        if self._rest(arm, plan):
            self._check_placed(plan)

    def _rest(self, arm, plan) -> bool:
        return self._gripper(arm, 0.0) and self._joint(arm, plan["staged"]) and self._joint(arm, [0.0] * 6)

    def _run_handover(self, giver, gp, taker, tp) -> None:
        """The giver picks and holds the cube out midway; the taker takes it on its free faces, the
        giver lets go and rests, and the taker places it."""
        down_speed = self.speed / 3
        if not self._run_pick(giver, gp):
            return
        self.goto("handing_over")                 # the cube turns with the tool, toward the taker
        if not self._joint(giver, gp["handover"]):
            return
        self.goto("receiving")
        if not (self._joint(taker, tp["staged"]) and self._joint(taker, tp["pre_handover"])):
            return
        meet = gp["points"]["handover"]
        for take in (1, 2):                       # a miss here is common and cheap to retry: the giver still holds it
            if not (self._gripper(taker, OPEN) and self._line(taker, tp, "handover", down_speed)):
                return
            gap = taker.close_on_object(stopping=self._halted)
            if self._halted():
                return
            if HELD_APERTURE[0] <= gap <= HELD_APERTURE[1]:
                break
            self.log.warning("the taker's fingers closed to %.1f mm, not on the cube (take %d)", gap * 1000, take)
            if not (self._gripper(taker, OPEN) and self._line(taker, tp, "pre_handover", down_speed)):
                return
        else:
            raise Missed(f"the handover missed: the taker's fingers closed to {gap * 1000:.1f} mm twice", meet,
                         radius=0.15)
        self.log.info("taken over: %s fingers %.1f mm apart", taker.name, gap * 1000)
        self.goto("exchanging")
        self.side = tp["side"]                    # the cube is the taker's now
        if not (self._gripper(giver, OPEN) and self._line(giver, gp, "handover_back", down_speed)):
            return
        why = self._check_held(taker, tp, gap, "after the giver let go")
        if why:                                   # it fell from the meeting point
            raise Missed(f"the handover failed: {why}", meet, radius=0.15)
        if not self._rest(giver, gp):
            return
        self.goto("moving")                       # straight over, the tool turning down on the way
        if self._line(taker, tp, "above_target", self.speed):
            self._run_place(taker, tp)

    def _halted(self) -> bool:
        return self.stopping or self.lost

    def _wait(self, seconds: float) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._halted():
                return False
            time.sleep(0.02)
        return True

    def _gripper(self, arm, aperture: float) -> bool:
        """Open or close and wait until done: an arm command sent meanwhile would cancel it."""
        arm.set_gripper(aperture, 1.0)
        if not self._wait(1.3):
            return False
        got = arm.gripper_aperture()
        if abs(got - aperture) > 0.004:
            raise RuntimeError(f"{arm.name}: gripper at {got * 1000:.1f} mm, asked for {aperture * 1000:.1f} mm")
        return True

    def _joint(self, arm, q) -> bool:
        """Joint to joint at JOINT_SPEED; checks the arm got there. Refused if the tool would leave the
        safety boundary on the way."""
        q = np.asarray(q, float)
        if (at := self._leaves(arm.name.removeprefix("follower_"), arm.positions()[:6], q)) is not None:
            raise RuntimeError(f"{arm.name}: the move would take the tool outside the safety boundary, at "
                               f"({at[0]:.3f}, {at[1]:.3f}); not moving")
        t = max(1.5, float(np.max(np.abs(q - np.array(arm.positions()[:6])))) / JOINT_SPEED)
        arm.move_arm(q, t)
        if not self._wait(t + 0.3):
            return False
        err = float(np.max(np.abs(np.array(arm.positions()[:6]) - q)))
        if err > 0.03:
            raise RuntimeError(f"{arm.name} did not reach its target (off by {np.degrees(err):.1f} deg)")
        return True

    def _line(self, arm, plan, to: str, speed: float, until=None) -> bool:
        """A straight tool line from the present pose to plan[to], streamed at 1/DT; cut short where
        `until()` turns true (the arm then holds where it is). Refused if it ends outside the safety
        boundary, unless it goes straight up or down (lines between points inside stay inside)."""
        q0 = np.array(arm.positions()[:6])
        end = plan["points"][to]
        if not self.robot.boundary.contains(end):
            start = (plan["T_W_B"] @ plan["kin"].fk(q0))[:3, 3]
            if np.hypot(*(end[:2] - start[:2])) > 0.002:
                raise RuntimeError(f"{arm.name}: the line to {to} ends outside the safety boundary, at "
                                   f"({end[0]:.3f}, {end[1]:.3f}); not moving")
        for p, q in self._steps(plan["kin"], plan["T_W_B"], q0, plan["points"][to], plan["R"][to], speed * DT):
            if q is None:
                raise RuntimeError(f"no IK along the line to {to} at {np.round(p, 3).tolist()}")
            arm.move_arm(q, DT * 2)
            if not self._wait(DT):
                return False
            if until and until():
                arm.move_arm(np.array(arm.positions()[:6]), DT * 2)      # stop pressing
                return self._wait(0.2)
        return self._wait(0.4)

    @staticmethod
    def _steps(kin, T_W_B, q, p1, R1, step: float):
        """(tool point, joints) along a straight tool line from where joints q have it to p1 (W), `step`
        (m) apart, the tool turning evenly to R1 on the way (at most TURN_SPEED); the joints None, and the
        walk ends, where IK fails."""
        T = T_W_B @ kin.fk(q)
        p0, R0, B_W = T[:3, 3], T[:3, :3], inv_T(T_W_B)
        turn = rotvec(R1 @ R0.T)
        n = max(1, int(np.ceil(max(np.linalg.norm(p1 - p0) / step, np.linalg.norm(turn) / (TURN_SPEED * DT)))))
        for k in range(1, n + 1):
            p = p0 + (p1 - p0) * k / n
            q = kin.ik(apply_T(B_W, p[None])[0], rotvec(B_W[:3, :3] @ rodrigues(turn * k / n) @ R0), q_seed=q, seeds=1)
            yield p, q
            if q is None:
                return

    def _touched(self, arm, plan):
        """For lowering: has the carried cube come to rest on something? The vertical force at the tool,
        from the joints' external efforts, jumps from its running median by CONTACT_N, for CONTACT_HOLD
        readings in a row, near the surface the cameras see under the target; elsewhere by CONTACT_FAR
        times that. (On the rig a lone 1.9 N jump 3 cm up let a cube drop.)"""
        kin, T_W_B = plan["kin"], plan["T_W_B"]
        z_B, seen, over = inv_T(T_W_B)[:3, :3] @ np.array([0.0, 0.0, 1.0]), [], [0]
        floor, support = plan["points"]["place"][2], self.support  # the tool at floor: the cube's bottom at -SET_PRESS

        def touched() -> bool:
            r = arm.reading()
            q = np.array(r["pos"][:6])
            g = kin.jacobian(q)[:3].T @ z_B                       # joint efforts per newton up at the tool
            f = float(g @ np.array(r["ext_eff"][:6]) / (g @ g))
            ref = float(np.median(seen[-10:])) if len(seen) >= CONTACT_WARMUP else None
            seen.append(f)
            z = apply_T(T_W_B, kin.fk(q)[:3, 3][None])[0][2]
            bottom = z - floor - SET_PRESS                        # the carried cube's bottom above the table
            near = bottom <= support + CONTACT_BAND
            over[0] = over[0] + 1 if ref is not None and abs(f - ref) >= CONTACT_N * (1 if near else CONTACT_FAR) else 0
            if over[0] < CONTACT_HOLD:
                return False
            self.log.info("felt the cube touch down (%.1f N from %.1f N), its bottom %.1f mm above the table "
                          "(the cameras see %s there)", f, ref, bottom * 1000,
                          f"a stack {support * 1000:.0f} mm high" if support else "the table")
            return True
        return touched

    # ---- tracking ----
    def _cubes(self) -> list[dict]:
        m = self.robot.topics.latest("locate_cubes")
        return ((m or {}).get("data") or {}).get("cubes", [])

    def _cube(self, track: int) -> dict | None:
        return next((c for c in self._cubes() if c["track"] == track), None)

    def _measure(self, arm, plan) -> dict | None:
        """The cube as this arm's wrist camera sees it from the look pose, relative to the gripper:
        forward kinematics and the hand-eye pose only, so errors in the table calibration (and the
        arm's sag) cancel. Returns the cube (in the plan's W: where to send the tool) with `dz`, the
        height error of the plan there, or None if the wrist camera doesn't find it. Every look is
        saved in LOGS with what came of it."""
        kin, T_W_B, T_ee_cam, cube = plan["kin"], plan["T_W_B"], plan["T_ee_cam"], plan["cube"]
        if cube["color"] is None:
            return None
        key = f"wrist_{self.side}"
        if not (self.robot.cameras and self.robot.cameras.status().get(key, {}).get("ok")):
            self.log.warning("wrist view: the %s camera is down", key)
            return None
        color, depth, K, _ = self.robot.cameras.capture(key, n=3)
        dist, model = self.robot.cameras.distortion(key)
        T_W_ee = T_W_B @ kin.fk(np.array(arm.positions()[:6]))
        seen, why, blob = self._find(color, depth, K, dist, model, T_W_ee, T_ee_cam, cube)
        self.look = why
        LOGS.mkdir(parents=True, exist_ok=True)
        np.savez(LOGS / f"{time.strftime('%Y%m%d_%H%M%S')}_{self.side}_look.npz", color=color, depth=depth, K=K,
                 T_W_ee=T_W_ee, T_ee_cam=T_ee_cam, blob=np.zeros(color.shape[:2], bool) if blob is None else blob,
                 cube=np.array([cube["x"], cube["y"], cube["level"], cube["yaw"] or 0.0]),
                 yaw=np.nan if seen is None else seen["yaw"], why=why)
        if seen is None:
            self.log.warning("wrist view: %s", why)
        return seen

    def _find(self, color, depth, K, dist, model, T_W_ee, T_ee_cam, cube) -> tuple[dict | None, str, np.ndarray | None]:
        """(the cube as measured, or None; what came of the look; the blob used)."""
        view = cubes.CameraView("wrist", color, depth, K, dist, model, T_W_ee @ T_ee_cam)
        labels = cubes._labels(view, (1.0, 1.0))
        if labels is None:
            return None, "no colour labels (no table in view to balance them)", None
        pc = apply_T(inv_T(T_W_ee @ T_ee_cam), np.array([[cube["x"], cube["y"], (cube["level"] + 1) * SIZE]]))[0]
        want = np.array([pc[0] / pc[2] * K[0, 0] + K[0, 2], pc[1] / pc[2] * K[1, 1] + K[1, 2]])
        face = (SIZE * K[0, 0] / pc[2]) ** 2                 # px of a whole top face at that range
        # the blob of its colour nearest to where the plan expects the cube; failing that, a blob of
        # any cube colour right there (lit from this side, a colour can read as its neighbour: red as
        # orange): the same cube by where it is
        mask = (labels == NAMES.index(cube["color"])).astype(np.uint8)
        n, comp, stats, cent = cv2.connectedComponentsWithStats(mask)
        big = [i for i in range(1, n) if stats[i, 4] > 150]
        if not big:
            n, comp, stats, cent = cv2.connectedComponentsWithStats((labels >= 0).astype(np.uint8))
            big = [i for i in range(1, n) if stats[i, 4] > 150 and np.linalg.norm(cent[i] - want) < 0.6 * np.sqrt(face)]
            if not big:
                return None, (f"no cube: no {cube['color']} blob, nor one of any cube colour at pixel "
                              f"{np.round(want).astype(int).tolist()}"), None
            i = min(big, key=lambda i: np.linalg.norm(cent[i] - want))
            read = NAMES[int(np.bincount(labels[comp == i], minlength=len(NAMES)).argmax())]
            self.log.warning("wrist view: the cube reads %s here (tracked as %s); it is where the cube should be",
                             read, cube["color"])
        i = min(big, key=lambda i: np.linalg.norm(cent[i] - want))
        blob = comp == i
        if stats[i, 4] < 0.4 * face:                         # a sliver (shadowed, half hidden): no yaw from it
            return None, f"only {stats[i, 4]} px of the cube's colour (a face is ~{face:.0f} px)", blob
        v, u = np.nonzero(blob)
        d = depth[v, u]
        ok = d > 0.05
        if ok.sum() < 50:
            return None, f"too little depth on the cube ({ok.sum()} px)", blob
        P_ee = apply_T(T_ee_cam, cubes._rays(K, dist, model, u[ok], v[ok]) * d[ok][:, None])   # x: along the fingers
        top = P_ee[P_ee[:, 0] < np.percentile(P_ee[:, 0], 5) + 0.004]
        centre_ee = np.array([np.median(top[:, 0]) + SIZE / 2, *np.median(top[:, 1:], axis=0)])
        centre_W = apply_T(T_W_ee, centre_ee[None])[0]
        yaw = cubes._yaw(apply_T(T_W_ee, top)[:, :2])     # the top face's edges, seen close: better than above
        return ({**cube, "x": float(centre_W[0]), "y": float(centre_W[1]),
                 "z_grasp": float(centre_W[2]) - SIZE / 2 + GRASP_TIP,
                 "dz": float(centre_W[2] - (cube["level"] + 0.5) * SIZE), "yaw": cube["yaw"] if yaw is None else yaw},
                "found", blob)

    def _watch(self, done: threading.Event) -> None:
        """Until the wrist camera has looked at the cube, follow its track; once lost for longer
        than lost_s, stop the task. If the tracker gives it a new id, the cube is one of its colour
        within 3 cm of where it was last. From the look on, the wrist camera and the grasp and
        placement checks say where the cube is (the gripper hides it from the others)."""
        seen = time.monotonic()
        while not done.wait(0.2) and self.state in ("planning", "staging", "looking"):
            try:
                now = self._cubes()
            except Exception:                          # the topic went away: nothing seen this time
                now = []
            mine = next((c for c in now if c["track"] == self.params.track), None)
            if mine is None:
                last = self.last
                mine = next((c for c in now if c["color"] == last["color"] and c["level"] == last["level"]
                             and np.hypot(c["x"] - last["x"], c["y"] - last["y"]) < 0.03), None)
                if mine is not None:
                    self.log.info("cube %d is now track %d", self.params.track, mine["track"])
                    self.params.track = mine["track"]
            if mine is not None:
                seen, self.last = time.monotonic(), mine
            elif time.monotonic() - seen > self.params.lost_s:
                self.log.warning("cube %d untracked for %.1f s: stopping", self.params.track, time.monotonic() - seen)
                self.lost = f"cube {self.params.track} untracked for more than {self.params.lost_s:.0f} s"
                return

    # ---- planning ----
    def _travel_z(self, cube: dict, target: np.ndarray) -> float:
        """Tool height for moves across the table: HOVER above the grasp, or higher where a column
        the cameras see lies within PATH_WIDTH of the way from the cube to the target, so that the
        carried cube (hanging GRASP_TIP below the tool point) clears it by TRAVEL_CLEAR. Low travel
        keeps the wrist off its limit far from the base."""
        a = np.array([cube["x"], cube["y"]])
        ab = target - a
        z = cube["level"] * SIZE + GRASP_TIP + HOVER
        for c in self._cubes():
            if c["track"] == cube["track"] or c["level"] is None:      # in the air: not a column on the way
                continue
            pc = np.array([c["x"], c["y"]])
            t = float(np.clip((pc - a) @ ab / max(ab @ ab, 1e-9), 0.0, 1.0))
            if np.linalg.norm(pc - (a + t * ab)) < PATH_WIDTH:
                z = max(z, (c["level"] + 1) * SIZE + GRASP_TIP + TRAVEL_CLEAR)
        return z

    def _plan(self, cube: dict):
        p = self.params
        kin, T_ee_cam = load_arm_model()
        doc = self.robot.coordinates
        bases = {s: np.array(doc["T_W_base"][s]) for s in ("left", "right")}
        target = self.target
        here = np.array([cube["x"], cube["y"]])
        sides = [p.arm] if p.arm in ("left", "right") else ["left", "right"]
        sides = [s for s in sides if self.robot.arm("follower", s)]
        why = []
        for side in sorted(sides, key=lambda s: max(np.linalg.norm(bases[s][:2, 3] - here),
                                                      np.linalg.norm(bases[s][:2, 3] - target))):
            other = bases["right" if side == "left" else "left"][:2, 3]
            r = [np.linalg.norm(bases[side][:2, 3] - xy) for xy in (here, target)]
            if min(np.linalg.norm(other - here), np.linalg.norm(other - target)) < OTHER_BASE:
                why.append(f"{side}: too close to the other arm's base")
                continue
            try:
                staged = np.array(self.robot.arm("follower", side).staged_positions[:6])
                return side, self._poses(kin, bases[side], T_ee_cam[side], staged, cube, target)
            except PlanningError as e:
                why.append(f"{side}: {e} (cube {r[0]:.2f} m, target {r[1]:.2f} m from its base)")
        raise PlanningError("no follower can do it: " + "; ".join(why or ["no follower connected"])
                            + ("" if p.handover else " (a handover may: handover=true)"))

    def _replan(self, plan, seen) -> dict:
        """The plan again for the cube as the wrist camera saw it: the whole of it for one arm, the
        pick for a giver (its handover doesn't depend on where the cube was)."""
        kin, T_W_B, T_ee_cam, staged = plan["kin"], plan["T_W_B"], plan["T_ee_cam"], plan["staged"]
        if "place" in plan:
            return self._poses(kin, T_W_B, T_ee_cam, staged, seen, self.target)
        for g, opening in self._grips(T_W_B, seen):
            pick = self._pick(kin, T_W_B, T_ee_cam, staged, seen, g, opening)
            if pick is not None:
                return self._merge(plan, pick)
        raise PlanningError("the cube as the wrist camera sees it can't be grasped")

    def _plan_handover(self, cube: dict):
        """((giver, its plan), (taker, its plan)). The giver reaches the cube, the taker the target.
        They meet where the reach map says both can (the shortest carry first): the giver's tool points
        at the taker, the taker's back at it turned 90 deg about that axis, so each pair of pads is on
        its own pair of faces."""
        kin, T_ee_cam = load_arm_model()
        bases = {s: np.array(self.robot.coordinates["T_W_base"][s]) for s in ("left", "right")}
        target, here = self.target, np.array([cube["x"], cube["y"]])
        if not all(self.robot.arm("follower", s) for s in bases):
            raise PlanningError("a handover needs both followers")
        why = []
        for giver in sorted(bases, key=lambda s: np.linalg.norm(bases[s][:2, 3] - here)):
            taker = "right" if giver == "left" else "left"
            bg, bt = bases[giver][:2, 3], bases[taker][:2, 3]
            r = [np.linalg.norm(bg - here), np.linalg.norm(bt - target)]
            if np.linalg.norm(bt - here) < OTHER_BASE or np.linalg.norm(bg - target) < OTHER_BASE:
                why.append(f"{giver} to {taker}: too close to the other arm's base")
                continue
            staged = {s: np.array(self.robot.arm("follower", s).staged_positions[:6]) for s in bases}
            pick = next((pk for g, w in self._grips(bases[giver], cube)
                         if (pk := self._pick(kin, bases[giver], T_ee_cam[giver], staged[giver], cube, g, w))), None)
            if pick is None:
                why.append(f"{giver}: no straight-down grasp within the joint limits (cube {r[0]:.2f} m from its base)")
                continue
            x = np.append((bt - bg) / np.linalg.norm(bt - bg), 0.0)          # from the giver toward the taker
            for H, y in ((H, y) for H in self._meeting_points(bases[giver], bases[taker], x, cube, target)[:MEET_TRIES]
                         for y in (np.array([0, 0, 1.0]), np.array([0, 0, -1.0]),
                                   np.cross([0, 0, 1.0], x), -np.cross([0, 0, 1.0], x))):
                H_taker = H - x * HANDOVER_DEPTH
                R_g = np.c_[x, y, np.cross(x, y)]                           # giver: closing along y
                R_t = np.c_[-x, np.cross(x, y), y]                           # taker: closing along the third axis
                qg = self._ik(kin, bases[giver], H, R_g, pick["above_cube"])
                qg_back = qg if qg is None else self._ik(kin, bases[giver], H - x * APPROACH, R_g, qg)
                qt_pre = self._ik(kin, bases[taker], H_taker + x * APPROACH, R_t, staged[taker])
                qt = qt_pre if qt_pre is None else self._ik(kin, bases[taker], H_taker, R_t, qt_pre)
                if any(q is None for q in (qg, qg_back, qt_pre, qt)):
                    continue
                place = self._place(kin, bases[taker], qt, target, self._release_yaws([k * np.pi / 2 for k in range(4)]))
                # the carry: a straight line to above the target, turning the tool down on the way (one
                # joint move there swung the cube down near the table)
                if place is None or any(q is None for _, q in self._steps(
                        kin, bases[taker], qt, place["points"]["above_target"], place["R"]["above_target"], 0.01)):
                    continue
                gp = self._merge(pick, {"points": {"handover": H, "handover_back": H - x * APPROACH},
                                        "R": {"handover": R_g, "handover_back": R_g},
                                        "handover": qg, "handover_back": qg_back})
                tp = self._merge({"kin": kin, "T_W_B": bases[taker], "T_ee_cam": T_ee_cam[taker], "cube": cube,
                                  "staged": staged[taker], "side": taker,
                                  "points": {"pre_handover": H_taker + x * APPROACH, "handover": H_taker},
                                  "R": {"pre_handover": R_t, "handover": R_t}, "pre_handover": qt_pre, "handover": qt},
                                 place)
                return (giver, gp), (taker, tp)
            why.append(f"{giver} to {taker}: no handover pose both arms reach, or the {taker} arm can't place "
                       f"there (target {r[1]:.2f} m from its base)")
        raise PlanningError("no handover works: " + "; ".join(why))

    def _meeting_points(self, T_g, T_t, x, cube: dict, target) -> list[np.ndarray]:
        """Where the giver's tool point can hold the cube out to the taker, best first: the giver there
        and backing off along x, the taker's tool point against it and coming in along -x, all within
        the reach map; at least HANDOVER_MIN_Z (and a cube over the travel height) up. The shorter the
        carry (the cube to there to the target), the better."""
        m = reach_map()
        mid = (T_g[:3, 3] + T_t[:3, 3]) / 2
        side = np.cross([0, 0, 1.0], x)
        u, v, z = np.meshgrid(np.arange(-0.26, 0.261, 0.02), np.arange(-0.3, 0.301, 0.02),
                              np.arange(max(HANDOVER_MIN_Z, self.travel + SIZE), 0.40, 0.02), indexing="ij")
        H = np.c_[mid[0] + u.ravel() * x[0] + v.ravel() * side[0], mid[1] + u.ravel() * x[1] + v.ravel() * side[1],
                  z.ravel()]
        X = np.broadcast_to(x, H.shape)
        H_t = H - X * HANDOVER_DEPTH
        comfy = dict(margin=2 * JOINT_MARGIN)       # inside the reach, not at its edge
        ok = (m.reaches(T_g, H, X, **comfy) & m.reaches(T_g, H - X * APPROACH, X, **comfy)
              & m.reaches(T_t, H_t, -X, **comfy) & m.reaches(T_t, H_t + X * APPROACH, -X, **comfy))
        H = H[ok]
        start = np.array([cube["x"], cube["y"], (cube["level"] + 0.5) * SIZE])
        end = np.array([*target, SIZE / 2])
        return list(H[np.argsort(np.linalg.norm(H - start, axis=1) + np.linalg.norm(H - end, axis=1))])

    def _poses(self, kin, T_W_B, T_ee_cam, staged, cube, target) -> dict:
        """Every waypoint (W) and its joint solution, for one arm that picks and places. Finger yaws
        grip the cube square and set it down square to W; the ones closest to pointing away from the
        base (least wrist turn) are tried first and the first that works is taken."""
        for g_pick, opening in self._grips(T_W_B, cube):
            pick = self._pick(kin, T_W_B, T_ee_cam, staged, cube, g_pick, opening)
            if pick is None:
                continue
            cube_yaw = np.radians(cube["yaw"] or 0.0)
            yaws = self._release_yaws([g_pick - cube_yaw + m * np.pi / 2 for m in range(4)])
            place = self._place(kin, T_W_B, pick["grasp"], target, yaws, cube.get("dz", 0.0))
            if place is not None:
                return self._merge(pick, place)
        raise PlanningError("no straight-down grasp and place within the joint limits")

    def _grips(self, T_W_B, cube) -> list[tuple[float, float]]:
        """(finger yaw, opening) square to the cube with room for the open fingers on the way down (no
        cube around in their way): OPEN before OPEN_NARROW, least wrist turn first. PlanningError if
        there is room nowhere."""
        heading = np.arctan2(cube["y"] - T_W_B[1, 3], cube["x"] - T_W_B[0, 3])
        yaws = sorted((np.radians(cube["yaw"] or 0.0) + k * np.pi / 2 for k in range(4)),
                      key=lambda g: abs(_wrap(g - heading)))
        tip = max(TCP_FLOOR, cube.get("z_grasp", cube["level"] * SIZE + GRASP_TIP))
        out = [(g, w) for w in (OPEN, OPEN_NARROW) for g in yaws if not self._crowded((cube["x"], cube["y"]), tip, g, w)]
        if not out:
            raise PlanningError(f"cube {cube['track']} is boxed in: no pair of its sides has room for the fingers")
        return out

    def _pick(self, kin, T_W_B, T_ee_cam, staged, cube, g, opening=OPEN) -> dict | None:
        """The pick waypoints at finger yaw g (look, above_cube, grasp), the fingers `opening` apart on
        the way down, or None if one isn't reachable. The look pose aims the wrist camera at the cube (it
        sits beside the fingers, tilted)."""
        z_grasp = max(TCP_FLOOR, cube.get("z_grasp", cube["level"] * SIZE + GRASP_TIP))
        points = {"above_cube": np.array([cube["x"], cube["y"], max(self.travel, z_grasp + HOVER)]),
                  "grasp": np.array([cube["x"], cube["y"], z_grasp])}
        R = {"above_cube": down(g), "grasp": down(g)}
        qs, seed = {}, staged
        for key in ("above_cube", "grasp"):
            qs[key] = seed = self._ik(kin, T_W_B, points[key], R[key], seed)
            if seed is None:
                return None
        look = self._aim(kin, T_W_B, T_ee_cam, cube, g, staged)
        if look is None:
            raise PlanningError("no look pose aims the wrist camera at the cube")
        points["look"], R["look"], qs["look"] = look
        return {"kin": kin, "T_W_B": T_W_B, "T_ee_cam": T_ee_cam, "cube": cube, "staged": staged,
                "points": points, "R": R, "open": opening, **qs}

    def _place(self, kin, T_W_B, seed, target, yaws, dz=0.0) -> dict | None:
        """The place waypoints (above_target, place) at the first workable finger yaw, least wrist
        turn first; `dz`: the arm's height error measured at the cube."""
        heading = np.arctan2(target[1] - T_W_B[1, 3], target[0] - T_W_B[0, 3])
        points = {"above_target": np.array([*target, self.travel]), "place": np.array([*target, max(TCP_FLOOR, GRASP_TIP - SET_PRESS + dz)])}
        for g in sorted(yaws, key=lambda g: abs(_wrap(g - heading))):
            R = {"above_target": down(g), "place": down(g)}
            qa = self._ik(kin, T_W_B, points["above_target"], R["above_target"], seed)
            if qa is None:
                continue
            qp = self._ik(kin, T_W_B, points["place"], R["place"], qa)
            if qp is not None:
                return {"points": points, "R": R, "above_target": qa, "place": qp}
        return None

    @staticmethod
    def _merge(*parts) -> dict:
        out = {}
        for part in parts:
            for k, v in part.items():
                out[k] = {**out.get(k, {}), **v} if k in ("points", "R") else v
        return out

    def _aim(self, kin, T_W_B, T_ee_cam, cube, yaw, seed):
        """(point, rotation, joints) of a straight-down tool, at the highest of LOOK_Z that works,
        whose wrist camera's optical axis meets the cube's top (the reference demo's look pose)."""
        top = (cube["level"] + 1) * SIZE
        R = down(yaw)
        for z in LOOK_Z:
            xy = np.array([cube["x"], cube["y"]], float)
            for _ in range(4):
                p = np.array([*xy, z + cube["level"] * SIZE])
                q = self._ik(kin, T_W_B, p, R, seed)
                if q is None:
                    break
                T = T_W_B @ kin.fk(q) @ T_ee_cam
                o, d = T[:3, 3], T[:3, 2]
                if d[2] > -0.1:
                    q = None
                    break                                     # the camera doesn't look down at the table
                hit = o + d * (top - o[2]) / d[2]
                xy += np.array([cube["x"], cube["y"]]) - hit[:2]
            if q is not None:
                return p, R, q
        return None

    def _ik(self, kin, T_W_B, p_W, R_W, seed):
        """Joints for the tool at (p_W, R_W), nearest `seed`, JOINT_MARGIN inside the limits and with
        the joints CLEARANCE over the table; or None, also for a tool point outside the safety boundary
        (noted in `fenced`). IK returns only the solution nearest where it starts, and a descent can
        miss one that is there: failing from `seed`, it starts again from the reach map's solution
        nearby, then from mid-range."""
        if not self.robot.boundary.contains(p_W):
            self.fenced = True
            return None
        T_B = inv_T(T_W_B) @ make_T(R_W, p_W)

        def starts():
            if seed is not None:
                yield seed
            if (near := reach_map().seed(T_W_B, p_W, R_W)) is not None:
                yield near
            yield None                            # mid-range
        for start in starts():
            q = kin.ik(T_B[:3, 3], rotvec(T_B[:3, :3]), q_seed=start, seeds=IK_SEEDS)
            if q is not None and (np.all(q >= kin.lo + JOINT_MARGIN) and np.all(q <= kin.hi - JOINT_MARGIN)
                    and np.all(apply_T(T_W_B, kin.link_points(q)[2:-1])[:, 2] >= CLEARANCE)):
                return q
        return None
