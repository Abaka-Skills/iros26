"""An arm controller as the driver sees it: modes, commanded positions reached over their goal
time (the motor targets), and the robot output. A follower's targets drive its servos in the
world's physics, and what it reads is what the world measures (a blocked link, fingers stopped on
a cube). A leader in external-effort mode (gravity comp) drifts as if an operator were moving it."""
import math
import threading
import time
from types import SimpleNamespace

from simulator.protocol import Mode

N = 7
BOOT = [0.0, 0.05, 0.02, 0.0, 0.0, 0.0, 0.0]    # resting on its stops at power-up


class ArmController:
    def __init__(self, ip: str, world=None):
        self.ip, self.world = ip, world          # world: only for followers
        self.plugged = True
        self.mode = self.gripper_mode = Mode("idle")
        self._lock = threading.Lock()
        self._start, self._goal = list(BOOT), list(BOOT)
        self._t0, self._dur = [0.0] * N, [0.0] * N     # each joint interpolates on its own
        self._free_t0 = 0.0
        self._last = (time.monotonic(), list(BOOT))

    def _pos(self) -> list[float]:
        now = time.monotonic()
        pos = []
        for a, b, t0, dur in zip(self._start, self._goal, self._t0, self._dur):
            k = 1.0 if dur <= 0 else min(1.0, (now - t0) / dur)
            pos.append(a + (b - a) * k * k * (3 - 2 * k))
        if self.mode == "external_effort" and not self.world:   # an operator moves the leader
            t = now - self._free_t0
            ramp = min(1.0, t / 1.5)
            wave = [0.35 * math.sin(0.5 * t), 0.3 * math.sin(0.37 * t + 1), 0.3 * math.sin(0.43 * t + 2),
                    0.25 * math.sin(0.61 * t), 0.4 * math.sin(0.29 * t), 0.5 * math.sin(0.7 * t), 0.0]
            pos = [p + ramp * w for p, w in zip(pos, wave)]
            pos[6] = 0.022 + 0.018 * math.sin(0.9 * t) * ramp
        return pos

    def motor(self) -> list[float]:
        """The motor targets now."""
        with self._lock:
            return self._pos()

    def _read(self, pos):
        """What the encoders report for targets `pos`: a follower's are measured in the world."""
        return self.world.reading(self.world.measured(self.ip, pos)) if self.world else pos

    def _hold(self, joints) -> None:
        here = self._pos()
        for j in joints:
            self._start[j] = self._goal[j] = here[j]
            self._dur[j] = 0.0

    def _go(self, joints, goals, goal_time) -> None:
        here, now = self._pos(), time.monotonic()
        for j, g in zip(joints, goals):
            self._start[j], self._goal[j], self._t0[j], self._dur[j] = here[j], float(g), now, goal_time

    # ---- the driver's calls ----
    def get_num_joints(self):
        return N

    def _set_modes(self, joints, mode, arm: bool, gripper: bool) -> None:
        with self._lock:
            self._hold(joints)
            if arm:
                self.mode, self._free_t0 = mode, time.monotonic()
            if gripper:
                self.gripper_mode = mode

    def _move(self, joints, goals, goal_time, blocking) -> None:
        with self._lock:
            self._go(joints, goals, goal_time)
        if blocking:
            time.sleep(goal_time)

    def set_all_modes(self, mode=Mode("idle")):
        self._set_modes(range(N), mode, arm=True, gripper=True)

    def set_arm_modes(self, mode=Mode("idle")):
        self._set_modes(range(N - 1), mode, arm=True, gripper=False)

    def set_gripper_mode(self, mode=Mode("idle")):
        self._set_modes([N - 1], mode, arm=False, gripper=True)

    def get_modes(self):
        return [self.mode] * (N - 1) + [self.gripper_mode]

    def set_all_positions(self, goal_positions, goal_time=2.0, blocking=True, **_):
        self._move(range(N), goal_positions, goal_time, blocking)

    def set_arm_positions(self, goal_positions, goal_time=2.0, blocking=True, **_):
        self._move(range(N - 1), goal_positions, goal_time, blocking)

    def set_gripper_position(self, goal_position, goal_time=2.0, blocking=True, **_):
        self._move([N - 1], [goal_position], goal_time, blocking)

    def set_all_external_efforts(self, efforts, goal_time=0.0, blocking=True):
        pass

    def set_gripper_external_effort(self, effort, goal_time=0.0, blocking=True):
        pass

    def get_all_positions(self):
        with self._lock:
            return self._read(self._pos())

    def get_gripper_position(self):
        return self.get_all_positions()[N - 1]

    def get_robot_output(self):
        with self._lock:
            pos, now = self._pos(), time.monotonic()
        t_prev, p_prev = self._last
        vel = [(a - b) / max(now - t_prev, 1e-3) for a, b in zip(pos, p_prev)]
        self._last = (now, pos)
        eff, ext = self.world.efforts(self.ip) if self.world else ([0.0] * N, [0.0] * N)
        return SimpleNamespace(
            header=SimpleNamespace(timestamp=now, id=0),
            joint=SimpleNamespace(all=SimpleNamespace(positions=self._read(pos), velocities=vel, efforts=eff,
                                                      external_efforts=ext)))

    def cleanup(self, reboot_controller=False):
        pass
