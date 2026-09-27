"""Simulated hardware for developing the panel without the rig: `python -m myaloha panel --sim`.

SimDriver stands in for trossen_arm.TrossenArmDriver: commanded positions are reached over their
goal_time, and an arm in external-effort mode (gravity comp) drifts as if a hand were moving it.
SimCamera produces labelled frames at the camera rate.
"""
import math
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import trossen_arm

N = 7


class SimDriver:
    ABSENT: set[str] = set()  # IPs that time out, like an unplugged arm

    def __init__(self):
        self.ip, self.configured = None, False
        self.mode = trossen_arm.Mode.idle
        self._lock = threading.Lock()
        self._start = self._goal = [0.0] * N
        self._t0, self._dur = 0.0, 0.0
        self._free_t0 = 0.0
        self._last = (time.monotonic(), [0.0] * N)

    def configure(self, model, end_effector, serv_ip, clear_error, timeout=20.0):
        if serv_ip in self.ABSENT:
            time.sleep(min(timeout, 0.5))
            raise RuntimeError(f"timed out connecting to {serv_ip}")
        self.ip, self.configured = serv_ip, True
        self._start = self._goal = [0.0, 0.05, 0.02, 0.0, 0.0, 0.0, 0.0]

    def get_is_configured(self):
        return self.configured

    def get_num_joints(self):
        return N

    def _pos(self) -> list[float]:
        now = time.monotonic()
        k = 1.0 if self._dur <= 0 else min(1.0, (now - self._t0) / self._dur)
        k = k * k * (3 - 2 * k)
        pos = [a + (b - a) * k for a, b in zip(self._start, self._goal)]
        if self.mode == trossen_arm.Mode.external_effort:  # a hand moves the floating arm
            t = now - self._free_t0
            ramp = min(1.0, t / 1.5)
            wave = [0.35 * math.sin(0.5 * t), 0.3 * math.sin(0.37 * t + 1), 0.3 * math.sin(0.43 * t + 2),
                    0.25 * math.sin(0.61 * t), 0.4 * math.sin(0.29 * t), 0.5 * math.sin(0.7 * t), 0.0]
            pos = [p + ramp * w for p, w in zip(pos, wave)]
            pos[6] = 0.022 + 0.018 * math.sin(0.9 * t) * ramp
        return pos

    def set_all_modes(self, mode=trossen_arm.Mode.idle):
        with self._lock:
            here = self._pos()
            self._start = self._goal = here
            self._dur = 0.0
            self.mode = mode
            self._free_t0 = time.monotonic()

    def get_modes(self):
        return [self.mode] * N

    def set_all_positions(self, goal_positions, goal_time=2.0, blocking=True, **_):
        with self._lock:
            self._start, self._goal = self._pos(), list(goal_positions)
            self._t0, self._dur = time.monotonic(), goal_time
        if blocking:
            time.sleep(goal_time)

    def set_all_external_efforts(self, efforts, goal_time=0.0, blocking=True):
        pass

    def get_all_positions(self):
        with self._lock:
            return self._pos()

    def get_robot_output(self):
        with self._lock:
            pos, now = self._pos(), time.monotonic()
        t_prev, p_prev = self._last
        vel = [(a - b) / max(now - t_prev, 1e-3) for a, b in zip(pos, p_prev)]
        self._last = (now, pos)
        zeros = [0.0] * N
        return SimpleNamespace(
            header=SimpleNamespace(timestamp=now, id=0),
            joint=SimpleNamespace(all=SimpleNamespace(positions=pos, velocities=vel, efforts=zeros, external_efforts=zeros)))

    def cleanup(self, reboot_controller=False):
        self.configured = False


class SimCamera:
    def __init__(self, key: str, label: str, fps: int = 30, width: int = 640, height: int = 480):
        self.key, self.label, self.fps, self.w, self.h = key, label, fps, width, height
        self.frame_lock = threading.Lock()
        self.latest_color_frame = self.latest_depth_frame = self.latest_timestamp = None
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        yy, xx = np.mgrid[0:self.h, 0:self.w]
        while not self._stop.wait(1 / self.fps):
            t = time.time()
            color = np.zeros((self.h, self.w, 3), np.uint8)
            color[..., 0] = (xx * 255 // self.w).astype(np.uint8)
            color[..., 1] = int(90 + 60 * math.sin(t))
            color[..., 2] = (yy * 255 // self.h).astype(np.uint8)
            cv2.putText(color, f"SIM {self.label}", (24, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
            cv2.putText(color, time.strftime("%H:%M:%S"), (24, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            r = np.hypot(xx - self.w / 2 - 120 * math.sin(t), yy - self.h / 2)
            depth = (200 + r * 1.5).astype(np.uint16)[..., None]  # mm
            with self.frame_lock:
                self.latest_color_frame, self.latest_depth_frame, self.latest_timestamp = color, depth, time.perf_counter()

    def disconnect(self):
        self._stop.set()


def install(absent_ips=()) -> None:
    """Swap the real arm driver for SimDriver. Call before creating Aloha."""
    SimDriver.ABSENT = set(absent_ips)
    trossen_arm.TrossenArmDriver = SimDriver


def add_cameras(rig, absent=()) -> None:
    for key, spec in rig.specs.items():
        if key in absent:
            rig.errors[key] = "sim: not connected"
        else:
            rig.add(key, SimCamera(key, spec.label, rig.fps, rig.width, rig.height), depth_mm_per_unit=1.0)
