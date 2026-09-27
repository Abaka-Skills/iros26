"""The four RealSense D405s, through LeRobot's RealSenseCamera (color + depth).

On macOS the cameras need root, and myaloha.realsense_macos.capture_realsense() must run before
any rs.context(); without root every camera is reported as unavailable instead of failing.
A watcher thread measures each camera's real frame rate from LeRobot's frame timestamps, and a
resume thread reopens any camera that goes down (a dropped cable, a replug) or comes up late.
"""
import logging
import os
import sys
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger("myaloha.camera")

STALE_S = 0.5  # no new frame for this long: the camera is down
RESUME_S = 3.0  # a camera down this long gets one reopen; after that, one whenever the USB devices change
RETRY_S = 60.0  # ... or, where USB can't be watched, at most this often
# A failed RealSense open blocks for seconds holding the GIL, stalling every thread in the process
# (arm monitor, API, emergency stop): never retry in a tight loop.


@dataclass(frozen=True)
class CamSpec:
    key: str
    serial: str
    label: str
    note: str


# Serials and roles from the camera bring-up; wrist sides from what each wrist sees (2026-09-26).
DEFAULT_CAMERAS = (
    CamSpec("high", "218622271652", "High", "top-down"),
    CamSpec("low", "230322270466", "Low", "table level"),
    CamSpec("wrist_left", "218622271803", "Wrist L", "left follower"),
    CamSpec("wrist_right", "218622272313", "Wrist R", "right follower"),
)


class CameraRig:
    def __init__(self, specs=DEFAULT_CAMERAS, width=640, height=480, fps=30, depth_range_mm=(70.0, 1000.0)):
        self.specs = {s.key: s for s in specs}
        self.width, self.height, self.fps = width, height, fps
        self.depth_range_mm = depth_range_mm
        self.cams: dict = {}  # key -> connected camera (LeRobot RealSenseCamera, or a sim camera)
        self.errors: dict[str, str] = {}
        self._depth_mm: dict[str, float] = {}  # mm per depth unit, per camera
        self.optics: dict[str, dict] = {}      # colour K, depth K, depth->colour transform
        self._rate: dict[str, dict] = {}
        self._jpeg: dict[tuple, tuple] = {}
        self._watch_stop = threading.Event()
        self._watcher: threading.Thread | None = None
        self._open_lock = threading.Lock()

    # ---- connections ----
    def connect(self) -> None:
        if _needs_root():
            for key in self.specs:
                self.errors[key] = "needs root (run the panel under sudo)"
            log.warning("cameras skipped: RealSense on macOS needs root")
            return
        self._open(list(self.specs))
        threading.Thread(target=self._resume, name="camera-resume", daemon=True).start()

    def _resume(self) -> None:
        """Reopen cameras that go down: once, then whenever the RealSense devices on USB change (an
        unplug and replug), or at most every RETRY_S where USB can't be watched."""
        from myaloha.realsense_macos import usb_addresses
        down_since: dict[str, float] = {}
        tried_at, tried_usb = 0.0, None
        while not self._watch_stop.wait(RESUME_S):
            now = time.monotonic()
            live = self.status()
            for key in self.specs:
                if live[key]["ok"]:
                    if key in down_since:
                        log.info("%s camera resumed", key)
                    down_since.pop(key, None)
                elif key not in down_since:
                    down_since[key] = now
                    tried_usb = None                                  # a new outage: try once
                    log.warning("%s camera down; reopening it", key)
            if not down_since:
                continue
            usb = usb_addresses()
            if tried_usb is None or (usb and usb != tried_usb) or (not usb and now - tried_at > RETRY_S):
                tried_at, tried_usb = now, usb
                back = self.reconnect()
                still = [k for k in down_since if k not in back]
                if still:
                    log.warning("%s still down; unplug and replug %s to retry", ", ".join(still),
                                "it" if len(still) == 1 else "them")

    def reconnect(self) -> list[str]:
        """Reopen every camera that is down (e.g. unplugged and plugged back in). Returns their keys."""
        with self._open_lock:     # the resume thread and the API may both ask
            live = self.status()
            down = [k for k in self.specs if not live[k]["ok"]]
            if not down or _needs_root():
                return []
            for key in down:
                cam = self.cams.pop(key, None)
                if cam is not None:
                    try:
                        cam.disconnect()
                    except Exception:
                        pass   # its read loop already died with the device
            self._open(down, quiet=True)
            return [k for k in down if k in self.cams]

    def _open(self, keys: list[str], quiet: bool = False) -> None:
        import pyrealsense2  # noqa: F401  (must load before capture_realsense)
        from lerobot.cameras.configs import ColorMode
        from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

        from myaloha.realsense_macos import capture_realsense

        n = capture_realsense()
        if n or not quiet:
            log.info("captured %d RealSense device(s) from macOS", n)
        for key in keys:
            spec = self.specs[key]
            cfg = RealSenseCameraConfig(serial_number_or_name=spec.serial, fps=self.fps, width=self.width,
                                        height=self.height, color_mode=ColorMode.BGR, use_depth=True)
            cam = RealSenseCamera(cfg)
            try:
                cam.connect(warmup=True)
                scale = cam.rs_profile.get_device().first_depth_sensor().get_depth_scale()
                self.add(key, cam, depth_mm_per_unit=scale * 1000, optics=read_optics(cam.rs_profile))
            except Exception as e:
                if not quiet or self.errors.get(key) != str(e):   # retries: say it once
                    log.warning("%s (%s) not connected: %s", key, spec.serial, e)
                self.errors[key] = str(e)

    def add(self, key: str, cam, depth_mm_per_unit: float, optics: dict | None = None) -> None:
        """Register a connected camera; anything with frame_lock/latest_*_frame/latest_timestamp works."""
        self.cams[key] = cam
        self._depth_mm[key] = depth_mm_per_unit
        self.optics[key] = optics or default_optics(self.width, self.height)
        self.errors.pop(key, None)
        log.info("%s camera connected (%s)", key, self.specs[key].serial)
        if not self._watcher:
            self._watcher = threading.Thread(target=self._watch, name="camera-watch", daemon=True)
            self._watcher.start()

    def disconnect(self) -> None:
        self._watch_stop.set()
        for key, cam in list(self.cams.items()):
            try:
                cam.disconnect()
            except Exception as e:
                log.warning("%s: disconnect failed: %s", key, e)
        self.cams.clear()

    # ---- frames ----
    def _latest(self, key: str, kind: str):
        cam = self.cams.get(key)
        if cam is None:
            return None, None
        with cam.frame_lock:
            frame = cam.latest_depth_frame if kind == "depth" else cam.latest_color_frame
            return frame, cam.latest_timestamp

    def jpeg(self, key: str, kind: str = "color", quality: int = 80) -> bytes | None:
        """The newest frame as JPEG (depth colorized), encoded once per frame."""
        frame, stamp = self._latest(key, kind)
        if frame is None:
            return None
        cached = self._jpeg.get((key, kind))
        if cached and cached[0] == stamp:
            return cached[1]
        img = self.colorize(frame, self._depth_mm[key]) if kind == "depth" else frame
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        data = buf.tobytes() if ok else None
        self._jpeg[(key, kind)] = (stamp, data)
        return data

    def capture(self, key: str, n: int = 5, after: float | None = None, timeout: float = 3.0):
        """(colour BGR, depth in metres aligned to colour, colour K, perf_counter time of the last
        frame) from `n` new frames taken after perf_counter time `after`; depth is the per-pixel
        median (0 = no reading)."""
        colors, depths, last = [], [], None
        end = time.monotonic() + timeout
        while True:
            if time.monotonic() > end:
                raise TimeoutError(f"{key}: no fresh frames")
            color, stamp = self._latest(key, "color")
            depth, _ = self._latest(key, "depth")
            if stamp is not None and stamp != last and (after is None or stamp > after):
                last = stamp
                colors.append(color.copy())
                depths.append(depth[..., 0].astype(np.float32) * self._depth_mm[key] / 1000)
                if len(depths) == n:
                    break
            time.sleep(0.005)
        if n == 1:
            depth = depths[0]
        else:
            d = np.stack(depths)
            d[d == 0] = np.nan
            with np.errstate(all="ignore"):
                depth = np.nan_to_num(np.nanmedian(d, axis=0), nan=0.0).astype(np.float32)
        o = self.optics[key]
        if not o["aligned"]:
            depth = align_depth(depth, o["K_depth"], o["K_color"], o["T_color_depth"], colors[-1].shape[:2])
        return colors[-1], depth, o["K_color"], last

    def distortion(self, key: str) -> tuple[np.ndarray, str]:
        """(coefficients k1 k2 p1 p2 k3, RealSense model name) of the colour stream."""
        o = self.optics[key]
        return o["dist_color"], o["model_color"]

    def colorize(self, depth: np.ndarray, mm_per_unit: float) -> np.ndarray:
        near, far = self.depth_range_mm
        mm = depth[..., 0].astype(np.float32) * mm_per_unit
        norm = np.clip((mm - near) / (far - near), 0, 1)
        img = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        img[mm <= 0] = 0  # no reading: black
        return img

    # ---- status ----
    def _watch(self) -> None:
        while not self._watch_stop.wait(1 / 90):
            now = time.monotonic()
            for key in list(self.cams):
                _, stamp = self._latest(key, "color")
                r = self._rate.setdefault(key, {"stamp": None, "seen": now, "n": 0, "t0": now, "fps": 0.0})
                if stamp is not None and stamp != r["stamp"]:
                    r["stamp"], r["seen"], r["n"] = stamp, now, r["n"] + 1
                if now - r["t0"] >= 1.0:
                    r["fps"], r["n"], r["t0"] = r["n"] / (now - r["t0"]), 0, now

    def status(self) -> dict:
        now, out = time.monotonic(), {}
        for key, spec in self.specs.items():
            r = self._rate.get(key)
            live = key in self.cams and r is not None and now - r["seen"] < STALE_S
            out[key] = {"serial": spec.serial, "label": spec.label, "note": spec.note,
                        "ok": live, "fps": round(r["fps"], 1) if live else 0.0,
                        "error": self.errors.get(key) or (None if live or key not in self.cams else "no frames")}
        return out


def _needs_root() -> bool:
    """RealSense on macOS works only as root (see realsense_macos)."""
    return sys.platform == "darwin" and os.geteuid() != 0


def _K(intr) -> np.ndarray:
    return np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.0]])


def read_optics(profile) -> dict:
    """Colour and depth intrinsics and the depth->colour transform from a RealSense profile."""
    import pyrealsense2 as rs
    cs = profile.get_stream(rs.stream.color).as_video_stream_profile()
    ds = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    ex = ds.get_extrinsics_to(cs)
    T = np.eye(4)
    T[:3, :3] = np.array(ex.rotation).reshape(3, 3).T  # librealsense stores column-major
    T[:3, 3] = ex.translation
    ci = cs.get_intrinsics()
    Kc, Kd = _K(ci), _K(ds.get_intrinsics())
    aligned = bool(np.allclose(T, np.eye(4), atol=1e-4) and np.allclose(Kc, Kd, atol=0.5))
    return {"K_color": Kc, "K_depth": Kd, "T_color_depth": T, "aligned": aligned,
            "dist_color": np.array(ci.coeffs, float), "model_color": str(ci.model).split(".")[-1]}


def default_optics(width: int, height: int) -> dict:
    K = np.array([[width * 0.6, 0, width / 2], [0, width * 0.6, height / 2], [0, 0, 1.0]])
    return {"K_color": K, "K_depth": K, "T_color_depth": np.eye(4), "aligned": True,
            "dist_color": np.zeros(5), "model_color": "brown_conrady"}


def align_depth(depth: np.ndarray, Kd, Kc, T_c_d, shape) -> np.ndarray:
    """Reproject a depth image into the colour camera (nearest surface wins)."""
    v, u = np.nonzero(depth > 0)
    z = depth[v, u]
    P = np.stack([(u - Kd[0, 2]) * z / Kd[0, 0], (v - Kd[1, 2]) * z / Kd[1, 1], z], -1) @ T_c_d[:3, :3].T + T_c_d[:3, 3]
    uc = np.round(P[:, 0] / P[:, 2] * Kc[0, 0] + Kc[0, 2]).astype(int)
    vc = np.round(P[:, 1] / P[:, 2] * Kc[1, 1] + Kc[1, 2]).astype(int)
    ok = (uc >= 0) & (uc < shape[1]) & (vc >= 0) & (vc < shape[0]) & (P[:, 2] > 0)
    out = np.zeros(shape, np.float32)
    order = np.argsort(-P[ok, 2])  # far first, so near overwrites
    out[vc[ok][order], uc[ok][order]] = P[ok, 2][order]
    return out
