"""The four RealSense D405s (colour + depth), read over the network device protocol from wherever
they are served: the camera relay on this machine (camrelay/, the only process that opens USB; on
macOS that needs root, so the backend never does) or a simulator. The backend reads them the same
way either way. A watcher thread measures each camera's real frame rate, and a resume thread
reconnects any camera that goes down (the relay drops a camera that is unplugged and serves it again
when it is back) or comes up late.
"""
import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger("myaloha.camera")

STALE_S = 0.5  # no new frame for this long: the camera is down
RESUME_S = 2.0  # a camera that is down is asked for again this often (a cheap refusal while it is away)
RELAY = "127.0.0.1:8820"  # where camrelay serves the rig's cameras


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
    def __init__(self, specs=DEFAULT_CAMERAS, width=640, height=480, fps=30, depth_range_mm=(70.0, 1000.0),
                 hardware: str = RELAY):
        """`hardware`: host:port where the cameras are served (the relay, or a simulator)."""
        self.specs = {s.key: s for s in specs}
        self.hardware = hardware
        self.width, self.height, self.fps = width, height, fps
        self.depth_range_mm = depth_range_mm
        self.cams: dict = {}  # key -> connected camera (a netdev.CameraClient)
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
        self._open(list(self.specs))
        threading.Thread(target=self._resume, name="camera-resume", daemon=True).start()

    def _resume(self) -> None:
        """Ask again, every RESUME_S, for each camera that is down: whoever serves the cameras says at
        once whether it has it (the relay follows the USB side), so asking is cheap."""
        down: set[str] = set()
        while not self._watch_stop.wait(RESUME_S):
            live = self.status()
            for key in self.specs:
                if live[key]["ok"] and key in down:
                    down.discard(key)
                    log.info("%s camera resumed", key)
                elif not live[key]["ok"] and key not in down:
                    down.add(key)
                    log.warning("%s camera down; asking for it every %.0f s", key, RESUME_S)
            if down:
                self.reconnect()

    def reconnect(self) -> list[str]:
        """Reopen every camera that is down (e.g. unplugged and plugged back in). Returns their keys."""
        with self._open_lock:     # the resume thread and the API may both ask
            live = self.status()
            down = [k for k in self.specs if not live[k]["ok"]]
            if not down:
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
        from myaloha.netdev import CameraClient
        for key in keys:
            spec = self.specs[key]
            try:
                cam = CameraClient(self.hardware, spec.serial)
                cam.connect(timeout=2.0)
                self.add(key, cam, cam.depth_mm_per_unit, cam.optics)
            except ConnectionRefusedError:
                self._fail(key, spec, f"nothing serves cameras at {self.hardware} (is camrelay running?)", quiet)
            except Exception as e:
                self._fail(key, spec, str(e), quiet)

    def _fail(self, key, spec, why: str, quiet: bool) -> None:
        if not quiet or self.errors.get(key) != why:           # retries: say it once
            log.warning("%s (%s) not connected: %s", key, spec.serial, why)
        self.errors[key] = why

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
