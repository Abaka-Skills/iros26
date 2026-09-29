"""RealSense cameras on USB as a relay source: which are attached, and each one opened and streaming
(colour BGR and depth, the same size and rate), its frames stamped on the machine's monotonic clock."""
import sys
import threading
import time

import numpy as np

MISSES = 3                 # frame waits in a row that time out: the camera is gone (unplugged, or wedged)


class RealSenseSource:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        import pyrealsense2 as rs                      # before capture_realsense (it uses the same libusb)
        self.rs, self.size, self.fps = rs, (width, height), fps
        self._listed = (None, set())                   # (the USB addresses, the serials) at the last listing

    def serials(self) -> set[str]:
        """Serials of the cameras attached now. On macOS, newly attached ones are first taken from the
        system's UVC driver (see realsense_macos): without that, librealsense can't power them up. There
        librealsense lists them again only when the cameras on USB change: listing powers each camera up,
        which fails ("failed to set power state") for those already streaming."""
        if sys.platform == "darwin":
            from camrelay.realsense_macos import capture_realsense, usb_addresses
            usb = usb_addresses()
            if usb == self._listed[0]:
                return set(self._listed[1])
            capture_realsense()
        else:
            usb = None
        serials = {d.get_info(self.rs.camera_info.serial_number) for d in self.rs.context().query_devices()}
        self._listed = (usb, serials)
        return set(serials)

    def open(self, serial: str) -> "Camera":
        return Camera(self.rs, serial, *self.size, self.fps)


class Camera:
    """One camera streaming; `frame(after)` waits for the next one. `alive` turns False when frames stop."""

    def __init__(self, rs, serial: str, width: int, height: int, fps: int):
        self.serial = serial
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipe.start(cfg)
        self.optics = read_optics(rs, profile)
        self.depth_mm_per_unit = profile.get_device().first_depth_sensor().get_depth_scale() * 1000
        self.alive, self.latest = True, None                # latest: (seq, t, colour, depth)
        self.cond = threading.Condition()
        threading.Thread(target=self._run, name=f"realsense-{serial}", daemon=True).start()

    def _run(self) -> None:
        seq, misses = 0, 0
        while self.alive:
            try:
                ok, frames = self.pipe.try_wait_for_frames(1000)
            except RuntimeError:
                ok = False
            if not ok:
                misses += 1
                if misses >= MISSES:
                    break
                continue
            misses = 0
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                continue
            seq += 1
            item = (seq, time.perf_counter(), np.asanyarray(color.get_data()).copy(),
                    np.asanyarray(depth.get_data()).copy())
            with self.cond:
                self.latest = item
                self.cond.notify_all()
        self.close()

    def frame(self, after: int, timeout: float = 1.0):
        """The first frame newer than seq `after`, or None (timed out, or the camera is gone)."""
        with self.cond:
            self.cond.wait_for(lambda: not self.alive or (self.latest and self.latest[0] > after), timeout)
            return self.latest if self.alive and self.latest and self.latest[0] > after else None

    def close(self) -> None:
        if not self.alive and getattr(self, "_closed", False):
            return
        self.alive, self._closed = False, True
        with self.cond:
            self.cond.notify_all()
        try:
            self.pipe.stop()
        except RuntimeError:
            pass                                          # already stopped with the device


def read_optics(rs, profile) -> dict:
    """Colour and depth intrinsics and the depth->colour transform, as lists (myaloha.cameras' form)."""
    cs = profile.get_stream(rs.stream.color).as_video_stream_profile()
    ds = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    ex = ds.get_extrinsics_to(cs)
    T = np.eye(4)
    T[:3, :3] = np.array(ex.rotation).reshape(3, 3).T    # librealsense stores column-major
    T[:3, 3] = ex.translation
    ci, di = cs.get_intrinsics(), ds.get_intrinsics()

    def K(i):
        return [[i.fx, 0.0, i.ppx], [0.0, i.fy, i.ppy], [0.0, 0.0, 1.0]]
    aligned = bool(np.allclose(T, np.eye(4), atol=1e-4) and np.allclose(K(ci), K(di), atol=0.5))
    return {"K_color": K(ci), "K_depth": K(di), "T_color_depth": T.tolist(), "aligned": aligned,
            "dist_color": [float(c) for c in ci.coeffs], "model_color": str(ci.model).split(".")[-1]}
