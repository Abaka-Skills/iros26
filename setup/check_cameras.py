"""Open every RealSense camera, grab one color + depth frame each, save them as PNGs.

librealsense on macOS needs root to claim the cameras, and only one process can hold them: stop the
camera relay first (sudo launchctl bootout system/com.myaloha.camrelay), then
  cd ~/Downloads/ALOHA && sudo .venv/bin/python setup/check_cameras.py
"""
import os
import sys

sys.dont_write_bytecode = True  # run under sudo; don't leave a root-owned __pycache__ in setup/

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "camrelay", "src"))
from camrelay.realsense_macos import capture_realsense  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_snapshots")
os.makedirs(OUT, exist_ok=True)

print(f"Captured {capture_realsense()} camera(s) from macOS")
devices = rs.context().query_devices()
print(f"Found {len(devices)} RealSense device(s)")


def snapshot(i):
    try:  # on macOS a single camera can fail here with "failed to set power state"
        dev = devices[i]
        serial = dev.get_info(rs.camera_info.serial_number)
        usb = dev.get_info(rs.camera_info.usb_type_descriptor)
    except RuntimeError as e:
        print(f"device {i}  FAILED to open: {e}")
        return
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline = rs.pipeline()
    try:
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()  # m per unit
        for _ in range(15):  # let auto-exposure settle
            frames = pipeline.wait_for_frames(timeout_ms=5000)
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data())
        cv2.imwrite(f"{OUT}/{serial}_color.png", color)
        cv2.imwrite(f"{OUT}/{serial}_depth.png", cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET))
        valid = depth[depth > 0]
        print(f"{serial}  USB {usb}  OK  color {color.shape}  median depth {np.median(valid) * depth_scale * 1000 if valid.size else 0:.0f} mm")
    except Exception as e:
        print(f"{serial}  USB {usb}  FAILED: {e}")
    finally:
        try:
            pipeline.stop()
        except RuntimeError:
            pass


try:
    for i in range(len(devices)):
        snapshot(i)
finally:
    if os.environ.get("SUDO_UID"):  # hand the snapshots back to the normal user
        for f in os.listdir(OUT):
            os.chown(os.path.join(OUT, f), int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"]))
        os.chown(OUT, int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"]))
print(f"Snapshots in {OUT}")
