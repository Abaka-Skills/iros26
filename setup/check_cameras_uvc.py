"""Grab one color frame from every camera macOS exposes (AVFoundation via OpenCV), save as PNGs.

Color only, no root. The RealSense SDK path (check_cameras.py) fails on macOS with
"failed to set power state" even under sudo. The terminal running this needs Camera
permission (System Settings > Privacy & Security > Camera); macOS prompts on first run.

  cd ~/Downloads/ALOHA && uv run python setup/check_cameras_uvc.py

Indices include the MacBook camera and any Continuity (iPhone) camera; the snapshots show which is which.
"""
import os
import time

import cv2

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_snapshots")
os.makedirs(OUT, exist_ok=True)

for i in range(8):
    cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    start = time.time()
    for _ in range(30):  # let auto-exposure settle
        ok, frame = cap.read()
    fps = 30 / (time.time() - start)
    if ok:
        cv2.imwrite(f"{OUT}/index{i}.png", frame)
        print(f"index {i}  OK  {frame.shape[1]}x{frame.shape[0]}  ~{fps:.0f} fps")
    else:
        print(f"index {i}  opened but no frame")
    cap.release()

print(f"Snapshots in {OUT}")
