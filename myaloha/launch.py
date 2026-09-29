"""Check the hardware, then serve the API and the panel on localhost and open it.

    uv run python -m myaloha panel                  # real rig: the arms, and the cameras from camrelay
    uv run python -m myaloha panel --no-cameras     # arms only
    uv run python -m myaloha panel --simulator 127.0.0.1:8810  # a simulator's arms and cameras

No root needed: the cameras are opened by the camera relay (camrelay/, a launchd daemon), which also
keeps the arms' network address on the USB Ethernet adapter.

Nothing here moves an arm: connecting only configures the drivers. Ctrl-C stops any running
mod (its arms hold), then releases the hardware.
"""
import logging
import os
import subprocess
import sys
import threading
import time
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
IFACE, HOST_IP = "en14", "192.168.10.200"

OK, NO, SKIP = "\033[32m✓\033[0m", "\033[31m✗\033[0m", "\033[33m–\033[0m"


def check_network(arms) -> None:
    out = subprocess.run(["ifconfig", IFACE], capture_output=True, text=True).stdout
    if f"inet {HOST_IP} " not in out:
        print(f"  {NO} {IFACE} has no {HOST_IP} (camrelay adds it when installed with --iface; or run "
              f"setup/find_arms.sh)")
    else:
        print(f"  {OK} {IFACE} {HOST_IP}")

    def ping(ip):
        return subprocess.run(["ping", "-c1", "-t1", ip], capture_output=True).returncode == 0

    with ThreadPoolExecutor(len(arms)) as pool:
        for arm, up in zip(arms, pool.map(ping, [a.ip for a in arms])):
            mark = OK if up else SKIP if arm.role == "leader" else NO  # leaders are optional
            print(f"  {mark} ping {arm.ip:<13} {arm.name}{'' if up else ' (unreachable)'}")


def report(robot) -> None:
    print("arms")
    for arm in robot.arms.values():
        s = arm.status()
        if s["connected"]:
            pos = " ".join(f"{p:+.2f}" for p in s["pos"])
            print(f"  {OK} {arm.name:<15} {arm.spec.ip:<13} {'/'.join(s['mode']):<16} [{pos}]")
        else:
            mark = SKIP if arm.spec.role == "leader" else NO
            print(f"  {mark} {arm.name:<15} {arm.spec.ip:<13} absent: {s['error']}")
    print("cameras")
    for key, c in robot.cameras.status().items():
        if c["ok"]:
            print(f"  {OK} {key:<15} {c['serial']:<13} {c['fps']:.0f} fps")
        else:
            print(f"  {NO} {key:<15} {c['serial']:<13} {c['error']}")
    print("mods")
    from myaloha.mods import MODS
    for name, cls in MODS.items():
        r = cls.requirements(robot)
        unmet = [n["text"] for n in r["needs"] if not n["met"]]
        print(f"  {OK if r['met'] else NO} {name:<15} {'ready' if r['met'] else 'blocked: needs ' + '; '.join(unmet)}")


def main(args) -> None:
    import uvicorn

    from myaloha import colors, cubes
    from myaloha.cameras import CameraRig
    from myaloha.mods import localize
    from myaloha.robot import DEFAULT_ARMS, Aloha
    from myaloha.safety import Boundary
    from myaloha.server import LogBuffer, create_app

    sys.stdout.reconfigure(line_buffering=True)
    logs = LogBuffer()
    root = logging.getLogger("myaloha")
    root.setLevel(logging.INFO)
    root.propagate = False  # LeRobot configures the root logger on import; don't print twice
    root.addHandler(logs)
    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter("%(asctime)s %(name)s %(message)s", "%H:%M:%S"))
    root.addHandler(console)

    calib = Path(args.calib) if args.calib else REPO / "calib" / (
        args.simulator.replace(":", "_") if args.simulator else "")      # each set of devices is its own rig
    localize.CALIB = calib / "coordinates"

    print("\nmyaloha preflight\nnetwork")
    if args.simulator:
        print(f"  {OK} simulator at {args.simulator}")
    else:
        check_network(DEFAULT_ARMS)

    rig = CameraRig(hardware=args.simulator or args.cameras)
    robot = Aloha(cameras=rig, hardware=args.simulator)
    robot.coordinates = localize.load_latest()
    print(f"calibration\n  {OK if robot.coordinates else SKIP} {os.path.relpath(localize.CALIB)}: "
          f"{robot.coordinates['id'] if robot.coordinates else 'none yet (run localize)'}")
    cubes.use_profile(colors.load(args.colors))
    print(f"  {OK} colours: {cubes.PALETTE.name} ({cubes.PALETTE.doc.get('note', '')})")
    robot.boundary = Boundary.load(calib / "boundary.yaml")
    print(f"  {OK} safety boundary: " + ", ".join(f"{k} {v * 100:.0f}" for k, v in robot.boundary.margins.items())
          + f" cm in from the table's edges{'' if robot.boundary.path.exists() else ' (default)'}")
    root.removeHandler(console)  # the report below says it more compactly
    robot.connect(timeout=args.timeout)
    if not args.simulator and len(robot.connected()) < len(robot.arms):
        time.sleep(1.5)  # right after a restart the controllers may still be closing the old session
        robot.connect(timeout=args.timeout)
    if args.no_cameras:
        rig.errors.update({k: "skipped (--no-cameras)" for k in rig.specs})
    else:
        rig.connect()
    robot.start_monitor(hz=args.hz)  # before the report: an arm without fresh output counts as lost
    time.sleep(1.2)  # let the camera watcher measure a frame rate
    report(robot)
    root.addHandler(console)

    url = f"http://127.0.0.1:{args.port}/"
    print(f"\npanel  {url}\napi    {url}docs")
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, [url]).start()
    from myaloha.topics.manager import TopicManager
    topics = TopicManager(robot, url.rstrip("/"))
    print(f"mcp    {url}mcp\n")
    try:
        uvicorn.run(create_app(robot, logs, topics=topics), host="127.0.0.1", port=args.port,
                    log_level="warning", timeout_graceful_shutdown=2)  # camera streams never end on their own
    finally:
        topics.stop_all()
        robot.stop()
        deadline = time.monotonic() + 10
        while robot.mods and time.monotonic() < deadline:  # let the mod put its arms on hold
            time.sleep(0.05)
        robot.disconnect()
        print("hardware released")
