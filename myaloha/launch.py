"""Check the hardware, then serve the API and the panel on localhost and open it.

    uv run python -m myaloha panel                  # real rig; re-runs itself under sudo for the cameras
    uv run python -m myaloha panel --no-cameras     # arms only, no sudo
    uv run python -m myaloha panel --sim            # no hardware
    uv run python -m myaloha panel --sim --absent leader_right,wrist_right

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


def ensure_root(args) -> None:
    """RealSense on macOS needs root: re-run this command under sudo, keeping the same Python."""
    if args.sim or args.no_cameras or sys.platform != "darwin" or os.geteuid() == 0:
        return
    print("Cameras need root on macOS; re-running under sudo (use --no-cameras to skip).")
    os.chdir(REPO)
    os.execvp("sudo", ["sudo", sys.executable, "-B", "-m", "myaloha", *sys.argv[1:]])


def check_network(arms) -> None:
    out = subprocess.run(["ifconfig", IFACE], capture_output=True, text=True).stdout
    if f"inet {HOST_IP} " not in out:
        if os.geteuid() == 0:
            subprocess.run(["ifconfig", IFACE, "alias", HOST_IP, "255.255.255.0"], check=False)
            print(f"  {OK} added {HOST_IP} to {IFACE}")
        else:
            print(f"  {NO} {IFACE} has no {HOST_IP}: run setup/find_arms.sh")
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


def open_browser(url: str) -> None:
    user = os.environ.get("SUDO_USER")
    if os.geteuid() == 0 and user:  # open in the desktop user's browser, not root's
        subprocess.run(["sudo", "-u", user, "open", url], check=False)
    else:
        webbrowser.open(url)


def main(args) -> None:
    ensure_root(args)
    import uvicorn

    from myaloha import sim
    from myaloha.cameras import CameraRig
    from myaloha.robot import DEFAULT_ARMS, Aloha
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

    absent = set(filter(None, args.absent.split(",")))
    if args.sim:
        sim.install(absent_ips=[a.ip for a in DEFAULT_ARMS if a.name in absent])

    print(f"\nmyaloha preflight{' (sim)' if args.sim else ''}\nnetwork")
    if args.sim:
        print(f"  {SKIP} simulated")
    else:
        check_network(DEFAULT_ARMS)

    from myaloha.mods.localize import load_latest

    rig = CameraRig()
    robot = Aloha(cameras=rig)
    robot.coordinates = load_latest()
    root.removeHandler(console)  # the report below says it more compactly
    robot.connect(timeout=args.timeout)
    if not args.sim and len(robot.connected()) < len(robot.arms):
        time.sleep(1.5)  # right after a restart the controllers may still be closing the old session
        robot.connect(timeout=args.timeout)
    if args.sim:
        sim.add_cameras(rig, absent)
    elif args.no_cameras:
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
        threading.Timer(1.0, open_browser, [url]).start()
    from myaloha.topics.manager import TopicManager
    topics = TopicManager(robot, url.rstrip("/"))
    print(f"mcp    {url}mcp\n")
    try:
        uvicorn.run(create_app(robot, logs, sim=args.sim, topics=topics), host="127.0.0.1", port=args.port,
                    log_level="warning", timeout_graceful_shutdown=2)  # camera streams never end on their own
    finally:
        topics.stop_all()
        robot.stop()
        deadline = time.monotonic() + 10
        while robot.mods and time.monotonic() < deadline:  # let the mod put its arms on hold
            time.sleep(0.05)
        robot.disconnect()
        print("hardware released")
