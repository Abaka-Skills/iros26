"""The environment for tests: the simulator (simulator/, a project of its own) run as a separate
process on free ports. The system under test connects to its devices by address, as with
`panel --simulator`, and never sees its ground truth; tests use the truth (`Sim.truth`) only to
judge what the system did."""
import json
import os
import shutil
import subprocess
import threading
import time
from collections import deque
from contextlib import contextmanager
from pathlib import Path

from myaloha import colors, cubes, netdev
from myaloha.cameras import CameraRig
from myaloha.robot import Aloha

SIMULATOR = Path(__file__).resolve().parent.parent / "simulator"


class Sim:
    """A running simulator: its device address, and its own control calls."""

    def __init__(self, proc: subprocess.Popen, address: str, page: str):
        self.proc, self.address, self.page = proc, address, page

    def call(self, name: str, *args):
        sock, _ = netdev.connect(self.address, {"open": "control"})
        conn = netdev.Connection(sock)
        try:
            return conn.call(name, *args)
        finally:
            conn.close()

    def truth(self) -> dict:
        return self.call("truth")

    def spawn(self, cubes: list) -> dict:
        return self.call("spawn", cubes)

    def unplug(self, name: str) -> None:
        self.call("unplug", name)

    def plug(self, name: str) -> None:
        self.call("plug", name)


@contextmanager
def simulated_rig(unplugged=(), cubes=None, noise=0.0, seed=1):
    """The simulator, started for the test and stopped after. `cubes`: [{color, x, y, yaw, level}]
    (default: none); `noise`: scale of its errors (0: exact)."""
    sets = {"listen": "127.0.0.1:0", "ui": "127.0.0.1:0", "seed": seed, "noise": noise,
            "unplugged": list(unplugged), "cubes.count": 0, "cubes.scene": cubes or []}
    cmd = [shutil.which("uv"), "run", "--project", str(SIMULATOR), "python", "-B", "-m", "simulator"]
    for key, value in sets.items():
        cmd += ["--set", f"{key}={json.dumps(value)}"]
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"}    # its own environment, not ours
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, cwd=SIMULATOR)
    lines: deque = deque(maxlen=200)
    address = page = None
    try:
        end = time.monotonic() + 120                    # the first run may set up its environment
        for line in proc.stdout:
            lines.append(line)
            if line.startswith("simulated rig at "):
                address = line.split()[3]
            elif line.startswith("page:"):
                page = line.split()[1]
                break
            if time.monotonic() > end:
                break
        if not address:
            raise RuntimeError("simulator did not start:\n" + "".join(lines))
        threading.Thread(target=lambda: lines.extend(proc.stdout), daemon=True).start()   # keep it draining
        yield Sim(proc, address, page)
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()


def connect(address: str) -> Aloha:
    """The robot on the simulator's devices, monitored, with frames flowing, recognising colours by
    the simulated rig's own profile (config/colors/sim.yaml, calibrated on it like any rig: `panel
    --simulator HOST:PORT --colors sim`)."""
    cubes.use_profile(colors.load("sim"))
    rig = CameraRig(hardware=address)
    robot = Aloha(cameras=rig, hardware=address)
    robot.connect(timeout=0.2)
    rig.connect()
    robot.start_monitor(hz=100)
    time.sleep(1.2)                           # a measured frame rate, fresh arm output
    return robot


def release(robot: Aloha) -> None:
    robot.stop()
    time.sleep(0.3)
    robot.disconnect()
