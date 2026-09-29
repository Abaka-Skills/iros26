"""The ALOHA robot: four WidowX AI arms and four cameras behind one object that owns their
connections and status.

Each arm is held as its LeRobot object (follower or leader teleop), but connecting only configures
the driver. LeRobot's own connect() and disconnect() move the arm, and here motion belongs to mods.
Leaders are optional: an arm that fails to connect is recorded, not raised.

Live state: a monitor thread copies each arm's robot output (cached by the driver, ~0.1 ms to
read) into Arm.live at `hz`, and re-reads the joint modes (a network round trip) at `mode_hz`.
An arm whose output timestamp stops advancing is reported as lost. Alongside, every RECONNECT_S an
arm that is lost or absent is connected again as soon as its controller answers (no motion).
"""
import logging
import socket
import threading
import time
from dataclasses import dataclass

import trossen_arm
from lerobot_robot_trossen.config_widowxai_follower import WidowXAIFollowerConfig
from lerobot_robot_trossen.widowxai_follower import WidowXAIFollower
from lerobot_teleoperator_trossen.config_widowxai_leader import WidowXAILeaderTeleopConfig
from lerobot_teleoperator_trossen.widowxai_leader import WidowXAILeaderTeleop

from myaloha import netdev
from myaloha.safety import Boundary

log = logging.getLogger("myaloha.robot")

STALE_S = 0.5  # no new robot output for this long: the arm is lost
RECONNECT_S = 3.0        # an arm lost or absent is tried again this often, once its controller answers
CONTROLLER_PORT = 50001  # the arm controller's TCP server (what the driver connects to)
# Shoulder and elbow (joint_1, joint_2) have lower limits of 0 rad (URDF). Released into gravity
# comp within this margin of them, the controller's limit push fights the compensation.
LIMIT_MARGIN = 0.1


class EmergencyStop(RuntimeError):
    """Raised for any arm command while the emergency stop is latched."""


@dataclass(frozen=True)
class ArmSpec:
    name: str
    ip: str
    role: str  # "leader" | "follower"
    side: str  # "left" | "right"


# Mapped 2026-09-26. Leader/follower: gripper efforts (.4/.5 load a real gripper). Follower
# left/right: base yaw in the top-down camera vs joint_0, and each wrist camera's view; left is
# the operator's left. Leader left/right follows from teleop pairing them without crossing.
DEFAULT_ARMS = (
    ArmSpec("leader_left", "192.168.10.3", "leader", "left"),
    ArmSpec("leader_right", "192.168.10.2", "leader", "right"),
    ArmSpec("follower_left", "192.168.10.5", "follower", "left"),
    ArmSpec("follower_right", "192.168.10.4", "follower", "right"),
)


class Arm:
    def __init__(self, spec: ArmSpec, hardware: str | None = None):
        self.spec = spec
        if spec.role == "follower":
            self.lerobot = WidowXAIFollower(WidowXAIFollowerConfig(ip_address=spec.ip, id=spec.name))
            self.end_effector = trossen_arm.StandardEndEffector.wxai_v0_follower
        else:
            self.lerobot = WidowXAILeaderTeleop(WidowXAILeaderTeleopConfig(ip_address=spec.ip, id=spec.name))
            self.end_effector = trossen_arm.StandardEndEffector.wxai_v0_leader
        self.hardware = hardware
        if hardware:   # the controllers are served over the network (myaloha.netdev)
            self.lerobot.driver = netdev.ArmClient(hardware)
        # The driver's thread safety is undocumented: every call goes through this lock.
        self.lock = threading.RLock()
        self.error: str | None = None
        self.halted = False  # emergency stop latched: every command is refused
        self.modes: list[str] | None = None
        self.live: dict | None = None  # pos, vel, eff, ext_eff, and when they were read
        self._stamp = (None, 0.0)  # last output timestamp and when it last changed
        self._polled = 0.0         # when the output was last read (or a read was tried)

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def driver(self) -> trossen_arm.TrossenArmDriver:
        return self.lerobot.driver

    @property
    def connected(self) -> bool:
        return self.driver.get_is_configured()

    @property
    def staged_positions(self) -> list[float]:
        return list(self.lerobot.config.staged_positions)

    def connect(self, timeout: float, quiet: bool = False) -> None:
        """Configure the driver (no motion), releasing a lost session first. `quiet`: a failure that
        repeats the last one is not logged."""
        if self.connected and self.lost:
            try:
                self.disconnect()
            except Exception as e:
                log.warning("%s: releasing the lost session failed: %s", self.name, e)
        try:
            with self.lock:
                self.driver.configure(trossen_arm.Model.wxai_v0, self.end_effector, self.spec.ip, True, timeout=timeout)
            self.error = None
            self.refresh(modes=True)
            log.info("%s connected (%s): %s", self.name, self.spec.ip, "/".join(self.modes))
        except Exception as e:
            if not (quiet and self.error == str(e)):
                log.warning("%s (%s) not connected: %s", self.name, self.spec.ip, e)
            self.error = str(e)

    def reachable(self) -> bool:
        """Worth a connect: the controller's port answers (cheap; a powered-off arm costs 0.3 s and no
        driver retries). Through a device server (the simulator) it answers at once anyway."""
        if self.hardware:
            return True
        try:
            socket.create_connection((self.spec.ip, CONTROLLER_PORT), timeout=0.3).close()
            return True
        except OSError:
            return False

    def refresh(self, modes: bool = False) -> None:
        """Copy the driver's latest output into `live`; with `modes`, re-read the joint modes too."""
        self._polled = time.monotonic()   # before the read: a read that fails still counts as a poll
        with self.lock:
            out = self.driver.get_robot_output()
            if modes:
                self.modes = sorted({m.name for m in self.driver.get_modes()})
        joints, now, t = out.joint.all, time.monotonic(), time.perf_counter()
        stamp = out.header.timestamp
        if stamp != self._stamp[0]:
            self._stamp = (stamp, now)
        self.live = {
            "pos": list(joints.positions), "vel": list(joints.velocities),
            "eff": list(joints.efforts), "ext_eff": list(joints.external_efforts),
            "t": t,  # perf_counter when read: the same clock as camera frame stamps
        }

    @property
    def lost(self) -> bool:
        """Polled for STALE_S without new output (reads failing, or the stamp not moving). An arm
        nobody polls (before the monitor starts) is not lost."""
        return self.live is not None and self._polled - self._stamp[1] > STALE_S

    def positions(self) -> list[float]:
        with self.lock:
            return list(self.driver.get_all_positions())

    def check(self) -> None:
        """Raise if the emergency stop is latched. Call under `lock` before any command."""
        if self.halted:
            raise EmergencyStop(f"{self.name}: emergency stop")

    def _set_modes(self, mode: trossen_arm.Mode) -> None:
        with self.lock:
            self.check()
            self.driver.set_all_modes(mode)
        self.modes = [mode.name]

    def move_to(self, positions: list[float], goal_time: float, blocking: bool = False) -> None:
        self._set_modes(trossen_arm.Mode.position)
        with self.lock:
            self.check()
            self.driver.set_all_positions(positions, goal_time=goal_time, blocking=blocking)

    def hold(self) -> None:
        """Stiffen in place: position mode at the current pose."""
        self.move_to(self.positions(), goal_time=0.2)

    # ---- arm and gripper apart: the arm moves while the gripper keeps a force hold ----
    def move_arm(self, q: list[float], goal_time: float) -> None:
        """Command the six arm joints only (position mode); returns at once."""
        with self.lock:
            self.check()
            self.driver.set_arm_modes(trossen_arm.Mode.position)
            self.driver.set_arm_positions(list(q), goal_time=goal_time, blocking=False)

    def gripper_aperture(self) -> float:
        """Distance between the finger pads (m): twice the carriage position."""
        with self.lock:
            return 2.0 * float(self.driver.get_gripper_position())

    def set_gripper(self, aperture: float, goal_time: float = 1.0) -> None:
        """Open or close to `aperture` (m) in position mode; returns at once."""
        with self.lock:
            self.check()
            self.driver.set_gripper_mode(trossen_arm.Mode.position)
            self.driver.set_gripper_position(min(max(aperture, 0.0), 0.08) / 2, goal_time=goal_time, blocking=False)

    def close_on_object(self, squeeze: float = 0.004, speed: float = 0.012, timeout: float = 4.0,
                        stopping=lambda: False) -> float:
        """Close until the fingers stall (pads on an object, or shut), then hold it: position mode
        aimed `squeeze` (m per finger) inside where they stopped, so the controller keeps pressing
        with a bounded force, as teleop does when the leader closes past the object. Returns the
        aperture (m). (Switching to external-effort mode instead let the fingers spring back open
        on the rig: no grip left on the cube.)"""
        with self.lock:
            self.check()
            w0 = self.gripper_aperture() / 2
            self.driver.set_gripper_mode(trossen_arm.Mode.position)
            self.driver.set_gripper_position(0.0, goal_time=max(1.0, w0 / speed), blocking=False)
        t0, hist, pos = time.monotonic(), [], w0
        while time.monotonic() - t0 < timeout and not stopping():
            time.sleep(0.02)
            pos = self.gripper_aperture() / 2
            hist.append(pos)
            if pos < 0.0008:                                                    # shut on nothing
                break
            if len(hist) >= 10 and hist[-10] - pos < 0.0004 and w0 - pos > 0.002:   # moved, then stalled
                break
        with self.lock:
            self.check()
            self.driver.set_gripper_position(max(pos - squeeze, 0.0), goal_time=0.3, blocking=False)
        time.sleep(0.4)
        return self.gripper_aperture()

    def free(self) -> None:
        """Gravity compensation: the arm floats and can be moved by hand. Refused on the
        shoulder/elbow limits (move to the staged pose first)."""
        pos = self.positions()
        if min(pos[1], pos[2]) < LIMIT_MARGIN:
            raise RuntimeError(f"{self.name}: shoulder/elbow at their limits "
                               f"({pos[1]:.2f}, {pos[2]:.2f} rad); stage the arm before gravity comp")
        self._set_modes(trossen_arm.Mode.external_effort)
        with self.lock:
            self.check()
            self.driver.set_all_external_efforts([0.0] * self.driver.get_num_joints(), goal_time=0.0, blocking=True)

    def halt(self) -> None:
        """Emergency stop: brake every joint now and refuse further commands until released."""
        self.halted = True  # before the lock: a command already queued on it will be refused
        with self.lock:
            self.driver.set_all_modes(trossen_arm.Mode.idle)
        self.modes = [trossen_arm.Mode.idle.name]

    def reading(self) -> dict:
        with self.lock:
            joints = self.driver.get_robot_output().joint.all
        return {
            "pos": list(joints.positions),
            "vel": list(joints.velocities),
            "eff": list(joints.efforts),
            "ext_eff": list(joints.external_efforts),
        }

    def disconnect(self) -> None:
        with self.lock:
            self.driver.cleanup()
        self.live, self.modes = None, None

    def status(self) -> dict:
        connected = self.connected
        return {
            "name": self.name, "ip": self.spec.ip, "role": self.spec.role, "side": self.spec.side,
            "connected": connected, "lost": connected and self.lost, "error": self.error, "halted": self.halted,
            "mode": self.modes if connected else None,
            **({k: self.live[k] for k in ("pos", "vel", "eff", "ext_eff", "t")} if connected and self.live else
               {"pos": None, "vel": None, "eff": None, "ext_eff": None, "t": None}),
        }


class Aloha:
    # The robot's major states; the running mod's own FSM refines "running".
    STATES = {
        "offline": "no arm connected",
        "idle": "arms connected, no mod running",
        "running": "a mod holds the robot",
        "estop": "emergency stop: arms braked, commands refused until reset",
    }

    def __init__(self, specs=DEFAULT_ARMS, cameras=None, hardware: str | None = None):
        """`hardware`: host:port where the arm controllers are served (myaloha.netdev); None: the
        controllers on the arms' own IPs, through the vendor driver."""
        self.arms = {spec.name: Arm(spec, hardware) for spec in specs}
        self.cameras = cameras  # a myaloha.cameras.CameraRig, or None
        # Running mods, outermost first. A mod may only start when the stack is empty or its
        # parent is on top (composition).
        self.mods: list = []
        self._mods_lock = threading.Lock()
        # One segment per major state; running segments name their mod and, once over, its result.
        self.timeline: list[dict] = []
        self._monitor: threading.Thread | None = None
        self._monitor_stop = threading.Event()
        self.estopped = False
        self.coordinates: dict | None = None  # latest localize result (W, base poses)
        self.boundary = Boundary()             # where planned tool points may be (launch loads the rig's own)
        self.last_run: dict | None = None      # status of the last top-level mod run, once it ended
        self.topics = None                     # the TopicManager, once the server runs (mods can read topics)

    # ---- connections ----
    def connect(self, timeout: float = 5.0) -> None:
        """Connect every arm that is not connected yet, in parallel, so absent arms (e.g. leaders
        off the network) cost one timeout in total. A lost arm is connected afresh. Sends no motion."""
        threads = [threading.Thread(target=a.connect, args=(timeout,)) for a in self.arms.values()
                   if not a.connected or a.lost]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self._mark()

    def disconnect(self) -> None:
        """Release the drivers. Sends no motion; leave arms somewhere safe first (e.g. the reset mod)."""
        self.stop_monitor()
        for arm in self.arms.values():
            if arm.connected:
                arm.disconnect()
        if self.cameras:
            self.cameras.disconnect()
        self._mark()

    def connected(self, role: str | None = None) -> list[Arm]:
        return [a for a in self.arms.values() if a.connected and role in (None, a.spec.role)]

    def arm(self, role: str, side: str) -> Arm | None:
        """The connected arm with this role and side, or None if it is absent."""
        return next((a for a in self.connected(role) if a.spec.side == side), None)

    # ---- live state ----
    def start_monitor(self, hz: float = 100.0, mode_hz: float = 2.0) -> None:
        if self._monitor:
            return
        self._monitor_stop.clear()
        self._monitor = threading.Thread(target=self._watch, args=(hz, mode_hz), name="arm-monitor", daemon=True)
        self._monitor.start()
        self._reconnector = threading.Thread(target=self._reconnect, name="arm-reconnect", daemon=True)
        self._reconnector.start()

    def stop_monitor(self) -> None:
        if self._monitor:
            self._monitor_stop.set()
            self._monitor.join()
            self._reconnector.join()
            self._monitor = None

    def _reconnect(self) -> None:
        """Every RECONNECT_S: an arm that dropped (lost) or is absent is connected again once its
        controller answers: the dead session released, the driver configured afresh. No motion."""
        while not self._monitor_stop.wait(RECONNECT_S):
            changed = False
            for a in self.arms.values():
                if self._monitor_stop.is_set():
                    break
                if (a.connected and not a.lost) or not a.reachable():
                    continue
                a.connect(timeout=2.0, quiet=True)   # logs "connected" when it is back
                changed |= a.connected
            if changed:
                self._mark()

    def _watch(self, hz: float, mode_hz: float) -> None:
        period, next_modes = 1 / hz, 0.0
        lost = set()
        while not self._monitor_stop.wait(period):
            modes = time.monotonic() >= next_modes
            if modes:
                next_modes = time.monotonic() + 1 / mode_hz
            for arm in self.connected():
                try:
                    arm.refresh(modes=modes)
                except Exception as e:
                    if arm.error != str(e):
                        log.error("%s: %s", arm.name, e)
                    arm.error = str(e)
                if arm.lost and arm.name not in lost:
                    lost.add(arm.name)
                    log.error("%s: no output for %.1f s; link lost?", arm.name, arm._polled - arm._stamp[1])
                elif not arm.lost and arm.name in lost:
                    lost.discard(arm.name)
                    log.info("%s: output resumed", arm.name)

    # ---- mods ----
    def claim(self, mod) -> None:
        if self.estopped:
            raise RuntimeError("emergency stop is active; reset it first")
        with self._mods_lock:
            if self.mods and self.mods[-1] is not mod.parent:
                raise RuntimeError(f"robot busy with {self.mods[-1].name!r}")
            self.mods.append(mod)
        if mod.parent is None:
            self._mark(mod=mod.name, source=mod.source)

    def release(self, mod) -> None:
        with self._mods_lock:
            self.mods.remove(mod)
        if mod.parent is None:
            self.last_run = mod.status()
            run = next((seg for seg in reversed(self.timeline) if seg.get("mod") == mod.name and "result" not in seg), None)
            if run:
                run["result"] = mod.state
            self._mark()

    def emergency_stop(self) -> None:
        """Brake every connected arm at once, then tell running mods to stop. Latched until
        reset_emergency_stop(); mods cannot start and arm commands raise EmergencyStop."""
        first = not self.estopped
        self.estopped = True
        if first:  # repeats (the panel sends over two paths) brake again but log once
            log.critical("EMERGENCY STOP")
        arms = self.connected()
        threads = [threading.Thread(target=self._halt, args=(a, first)) for a in arms]
        for t in threads:
            t.start()
        for mod in list(self.mods):
            mod.stop()
        for t in threads:
            t.join()
        self._mark()

    def _halt(self, arm: Arm, report: bool) -> None:
        try:
            arm.halt()
            if report:
                log.critical("%s braked", arm.name)
        except Exception as e:
            log.critical("%s: brake failed: %s", arm.name, e)

    def reset_emergency_stop(self) -> None:
        """Accept commands again. Arms stay braked until a mod commands them."""
        if not self.estopped:
            return
        if self.mods:
            raise RuntimeError("a mod is still winding down; try again in a moment")
        for arm in self.arms.values():
            arm.halted = False
        self.estopped = False
        log.warning("emergency stop reset; arms stay braked until a mod moves them")
        self._mark()

    def stop(self) -> None:
        """Ask every running mod to stop; each leaves its arms safe on the way out."""
        for mod in list(self.mods):
            mod.stop()

    # ---- status ----
    @property
    def state(self) -> str:
        if self.estopped:
            return "estop"
        if self.mods:
            return "running"
        return "idle" if self.connected() else "offline"

    def _mark(self, mod: str | None = None, source: str | None = None) -> None:
        state = self.state if mod is None else "running"
        last = self.timeline[-1] if self.timeline else None
        if last and last["state"] == state and last.get("mod") == mod and "result" not in last:
            return
        self.timeline.append({"state": state, "t0": time.time(), **({"mod": mod, "source": source} if mod else {})})
        del self.timeline[:-200]

    def status(self) -> dict:
        return {
            "state": self.state,
            "arms": {name: arm.status() for name, arm in self.arms.items()},
            "cameras": self.cameras.status() if self.cameras else {},
            "coordinates": self.coordinates and {"id": self.coordinates["id"]},
            "boundary": self.boundary.status(),
            "mods": [m.status() for m in self.mods],
        }
