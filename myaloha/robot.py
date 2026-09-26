"""The ALOHA robot: four WidowX AI arms behind one object that owns their connections and status.

Each arm is held as its LeRobot object (follower or leader teleop), but connecting only configures
the driver. LeRobot's own connect() and disconnect() move the arm, and here motion belongs to mods.
Leaders are optional: an arm that fails to connect is recorded, not raised.
"""
import threading
from dataclasses import dataclass

import trossen_arm
from lerobot_robot_trossen.config_widowxai_follower import WidowXAIFollowerConfig
from lerobot_robot_trossen.widowxai_follower import WidowXAIFollower
from lerobot_teleoperator_trossen.config_widowxai_leader import WidowXAILeaderTeleopConfig
from lerobot_teleoperator_trossen.widowxai_leader import WidowXAILeaderTeleop


@dataclass(frozen=True)
class ArmSpec:
    name: str
    ip: str
    role: str  # "leader" | "follower"
    side: str  # "left" | "right"


# Placeholder: which IP is which arm is not mapped yet (see PROGRESS.md).
DEFAULT_ARMS = (
    ArmSpec("leader_left", "192.168.10.2", "leader", "left"),
    ArmSpec("leader_right", "192.168.10.3", "leader", "right"),
    ArmSpec("follower_left", "192.168.10.4", "follower", "left"),
    ArmSpec("follower_right", "192.168.10.5", "follower", "right"),
)


class Arm:
    def __init__(self, spec: ArmSpec):
        self.spec = spec
        if spec.role == "follower":
            self.lerobot = WidowXAIFollower(WidowXAIFollowerConfig(ip_address=spec.ip, id=spec.name))
            self.end_effector = trossen_arm.StandardEndEffector.wxai_v0_follower
        else:
            self.lerobot = WidowXAILeaderTeleop(WidowXAILeaderTeleopConfig(ip_address=spec.ip, id=spec.name))
            self.end_effector = trossen_arm.StandardEndEffector.wxai_v0_leader
        self.error: str | None = None

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

    def connect(self, timeout: float) -> None:
        try:
            self.driver.configure(trossen_arm.Model.wxai_v0, self.end_effector, self.spec.ip, True, timeout=timeout)
            self.error = None
        except Exception as e:
            self.error = str(e)

    def positions(self) -> list[float]:
        return list(self.driver.get_all_positions())

    def move_to(self, positions: list[float], goal_time: float, blocking: bool = False) -> None:
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(positions, goal_time=goal_time, blocking=blocking)

    def hold(self) -> None:
        """Stiffen in place: position mode at the current pose."""
        self.move_to(self.positions(), goal_time=0.2)

    def free(self) -> None:
        """Gravity compensation: the arm floats and can be moved by hand."""
        self.driver.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver.set_all_external_efforts([0.0] * self.driver.get_num_joints(), goal_time=0.0, blocking=True)

    def reading(self) -> dict:
        joints = self.driver.get_robot_output().joint.all
        return {
            "pos": list(joints.positions),
            "vel": list(joints.velocities),
            "eff": list(joints.efforts),
            "ext_eff": list(joints.external_efforts),
        }

    def status(self) -> dict:
        s = {"name": self.name, "ip": self.spec.ip, "role": self.spec.role, "side": self.spec.side,
             "connected": False, "error": self.error, "mode": None, "pos": None}
        if self.connected:
            s["connected"] = True
            s["mode"] = sorted({m.name for m in self.driver.get_modes()})
            s["pos"] = self.positions()
        return s


class Aloha:
    # The robot's major states; the running mod's own FSM refines "running".
    STATES = {
        "offline": "no arm connected",
        "idle": "arms connected, no mod running",
        "running": "a mod holds the robot",
    }

    def __init__(self, specs=DEFAULT_ARMS):
        self.arms = {spec.name: Arm(spec) for spec in specs}
        # Running mods, outermost first. A mod may only start when the stack is empty or its
        # parent is on top (composition).
        self.mods: list = []
        self._mods_lock = threading.Lock()

    def connect(self, timeout: float = 5.0) -> None:
        for arm in self.arms.values():
            if not arm.connected:
                arm.connect(timeout)

    def disconnect(self) -> None:
        """Release the drivers. Sends no motion; leave arms somewhere safe first (e.g. the reset mod)."""
        for arm in self.arms.values():
            if arm.connected:
                arm.driver.cleanup()

    def connected(self, role: str | None = None) -> list[Arm]:
        return [a for a in self.arms.values() if a.connected and role in (None, a.spec.role)]

    def arm(self, role: str, side: str) -> Arm | None:
        """The connected arm with this role and side, or None if it is absent."""
        return next((a for a in self.connected(role) if a.spec.side == side), None)

    def claim(self, mod) -> None:
        with self._mods_lock:
            if self.mods and self.mods[-1] is not mod.parent:
                raise RuntimeError(f"robot busy with {self.mods[-1].name!r}")
            self.mods.append(mod)

    def release(self, mod) -> None:
        with self._mods_lock:
            self.mods.remove(mod)

    @property
    def state(self) -> str:
        if self.mods:
            return "running"
        return "idle" if self.connected() else "offline"

    def status(self) -> dict:
        return {
            "state": self.state,
            "arms": {name: arm.status() for name, arm in self.arms.items()},
            "mods": [m.status() for m in self.mods],
        }
