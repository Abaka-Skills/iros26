from dataclasses import dataclass, field

from myaloha.mods.base import Mod, Need


class Reset(Mod):
    """Move every connected arm to the all-zero pose. Absent arms are skipped."""

    name = "reset"
    category = "motion"

    @dataclass
    class Params:
        goal_time: float = field(default=3.0, metadata={"help": "Seconds per move."})
        via_staged: bool = field(default=True, metadata={"help": "Pass through LeRobot's staged pose first, as its disconnect() does."})

    STATES = {
        "idle": "not started",
        "checking": "finding the connected arms",
        "staging": "connected arms move to the staged pose (position mode)",
        "zeroing": "connected arms move to all-zero (position mode)",
    }
    TRANSITIONS = {
        "idle": {"checking"},
        "checking": {"staging", "zeroing"},
        "staging": {"zeroing"},
    }

    REQUIRES = (Need("at least one arm", tuple(
        (name.replace("_", " "), "", (name,)) for name in ("leader_left", "leader_right", "follower_left", "follower_right"))),)

    skipped: tuple[str, ...] = ()

    def execute(self) -> None:
        self.goto("checking")
        arms = self.robot.connected()
        self.skipped = tuple(n for n, a in self.robot.arms.items() if not a.connected)
        if not arms:
            raise RuntimeError("no arm connected")

        if self.params.via_staged:
            self.goto("staging")
            if not self._move({a: a.staged_positions for a in arms}):
                return
        self.goto("zeroing")
        self._move({a: [0.0] * len(a.staged_positions) for a in arms})

    def _move(self, targets: dict) -> bool:
        for arm, q in targets.items():
            arm.move_to(q, self.params.goal_time)
        if self.sleep(self.params.goal_time):
            return True
        for arm in targets:  # stopped mid-move: freeze where they are
            arm.hold()
        return False

    def status(self) -> dict:
        return {**super().status(), "skipped": list(self.skipped)}
