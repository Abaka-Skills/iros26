import time
from dataclasses import dataclass

from myaloha.mods.base import Mod


class Instruction(Mod):
    """Float the followers so they can be moved by hand, and measure them while they are.

    `readings` holds the latest joint state per arm; `ranges` holds each joint's min and max
    position seen during the run.
    """

    name = "instruction"
    category = "teach"

    @dataclass
    class Params:
        duration_s: float = 0.0  # 0 runs until stopped
        sample_hz: float = 30.0
        sides: tuple[str, ...] = ("left", "right")

    STATES = {
        "idle": "not started",
        "entering": "selected followers switch to gravity comp (external effort 0)",
        "free": "followers float and are moved by hand; joint state sampled",
        "exiting": "followers hold their current pose (position)",
    }
    TRANSITIONS = {
        "idle": {"entering"},
        "entering": {"free", "exiting"},
        "free": {"exiting"},
    }

    readings: dict[str, dict] = {}
    ranges: dict[str, list[tuple[float, float]]] = {}

    def execute(self) -> None:
        arms = [a for side in self.params.sides if (a := self.robot.arm("follower", side))]
        if not arms:
            raise RuntimeError(f"no follower connected on {', '.join(self.params.sides)}")
        self.readings, self.ranges = {}, {}

        self.goto("entering")
        try:
            for arm in arms:
                arm.free()
            self.goto("free")
            period = 1 / self.params.sample_hz
            end = time.monotonic() + self.params.duration_s if self.params.duration_s > 0 else float("inf")
            while not self.stopping and (tick := time.monotonic()) < end:
                for arm in arms:
                    self._sample(arm)
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
        finally:
            self.goto("exiting")
            for arm in arms:
                arm.hold()

    def _sample(self, arm) -> None:
        r = self.readings[arm.name] = arm.reading()
        prev = self.ranges.get(arm.name) or [(p, p) for p in r["pos"]]
        self.ranges[arm.name] = [(min(lo, p), max(hi, p)) for (lo, hi), p in zip(prev, r["pos"])]

    def status(self) -> dict:
        return {**super().status(), "readings": self.readings, "ranges": self.ranges}
