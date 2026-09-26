import time
from dataclasses import dataclass

from myaloha.mods.base import Mod


class Teleop(Mod):
    """Followers mirror their leaders' joints for a fixed duration.

    Runs every side in `sides` whose leader and follower are both connected, and fails only when
    no side has a complete pair.
    """

    name = "teleop"
    category = "teleop"

    @dataclass
    class Params:
        duration_s: float = 30.0
        align_time: float = 2.0  # seconds for followers to reach the leaders' pose before mirroring
        sides: tuple[str, ...] = ("left", "right")

    STATES = {
        "idle": "not started",
        "pairing": "matching each side's leader and follower; incomplete sides are skipped",
        "aligning": "leaders hold still (position); followers move to the leaders' pose",
        "following": "leaders float under the operator's hand (gravity comp); followers track them (position)",
        "releasing": "leaders and followers hold their last pose (position)",
    }
    TRANSITIONS = {
        "idle": {"pairing"},
        "pairing": {"aligning"},
        "aligning": {"following", "releasing"},
        "following": {"releasing"},
    }

    skipped: dict[str, str] = {}

    def execute(self) -> None:
        self.goto("pairing")
        pairs, self.skipped = [], {}
        for side in self.params.sides:
            leader, follower = self.robot.arm("leader", side), self.robot.arm("follower", side)
            if leader and follower:
                pairs.append((leader, follower))
            else:
                self.skipped[side] = f"{'leader' if not leader else 'follower'}_{side} not connected"
        if not pairs:
            raise RuntimeError(f"no complete leader/follower pair ({'; '.join(self.skipped.values())})")

        self.goto("aligning")
        try:
            for leader, follower in pairs:
                leader.hold()
                follower.move_to(leader.positions(), self.params.align_time)
            if not self.sleep(self.params.align_time):
                return
            for leader, _ in pairs:
                leader.free()

            self.goto("following")
            period = 1 / pairs[0][1].lerobot.config.loop_rate
            end = time.monotonic() + self.params.duration_s
            while not self.stopping and (tick := time.monotonic()) < end:
                for leader, follower in pairs:
                    follower.lerobot.send_action(leader.lerobot.get_action())
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
        finally:
            self.goto("releasing")
            for leader, follower in pairs:
                leader.hold()
                follower.hold()

    def status(self) -> dict:
        return {**super().status(), "skipped": self.skipped}
