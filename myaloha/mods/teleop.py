import time
from dataclasses import dataclass, field

from myaloha.mods.base import Mod, Need


class Teleop(Mod):
    """Followers mirror their leaders' joints for a fixed duration.

    Runs every side in `sides` whose leader and follower are both connected, and fails only when
    no side has a complete pair.
    """

    name = "teleop"
    category = "teleop"

    @dataclass
    class Params:
        duration_s: float = field(default=30.0, metadata={"help": "Seconds of teleoperation."})
        stage_time: float = field(default=2.0, metadata={"help": "Seconds for leaders and followers to reach the staged pose."})
        sides: tuple[str, ...] = field(default=("left", "right"), metadata={"help": "Leader/follower pairs to run: left, right or both."})

    STATES = {
        "idle": "not started",
        "pairing": "matching each side's leader and follower; incomplete sides are skipped",
        "staging": "leaders and followers move to the staged pose, clear of the joint limits (position)",
        "following": "leaders float under the operator's hand (gravity comp); followers track them (position)",
        "releasing": "leaders and followers hold their last pose (position)",
    }
    TRANSITIONS = {
        "idle": {"pairing"},
        "pairing": {"staging"},
        "staging": {"following", "releasing"},
        "following": {"releasing"},
    }

    REQUIRES = (Need("a leader+follower pair on a requested side", (
        ("left pair", "left", ("leader_left", "follower_left")),
        ("right pair", "right", ("leader_right", "follower_right")))),)

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
                self.log.warning("skip %s: %s", side, self.skipped[side])
        if not pairs:
            raise RuntimeError(f"no complete leader/follower pair ({'; '.join(self.skipped.values())})")

        # Stage both arms of each pair together: released at the zero pose, a leader sits on the
        # shoulder/elbow limits and the controller's limit push fights gravity comp (it shakes or
        # floats up). Both at the staged pose also means the follower starts where the leader is.
        self.goto("staging")
        try:
            for leader, follower in pairs:
                leader.move_to(leader.staged_positions, self.params.stage_time)
                follower.move_to(follower.staged_positions, self.params.stage_time)
            if not self.sleep(self.params.stage_time):
                return
            for leader, _ in pairs:
                leader.free()

            self.goto("following")
            period = 1 / pairs[0][1].lerobot.config.loop_rate
            end = time.monotonic() + self.params.duration_s
            while not self.stopping and (tick := time.monotonic()) < end:
                for leader, follower in pairs:
                    with leader.lock:
                        action = leader.lerobot.get_action()
                    with follower.lock:
                        follower.check()
                        follower.lerobot.send_action(action)
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
        finally:
            self.goto("releasing")
            for leader, follower in pairs:
                leader.hold()
                follower.hold()

    def status(self) -> dict:
        return {**super().status(), "skipped": self.skipped}
