"""The shared interface of every mod.

A mod is built with the robot and its params, then run once:

    mod = Reset(robot, Reset.Params(goal_time=4.0))
    mod.run()            # blocks; call mod.stop() from another thread to end early

Each mod declares its own finite-state machine over the arms: STATES names every state and says
what the arms are doing in it, TRANSITIONS lists the allowed moves. Every mod starts in "idle".
The terminal states "done", "stopped" and "failed" are shared and reachable from any state, so
mods list only their own path in TRANSITIONS.
"""
import logging
import threading
import time
from dataclasses import asdict, dataclass

logger = logging.getLogger(__name__)

TERMINAL = {
    "done": "finished normally",
    "stopped": "ended early on request; arms left safe",
    "failed": "ended on an error; see error",
}


class Mod:
    name: str = "mod"
    category: str = "general"  # groups mods in the panel

    @dataclass
    class Params:
        pass

    STATES: dict[str, str] = {"idle": "not started"}
    TRANSITIONS: dict[str, set[str]] = {"idle": set()}

    def __init__(self, robot, params=None, parent: "Mod | None" = None):
        self.robot = robot
        self.params = params if params is not None else self.Params()
        self.parent = parent
        self.state = "idle"
        self.error: str | None = None
        self.history: list[tuple[str, float]] = [("idle", time.time())]
        self._stop = threading.Event()

    # ---- FSM ----
    def goto(self, state: str) -> None:
        if state not in TERMINAL and state not in self.TRANSITIONS.get(self.state, ()):
            raise RuntimeError(f"{self.name}: illegal transition {self.state} -> {state}")
        logger.info("%s: %s -> %s", self.name, self.state, state)
        self.state = state
        self.history.append((state, time.time()))

    # ---- lifecycle ----
    def run(self) -> str:
        """Execute to a terminal state and return it. Raises only if the robot is busy."""
        self.robot.claim(self)
        try:
            self.execute()
            self.goto("stopped" if self.stopping else "done")
        except Exception as e:
            logger.exception("%s failed in %s", self.name, self.state)
            self.error = f"{type(e).__name__}: {e}"
            self.goto("failed")
        finally:
            self.robot.release(self)
        return self.state

    def execute(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        self._stop.set()

    @property
    def stopping(self) -> bool:
        return self._stop.is_set() or (self.parent is not None and self.parent.stopping)

    def sleep(self, seconds: float) -> bool:
        """Wait, waking early on stop. Returns False if stopped."""
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            if self.stopping:
                return False
            time.sleep(min(left, 0.02))
        return not self.stopping

    # ---- reporting ----
    def status(self) -> dict:
        return {
            "name": self.name,
            "category": self.category,
            "parent": self.parent.name if self.parent else None,
            "state": self.state,
            "since": self.history[-1][1],
            "history": list(self.history),
            "error": self.error,
            "params": asdict(self.params),
            "states": {**self.STATES, **TERMINAL},
            "transitions": {k: sorted(v) for k, v in self.TRANSITIONS.items()},
        }
