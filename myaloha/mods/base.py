"""The shared interface of every mod.

A mod is built with the robot and its params, then run once:

    mod = Reset(robot, Reset.Params(goal_time=4.0))
    mod.run()            # blocks; call mod.stop() from another thread to end early

Each mod declares its own finite-state machine over the arms: STATES names every state and says
what the arms are doing in it, TRANSITIONS lists the allowed moves. Every mod starts in "idle".
The terminal states "done", "stopped" and "failed" are shared and reachable from any state, so
mods list only their own path in TRANSITIONS.
"""
import dataclasses
import logging
import threading
import time
from dataclasses import asdict, dataclass

from myaloha.robot import EmergencyStop


TERMINAL = {
    "done": "finished normally",
    "stopped": "ended early on request; arms left safe",
    "failed": "ended on an error; see error",
}


class RequirementError(RuntimeError):
    """The connected hardware does not meet a mod's minimal requirement."""


@dataclass(frozen=True)
class Need:
    """Minimal hardware: at least `least` of `options` must be up. An option is (label, side,
    device names); all its devices must be up: arms connected, cameras streaming. Options with
    a side count only when the mod's `sides` param asks for that side."""

    text: str
    options: tuple[tuple[str, str, tuple[str, ...]], ...]
    least: int = 1

    def evaluate(self, robot, sides=None) -> dict:
        items = []
        for label, side, arms in self.options:
            if sides is not None and side and side not in sides:
                continue
            missing = [a for a in arms if not _up(robot, a)]
            items.append({"label": label, "side": side, "ok": not missing, "missing": missing})
        return {"text": self.text, "met": sum(i["ok"] for i in items) >= self.least, "items": items}


def first_paragraph(cls) -> str:
    return " ".join((cls.__doc__ or "").strip().split("\n\n")[0].split())


def evaluate(needs, robot, sides=None) -> dict:
    """Check `needs` against the connected hardware: {"met", "needs": [each Need's evaluation]}."""
    out = [n.evaluate(robot, sides) for n in needs]
    return {"met": all(n["met"] for n in out), "needs": out}


def _up(robot, name: str) -> bool:
    if name == "coordinates":   # not a device: a finished localize run
        return bool(robot.coordinates)
    if name in robot.arms:
        return robot.arms[name].connected and not robot.arms[name].lost
    cams = robot.cameras.status() if robot.cameras else {}
    return bool(cams.get(name, {}).get("ok"))


class Mod:
    name: str = "mod"
    category: str = "general"  # groups mods in the panel
    # Minimal hardware, checked before the robot is claimed; every Need must be met.
    REQUIRES: tuple[Need, ...] = ()

    @dataclass
    class Params:
        pass

    STATES: dict[str, str] = {"idle": "not started"}
    TRANSITIONS: dict[str, set[str]] = {"idle": set()}

    def __init__(self, robot, params=None, parent: "Mod | None" = None, source: str = "api"):
        self.robot = robot
        self.source = source  # who started it: "api" (panel, scripts) or "agent" (MCP)
        self.params = params if params is not None else self.Params()
        self.parent = parent
        self.state = "idle"
        self.error: str | None = None
        self.history: list[tuple[str, float]] = [("idle", time.time())]
        self._stop = threading.Event()
        self.log = logging.getLogger(f"myaloha.mod.{self.name}")

    # ---- FSM ----
    def goto(self, state: str) -> None:
        if state not in TERMINAL and state not in self.TRANSITIONS.get(self.state, ()):
            raise RuntimeError(f"{self.name}: illegal transition {self.state} -> {state}")
        self.log.info("%s → %s", self.state, state)
        self.state = state
        self.history.append((state, time.time()))

    @classmethod
    def catalog(cls) -> dict:
        """What the panel and agents are told about the mod: doc, params, states, requirements."""
        defaults = cls.Params()
        return {
            "name": cls.name,
            "category": cls.category,
            "doc": first_paragraph(cls),
            "params": [{"name": f.name, "type": type(getattr(defaults, f.name)).__name__,
                        "default": getattr(defaults, f.name), **dict(f.metadata)} for f in dataclasses.fields(cls.Params)],
            "states": {**cls.STATES, **TERMINAL},
            "transitions": {k: sorted(v) for k, v in cls.TRANSITIONS.items()},
            "requires": [{"text": n.text, "least": n.least,
                          "options": [{"label": label, "side": side, "arms": list(arms)} for label, side, arms in n.options]}
                         for n in cls.REQUIRES],
        }

    @classmethod
    def requirements(cls, robot, params=None) -> dict:
        """Evaluate REQUIRES against the connected hardware (default params if none given)."""
        sides = getattr(params if params is not None else cls.Params(), "sides", None)
        return evaluate(cls.REQUIRES, robot, sides)

    def check_requirements(self) -> None:
        r = self.requirements(self.robot, self.params)
        if r["met"]:
            return
        reasons = []
        for need in (n for n in r["needs"] if not n["met"]):
            missing = [f"{i['label']}: no {', '.join(i['missing'])}" for i in need["items"] if not i["ok"]]
            reasons.append(f"{need['text']} ({'; '.join(missing) or 'no side requested'})")
        raise RequirementError(f"{self.name} needs {' and '.join(reasons)}")

    @classmethod
    def params_from(cls, raw: dict):
        """Build Params from strings (CLI) or JSON values (API), typed by each field's default."""
        fields = {f.name for f in dataclasses.fields(cls.Params)}
        defaults, values = cls.Params(), {}
        for key, v in raw.items():
            if key not in fields:
                raise ValueError(f"{cls.name} has no param {key!r}; params: {', '.join(fields)}")
            default = getattr(defaults, key)
            if isinstance(default, bool):
                values[key] = v.lower() in ("1", "true", "yes", "on") if isinstance(v, str) else bool(v)
            elif isinstance(default, tuple):
                values[key] = tuple(v.split(",")) if isinstance(v, str) else tuple(v)
            else:
                values[key] = type(default)(v)
        return cls.Params(**values)

    # ---- lifecycle ----
    def run(self) -> str:
        """Execute to a terminal state and return it. Raises only if the hardware falls short
        of REQUIRES or the robot is busy."""
        self.check_requirements()
        self.robot.claim(self)
        return self._run()

    def start(self) -> threading.Thread:
        """Check REQUIRES and claim the robot now (raising if either fails), then run in a
        background thread."""
        self.check_requirements()
        self.robot.claim(self)
        thread = threading.Thread(target=self._run, name=f"mod-{self.name}", daemon=True)
        thread.start()
        return thread

    def _run(self) -> str:
        self.log.info("run %s%s", asdict(self.params), " (started by an agent)" if self.source == "agent" else "")
        try:
            self.execute()
            self.goto("stopped" if self.stopping else "done")
        except EmergencyStop as e:
            self.error = str(e)
            self.log.warning("halted by emergency stop in %s", self.state)
            self.goto("failed")
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            self.log.error("failed in %s: %s", self.state, self.error, exc_info=True)
            self.goto("failed")
        finally:
            self.robot.release(self)
        return self.state

    def execute(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        if not self._stop.is_set():
            self.log.info("stop requested")
        self._stop.set()

    @property
    def stopping(self) -> bool:
        return (self._stop.is_set() or self.robot.estopped
                or (self.parent is not None and self.parent.stopping))

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
            "source": self.source,
            "state": self.state,
            "since": self.history[-1][1],
            "history": list(self.history),
            "error": self.error,
            "params": asdict(self.params),
            "states": {**self.STATES, **TERMINAL},
            "transitions": {k: sorted(v) for k, v in self.TRANSITIONS.items()},
        }
