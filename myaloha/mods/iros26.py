"""iros26: pick and place, chosen on a page. While it runs the backend serves pickandplay.html,
where the operator clicks a cube, then where it goes; each choice arrives through
POST /api/mods/iros26/input and is carried out by a pick_place run (its child)."""
import threading
from dataclasses import dataclass, field, fields

from myaloha.mods.base import Mod, RequirementError
from myaloha.mods.pick_place import PickPlace

_PP = {f.name: f.metadata for f in fields(PickPlace.Params)}


class Iros26(Mod):
    """Interactive pick and place: while it runs, /pickandplay.html is served, where the operator
    chooses a cube and then where to put it (on the table, or on another cube). Each choice is
    carried out by pick_place, then the next one is awaited, until stopped. A choice that fails is
    reported on the page and the next can follow."""

    name = "iros26"
    category = "manipulation"

    @dataclass
    class Params:
        arm: str = field(default="auto", metadata=_PP["arm"])
        speed: float = field(default=0.05, metadata=_PP["speed"])
        tries: int = field(default=3, metadata=_PP["tries"])
        handover: bool = field(default=True, metadata=_PP["handover"])
        dry_run: bool = field(default=False, metadata=_PP["dry_run"])

    STATES = {
        "idle": "not started",
        "waiting": "waiting for a cube and a target, chosen on /pickandplay.html",
        "moving": "pick_place moves the chosen cube to the target",
    }
    TRANSITIONS = {
        "idle": {"waiting"},
        "waiting": {"moving"},
        "moving": {"waiting"},
    }

    REQUIRES = PickPlace.REQUIRES

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.choice: dict | None = None        # the one being carried out: {track, target_x, target_y}
        self.rounds: list[dict] = []           # every choice so far, with how it ended
        self._chosen = threading.Event()
        self._lock = threading.Lock()

    def input(self, data: dict) -> dict:
        """A choice from the page: {track, target_x, target_y}; taken only while waiting."""
        try:
            choice = {"track": int(data["track"]), "target_x": float(data["target_x"]),
                      "target_y": float(data["target_y"])}
        except (KeyError, TypeError, ValueError):
            raise ValueError("a choice is {track, target_x, target_y}") from None
        with self._lock:
            if self.state != "waiting" or self._chosen.is_set():
                raise RuntimeError(f"iros26 is {self.state}, not waiting for a choice")
            self.choice = choice
            self._chosen.set()
        return self.status()

    def execute(self) -> None:
        p = self.params
        while True:
            self.goto("waiting")
            while not self._chosen.is_set():
                if not self.sleep(0.1):
                    return
            self.goto("moving")
            child = PickPlace(self.robot, PickPlace.Params(**self.choice, arm=p.arm, speed=p.speed, tries=p.tries,
                                                           handover=p.handover, dry_run=p.dry_run), parent=self)
            try:
                result = child.run()
            except RequirementError as e:          # e.g. locate_cubes stopped meanwhile: this choice only
                result, child.error = "failed", str(e)
            with self._lock:
                self.rounds.append({**self.choice, "result": result, "error": child.error})
                self.choice = None
                self._chosen.clear()
            if self.stopping:
                return

    def status(self) -> dict:
        return {**super().status(), "choice": self.choice, "rounds": self.rounds}
