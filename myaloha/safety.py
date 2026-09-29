"""The safety boundary: a rectangle on the table, in W, that no tool point a mod plans may leave. Its
sides are the table's edges (the panel's rig.table in panel/config.json, as the panel draws it)
moved in by a margin each: up (+y, the far edge: up in the panel's top view), down (-y, the
operator's edge), left (-x, the left follower's side) and right (+x). Set from the panel
(PUT /api/boundary) and kept per rig, beside its calibration (boundary.yaml)."""
import json
from dataclasses import dataclass, field
from functools import cache
from pathlib import Path

import numpy as np
import yaml

CONFIG = Path(__file__).resolve().parent.parent / "panel" / "config.json"
SIDES = ("up", "down", "left", "right")


@cache
def table() -> tuple[float, float, float, float]:
    """(x0, y0, x1, y1) of the table top in W (m), as the panel draws it."""
    t = json.loads(CONFIG.read_text())["rig"]["table"]
    return -t["w"] / 2, -t["z"] - t["d"] / 2, t["w"] / 2, -t["z"] + t["d"] / 2


@dataclass
class Boundary:
    up: float = 0.03
    down: float = 0.03
    left: float = 0.03
    right: float = 0.03
    path: Path | None = field(default=None, repr=False)   # where it is kept; None: not kept

    @classmethod
    def load(cls, path: Path) -> "Boundary":
        """The boundary kept at `path`, or the default one (kept there once changed)."""
        doc = yaml.safe_load(path.read_text()) if path.exists() else {}
        return cls(**{k: float(doc[k]) for k in SIDES if k in doc}, path=path)

    @property
    def margins(self) -> dict:
        return {k: getattr(self, k) for k in SIDES}

    def box(self) -> tuple[float, float, float, float]:
        """(x0, y0, x1, y1) in W (m)."""
        x0, y0, x1, y1 = table()
        return x0 + self.left, y0 + self.down, x1 - self.right, y1 - self.up

    def contains(self, P) -> np.ndarray:
        """Whether each point (..., 2 or 3; W) is inside, in xy."""
        P = np.asarray(P, float)
        x0, y0, x1, y1 = self.box()
        return (P[..., 0] >= x0) & (P[..., 0] <= x1) & (P[..., 1] >= y0) & (P[..., 1] <= y1)

    def set(self, **margins: float) -> None:
        """Change some margins (m), and keep them. ValueError for an unknown side, a negative margin,
        or margins that leave no room on the table."""
        if unknown := set(margins) - set(SIDES):
            raise ValueError(f"no side {', '.join(sorted(unknown))}; sides: {', '.join(SIDES)}")
        new = {**self.margins, **{k: float(v) for k, v in margins.items()}}
        if any(not np.isfinite(v) or v < 0 for v in new.values()):
            raise ValueError("margins are distances in from the table's edges: 0 or more")
        x0, y0, x1, y1 = table()
        if new["left"] + new["right"] >= x1 - x0 or new["up"] + new["down"] >= y1 - y0:
            raise ValueError("those margins leave nothing of the table")
        for k, v in new.items():
            setattr(self, k, v)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(yaml.safe_dump(self.margins))

    def status(self) -> dict:
        return {"margins": self.margins, "box": [round(v, 4) for v in self.box()], "table": list(table())}
