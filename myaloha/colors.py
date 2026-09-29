"""Colour recognition profiles: how cube colours look on a rig (its table, its light, its cameras).

A profile is a YAML file in config/colors/ (NAME.yaml): prototype colours per cube colour and for
things that are not cubes, the table colour frames are balanced to, and the classifier's weights
and thresholds. `default` is the lab's (the wooden table, 2026-09-26). Make one for another setup
with setup/calibrate_colors.py and launch with `panel --colors NAME`.

Format (OpenCV Lab: L 0-255, a and b centred on 0):
    note: where and how it was measured
    balance: tint | full   # match the table's tint only, or its brightness too
    table: [L, a, b]       # the table colour frames are balanced to
    l_weight: 0.4          # lightness counts this much against hue (shading)
    max_dist: 28           # a pixel further than this from every prototype is no colour
    min_chroma: 15         # greyer pixels are never cubes
    colors: {red: [[L, a, b], ...], ...}   # one or more prototypes per cube colour
    others: {wood: [[L, a, b], ...], ...}  # not cubes: pixels nearest these are dropped
"""
from pathlib import Path

import yaml

PROFILES = Path(__file__).resolve().parent.parent / "config" / "colors"


def path(name: str) -> Path:
    """A profile's file: a name in config/colors/, or a path to a .yaml file."""
    return Path(name) if name.endswith((".yaml", ".yml")) else PROFILES / f"{name}.yaml"


def load(name: str = "default") -> dict:
    p = path(name)
    if not p.exists():
        have = sorted(q.stem for q in PROFILES.glob("*.yaml"))
        raise FileNotFoundError(f"no colour profile {name!r} ({p}); have: {', '.join(have)}")
    return {"name": p.stem, **yaml.safe_load(p.read_text())}


def save(name: str, doc: dict) -> Path:
    p = path(name)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(yaml.safe_dump({k: v for k, v in doc.items() if k != "name"}, sort_keys=False,
                                default_flow_style=None, width=110))
    return p
