"""A simulated rig: the same devices as the real one (arm controllers, RealSense cameras) served over
the network device protocol, so a backend runs against it unchanged. A project of its own: it
shares nothing with the backend but the protocol. See rig.yaml for everything it simulates and how
wrong each reading is."""
from pathlib import Path

import yaml

DEFAULT_CONFIG = Path(__file__).resolve().with_name("rig.yaml")


def load_config(path=DEFAULT_CONFIG, overrides: dict | None = None) -> dict:
    """The rig config, with `overrides` ({"errors.depth": 0, "noise": 0, ...}, dotted keys) applied.
    Paths in it are relative to its own directory (kept as cfg["_dir"])."""
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    cfg["_dir"] = path.parent
    for key, value in (overrides or {}).items():
        *parents, last = key.split(".")
        node = cfg
        for p in parents:
            node = node[p]
        if last not in node:
            raise KeyError(f"no setting {key!r} in {path}")
        node[last] = value
    return cfg
