"""Run a mod from the terminal.

From ALOHA/ (after setup/find_arms.sh):
    uv run python -m myaloha status
    uv run python -m myaloha run reset goal_time=4
    uv run python -m myaloha run teleop duration_s=60 sides=left
    uv run python -m myaloha run instruction      # Ctrl-C ends it

`status` connects and prints without moving anything. Ctrl-C during `run` stops the mod cleanly.
"""
import argparse
import dataclasses
import json
import logging
import threading

from myaloha.mods import MODS
from myaloha.robot import Aloha


def parse_params(mod_cls, pairs: list[str]):
    fields = {f.name: f for f in dataclasses.fields(mod_cls.Params)}
    defaults = mod_cls.Params()
    values = {}
    for pair in pairs:
        key, _, raw = pair.partition("=")
        if key not in fields:
            raise SystemExit(f"{mod_cls.name} has no param {key!r}; params: {', '.join(fields)}")
        default = getattr(defaults, key)
        if isinstance(default, bool):
            values[key] = raw.lower() in ("1", "true", "yes", "on")
        elif isinstance(default, tuple):
            values[key] = tuple(raw.split(","))
        else:
            values[key] = type(default)(raw)
    return mod_cls.Params(**values)


def main() -> None:
    p = argparse.ArgumentParser(prog="python -m myaloha")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    run = sub.add_parser("run")
    run.add_argument("mod", choices=MODS)
    run.add_argument("params", nargs="*", help="key=value")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    robot = Aloha()
    robot.connect()
    try:
        if args.cmd == "status":
            print(json.dumps(robot.status()["arms"], indent=2))
            return
        mod = MODS[args.mod](robot, parse_params(MODS[args.mod], args.params))
        worker = threading.Thread(target=mod.run)
        worker.start()
        try:
            while worker.is_alive():
                worker.join(0.2)
        except KeyboardInterrupt:
            mod.stop()
            worker.join()
        print(json.dumps({k: v for k, v in mod.status().items() if k not in ("states", "transitions")}, indent=2))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
