"""Run the simulated rig, or talk to a running one. From this project's directory:

    uv run python -m simulator                              # serve rig.yaml's world
    uv run python -m simulator --config my.yaml --set seed=3 --set noise=0
    uv run python -m simulator truth                        # where every cube really is
    uv run python -m simulator spawn [--count 6] [--seed 4] # a new scene
    uv run python -m simulator unplug wrist_left            # and `plug`: drop / restore a device

(or from anywhere: uv run --project <this directory> python -m simulator ...)
The devices are served at the config's `listen` address, its own page (ground truth, cameras,
controls) at `ui`.
"""
import argparse
import json
import logging
import sys
import time

import yaml

from simulator import DEFAULT_CONFIG, load_config
from simulator.protocol import call


def main() -> None:
    p = argparse.ArgumentParser(prog="python -m simulator")
    p.add_argument("cmd", nargs="?", default="serve", choices=("serve", "truth", "spawn", "unplug", "plug", "info"))
    p.add_argument("device", nargs="?", help="unplug/plug: an arm (IP or name) or camera (serial or name)")
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override a config setting (dotted key, YAML value), e.g. errors.depth=0")
    p.add_argument("--address", help="of a running simulator (default: the config's listen)")
    p.add_argument("--count", type=int, help="spawn: number of cubes")
    p.add_argument("--seed", type=int, help="spawn: seed for the new scene")
    args = p.parse_args()
    overrides = {k: yaml.safe_load(v) for k, _, v in (s.partition("=") for s in args.set)}
    cfg = load_config(args.config, overrides)

    if args.cmd != "serve":
        address = args.address or cfg["listen"]
        if args.cmd in ("unplug", "plug"):
            out = call(address, args.cmd, args.device)
        elif args.cmd == "spawn":
            out = call(address, "spawn", args.count, args.seed)
        else:
            out = call(address, args.cmd)
        print(json.dumps(out, indent=2))
        return

    from simulator import ui
    from simulator.server import Simulator
    sys.stdout.reconfigure(line_buffering=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s", datefmt="%H:%M:%S")
    sim = Simulator(cfg)
    address = sim.start()
    page = ui.serve(sim, cfg["ui"])
    print(f"simulated rig at {address} (seed {sim.world.seed}, noise x{cfg['noise']})\n"
          f"page:    http://{page}/\nCtrl-C stops it")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.stop()


if __name__ == "__main__":
    main()
