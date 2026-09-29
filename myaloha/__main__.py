"""Command line.

From the repo root (after setup/find_arms.sh):
    uv run python -m myaloha panel                     # check hardware, serve API + panel, open it
    uv run python -m myaloha panel --simulator HOST:PORT  # read the arms and cameras from a simulator

The rig's cameras come from the camera relay (camrelay/, a daemon on this machine; the backend never
needs root). The arms are reached directly over their network.
    uv run python -m myaloha status                    # connect and print; moves nothing
    uv run python -m myaloha run reset goal_time=4
    uv run python -m myaloha run teleop duration_s=60 sides=left
    uv run python -m myaloha run instruction           # Ctrl-C ends it

Ctrl-C during `run` stops the mod cleanly.
"""
import argparse
import json
import logging

from myaloha.mods import MODS


def main() -> None:
    p = argparse.ArgumentParser(prog="python -m myaloha")
    sub = p.add_subparsers(dest="cmd", required=True)
    panel = sub.add_parser("panel", help="check the hardware, then serve the API and panel on localhost")
    panel.add_argument("--port", type=int, default=8765)
    panel.add_argument("--hz", type=float, default=100, help="arm monitor rate")
    panel.add_argument("--timeout", type=float, default=3.0, help="per-arm connect timeout, s")
    panel.add_argument("--no-cameras", action="store_true", help="skip the cameras")
    panel.add_argument("--cameras", metavar="HOST:PORT", default="127.0.0.1:8820",
                       help="where the rig's cameras are served (the camera relay; default 127.0.0.1:8820)")
    panel.add_argument("--no-browser", action="store_true")
    panel.add_argument("--simulator", metavar="HOST:PORT",
                       help="read the arms and cameras from a simulator there instead of the rig")
    panel.add_argument("--colors", default="default",
                       help="colour recognition profile: a name in config/colors/ or a .yaml path (default: default)")
    panel.add_argument("--calib", help="calibration directory (default: calib/, or calib/HOST_PORT with --simulator)")
    sub.add_parser("status")
    run = sub.add_parser("run")
    run.add_argument("mod", choices=MODS)
    run.add_argument("params", nargs="*", help="key=value")
    args = p.parse_args()

    if args.cmd == "panel":
        from myaloha import launch
        launch.main(args)
        return

    from myaloha.robot import Aloha

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s", datefmt="%H:%M:%S")
    robot = Aloha()
    robot.connect()
    try:
        if args.cmd == "status":
            print(json.dumps(robot.status()["arms"], indent=2))
            return
        cls = MODS[args.mod]
        try:
            params = cls.params_from(dict(pair.partition("=")[::2] for pair in args.params))
        except (ValueError, TypeError) as e:
            raise SystemExit(str(e))
        mod = cls(robot, params)
        try:
            worker = mod.start()
        except RuntimeError as e:  # requirement not met, busy, or emergency stop
            raise SystemExit(str(e))
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
