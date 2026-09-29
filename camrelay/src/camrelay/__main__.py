"""camrelay: serve the RealSense cameras on localhost (python -m camrelay [--listen HOST:PORT]).

Needs root on macOS (librealsense takes the cameras from the system's UVC driver). Installed as a
launchd daemon by camrelay/install.sh: started at boot, restarted if it dies, following the cameras as
they are plugged and unplugged. With --iface and --host-ip it also keeps the arms' network address on
the USB Ethernet adapter (it is lost when the adapter is replugged)."""
import argparse
import logging
import re
import subprocess
import threading

from camrelay.relay import Relay

log = logging.getLogger("camrelay")


def keep_address(iface: str, ip: str, every: float = 5.0) -> None:
    """Add `ip`/24 to `iface` whenever the interface is there without it (macOS ifconfig)."""
    while True:
        out = subprocess.run(["ifconfig", iface], capture_output=True, text=True)
        if out.returncode == 0 and not re.search(rf"inet {re.escape(ip)} ", out.stdout):
            r = subprocess.run(["ifconfig", iface, "alias", ip, "255.255.255.0"], capture_output=True, text=True)
            log.info("%s: added %s%s", iface, ip, "" if r.returncode == 0 else f" (failed: {r.stderr.strip()})")
        threading.Event().wait(every)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default="127.0.0.1:8820")
    ap.add_argument("--size", default="640x480", help="colour and depth size, WIDTHxHEIGHT")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--iface", help="also keep --host-ip on this network interface (the arms' adapter)")
    ap.add_argument("--host-ip", default="192.168.10.200")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s", datefmt="%H:%M:%S")
    from camrelay.realsense import RealSenseSource
    w, h = (int(v) for v in args.size.split("x"))
    relay = Relay(RealSenseSource(w, h, args.fps), args.listen).start()
    log.info("serving cameras at %s", relay.address)
    if args.iface:
        threading.Thread(target=keep_address, args=(args.iface, args.host_ip), daemon=True).start()
    relay.stopped.wait()
    if relay.stuck:
        raise SystemExit(1)                    # launchd (KeepAlive) starts a fresh relay


if __name__ == "__main__":
    main()
