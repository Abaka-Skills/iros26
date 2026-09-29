"""The simulator's devices over the network device protocol (simulator.protocol): arm controller
sessions by IP, camera streams by serial, and a control connection for the simulator's own calls
(truth, spawn, unplug, plug), which the backend never uses."""
from __future__ import annotations

import logging
import socket
from contextlib import contextmanager
import threading
import time

from simulator.protocol import close as _close
from simulator.protocol import decode, encode, recv, send
from simulator.controller import ArmController
from simulator.world import World

log = logging.getLogger("simulator")


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.world = World(cfg)
        self.arms = {ip: ArmController(ip, self.world if a["role"] == "follower" else None)
                     for ip, a in cfg["arms"].items()}
        self.world.controllers = {ip: c for ip, c in self.arms.items() if c.world}
        self.latest: dict = {}                   # serial -> (time, colour) of the last streamed frame
        self.unplugged: set[str] = set()
        self._sock: socket.socket | None = None
        self._conns: set[tuple[str, socket.socket]] = set()      # (device id, socket) of every session
        self._lock = threading.Lock()
        self._stopped = threading.Event()
        for name in cfg["unplugged"]:
            self.unplug(name)

    # ---- serving ----
    def start(self, listen: str | None = None) -> str:
        """Serve in the background; returns the address (host:port, port 0 picks a free one)."""
        host, _, port = (listen or self.cfg["listen"]).rpartition(":")
        self._sock = socket.create_server((host, int(port)))
        threading.Thread(target=self._accept, name="sim-accept", daemon=True).start()
        return f"{host}:{self._sock.getsockname()[1]}"

    def stop(self) -> None:
        self._stopped.set()
        if self._sock:
            self._sock.close()
        with self._lock:
            for _, s in list(self._conns):
                _close(s)
        self.world.stop()
        self.world.scene.close()

    def _accept(self) -> None:
        while True:
            try:
                sock, _ = self._sock.accept()
            except OSError:
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self._serve, args=(sock,), daemon=True).start()

    def _serve(self, sock: socket.socket) -> None:
        try:
            hello, _ = recv(sock)
            kind = hello.get("open")
            if kind == "arm":
                self._arm(sock, hello)
            elif kind == "camera":
                self._camera(sock, hello["serial"])
            elif kind == "control":
                send(sock, {})
                self._calls(sock, self, ("truth", "spawn", "unplug", "plug", "info"))
            else:
                send(sock, {"error": f"unknown device kind {kind!r}"})
        except OSError:
            pass                                   # the other end went away
        except Exception:
            log.exception("device session failed")
        finally:
            _close(sock)

    @contextmanager
    def _session(self, dev: str, sock):
        """A device session, closed by unplug() and stop()."""
        entry = (dev, sock)
        with self._lock:
            self._conns.add(entry)
        try:
            yield
        finally:
            with self._lock:
                self._conns.discard(entry)

    def _arm(self, sock, hello) -> None:
        ip = hello["ip"]
        if ip not in self.arms or ip in self.unplugged:
            time.sleep(min(hello.get("timeout", 1.0), 0.5))
            send(sock, {"error": f"timed out connecting to {ip}"})
            return
        send(sock, {})
        with self._session(ip, sock):
            self._calls(sock, self.arms[ip])

    def _calls(self, sock, target, allowed=None) -> None:
        while True:
            head, _ = recv(sock)
            name = head["call"]
            try:
                if name.startswith("_") or (allowed and name not in allowed):
                    raise AttributeError(f"no call {name!r}")
                out = {"result": encode(getattr(target, name)(*decode(head["args"]), **decode(head["kwargs"])))}
            except Exception as e:
                out = {"error": f"{type(e).__name__}: {e}"}
            send(sock, out)

    def _camera(self, sock, serial: str) -> None:
        if serial not in self.world.cams or serial in self.unplugged:
            send(sock, {"error": f"no camera {serial}"})
            return
        send(sock, {"optics": encode(self.world.optics(serial)), "depth_mm_per_unit": 0.1})
        period = 1 / self.cfg["render_fps"]
        with self._session(serial, sock):
            while not self._stopped.is_set():
                t0 = time.monotonic()
                f = self.world.frame(serial)
                if f is not None:
                    color, depth = f
                    self.latest[serial] = (time.monotonic(), color)
                    send(sock, {"size": [color.shape[1], color.shape[0]]}, color.tobytes(), depth.tobytes())
                time.sleep(max(0.0, period - (time.monotonic() - t0)) if f is not None else 0.5)

    def colour(self, serial: str):
        """The camera's latest colour frame: the one just streamed, else a fresh render (None if
        its arm is unplugged)."""
        t, color = self.latest.get(serial, (0.0, None))
        if time.monotonic() - t < 1.0:
            return color
        f = self.world.frame(serial)
        return f and f[0]

    # ---- control calls ----
    def _device(self, name: str) -> str:
        """An arm IP or camera serial, from itself or its name in the config."""
        for ip, a in self.cfg["arms"].items():
            if name in (ip, a["name"]):
                return ip
        for serial, c in self.cfg["cameras"].items():
            if name in (serial, c["name"]):
                return serial
        raise KeyError(f"no device {name!r}")

    def truth(self) -> dict:
        return self.world.truth()

    def spawn(self, scene=None, seed: int | None = None) -> dict:
        """New cubes: a count, or a list of {color, x, y, yaw, level}; default the config's."""
        self.world.spawn(scene if scene is not None else (self.cfg["cubes"]["scene"] or self.cfg["cubes"]["count"]), seed)
        return self.world.truth()

    def unplug(self, name: str) -> None:
        """Take a device off: its sessions drop, and it can't be opened until plugged back."""
        dev = self._device(name)
        self.unplugged.add(dev)
        if dev in self.arms:
            self.arms[dev].plugged = False
        with self._lock:
            for d, s in list(self._conns):
                if d == dev:
                    _close(s)
        log.info("unplugged %s", name)

    def plug(self, name: str) -> None:
        dev = self._device(name)
        self.unplugged.discard(dev)
        if dev in self.arms:
            self.arms[dev].plugged = True
        log.info("plugged %s", name)

    def info(self) -> dict:
        return {"seed": self.world.seed, "unplugged": sorted(self.unplugged),
                "arms": {ip: a["name"] for ip, a in self.cfg["arms"].items()},
                "cameras": {s: c["name"] for s, c in self.cfg["cameras"].items()}}

