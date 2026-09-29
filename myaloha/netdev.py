"""Arms and cameras served over TCP by another process: a simulator (`panel --simulator HOST:PORT`),
or, for the cameras, the camera relay on this machine (camrelay/).

One protocol for every device: a connection opens with {"open": "arm" | "camera", ...} and then
carries length-prefixed messages, each a JSON head plus optional binary blobs.

- arm: ArmClient stands in for trossen_arm.TrossenArmDriver; each call is one request/reply
  ({"call", "args", "kwargs"} -> {"result"} or {"error"}), and blocking calls block the reply.
- camera: CameraClient; the reply to the open carries the optics, then frames stream (colour BGR
  uint8 and depth uint16 blobs), stamped with "t" when the server sends one (when the camera delivered
  the frame, on this machine's monotonic clock: time.perf_counter), else on arrival.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
import time
from types import SimpleNamespace

import numpy as np
import trossen_arm


# ---- framing ----
def send(sock: socket.socket, head: dict, *blobs: bytes) -> None:
    h = json.dumps({**head, "blobs": [len(b) for b in blobs]} if blobs else head).encode()
    sock.sendall(b"".join([struct.pack("!I", len(h)), h, *blobs]))


def recv(sock: socket.socket) -> tuple[dict, list[bytearray]]:
    head = json.loads(_exact(sock, struct.unpack("!I", _exact(sock, 4))[0]))
    return head, [_exact(sock, n) for n in head.pop("blobs", [])]


def _exact(sock, n: int) -> bytearray:
    buf, got = bytearray(n), 0
    view = memoryview(buf)
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if not k:
            raise ConnectionError("connection closed")
        got += k
    return buf


def close(sock: socket.socket | None) -> None:
    """Shut down and close, whatever state the socket is in."""
    if sock is None:
        return
    for end in (lambda: sock.shutdown(socket.SHUT_RDWR), sock.close):
        try:
            end()
        except OSError:
            pass


def encode(v):
    """Driver values as JSON: modes by name, output structures as plain dicts."""
    if isinstance(v, trossen_arm.Mode):
        return {"$mode": v.name}
    if isinstance(v, SimpleNamespace):
        return {"$ns": {k: encode(x) for k, x in vars(v).items()}}
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, (list, tuple)):
        return [encode(x) for x in v]
    if isinstance(v, dict):
        return {k: encode(x) for k, x in v.items()}
    return v


def decode(v):
    if isinstance(v, dict):
        if "$mode" in v:
            return trossen_arm.Mode.__members__[v["$mode"]]
        if "$ns" in v:
            return SimpleNamespace(**{k: decode(x) for k, x in v["$ns"].items()})
        return {k: decode(x) for k, x in v.items()}
    if isinstance(v, list):
        return [decode(x) for x in v]
    return v


def connect(address: str, hello: dict, timeout: float = 5.0) -> tuple[socket.socket, dict]:
    """A connection to `address` (host:port) opened with `hello`, and the server's reply to it."""
    host, _, port = address.rpartition(":")
    sock = socket.create_connection((host or "127.0.0.1", int(port)), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        send(sock, hello)
        head, _ = recv(sock)
    except Exception:
        sock.close()
        raise
    if "error" in head:
        sock.close()
        raise RuntimeError(head["error"])
    sock.settimeout(None)
    return sock, head


class Connection:
    """Request/reply calls over one connection."""

    def __init__(self, sock: socket.socket):
        self.sock, self.lock = sock, threading.Lock()

    def call(self, name: str, *args, **kwargs):
        with self.lock:
            send(self.sock, {"call": name, "args": encode(args), "kwargs": encode(kwargs)})
            head, _ = recv(self.sock)
        if "error" in head:
            raise RuntimeError(head["error"])
        return decode(head.get("result"))

    def close(self) -> None:
        close(self.sock)


class ArmClient:
    """trossen_arm.TrossenArmDriver, for an arm controller served at `address`: configure() opens
    a session with the controller at `serv_ip`; every other method is forwarded as is."""

    def __init__(self, address: str):
        self.address = address
        self.conn: Connection | None = None

    def configure(self, model, end_effector, serv_ip, clear_error, timeout=20.0):
        self.cleanup()
        sock, _ = connect(self.address, {"open": "arm", "ip": serv_ip, "clear_error": clear_error,
                                         "timeout": timeout}, timeout=timeout + 2)
        self.conn = Connection(sock)

    def get_is_configured(self) -> bool:
        return self.conn is not None

    def cleanup(self, reboot_controller=False):
        if self.conn:
            try:
                self.conn.call("cleanup", reboot_controller=reboot_controller)
            except (OSError, RuntimeError):
                pass   # the session is gone either way
            self.conn.close()
            self.conn = None

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)

        def forward(*args, **kwargs):
            if self.conn is None:
                raise RuntimeError("driver not configured")
            try:
                return self.conn.call(name, *args, **kwargs)
            except OSError as e:                  # the controller went away: the session is dead
                raise RuntimeError(f"{name}: {e}") from e
        return forward


class CameraClient:
    """A camera served at `address`, with the frame attributes CameraRig reads. Frames stop when the
    connection drops; open a new client to resume."""

    def __init__(self, address: str, serial: str):
        self.address, self.serial = address, serial
        self.frame_lock = threading.Lock()
        self.latest_color_frame = self.latest_depth_frame = self.latest_timestamp = None
        self.optics: dict = {}
        self.depth_mm_per_unit = 1.0
        self.sock: socket.socket | None = None

    def connect(self, timeout: float = 5.0) -> None:
        self.sock, head = connect(self.address, {"open": "camera", "serial": self.serial}, timeout)
        self.optics = {k: np.array(v) if isinstance(v, list) else v for k, v in head["optics"].items()}
        self.depth_mm_per_unit = head["depth_mm_per_unit"]
        threading.Thread(target=self._run, name=f"netcam-{self.serial}", daemon=True).start()

    def _run(self) -> None:
        try:
            while True:
                head, (color, depth) = recv(self.sock)
                h, w = head["size"][1], head["size"][0]
                c = np.frombuffer(color, np.uint8).reshape(h, w, 3)
                d = np.frombuffer(depth, np.uint16).reshape(h, w, 1)
                with self.frame_lock:
                    self.latest_color_frame, self.latest_depth_frame = c, d
                    self.latest_timestamp = head.get("t") or time.perf_counter()
        except (OSError, ValueError):
            pass                                  # the stream ended: frames stop

    def disconnect(self) -> None:
        close(self.sock)

