"""The network device protocol, simulator side. The same wire format the backend's device clients
speak; this is the simulator's own implementation of it.

A connection opens with {"open": "arm" | "camera" | "control", ...} and then carries messages:
a 4-byte big-endian length, a JSON head, then the binary blobs the head lists by size ("blobs").
Values: an arm mode is {"$mode": name}; a structure (the robot output) is {"$ns": {...}}.
"""
from __future__ import annotations

import json
import socket
import struct
from types import SimpleNamespace

import numpy as np


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


class Mode(str):
    """An arm mode by name ("idle", "position", "external_effort", ...)."""


def encode(v):
    if isinstance(v, Mode):
        return {"$mode": str(v)}
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
            return Mode(v["$mode"])
        if "$ns" in v:
            return SimpleNamespace(**{k: decode(x) for k, x in v["$ns"].items()})
        return {k: decode(x) for k, x in v.items()}
    if isinstance(v, list):
        return [decode(x) for x in v]
    return v


def call(address: str, name: str, *args, timeout: float = 5.0):
    """One control call to the simulator at `address` (host:port)."""
    host, _, port = address.rpartition(":")
    with socket.create_connection((host or "127.0.0.1", int(port)), timeout=timeout) as sock:
        send(sock, {"open": "control"})
        head, _ = recv(sock)
        if "error" not in head:
            send(sock, {"call": name, "args": encode(args), "kwargs": {}})
            head, _ = recv(sock)
    if "error" in head:
        raise RuntimeError(head["error"])
    return decode(head.get("result"))
