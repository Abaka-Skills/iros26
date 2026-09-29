"""The network device protocol's framing (myaloha.netdev has the backend's side): a 4-byte big-endian
length, a JSON head listing the binary blobs that follow ({"blobs": [sizes]}), then the blobs."""
import json
import socket
import struct


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
    if sock is None:
        return
    for end in (lambda: sock.shutdown(socket.SHUT_RDWR), sock.close):
        try:
            end()
        except OSError:
            pass
