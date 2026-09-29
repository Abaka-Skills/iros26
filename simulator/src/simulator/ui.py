"""The simulator's own page: ground truth (a top view of the table), what each camera sees, and
the scene and device controls. Served by the simulator process; nothing of the backend's."""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2

PAGE = Path(__file__).with_name("ui.html")


def serve(sim, listen: str) -> str:
    """Serve the page for `sim` in the background; returns its address."""
    host, _, port = listen.rpartition(":")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def _send(self, code: int, body: bytes, kind: str = "application/json") -> None:
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode())

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            elif url.path == "/api/truth":
                self._json(sim.truth())
            elif url.path == "/api/info":
                self._json({**sim.info(), "table": sim.cfg["table"], "colors": sim.cfg["cubes"]["colors"],
                            "size": sim.cfg["cubes"]["size"],
                            "mounts": {a["name"]: a["mount"] for a in sim.cfg["arms"].values() if "mount" in a}})
            elif url.path.startswith("/api/frame/"):
                color = sim.colour(sim._device(url.path.rsplit("/", 1)[1].removesuffix(".jpg")))
                if color is None:
                    self._json({"error": "no frame"}, 404)
                    return
                ok, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self._send(200, buf.tobytes(), "image/jpeg")
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            try:
                if url.path == "/api/spawn":
                    self._json(sim.spawn(int(q["count"]) if q.get("count") else None,
                                         int(q["seed"]) if q.get("seed") else None))
                elif url.path.startswith(("/api/unplug/", "/api/plug/")):
                    _, _, action, name = url.path.split("/", 3)
                    getattr(sim, action)(name)
                    self._json(sim.info())
                else:
                    self._json({"error": "not found"}, 404)
            except (KeyError, ValueError) as e:
                self._json({"error": str(e)}, 400)

    server = ThreadingHTTPServer((host, int(port)), Handler)
    threading.Thread(target=server.serve_forever, name="sim-ui", daemon=True).start()
    return f"{host}:{server.server_address[1]}"
