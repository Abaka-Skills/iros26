"""The shared interface of every topic.

A topic is a perception process: it reads the robot's API (camera frames, arm state,
coordinates) and other topics, and serves what it computes on its own localhost port. The
backend starts and stops topics (myaloha.topics.manager); a topic never commands the robot.

    GET /health    {"name", "state": "running" | "waiting", "waiting": reason, "seq", "hz"}
    GET /latest    the newest result: {"seq", "t" (robot clock), "wall", "data"}; before the
                   first one, data is null and "waiting" says why
    WS  /stream    every new result as it is published (the latest one first)

Subclasses set `name`, DEPENDS (topics that must run first), REQUIRES (hardware, as for mods),
HZ and OUTPUT (what one result holds; shown to agents), and implement step(), which returns one
result or None (after calling wait(reason)); setup() runs once in the process before it.
"""
import asyncio
import io
import logging
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import httpx
import numpy as np

from myaloha.mods.base import Need, first_paragraph


@dataclass
class Frame:
    key: str
    color: np.ndarray       # BGR
    depth: np.ndarray       # metres, aligned to colour (0 = no reading)
    K: np.ndarray
    dist: np.ndarray
    model: str
    t: float                # robot clock (perf_counter in the backend) of the last frame used


class Topic:
    name: str = "topic"
    DEPENDS: tuple[str, ...] = ()
    REQUIRES: tuple[Need, ...] = ()
    HZ: float = 5.0
    # what one result holds, for the panel and the MCP read tool
    OUTPUT: str = ""

    @classmethod
    def catalog(cls) -> dict:
        """What the panel and agents are told about the topic."""
        return {"name": cls.name, "doc": first_paragraph(cls), "depends": list(cls.DEPENDS),
                "target_hz": cls.HZ, "output": cls.OUTPUT,
                "requires": [{"text": n.text, "least": n.least} for n in cls.REQUIRES]}

    def __init__(self, backend: str):
        self.http = httpx.Client(base_url=backend, timeout=5.0)
        self.log = logging.getLogger(f"myaloha.topic.{self.name}")
        self.waiting: str | None = None

    # ---- to implement ----
    def setup(self) -> None:
        """Called once in the topic's process before the first step."""

    def step(self) -> dict | None:
        raise NotImplementedError

    # ---- helpers ----
    def wait(self, reason: str) -> None:
        """Report that the topic can't produce a result right now, and why."""
        if reason != self.waiting:
            self.log.info("waiting: %s", reason)
        self.waiting = reason

    def get(self, path: str):
        r = self.http.get("/api" + path)
        r.raise_for_status()
        return r.json()

    def frame(self, key: str, n: int = 1) -> Frame:
        """The newest `n` frames of a camera (median depth), with its optics."""
        r = self.http.get(f"/api/cameras/{key}/raw", params={"n": n})
        r.raise_for_status()
        z = np.load(io.BytesIO(r.content))
        return Frame(key, z["color"], z["depth"], z["K"], z["dist"], str(z["model"]), float(z["t"]))

    def topic_url(self, name: str) -> str | None:
        """Base URL of another running topic, or None."""
        t = next((t for t in self.get("/topics") if t["name"] == name), None)
        return t["url"] if t and t["state"] in ("running", "waiting") else None

    def subscribe(self, name: str, keep_s: float = 1.0) -> "Subscription":
        return Subscription(self, name, keep_s)


class Subscription:
    """Another topic's results, kept for `keep_s` seconds from its /stream (reconnects on its own)."""

    def __init__(self, topic: Topic, name: str, keep_s: float):
        self.topic, self.name, self.keep_s = topic, name, keep_s
        self.buffer: deque = deque()
        self.connected = False
        threading.Thread(target=self._run, name=f"sub-{name}", daemon=True).start()

    def _run(self) -> None:
        import json

        from websockets.sync.client import connect
        while True:
            try:
                url = self.topic.topic_url(self.name)
                if url:
                    with connect(url.replace("http", "ws", 1) + "/stream", open_timeout=2) as ws:
                        self.connected = True
                        for msg in ws:
                            m = json.loads(msg)
                            self.buffer.append(m)
                            while self.buffer and m["t"] - self.buffer[0]["t"] > self.keep_s:
                                self.buffer.popleft()
            except Exception:
                pass
            self.connected = False
            time.sleep(0.5)

    @property
    def latest(self) -> dict | None:
        return self.buffer[-1] if self.connected and self.buffer else None

    def at(self, t: float) -> dict | None:
        """The result published nearest to robot time `t`."""
        items = list(self.buffer) if self.connected else []
        return min(items, key=lambda m: abs(m["t"] - t)) if items else None


# ---------------- the topic's process ----------------

def serve(topic: Topic, port: int) -> None:
    """Run `topic` at its HZ and serve its results on 127.0.0.1:`port` until the parent exits."""
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.middleware.cors import CORSMiddleware

    state = {"seq": 0, "latest": None, "hz": 0.0}
    parent = os.getppid()

    def loop():
        topic.setup()
        period, stamps = 1 / topic.HZ, deque(maxlen=20)
        failed = None
        while True:
            t0 = time.monotonic()
            try:
                data = topic.step()
                failed = None
            except Exception as e:
                data = None
                if str(e) != failed:
                    topic.log.error("step failed: %s: %s", type(e).__name__, e)
                failed = str(e)
                topic.wait(f"error: {e}")
            if data is not None:
                topic.waiting = None
                t = data.pop("t", None)
                state["seq"] += 1
                state["latest"] = {"seq": state["seq"], "t": t, "wall": time.time(), "data": data}
                stamps.append(t0)
                state["hz"] = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def orphan_watch():  # the backend died without stopping us: exit too
        while os.getppid() == parent:
            time.sleep(1.0)
        os._exit(0)

    app = FastAPI(title=f"myaloha topic: {topic.name}")
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["GET"], allow_headers=["*"])

    @app.get("/health")
    def health():
        return {"name": topic.name, "state": "waiting" if topic.waiting else "running", "waiting": topic.waiting,
                "seq": state["seq"], "hz": round(state["hz"], 1)}

    @app.get("/latest")
    def latest():
        return state["latest"] or {"seq": 0, "t": None, "wall": None, "data": None, "waiting": topic.waiting}

    @app.websocket("/stream")
    async def stream(ws: WebSocket):
        await ws.accept()
        sent = 0
        try:
            while True:
                m = state["latest"]
                if m and m["seq"] != sent:
                    sent = m["seq"]
                    await ws.send_json(m)
                await asyncio.sleep(0.01)
        except (WebSocketDisconnect, RuntimeError):
            pass

    threading.Thread(target=loop, name="topic-loop", daemon=True).start()
    threading.Thread(target=orphan_watch, name="orphan-watch", daemon=True).start()
    topic.log.info("serving on 127.0.0.1:%d", port)
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning", timeout_graceful_shutdown=0.5)


def configure_logging() -> None:
    """One line per record on stdout, `LEVEL message`: the backend parses these into its log."""
    import warnings
    # numpy 2 on macOS (Accelerate) flags finite matmuls as overflow / divide by zero: noise, not errors
    warnings.filterwarnings("ignore", message=".*encountered in matmul", category=RuntimeWarning)
    root = logging.getLogger("myaloha")
    root.setLevel(logging.INFO)
    h = logging.StreamHandler(sys.stdout)
    h.setFormatter(logging.Formatter("%(levelname)s %(message)s"))
    root.addHandler(h)
    root.propagate = False
    sys.stdout.reconfigure(line_buffering=True)
