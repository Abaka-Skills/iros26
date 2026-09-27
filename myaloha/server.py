"""The robot's API; the panel is only a client of it. Serves on localhost only.

    REST       read state, run and stop mods, start and stop topics   /api/...
    MCP        mods and topic reads for coding agents                 /mcp
    WebSocket  live state at `hz`, plus log lines and timeline changes /api/stream
    MJPEG      one stream per camera, color or depth                   /api/cameras/{key}/stream
    Camera WS  every camera over one socket (the panel uses this)      /api/cameras/ws
    Docs       every endpoint, interactive                             /docs
    Static     the panel (index.html) and the visitor page             /
"""
import asyncio
import collections
import contextlib
import io
import logging
import os
import signal
import threading
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
from fastapi import Body, FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from myaloha import mcp_server
from myaloha.mods import MODS
from myaloha.mods.base import RequirementError
from myaloha.topics import TOPICS
from myaloha.topics.manager import TopicError, TopicManager

PANEL = Path(__file__).resolve().parent.parent / "panel"
logger = logging.getLogger("myaloha.server")


class LogBuffer(logging.Handler):
    """The last `size` myaloha log records, numbered so clients can ask for what they missed."""

    def __init__(self, size: int = 2000):
        super().__init__()
        self.records: collections.deque = collections.deque(maxlen=size)
        self.seq = 0
        self._lock = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        with self._lock:
            self.seq += 1
            self.records.append({"seq": self.seq, "t": record.created, "level": record.levelname.lower(),
                                 "src": record.name.rsplit(".", 1)[-1], "msg": record.getMessage()})

    def since(self, seq: int, limit: int = 500) -> list[dict]:
        with self._lock:
            return [r for r in self.records if r["seq"] > seq][-limit:]


def create_app(robot, logs: LogBuffer, sim: bool = False, topics: TopicManager | None = None) -> FastAPI:
    topics = topics or TopicManager(robot, "http://127.0.0.1:8765")
    mcp = mcp_server.build(robot, topics)
    mcp_http = mcp.streamable_http_app(streamable_http_path="/mcp")

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        async with mcp.session_manager.run():
            yield
        topics.stop_all()

    app = FastAPI(title="myaloha", description="Robot API for the bimanual WidowX AI rig. Localhost only. "
                  "Coding agents: the MCP server is at /mcp.", lifespan=lifespan)
    app.router.routes.extend(mcp_http.routes)   # at /mcp, not /mcp/ (a mount would redirect)

    @app.middleware("http")
    async def revalidate_pages(request, call_next):
        """Page files change with the repo: make browsers revalidate instead of reusing a cached copy."""
        response = await call_next(request)
        if not request.url.path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-cache"
        return response

    def arm_or_404(name: str):
        if name not in robot.arms:
            raise HTTPException(404, f"no arm {name!r}; arms: {', '.join(robot.arms)}")
        return robot.arms[name]

    def cam_or_404(key: str):
        if not robot.cameras or key not in robot.cameras.specs:
            raise HTTPException(404, f"no camera {key!r}")
        return key

    def mod_or_404(name: str):
        if name not in MODS:
            raise HTTPException(404, f"no mod {name!r}; mods: {', '.join(MODS)}")
        return MODS[name]

    # ---- robot ----
    @app.get("/api/robot", tags=["robot"], summary="Static description: major states, arm and camera specs")
    def robot_info():
        return {
            "sim": sim,
            "states": robot.STATES,
            "arms": [asdict(a.spec) for a in robot.arms.values()],
            "cameras": [asdict(s) for s in robot.cameras.specs.values()] if robot.cameras else [],
        }

    def live() -> dict:
        return {**robot.status(), "requirements": {name: cls.requirements(robot) for name, cls in MODS.items()},
                "topics": topics.status(), "last_run": robot.last_run}

    @app.get("/api/status", tags=["robot"],
             summary="Everything live: major state, arms, cameras, running mods, each mod's requirement check")
    def status():
        return live()

    @app.get("/api/timeline", tags=["robot"], summary="Major-state history, one segment per state or mod run")
    def timeline():
        return robot.timeline

    @app.post("/api/stop", tags=["robot"], summary="Stop every running mod (each leaves its arms holding)")
    def stop():
        robot.stop()
        return [m.status() for m in robot.mods]

    @app.post("/api/estop", tags=["safety"],
              summary="EMERGENCY STOP: brake every arm now; latched until /api/estop/reset")
    async def estop():
        # async on purpose: runs on the event loop at once instead of queuing for a worker thread
        robot.emergency_stop()
        return {"estopped": True}

    @app.post("/api/estop/reset", tags=["safety"],
              summary="Release the emergency stop; arms stay braked until a mod moves them")
    def estop_reset():
        try:
            robot.reset_emergency_stop()
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        return {"estopped": False}

    @app.post("/api/shutdown", tags=["robot"],
              summary="Stop mods, release the hardware and exit the server (same as Ctrl-C)")
    def shutdown():
        threading.Timer(0.2, os.kill, [os.getpid(), signal.SIGINT]).start()
        return {"shutting_down": True}

    @app.get("/api/coordinates", tags=["robot"],
             summary="Latest localize result: W, follower base poses and fixed cameras in W, evaluation report")
    def coordinates():
        if not robot.coordinates:
            raise HTTPException(404, "no coordinates yet; run the localize mod")
        return robot.coordinates

    @app.get("/api/coordinates/matches.jpg", tags=["robot"], summary="The SIFT matches behind the latest result",
             responses={200: {"content": {"image/jpeg": {}}}})
    def coordinates_matches():
        path = robot.coordinates and PANEL.parent / robot.coordinates["frames"] / "matches.jpg"
        if not path or not path.exists():
            raise HTTPException(404, "no match picture")
        return FileResponse(path, media_type="image/jpeg")

    # ---- arms ----
    @app.get("/api/arms", tags=["arms"], summary="All arms: connection, mode, joint pos/vel/effort")
    def arms():
        return {name: arm.status() for name, arm in robot.arms.items()}

    @app.get("/api/arms/{name}", tags=["arms"])
    def arm(name: str):
        return arm_or_404(name).status()

    @app.post("/api/arms/connect", tags=["arms"], summary="Try to connect every absent arm (no motion)")
    async def connect_arms():
        await run_in_threadpool(robot.connect)
        return {name: a.status() for name, a in robot.arms.items()}

    # ---- cameras ----
    @app.get("/api/cameras", tags=["cameras"], summary="All cameras: ok, measured fps, error")
    def cameras():
        return robot.cameras.status() if robot.cameras else {}

    @app.post("/api/cameras/connect", tags=["cameras"],
              summary="Reopen every camera that is down (e.g. after it was unplugged and plugged back in)")
    async def connect_cameras():
        if not robot.cameras:
            raise HTTPException(404, "no cameras")
        reopened = await run_in_threadpool(robot.cameras.reconnect)
        return {"reopened": reopened, "cameras": robot.cameras.status()}

    @app.get("/api/cameras/{key}/frame", tags=["cameras"], summary="Newest frame as JPEG",
             responses={200: {"content": {"image/jpeg": {}}}})
    def frame(key: str, kind: str = Query("color", pattern="^(color|depth)$")):
        data = robot.cameras.jpeg(cam_or_404(key), kind)
        if data is None:
            raise HTTPException(503, f"{key}: no frame")
        return Response(data, media_type="image/jpeg")

    @app.get("/api/cameras/{key}/optics", tags=["cameras"],
             summary="Colour intrinsics: K, lens distortion (k1 k2 p1 p2 k3) and model, image size")
    def optics(key: str):
        cam_or_404(key)
        if key not in robot.cameras.cams:
            raise HTTPException(503, f"{key}: not connected")
        dist, model = robot.cameras.distortion(key)
        return {"K": robot.cameras.optics[key]["K_color"].tolist(), "dist": [float(v) for v in dist],
                "model": model, "size": [robot.cameras.width, robot.cameras.height]}

    @app.get("/api/cameras/{key}/raw", tags=["cameras"],
             summary="The next `n` frames for perception (topics): an .npz with color (BGR), depth "
                     "(metres, aligned to colour, median of n), K, dist, model, t (robot clock)",
             responses={200: {"content": {"application/octet-stream": {}}}})
    def raw(key: str, n: int = Query(1, ge=1, le=30)):
        cam_or_404(key)
        if not robot.cameras.status()[key]["ok"]:
            raise HTTPException(503, f"{key}: no frames")
        color, depth, K, t = robot.cameras.capture(key, n=n)
        dist, model = robot.cameras.distortion(key)
        buf = io.BytesIO()
        np.savez(buf, color=color, depth=depth, K=K, dist=np.asarray(dist, float), model=np.array(model), t=np.array(t))
        return Response(buf.getvalue(), media_type="application/octet-stream")

    @app.get("/api/cameras/{key}/stream", tags=["cameras"], summary="MJPEG stream; use as an <img> src")
    def stream_camera(key: str, kind: str = Query("color", pattern="^(color|depth)$"), fps: float = Query(15, gt=0, le=60)):
        cam_or_404(key)
        if robot.cameras.jpeg(key, kind) is None:
            raise HTTPException(503, f"{key}: no frame")

        def frames():
            last, seen = None, time.monotonic()
            while time.monotonic() - seen < 5:  # frames stopped: end the stream
                data = robot.cameras.jpeg(key, kind)
                if data is not None and data is not last:
                    last, seen = data, time.monotonic()
                    yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n%s\r\n" % (len(data), data)
                time.sleep(1 / fps)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.websocket("/api/cameras/ws")
    async def camera_socket(ws: WebSocket, fps: float = 15):
        """Every camera over one socket; browsers allow only six HTTP connections per host, so
        one MJPEG stream per tile starves other requests. Each binary message is
        [key length][key][kind: 0 color, 1 depth][JPEG]. Send {"key": k, "kind": "color"|"depth"}
        to switch a camera; all start as color."""
        await ws.accept()
        cams = robot.cameras
        kinds = {k: "color" for k in cams.specs} if cams else {}
        sent: dict[str, bytes] = {}

        async def listen():
            while True:
                m = await ws.receive_json()
                if m.get("key") in kinds and m.get("kind") in ("color", "depth"):
                    kinds[m["key"]] = m["kind"]

        listener = asyncio.create_task(listen())
        period = 1 / max(1.0, min(fps, 30.0))
        try:
            while True:
                pending = list(kinds.items())
                frames = await asyncio.gather(*(asyncio.to_thread(cams.jpeg, k, kind) for k, kind in pending))
                for (key, kind), data in zip(pending, frames):
                    if data is not None and data is not sent.get(key):
                        sent[key] = data
                        head = key.encode()
                        await ws.send_bytes(bytes([len(head)]) + head + bytes([kind == "depth"]) + data)
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            listener.cancel()

    # ---- mods ----
    @app.get("/api/mods", tags=["mods"], summary="Catalog: every mod with its params, states and transitions")
    def mods():
        return [cls.catalog() for cls in MODS.values()]

    @app.get("/api/mods/running", tags=["mods"], summary="Running mods, outermost first")
    def running():
        return [m.status() for m in robot.mods]

    @app.get("/api/mods/{name}", tags=["mods"])
    def mod(name: str):
        return mod_or_404(name).catalog()

    @app.post("/api/mods/{name}/run", tags=["mods"], status_code=202,
              summary="Start a mod with params (defaults for any left out). 412: hardware below the "
                      "mod's REQUIRES; 409: robot busy or emergency stop")
    def run(name: str, params: dict = Body(default={}, embed=True)):
        cls = mod_or_404(name)
        try:
            p = cls.params_from(params)
        except (ValueError, TypeError) as e:
            raise HTTPException(422, str(e))
        m = cls(robot, p)
        try:
            m.start()
        except RequirementError as e:
            raise HTTPException(412, str(e))
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        return m.status()

    # ---- topics ----
    def topic_or_404(name: str):
        if name not in TOPICS:
            raise HTTPException(404, f"no topic {name!r}; topics: {', '.join(TOPICS)}")
        return name

    @app.get("/api/topics", tags=["topics"],
             summary="Every topic: what it computes, what it depends on, its state and URL when running")
    def topic_list():
        st = topics.status()
        return [{**cls.catalog(), **st[name]} for name, cls in TOPICS.items()]

    @app.post("/api/topics/{name}/start", tags=["topics"],
              summary="Start a topic. 409 with `missing` if topics it depends on are stopped, unless "
                      "with_deps (then they start first); 412 with `unmet` if its requirements are not met")
    def topic_start(name: str, with_deps: bool = False):
        try:
            topics.start(topic_or_404(name), with_deps)
        except TopicError as e:
            raise HTTPException(412 if "unmet" in e.detail else 409, e.detail)
        return topics.status()[name]

    @app.post("/api/topics/{name}/stop", tags=["topics"],
              summary="Stop a topic. 409 with `dependents` if running topics depend on it, unless "
                      "cascade (then they stop too)")
    def topic_stop(name: str, cascade: bool = False):
        try:
            return {"stopped": topics.stop(topic_or_404(name), cascade)}
        except TopicError as e:
            raise HTTPException(409, e.detail)

    @app.post("/api/topics/{name}/restart", tags=["topics"], summary="Stop and start a topic (its dependents reconnect)")
    def topic_restart(name: str):
        try:
            topics.restart(topic_or_404(name))
        except TopicError as e:
            raise HTTPException(409, e.detail)
        return topics.status()[name]

    # ---- log ----
    @app.get("/api/log", tags=["log"], summary="Log records after `after` (a seq number)")
    def log(after: int = 0, limit: int = Query(500, le=2000)):
        return {"seq": logs.seq, "records": logs.since(after, limit)}

    # ---- live stream ----
    @app.websocket("/api/stream")
    async def stream(ws: WebSocket, hz: float = 30):
        """Pushes {"type": "state", ...} at `hz`, {"type": "log", records} for new log lines
        (the last 200 on connect) and {"type": "timeline", timeline} whenever it changes.
        Accepts {"type": "estop"}: the lowest-latency way to trigger the emergency stop."""
        await ws.accept()
        seq, tl_key, period = max(0, logs.seq - 200), None, 1 / max(1.0, min(hz, 60.0))

        async def listen():
            while True:
                if (await ws.receive_json()).get("type") == "estop":
                    robot.emergency_stop()

        listener = asyncio.create_task(listen())
        try:
            while True:
                if records := logs.since(seq):
                    seq = records[-1]["seq"]
                    await ws.send_json({"type": "log", "records": records})
                tl = robot.timeline
                key = (len(tl), tl[-1].get("result") if tl else None)
                if key != tl_key:
                    tl_key = key
                    await ws.send_json({"type": "timeline", "timeline": tl})
                await ws.send_json({"type": "state", "t": time.time(), **live()})
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            listener.cancel()

    app.mount("/", StaticFiles(directory=PANEL, html=True), name="panel")
    return app
