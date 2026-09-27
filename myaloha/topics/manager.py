"""Topics as child processes of the backend: start (after the topics they depend on), watch,
stop (with the topics that depend on them), and forward their log lines into the backend's log.

Each topic runs `python -m myaloha.topics <name> --port P --backend URL` on a free localhost
port. Under sudo (the cameras need root on macOS) topics run as the desktop user: they only
read the API. Nothing restarts on its own: a topic that exits unexpectedly is `crashed`.
"""
import collections
import logging
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import httpx

from myaloha.mods.base import evaluate
from myaloha.topics import TOPICS

log = logging.getLogger("myaloha.topics")
REPO = Path(__file__).resolve().parents[2]
LEVELS = logging.getLevelNamesMapping()


class TopicError(RuntimeError):
    """A start or stop the caller must decide on: missing dependencies, or running dependents."""

    def __init__(self, message: str, **detail):
        super().__init__(message)
        self.detail = {"message": message, **detail}


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Proc:
    def __init__(self, name: str, popen: subprocess.Popen, port: int):
        self.name, self.popen, self.port = name, popen, port
        self.since = time.time()
        self.state = "starting"            # starting | running | waiting | crashed
        self.waiting: str | None = None
        self.hz = 0.0
        self.tail: collections.deque = collections.deque(maxlen=20)
        self.stopping = False


class TopicManager:
    def __init__(self, robot, backend_url: str):
        self.robot, self.backend = robot, backend_url
        self.procs: dict[str, Proc] = {}
        self.crashed: dict[str, dict] = {}   # name -> last crash (exit code, time, last lines)
        self._lock = threading.RLock()
        self._http = httpx.Client(timeout=0.3)
        threading.Thread(target=self._watch, name="topic-watch", daemon=True).start()

    # ---- queries ----
    @staticmethod
    def dependents(name: str) -> list[str]:
        """Topics that depend on `name`, directly or not."""
        out = []
        for other, cls in TOPICS.items():
            if name in cls.DEPENDS:
                out += [other, *TopicManager.dependents(other)]
        return list(dict.fromkeys(out))

    @staticmethod
    def dependencies(name: str) -> list[str]:
        """Everything `name` needs, in start order."""
        out = []
        for dep in TOPICS[name].DEPENDS:
            out += [*TopicManager.dependencies(dep), dep]
        return list(dict.fromkeys(out))

    def running(self, name: str) -> bool:
        p = self.procs.get(name)
        return bool(p and p.popen.poll() is None)

    def requirements(self, name: str) -> dict:
        return evaluate(TOPICS[name].REQUIRES, self.robot)

    def status(self) -> dict:
        out = {}
        for name, cls in TOPICS.items():
            p = self.procs.get(name)
            if p:
                s = {"state": p.state, "since": p.since, "pid": p.popen.pid, "port": p.port,
                     "url": f"http://127.0.0.1:{p.port}", "waiting": p.waiting, "hz": p.hz}
            elif name in self.crashed:
                s = {"state": "crashed", **self.crashed[name], "url": None}
            else:
                s = {"state": "stopped", "url": None}
            out[name] = {"name": name, **s, "requirements": self.requirements(name)}
        return out

    # ---- control ----
    def start(self, name: str, with_deps: bool = False) -> None:
        with self._lock:
            missing = [d for d in self.dependencies(name) if not self.running(d)]
            if missing and not with_deps:
                raise TopicError(f"{name} needs {', '.join(missing)} running", missing=missing)
            for n in [*missing, name]:
                if not self.running(n):
                    self._spawn(n)

    def stop(self, name: str, cascade: bool = False) -> list[str]:
        """Stop `name`; with `cascade`, the running topics that depend on it first. Returns what stopped."""
        with self._lock:
            deps = [d for d in self.dependents(name) if self.running(d)]
            if deps and not cascade:
                raise TopicError(f"{', '.join(deps)} depend{'s' if len(deps) == 1 else ''} on {name}", dependents=deps)
            stopped = [n for n in [*reversed(deps), name] if n in self.procs]
            for n in stopped:
                self._kill(n)
            self.crashed.pop(name, None)
            return stopped

    def restart(self, name: str) -> None:
        """Restart `name` alone; topics subscribed to it reconnect on their own."""
        with self._lock:
            if name in self.procs:
                self._kill(name)
            self.start(name)

    def stop_all(self) -> None:
        with self._lock:
            for name in list(self.procs):
                self._kill(name)

    # ---- processes ----
    def _spawn(self, name: str) -> None:
        req = self.requirements(name)
        if not req["met"]:
            unmet = [n["text"] for n in req["needs"] if not n["met"]]
            raise TopicError(f"{name} needs {' and '.join(unmet)}", unmet=unmet)
        port = free_port()
        kw = {}
        if os.geteuid() == 0 and os.environ.get("SUDO_UID"):   # root only for the cameras
            kw = {"user": int(os.environ["SUDO_UID"]), "group": int(os.environ["SUDO_GID"])}
        popen = subprocess.Popen([sys.executable, "-B", "-m", "myaloha.topics", name, "--port", str(port),
                                  "--backend", self.backend], cwd=REPO, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True, **kw)
        p = Proc(name, popen, port)
        self.procs[name] = p
        self.crashed.pop(name, None)
        threading.Thread(target=self._pipe, args=(p,), name=f"topic-{name}-out", daemon=True).start()
        log.info("%s started (pid %d, port %d)", name, popen.pid, port)

    def _kill(self, name: str) -> None:
        p = self.procs.pop(name)
        p.stopping = True
        p.popen.terminate()
        try:
            p.popen.wait(2)
        except subprocess.TimeoutExpired:
            p.popen.kill()
            p.popen.wait()
        log.info("%s stopped", name)

    def _pipe(self, p: Proc) -> None:
        """The topic's stdout into the backend log, as `myaloha.topic.<name>`."""
        out = logging.getLogger(f"myaloha.topic.{p.name}")
        for line in p.popen.stdout:
            line = line.rstrip()
            if not line:
                continue
            p.tail.append(line)
            level, _, msg = line.partition(" ")
            if level in LEVELS:
                out.log(LEVELS[level], msg)
            else:                                   # a traceback or other raw output
                out.warning(line)

    def _watch(self) -> None:
        while True:
            time.sleep(0.5)
            for name, p in list(self.procs.items()):
                code = p.popen.poll()
                if code is not None:
                    if not p.stopping:
                        with self._lock:
                            if self.procs.get(name) is p:
                                del self.procs[name]
                                self.crashed[name] = {"code": code, "since": time.time(), "tail": list(p.tail)[-8:]}
                        log.error("%s crashed (exit %s): %s", name, code, " | ".join(list(p.tail)[-3:]) or "no output")
                    continue
                try:
                    h = self._http.get(f"http://127.0.0.1:{p.port}/health").json()
                    p.state, p.waiting, p.hz = h["state"], h["waiting"], h["hz"]
                except Exception:
                    pass   # still starting (or busy): keep the last state
