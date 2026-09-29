"""The relay: a source's cameras served over the network device protocol, following them as they come
and go. A manager polls the source every POLL_S: a camera newly attached is opened, one gone (detached,
or its frames stopped) is closed and dropped, to be opened again when it is back. Each connection
serves one camera: the hello {"open": "camera", "serial"} is answered with its optics and depth scale,
then frames stream (colour BGR uint8 and depth uint16 blobs, with "t", when the camera delivered them,
on the machine's monotonic clock) until the camera goes; the client connects again later. A hello
{"open": "status"} is answered with what is attached and served."""
import logging
import socket
import threading
import time

from camrelay.protocol import close, recv, send

log = logging.getLogger("camrelay")
POLL_S = 1.0
RETRY_S = 5.0              # a camera that failed to open is tried again after this long
LIST_FAILS = 3             # listings failing in a row: the USB state is stuck; the relay stops (to be restarted)


class Relay:
    def __init__(self, source, listen: str = "127.0.0.1:8820"):
        self.source = source
        host, port = listen.rsplit(":", 1)
        self.srv = socket.create_server((host, int(port)))
        self.address = f"{host}:{self.srv.getsockname()[1]}"
        self.cams: dict = {}                   # serial -> Camera, streaming
        self.attached: set[str] = set()
        self.failed: dict[str, tuple[float, str]] = {}     # serial -> (when, why) its last open failed
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.stuck, self._list_fails = False, 0

    def start(self) -> "Relay":
        threading.Thread(target=self._manage, name="camrelay-manage", daemon=True).start()
        threading.Thread(target=self._serve, name="camrelay-serve", daemon=True).start()
        return self

    def stop(self) -> None:
        self.stopped.set()
        close(self.srv)
        with self.lock:
            for cam in self.cams.values():
                cam.close()
            self.cams.clear()

    # ---- following the cameras ----
    def _manage(self) -> None:
        while not self.stopped.is_set():
            self.poll()
            self.stopped.wait(POLL_S)

    def poll(self) -> None:
        try:
            attached = set(self.source.serials())
            self._list_fails = 0
        except Exception as e:                              # the USB layer hiccuped: next time
            self._list_fails += 1
            log.warning("listing the cameras failed: %s", e)
            if self._list_fails >= LIST_FAILS:              # it didn't: after a USB reset librealsense stayed
                log.error("listing failed %d times in a row: stopping, to start again afresh", self._list_fails)
                self.stuck = True                           # failing until the process started again
                self.stop()
            return
        with self.lock:
            self.attached = attached
            for serial in [s for s, c in self.cams.items() if not c.alive or s not in attached]:
                self.cams.pop(serial).close()
                log.warning("camera %s gone", serial)
        now = time.monotonic()
        for serial in sorted(attached - set(self.cams)):
            if now - self.failed.get(serial, (-RETRY_S, ""))[0] < RETRY_S:
                continue
            try:
                cam = self.source.open(serial)
            except Exception as e:
                self.failed[serial] = (now, str(e))
                log.warning("camera %s attached, but it did not open: %s", serial, e)
                continue
            self.failed.pop(serial, None)
            with self.lock:
                self.cams[serial] = cam
            log.info("camera %s streaming", serial)

    # ---- serving ----
    def _serve(self) -> None:
        while not self.stopped.is_set():
            try:
                sock, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._session, args=(sock,), daemon=True).start()

    def _session(self, sock) -> None:
        try:
            head, _ = recv(sock)
            if head.get("open") == "status":
                with self.lock:
                    send(sock, {"attached": sorted(self.attached), "streaming": sorted(self.cams),
                                "failed": {s: why for s, (_, why) in self.failed.items()}})
            elif head.get("open") == "camera":
                self._camera(sock, str(head.get("serial")))
            else:
                send(sock, {"error": f"the relay serves cameras only, not {head.get('open')!r}"})
        except (OSError, ValueError):
            pass                                            # the client went away
        finally:
            close(sock)

    def _camera(self, sock, serial: str) -> None:
        with self.lock:
            cam = self.cams.get(serial)
            why = ("not attached" if serial not in self.attached else
                   f"attached, not open: {self.failed[serial][1]}" if serial in self.failed else "opening")
        if cam is None:
            send(sock, {"error": f"camera {serial}: {why}"})
            return
        send(sock, {"optics": cam.optics, "depth_mm_per_unit": cam.depth_mm_per_unit})
        seq = 0
        while cam.alive and not self.stopped.is_set():
            f = cam.frame(seq)
            if f is None:
                continue
            seq, t, color, depth = f
            send(sock, {"size": [color.shape[1], color.shape[0]], "t": t}, color.tobytes(), depth.tobytes())
