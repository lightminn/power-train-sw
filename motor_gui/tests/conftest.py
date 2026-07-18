"""Host-only dependency stubs for the motor_gui test process.

Production containers provide python-can and FastAPI.  The workstation conda
environment used by these tests may not, so install small in-process stand-ins
only when the real packages are unavailable.
"""

from __future__ import annotations

import asyncio
import inspect
import queue
import sys
import threading
import time
import types
from pathlib import Path


def _install_can_stub() -> None:
    try:
        __import__("can")
        return
    except ModuleNotFoundError:
        pass

    can = types.ModuleType("can")

    class CanError(Exception):
        pass

    class Message:
        def __init__(
            self,
            arbitration_id=0,
            data=b"",
            is_extended_id=False,
            is_remote_frame=False,
            timestamp=0.0,
            dlc=None,
            **_kwargs,
        ):
            self.arbitration_id = int(arbitration_id)
            self.data = bytes(data)
            self.is_extended_id = bool(is_extended_id)
            self.is_remote_frame = bool(is_remote_frame)
            self.timestamp = float(timestamp)
            self.dlc = len(self.data) if dlc is None else int(dlc)

    def _no_real_bus(*_args, **_kwargs):
        raise RuntimeError("python-can stub cannot open a real bus")

    can.CanError = CanError
    can.Message = Message
    can.interface = types.SimpleNamespace(Bus=_no_real_bus)
    sys.modules["can"] = can


def _install_fastapi_stub() -> None:
    try:
        __import__("fastapi")
        return
    except ModuleNotFoundError:
        pass

    fastapi = types.ModuleType("fastapi")
    staticfiles = types.ModuleType("fastapi.staticfiles")
    testclient = types.ModuleType("fastapi.testclient")

    class WebSocketDisconnect(Exception):
        pass

    class WebSocket:
        pass

    class StaticFiles:
        def __init__(self, directory, html=False):
            self.directory = str(directory)
            self.html = bool(html)

        async def get_response(self, path, scope):  # pragma: no cover - API shape only
            raise NotImplementedError

    class FastAPI:
        def __init__(self, **_kwargs):
            self._events = {"startup": [], "shutdown": []}
            self._routes = {"GET": {}, "POST": {}}
            self._websockets = {}
            self._mounts = []

        def on_event(self, event):
            def decorate(func):
                self._events[event].append(func)
                return func
            return decorate

        def _route(self, method, path):
            def decorate(func):
                self._routes[method][path] = func
                return func
            return decorate

        def get(self, path):
            return self._route("GET", path)

        def post(self, path):
            return self._route("POST", path)

        def websocket(self, path):
            def decorate(func):
                self._websockets[path] = func
                return func
            return decorate

        def mount(self, path, app, name=None):
            self._mounts.append((path, app, name))

    class _Response:
        def __init__(self, payload=None, *, text="", status_code=200):
            self._payload = payload
            self.text = text
            self.status_code = status_code

        def json(self):
            return self._payload

    class _WebSocketBridge:
        def __init__(self):
            self.messages = queue.Queue()
            self.closed = threading.Event()

        async def accept(self):
            return None

        async def send_json(self, payload):
            if self.closed.is_set():
                raise WebSocketDisconnect
            self.messages.put(payload)

    class _WebSocketSession:
        def __init__(self, endpoint):
            self._endpoint = endpoint
            self._bridge = _WebSocketBridge()
            self._error = None
            self._thread = None
            self._worker = next(
                (
                    cell.cell_contents
                    for cell in (endpoint.__closure__ or ())
                    if hasattr(cell.cell_contents, "latest")
                ),
                None,
            )

        def __enter__(self):
            def run():
                try:
                    asyncio.run(self._endpoint(self._bridge))
                except BaseException as exc:  # surfaced by receive_json/exit
                    self._error = exc

            self._thread = threading.Thread(target=run, daemon=True)
            self._thread.start()
            return self

        def receive_json(self):
            try:
                return self._bridge.messages.get(timeout=0.2)
            except queue.Empty:
                if self._error is not None:
                    raise self._error
                deadline = time.monotonic() + 1.8
                while self._worker is not None and time.monotonic() < deadline:
                    sample = self._worker.latest()
                    if sample is not None:
                        return sample
                    time.sleep(0.01)
                raise

        def __exit__(self, *_exc):
            self._bridge.closed.set()
            if self._thread is not None:
                self._thread.join(timeout=1.0)
            if self._error is not None and not isinstance(
                self._error, WebSocketDisconnect
            ):
                raise self._error

    class TestClient:
        __test__ = False

        def __init__(self, app):
            self.app = app

        def __enter__(self):
            for func in self.app._events["startup"]:
                func()
            return self

        def __exit__(self, *_exc):
            for func in reversed(self.app._events["shutdown"]):
                func()

        def get(self, path):
            endpoint = self.app._routes["GET"].get(path)
            if endpoint is not None:
                return _Response(endpoint())
            for mount_path, static, _name in self.app._mounts:
                if not path.startswith(mount_path):
                    continue
                rel = path[len(mount_path):].lstrip("/")
                if not rel and static.html:
                    rel = "index.html"
                target = Path(static.directory) / rel
                if target.is_file():
                    return _Response(text=target.read_text(encoding="utf-8"))
            return _Response(status_code=404)

        def post(self, path, json=None):
            endpoint = self.app._routes["POST"][path]
            if inspect.signature(endpoint).parameters:
                return _Response(endpoint(json))
            return _Response(endpoint())

        def websocket_connect(self, path):
            return _WebSocketSession(self.app._websockets[path])

    fastapi.FastAPI = FastAPI
    fastapi.WebSocket = WebSocket
    fastapi.WebSocketDisconnect = WebSocketDisconnect
    staticfiles.StaticFiles = StaticFiles
    testclient.TestClient = TestClient
    sys.modules["fastapi"] = fastapi
    sys.modules["fastapi.staticfiles"] = staticfiles
    sys.modules["fastapi.testclient"] = testclient


_install_can_stub()
_install_fastapi_stub()
