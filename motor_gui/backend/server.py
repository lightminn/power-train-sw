from __future__ import annotations

import argparse
import asyncio
import contextlib
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles


class _NoCacheStatic(StaticFiles):
    async def get_response(self, path, scope):
        resp = await super().get_response(path, scope)
        resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        return resp

from .worker import HardwareWorker
from .recorder import Recorder

_FRONTEND = Path(__file__).resolve().parents[1] / "frontend"


def _make_transport(track: str):
    if track == "fake":
        from .transport.fake import FakeTransport
        return FakeTransport()
    if track == "usb":
        from .transport.usb_odrive import UsbOdriveBackend
        return UsbOdriveBackend()
    if track == "ak":
        from .transport.can_device import CanTransport
        from .transport.ak_device import AkDevice
        return CanTransport([AkDevice()], track="ak")
    if track == "odrive_can":
        from .transport.can_device import CanTransport
        from .transport.odrive_can_device import OdriveCanDevice
        return CanTransport([OdriveCanDevice()], track="can")
    if track == "can":
        from .transport.can_bus import CanBackend
        return CanBackend()
    raise ValueError(f"unknown track: {track!r}")


def create_app(track: str = "fake") -> FastAPI:
    app = FastAPI(title="motor_gui", version="0.1")
    worker = HardwareWorker(_make_transport(track))
    recorder = Recorder(worker)

    @app.on_event("startup")
    def _startup() -> None:
        worker.start()

    @app.on_event("shutdown")
    def _shutdown() -> None:
        recorder.stop()
        worker.stop()

    @app.get("/api/capabilities")
    def capabilities() -> dict:
        return worker.capabilities()

    @app.get("/api/tunables")
    def tunables() -> dict:
        return worker.tunables()

    @app.post("/api/command")
    def command(envelope: dict) -> dict:
        if envelope.get("op") == "estop":
            target = envelope.get("target")
            caps = worker.capabilities()
            if target not in caps.get("commands", {}):
                return {"ok": False, "target": target,
                        "op": "estop", "detail": f"unknown target: {target!r}"}
            worker.estop()
            return {"ok": True, "target": target,
                    "op": "estop", "detail": "estop latched"}
        return worker.submit(envelope)

    @app.post("/api/reconnect")
    def reconnect() -> dict:
        return worker.reconnect()

    @app.post("/api/record/start")
    def record_start(body: dict) -> dict:
        path = body.get("path")
        if not path:
            from pathlib import Path as _P
            import time as _t
            logdir = _P(__file__).resolve().parents[2] / "logs"
            path = str(logdir / f"motor_{track}_{_t.strftime('%Y%m%d_%H%M%S')}.csv")
        return recorder.start(path, body.get("fmt", "csv"))

    @app.post("/api/record/stop")
    def record_stop() -> dict:
        return recorder.stop()

    @app.websocket("/ws/telemetry")
    async def telemetry(ws: WebSocket) -> None:
        await ws.accept()
        loop = asyncio.get_running_loop()
        q: asyncio.Queue = asyncio.Queue(maxsize=10)

        def _on_sample(s: dict) -> None:
            def _put() -> None:
                if q.full():
                    with contextlib.suppress(asyncio.QueueEmpty):
                        q.get_nowait()
                q.put_nowait(s)
            loop.call_soon_threadsafe(_put)

        worker.subscribe(_on_sample)
        try:
            while True:
                s = await q.get()
                await ws.send_json(s)
        except WebSocketDisconnect:
            pass
        finally:
            worker.unsubscribe(_on_sample)

    if _FRONTEND.exists():
        app.mount("/", _NoCacheStatic(directory=str(_FRONTEND), html=True),
                  name="frontend")
    return app


def main() -> None:
    import uvicorn
    p = argparse.ArgumentParser(description="motor_gui backend")
    p.add_argument("--track", choices=["fake", "usb", "can", "ak", "odrive_can"], default="fake")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args()
    uvicorn.run(create_app(track=args.track), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
