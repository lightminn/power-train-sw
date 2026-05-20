from __future__ import annotations

import csv
import logging
import queue
import threading
from pathlib import Path

_log = logging.getLogger(__name__)
_Q_MAXSIZE = 10_000   # 100 Hz × 100 s 버퍼


class Recorder:
    """worker 의 sample 스트림을 tap 해서 CSV/parquet 로 기록. 토글식 (기본 off)."""

    def __init__(self, worker) -> None:
        self._worker = worker
        self._q: queue.Queue | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._path: str | None = None
        self._fmt = "csv"

    def start(self, path: str, fmt: str = "csv") -> dict:
        if self._running.is_set():
            return {"ok": False, "detail": "already recording"}
        if fmt not in ("csv", "parquet"):
            return {"ok": False, "detail": f"unsupported fmt: {fmt}"}
        self._path, self._fmt = path, fmt
        self._q = queue.Queue(maxsize=_Q_MAXSIZE)
        self._worker.subscribe(self._on_sample)
        self._running.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return {"ok": True, "detail": f"recording → {path} ({fmt})"}

    def stop(self) -> dict:
        if not self._running.is_set():
            return {"ok": True, "detail": "not recording"}
        self._running.clear()
        self._worker.unsubscribe(self._on_sample)
        if self._thread:
            self._thread.join(timeout=2.0)
        return {"ok": True, "detail": f"stopped → {self._path}"}

    def _on_sample(self, s: dict) -> None:
        if self._q is not None:
            try:
                self._q.put_nowait(s)
            except queue.Full:
                pass

    def _run(self) -> None:
        rows: list[dict] = []
        # 첫 샘플 대기 — 샘플 없이 stop 되면 좀비 없이 종료
        first = None
        while self._running.is_set():
            try:
                first = self._q.get(timeout=0.1)
                break
            except queue.Empty:
                continue
        if first is None:
            return
        cols = list(first.keys())
        rows.append(first)
        while self._running.is_set() or not self._q.empty():
            try:
                rows.append(self._q.get(timeout=0.1))
            except queue.Empty:
                continue
        try:
            Path(self._path).parent.mkdir(parents=True, exist_ok=True)
            if self._fmt == "csv":
                with open(self._path, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
                    w.writeheader()
                    for r in rows:
                        w.writerow(r)
            else:  # parquet
                import pyarrow as pa
                import pyarrow.parquet as pq
                table = pa.Table.from_pylist(
                    [{c: r.get(c) for c in cols} for r in rows])
                pq.write_table(table, self._path)
        except Exception as e:
            _log.exception("recorder write failed: %s", e)
