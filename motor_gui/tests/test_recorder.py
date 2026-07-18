import csv
import struct
import time

import can

from motor_gui.backend.transport.fake import FakeTransport
from motor_gui.backend.transport.odrive_can_device import (
    C_GET_ENC_EST,
    NODE_ID,
    OdriveCanDevice,
)
from motor_gui.backend.worker import HardwareWorker
from motor_gui.backend.recorder import Recorder


def test_records_csv_rows(tmp_path):
    w = HardwareWorker(FakeTransport(), rate_hz=200)
    w.start()
    rec = Recorder(w)
    path = str(tmp_path / "log.csv")
    try:
        assert rec.start(path, "csv")["ok"] is True
        time.sleep(0.3)
        rec.stop()
    finally:
        w.stop()
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0][0] == "t_mono"          # 헤더
    assert len(rows) > 5                    # 데이터 행


def test_stop_without_samples_does_not_hang(tmp_path):
    # worker 미기동 → 샘플 0개. start 후 즉시 stop 해도 좀비/행 없이 빠르게 종료.
    w = HardwareWorker(FakeTransport(), rate_hz=200)
    rec = Recorder(w)
    path = str(tmp_path / "empty.csv")
    assert rec.start(path, "csv")["ok"] is True
    t0 = time.monotonic()
    res = rec.stop()
    assert res["ok"] is True
    assert time.monotonic() - t0 < 1.5     # 무한 블록 아님


def test_unsupported_fmt_rejected(tmp_path):
    w = HardwareWorker(FakeTransport(), rate_hz=200)
    rec = Recorder(w)
    res = rec.start(str(tmp_path / "x.bin"), "bin")
    assert res["ok"] is False
    assert "unsupported" in res["detail"]


def test_records_converted_wheel_velocity(tmp_path):
    class ManualWorker:
        def subscribe(self, callback):
            self.callback = callback

        def unsubscribe(self, callback):
            assert callback == self.callback

    worker = ManualWorker()
    recorder = Recorder(worker)
    path = str(tmp_path / "wheel.csv")
    device = OdriveCanDevice(gear_ratio=5.0)
    device.on_rx(can.Message(
        arbitration_id=(NODE_ID << 5) | C_GET_ENC_EST,
        data=struct.pack("<ff", 0.0, 10.0),
        is_extended_id=False,
    ))

    recorder.start(path, "csv")
    worker.callback({"t_mono": 1.0, **device.sample()})
    time.sleep(0.05)
    recorder.stop()

    with open(path, newline="") as f:
        row = next(csv.DictReader(f))
    assert float(row["odrive.vel"]) == 2.0
