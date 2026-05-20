import csv
import time

from motor_gui.backend.transport.fake import FakeTransport
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
