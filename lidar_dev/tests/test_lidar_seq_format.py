# l515-seq v1 기록 형식 시험(장치 없이). 실행: python -m pytest lidar_dev/tests -q  (numpy·opencv 필요, terrain 호환 시험은 open3d 필요)
import csv, json, os, stat, sys

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import seq_format as sf  # noqa: E402

W, H = 640, 480
K = dict(width=W, height=H, fx=464.5, fy=464.2, ppx=351.9, ppy=245.5, model="none", coeffs=[0.0] * 5)
CK = dict(width=1280, height=720, fx=900.0, fy=900.0, ppx=640.0, ppy=360.0, model="none", coeffs=[0.0] * 5)


def floor_depth(h=0.45, pitch_deg=20.0, k=K, scale=sf.L515_DEPTH_SCALE):
    """카메라 높이 h, 아래로 pitch만큼 숙인 카메라가 본 평평한 바닥의 깊이(uint16)."""
    u, v = np.meshgrid(np.arange(k["width"]), np.arange(k["height"]))
    dy = (v - k["ppy"]) / k["fy"]
    th = np.radians(pitch_deg)
    den = dy * np.cos(th) + np.sin(th)  # 광선의 아래 방향 성분(z = 1 기준)
    z = np.where(den > 1e-3, h / np.maximum(den, 1e-3), 0.0)
    z[(z > 6.0) | (z < 0.25)] = 0
    return np.round(z / scale).astype(np.uint16)


def make_seq(folder, n=10, streams=("depth", "ir", "color"), depth=None, exts=None):
    meta = {"format": sf.FORMAT, "format_version": sf.FORMAT_VERSION, "depth_scale_m": sf.L515_DEPTH_SCALE,
            "depth_intrinsics": K, "color_intrinsics": CK if "color" in streams else None,
            "streams": {s: ({"w": 1} if s in streams else None) for s in sf.STREAMS},
            "device": {"product_line": "L500"}, "imu": {"available": False},
            "stats": {"saved": n, "error": None, "write_errors": 0}, "file_ext": exts or {}}
    os.makedirs(folder, exist_ok=True)
    sf.write_json_atomic(os.path.join(folder, "meta.json"), meta)
    w = sf.SeqWriter(folder, streams, workers=3, queue_size=n + 2, exts=exts)
    d = floor_depth() if depth is None else depth
    for i in range(n):
        imgs = {"depth": d.copy()}
        if "ir" in streams:
            imgs["ir"] = np.full((H, W), 100 + i, np.uint8)
        if "color" in streams:
            imgs["color"] = np.full((720, 1280, 3), i, np.uint8)
        if "depth_aligned" in streams:
            imgs["depth_aligned"] = np.full((720, 1280), 4000 + i, np.uint16)
        row = {"host_mono_s": f"{i * 0.0667:.6f}", "host_wall": "x"}
        for s in sf.STREAM_TS:
            if s in streams:
                row.update({f"{s}_ts_ms": f"{1000 + i * 66.67 + (10 if s == 'color' else 0):.3f}",
                            f"{s}_ts_domain": "global_time", f"{s}_fn": 30 + 2 * i})
        assert w.submit(30 + 2 * i, imgs, row)
    st = w.close()
    return meta, st


def test_roundtrip_passes_check(tmp_path):
    f = str(tmp_path / "seq")
    _, st = make_seq(f, n=10)
    assert st["saved"] == 10 and st["error"] is None
    E, Wn, S = sf.check_sequence(f)
    assert E == [], E
    assert S["frames"] == 10
    assert S["depth_ir_skew_ms"]["max"] == 0.0 and abs(S["depth_color_skew_ms"]["median"] - 10) < 1e-6
    rows = list(csv.DictReader(open(os.path.join(f, "frames.csv"))))
    assert [int(r["save_idx"]) for r in rows] == list(range(10))
    a = cv2.imread(os.path.join(f, "depth", "000003.png"), cv2.IMREAD_UNCHANGED)
    assert a.dtype == np.uint16 and a.shape == (H, W) and np.array_equal(a, floor_depth())


def test_depth_only(tmp_path):
    f = str(tmp_path / "seq")
    make_seq(f, n=4, streams=("depth",))
    E, _, _ = sf.check_sequence(f)
    assert E == [], E


def test_extrinsics_col_major_to_row_major():
    th = np.radians(30.0)
    R = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]]) @ \
        np.array([[1, 0, 0], [0, np.cos(0.2), -np.sin(0.2)], [0, np.sin(0.2), np.cos(0.2)]])
    t = [0.015, -0.002, 0.004]
    d = sf.extrinsics_dict(R.reshape(-1, order="F").tolist(), t)  # SDK는 열 우선
    assert np.allclose(np.asarray(d["R_row_major"]).reshape(3, 3), R)
    p = np.array([0.3, -0.1, 1.7])
    T = sf.extrinsics_matrix(d)
    assert np.allclose((T @ np.r_[p, 1])[:3], R @ p + t)
    # 열 우선 배열을 행 우선으로 잘못 읽으면 전치 → 다른 점이 나와야 한다(대조)
    assert not np.allclose(np.asarray(d["R_sdk_col_major"]).reshape(3, 3) @ p, R @ p)


def test_legacy_terrain_reads_recording(tmp_path):
    """인수받은 terrain.py가 새 기록을 읽어 합성 바닥에서 OK와 맞는 카메라 높이·기울기를 낸다."""
    pytest.importorskip("open3d")
    import terrain
    f = str(tmp_path / "seq")
    make_seq(f, n=3, streams=("depth",), depth=floor_depth(h=0.45, pitch_deg=20.0))
    det = terrain.load_detector(f)
    r = det.process(cv2.imread(os.path.join(f, "depth", "000000.png"), cv2.IMREAD_UNCHANGED))
    assert r["status"] == "OK", r
    assert abs(r["camera_height_m"] - 0.45) < 0.01, r
    assert abs(r["camera_tilt_deg"] - 20.0) < 1.0, r


# ---- 음성 대조: 알려진 결함을 넣으면 check_sequence가 실패해야 한다

def _broken(tmp_path, n=6):
    f = str(tmp_path / "seq")
    make_seq(f, n=n)
    return f


def test_neg_missing_file(tmp_path):
    f = _broken(tmp_path)
    os.remove(os.path.join(f, "depth", "000002.png"))
    E, _, _ = sf.check_sequence(f)
    assert any("depth/" in e for e in E), E


def test_neg_wrong_dtype(tmp_path):
    f = _broken(tmp_path)
    cv2.imwrite(os.path.join(f, "depth", "000000.png"), np.zeros((H, W), np.uint8))
    E, _, _ = sf.check_sequence(f)
    assert any("dtype" in e for e in E), E


def test_neg_wrong_size(tmp_path):
    f = _broken(tmp_path)
    cv2.imwrite(os.path.join(f, "depth", "000000.png"), np.zeros((240, 320), np.uint16))
    E, _, _ = sf.check_sequence(f)
    assert any("크기" in e for e in E), E


def test_neg_bad_scale(tmp_path):
    f = _broken(tmp_path)
    m = json.load(open(os.path.join(f, "meta.json")))
    m["depth_scale_m"] = 0
    sf.write_json_atomic(os.path.join(f, "meta.json"), m)
    E, _, _ = sf.check_sequence(f)
    assert any("depth_scale_m" in e for e in E), E


def test_neg_timestamp_backwards(tmp_path):
    f = _broken(tmp_path)
    p = os.path.join(f, "frames.csv")
    rows = list(csv.DictReader(open(p)))
    rows[3]["depth_ts_ms"], rows[4]["depth_ts_ms"] = rows[4]["depth_ts_ms"], rows[3]["depth_ts_ms"]
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, sf.FRAME_COLS)
        w.writeheader()
        w.writerows(rows)
    E, _, _ = sf.check_sequence(f)
    assert any("depth 타임스탬프 역행" in e for e in E), E


def test_neg_leftover_tmp(tmp_path):
    f = _broken(tmp_path)
    open(os.path.join(f, "depth", ".000006.tmp.png"), "wb").close()
    E, _, _ = sf.check_sequence(f)
    assert any("임시 파일" in e for e in E), E


@pytest.mark.skipif(hasattr(os, "geteuid") and os.geteuid() == 0, reason="root는 쓰기 금지를 무시한다")
def test_neg_write_failure_stops(tmp_path):
    f = str(tmp_path / "seq")
    w = sf.SeqWriter(f, ("depth",), workers=1, queue_size=4)
    os.chmod(os.path.join(f, "depth"), stat.S_IRUSR | stat.S_IXUSR)  # 쓰기 금지
    try:
        assert w.submit(0, {"depth": floor_depth()}, {"host_mono_s": "0"})
        w.q.join()
        assert w.error and "저장 실패" in w.error
        assert not w.submit(1, {"depth": floor_depth()}, {"host_mono_s": "0.1"})  # 오류 뒤에는 거부
        st = w.close()
        assert st["saved"] == 0 and st["write_errors"] == 1
    finally:
        os.chmod(os.path.join(f, "depth"), stat.S_IRWXU)


def test_queue_full_drops_without_blocking(tmp_path):
    f = str(tmp_path / "seq")
    w = sf.SeqWriter(f, ("depth",), workers=0, queue_size=2)  # 저장 스레드 없음 → 큐가 바로 참
    ok = [w.submit(i, {"depth": floor_depth()}, {"host_mono_s": str(i)}) for i in range(5)]
    assert ok == [True, True, False, False, False] and w.dropped_queue == 3
    for _ in range(2):  # 저장 스레드가 없으니 큐를 직접 비우고 닫는다(수집 로그 스레드가 다 쓴 뒤 닫힘)
        w.q.get_nowait()
        w.q.task_done()
    w.close()
    log = list(csv.DictReader(open(os.path.join(f, "capture_log.csv"))))
    assert [r["status"] for r in log] == ["queued", "queued", "drop_queue", "drop_queue", "drop_queue"]


def _edit_meta(f, fn):
    m = json.load(open(os.path.join(f, "meta.json")))
    fn(m)
    sf.write_json_atomic(os.path.join(f, "meta.json"), m)


def test_neg_empty_recording(tmp_path):
    f = str(tmp_path / "seq")
    make_seq(f, n=0)
    E, _, _ = sf.check_sequence(f)
    assert any("0개" in e for e in E), E


def test_neg_error_stats(tmp_path):
    f = _broken(tmp_path)
    _edit_meta(f, lambda m: m["stats"].update(error="저장 실패(save_idx 3): disk full", write_errors=1))
    E, _, _ = sf.check_sequence(f)
    assert any("기록 중 오류" in e for e in E), E


def test_neg_missing_stats(tmp_path):
    f = _broken(tmp_path)
    _edit_meta(f, lambda m: m.update(stats=None))
    E, _, _ = sf.check_sequence(f)
    assert any("정상 종료되지 않음" in e for e in E), E


def test_neg_saved_mismatch(tmp_path):
    f = _broken(tmp_path)
    _edit_meta(f, lambda m: m["stats"].update(saved=99))
    E, _, _ = sf.check_sequence(f)
    assert any("stats.saved" in e for e in E), E


def test_neg_l515_wrong_scale(tmp_path):
    f = _broken(tmp_path)
    _edit_meta(f, lambda m: m.update(depth_scale_m=0.001))  # D400 계열 단위로 잘못 기록된 경우
    E, _, _ = sf.check_sequence(f)
    assert any("0.00025" in e for e in E), E


def test_neg_missing_timestamp(tmp_path):
    f = _broken(tmp_path)
    p = os.path.join(f, "frames.csv")
    rows = list(csv.DictReader(open(p)))
    rows[2]["ir_ts_ms"] = ""
    rows[4]["color_fn"] = "nan"
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, sf.FRAME_COLS)
        w.writeheader()
        w.writerows(rows)
    E, _, _ = sf.check_sequence(f)
    assert any(e.startswith("ir 타임스탬프") for e in E) and any(e.startswith("color 타임스탬프") for e in E), E


def test_imu_sorted_numerically(tmp_path):
    f = str(tmp_path / "seq")
    w = sf.SeqWriter(f, ("depth",), workers=1, queue_size=2)
    for t in ("100000.000000", "99999.990000", "99999.995000"):  # 문자열 정렬이면 100000이 먼저 온다
        w.add_imu({"host_mono_s": t, "kind": "accel", "ts_ms": t, "ts_domain": "global_time", "fn": 1, "x": 0, "y": 0, "z": 9.8})
    w.close()
    got = [r["host_mono_s"] for r in csv.DictReader(open(os.path.join(f, "imu.csv")))]
    assert got == ["99999.990000", "99999.995000", "100000.000000"]


def test_aligned_depth_and_png_color(tmp_path):
    """--align-to-color·--color-png(10-20): 컬러 해상도 정렬 깊이와 무손실 컬러가 점검을 통과하고 값이 그대로 읽힌다."""
    f = str(tmp_path / "seq")
    make_seq(f, n=5, streams=("depth", "ir", "color", "depth_aligned"), exts={"color": ".png"})
    E, Wn, S = sf.check_sequence(f)
    assert E == [], E
    a = cv2.imread(os.path.join(f, "depth_aligned", "000002.png"), cv2.IMREAD_UNCHANGED)
    assert a.dtype == np.uint16 and a.shape == (720, 1280) and int(a[0, 0]) == 4002
    c = cv2.imread(os.path.join(f, "color", "000004.png"), cv2.IMREAD_UNCHANGED)
    assert c.shape == (720, 1280, 3) and int(c[0, 0, 0]) == 4   # 무손실


def test_neg_aligned_without_color_intrinsics(tmp_path):
    f = str(tmp_path / "seq")
    make_seq(f, n=3, streams=("depth", "depth_aligned"))
    E, _, _ = sf.check_sequence(f)
    assert any("color_intrinsics" in e for e in E), E
