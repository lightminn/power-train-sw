# L1 지도(map_seq) 시험: 정답을 아는 합성 기록(synth_seq)으로 깨끗한 기준 → 실패 주입 → 복귀를 확인한다. open3d 없으면 skip.
import csv, json, math, os, shutil, sys

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
pytest.importorskip("open3d")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import seq_format as sf  # noqa: E402
import synth_seq as ss  # noqa: E402
import map_seq as ms  # noqa: E402


def make(folder, n=16, poses=None, frame_fn=None):
    """합성 기록. frame_fn(i, T) → (depth, ir)로 특정 프레임을 바꿔 넣을 수 있다."""
    poses = poses or ss.trajectory(n)
    meta = {"format": sf.FORMAT, "format_version": sf.FORMAT_VERSION, "device": {"product_line": "L500"},
            "depth_scale_m": ss.SCALE, "depth_intrinsics": ss.K, "infrared_intrinsics": ss.K, "color_intrinsics": None,
            "streams": {"depth": {"w": 640}, "ir": {"w": 640}, "color": None}, "imu": {"available": False}}
    os.makedirs(folder, exist_ok=True)
    w = sf.SeqWriter(folder, ("depth", "ir"), workers=4, queue_size=len(poses) + 2)
    for i, T in enumerate(poses):
        d, ir = frame_fn(i, T) if frame_fn else ss.render(T)
        ts = f"{1000 + i * 66.67:.3f}"
        w.submit(i, {"depth": d, "ir": ir}, {"host_mono_s": f"{i / 15:.6f}", "depth_ts_ms": ts, "depth_ts_domain": "syn", "depth_fn": 2 * i,
                                           "ir_ts_ms": ts, "ir_ts_domain": "syn", "ir_fn": 2 * i})
    st = w.close()
    meta["stats"] = {"saved": st["saved"], "error": None, "write_errors": 0}
    sf.write_json_atomic(os.path.join(folder, "meta.json"), meta)
    return poses


def err(Ta, Tb):
    D = np.linalg.inv(Ta) @ Tb
    return np.linalg.norm(D[:3, 3]), math.degrees(math.acos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))


def rel(poses, est, i, ref=0):
    return err(np.linalg.inv(poses[ref]) @ poses[i], np.linalg.inv(est[ref]) @ est[i])


def states(out):
    return {int(r["idx"]): r for r in csv.DictReader(open(os.path.join(out, "poses.csv")))}


@pytest.fixture(scope="module")
def clean(tmp_path_factory):
    d = tmp_path_factory.mktemp("clean")
    seq, out = str(d / "seq"), str(d / "map")
    poses = make(seq, 16)
    return poses, ms.run(seq, out), out


def test_clean_baseline_tracks_all_and_box_height(clean):
    poses, s, out = clean
    assert s["tracked"] == 16 and s["lost"] == 0 and s["alignment_valid"] and s["end_frame_tracked"]
    seg = s["segments"][0]
    assert abs(seg["camera_height_m"] - 0.46) < 0.005 and abs(seg["tilt_deg"] - 20.0) < 0.5 and seg["below_frac"] < 0.01
    est = np.load(os.path.join(out, "poses_T_world_cam.npy"))
    worst = max(rel(poses, est, i) for i in range(16))
    assert worst[0] < 0.01 and worst[1] < 0.3, worst
    import open3d as o3d
    P = np.asarray(o3d.io.read_point_cloud(os.path.join(out, "map_points.ply")).points)
    x0, x1, y0, y1, h = ss.BOXES[0]  # 10 cm 상자
    m = (P[:, 0] > x0 + 0.03) & (P[:, 0] < x1 - 0.03) & (P[:, 1] > y0 + 0.03) & (P[:, 1] < y1 - 0.03) & (P[:, 2] > 0.02)
    assert m.sum() > 20 and abs(np.median(P[m, 2]) - h) < 0.01


def test_plausible_wrong_frame_rejected_then_recovers(tmp_path, clean):
    """음성 대조: 7번에 0.4 m 옆 자세에서 찍은(그럴듯하지만 틀린) 프레임 → 7번만 잃고 8번부터 복귀, 복귀 뒤 자세 정확.
    궤적은 40프레임(약 0.45 m/s) — 걸음 상한(속도 1 m/s)보다 느린 실제 속도대. 16프레임(1.2 m/s)은 상한 밖이다."""
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    def fn(i, T):
        if i == 7:
            Tw = T.copy()
            Tw[1, 3] += 0.4
            return ss.render(Tw)
        return ss.render(T)
    poses = make(seq, 40, frame_fn=fn)
    s = ms.run(seq, out)
    st = states(out)
    assert st[7]["state"].startswith("TRACKING_LOST") and st[7]["integrated"] == "False"
    assert st[8]["state"].startswith("RECOVERED") and s["lost"] == 1 and s["tracked"] == 39
    est = np.load(os.path.join(out, "poses_T_world_cam.npy"))
    assert np.isnan(est[7]).all()
    worst = max(rel(poses, est, i) for i in range(40) if i != 7)
    assert worst[0] < 0.01 and worst[1] < 0.3, worst


def test_long_loss_opens_new_segment(tmp_path):
    """1 s 넘게(20프레임 ≈ 1.3 s) 깊이가 없으면 손실 → 첫 정상 프레임에서 새 구간(자기 초기 평면), 주 구간은 더 긴 쪽."""
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    def fn(i, T):
        d, ir = ss.render(T)
        if 6 <= i < 26:
            d = np.zeros_like(d)
        return d, ir
    make(seq, 40, frame_fn=fn)
    s = ms.run(seq, out)
    st = states(out)
    assert len(s["segments"]) == 2 and all(x["alignment_valid"] for x in s["segments"]), s["segments"]
    assert st[26]["state"] == "SEGMENT_START" and s["main_segment"] == 1
    assert all(st[i]["state"].startswith(("TRACKING_LOST", "SKIP")) for i in range(6, 26))


def test_upside_down_start_is_unaligned_and_l2_refuses(tmp_path):
    """음성 대조: 카메라를 뒤집으면(roll 180°) 하단이 천장 → 초기 평면 법선이 위가 아님 → 정렬 실패, L2 거부."""
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    flip = np.diag([-1.0, -1.0, 1.0, 1.0])  # optical z축 둘레 180°
    poses = [T @ flip for T in ss.trajectory(6)]
    make(seq, 6, poses=poses)
    s = ms.run(seq, out)
    assert not s["alignment_valid"] and s["frame_id"] == "unaligned" and "위 방향" in s["segments"][0]["reason"]
    assert not os.path.exists(os.path.join(out, "map_points.ply"))
    import obstacles_map as om
    crit = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "obstacle_criteria_v0.json")
    with pytest.raises(SystemExit):
        om.run(out, crit)


def test_odometry_failure_is_not_overridden(tmp_path, monkeypatch):
    """음성 대조: 5번째 정합 호출에서 오도메트리 실패를 주입 → 기하 지표가 좋아도 그 프레임은 TRACKING_LOST:odom_fail."""
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    make(seq, 40, poses=ss.trajectory(40)[:12])   # 실제 속도대(약 0.45 m/s) 궤적의 앞 12프레임
    real, calls = ms._odometry, {"n": 0}
    def fake(*a, **k):
        calls["n"] += 1
        ok, T, info = real(*a, **k)
        return (False, T, info) if calls["n"] == 5 else (ok, T, info)
    monkeypatch.setattr(ms, "_odometry", fake)
    ms.run(seq, out)
    st = states(out)
    assert st[5]["state"] == "TRACKING_LOST:odom_fail" and st[6]["state"].startswith("RECOVERED")


def test_rejects_sequence_without_ir(tmp_path):
    seq = str(tmp_path / "seq")
    make(seq, 3)
    shutil.rmtree(os.path.join(seq, "infrared"))
    m = json.load(open(os.path.join(seq, "meta.json")))
    m["streams"]["ir"] = None
    sf.write_json_atomic(os.path.join(seq, "meta.json"), m)
    with pytest.raises(SystemExit):
        ms.run(seq, str(tmp_path / "map"))


def test_preprocess_applies_range_limits():
    d = np.full((10, 10), 400, np.uint16)          # 0.1 m — 하한 0.25 m 밖
    d[5, 5] = 4000                                   # 1.0 m
    out = ms.preprocess(d, ss.SCALE)
    assert out[0, 0] == 0 and (out > 0).sum() <= 1
