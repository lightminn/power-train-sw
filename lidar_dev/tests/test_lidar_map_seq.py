# L1 지도(map_seq) 시험: 정답을 아는 합성 기록(synth_seq)으로 자세·치수 오차를 숫자로 확인. open3d 없으면 skip.
import json, math, os, shutil, sys

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
pytest.importorskip("open3d")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import seq_format as sf  # noqa: E402
import synth_seq as ss  # noqa: E402
import map_seq as ms  # noqa: E402


def make(folder, n=16):
    poses = ss.trajectory(n)
    meta = {"format": sf.FORMAT, "format_version": sf.FORMAT_VERSION, "device": {"product_line": "L500"},
            "depth_scale_m": ss.SCALE, "depth_intrinsics": ss.K, "infrared_intrinsics": ss.K, "color_intrinsics": None,
            "streams": {"depth": {"w": 640}, "ir": {"w": 640}, "color": None}, "imu": {"available": False}}
    os.makedirs(folder, exist_ok=True)
    w = sf.SeqWriter(folder, ("depth", "ir"), workers=4, queue_size=n + 2)
    for i, T in enumerate(poses):
        d, ir = ss.render(T)
        ts = f"{1000 + i * 66.67:.3f}"
        w.submit(i, {"depth": d, "ir": ir}, {"host_mono_s": f"{i / 15:.6f}", "depth_ts_ms": ts, "depth_ts_domain": "syn", "depth_fn": 2 * i,
                                           "ir_ts_ms": ts, "ir_ts_domain": "syn", "ir_fn": 2 * i})
    st = w.close()
    meta["stats"] = {"saved": st["saved"], "error": None, "write_errors": 0}
    sf.write_json_atomic(os.path.join(folder, "meta.json"), meta)
    return poses


def rel_err(poses, est):
    te, re = [], []
    for i, T in enumerate(poses):
        if np.isnan(est[i]).any():
            continue
        D = np.linalg.inv(np.linalg.inv(poses[0]) @ T) @ (np.linalg.inv(est[0]) @ est[i])
        te.append(np.linalg.norm(D[:3, 3]))
        re.append(math.degrees(math.acos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1))))
    return max(te), max(re), len(te)


def test_synthetic_trajectory_and_box_height(tmp_path):
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    poses = make(seq, 16)
    s = ms.run(seq, out)
    assert s["tracked"] == 16 and s["lost"] == 0 and s["gravity"] == "floor_plane"
    assert abs(s["floor"]["camera_height_m"] - 0.46) < 0.005 and abs(s["floor"]["tilt_deg"] - 20.0) < 0.5
    t_max, r_max, n = rel_err(poses, np.load(os.path.join(out, "poses_T_world_cam.npy")))
    assert n == 16 and t_max < 0.01 and r_max < 0.3, (t_max, r_max)
    import open3d as o3d
    P = np.asarray(o3d.io.read_point_cloud(os.path.join(out, "map_points.ply")).points)
    x0, x1, y0, y1, h = ss.BOXES[0]  # 10 cm 상자
    m = (P[:, 0] > x0 + 0.03) & (P[:, 0] < x1 - 0.03) & (P[:, 1] > y0 + 0.03) & (P[:, 1] < y1 - 0.03) & (P[:, 2] > 0.02)
    assert m.sum() > 20 and abs(np.median(P[m, 2]) - h) < 0.01


def test_corrupted_frame_is_marked_lost_not_integrated(tmp_path):
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    make(seq, 12)
    rng = np.random.default_rng(0)  # 7번 프레임을 무작위 깊이로 바꿔 넣는다(음성 대조)
    cv2.imwrite(os.path.join(seq, "depth", "000007.png"), rng.integers(1000, 16000, (480, 640), dtype=np.uint16))
    s = ms.run(seq, out)
    import csv
    rows = {int(r["idx"]): r for r in csv.DictReader(open(os.path.join(out, "poses.csv")))}
    assert rows[7]["state"].startswith("TRACKING_LOST") and s["lost"] >= 1
    assert np.isnan(np.load(os.path.join(out, "poses_T_world_cam.npy"))[7]).all()


def test_rejects_sequence_without_ir(tmp_path):
    seq = str(tmp_path / "seq")
    make(seq, 3)
    shutil.rmtree(os.path.join(seq, "infrared"))
    m = json.load(open(os.path.join(seq, "meta.json")))
    m["streams"]["ir"] = None
    sf.write_json_atomic(os.path.join(seq, "meta.json"), m)
    with pytest.raises(SystemExit):
        ms.run(seq, str(tmp_path / "map"))
