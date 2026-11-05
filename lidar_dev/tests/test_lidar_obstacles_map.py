# L2 장애물 분류(obstacles_map) 시험: 합성 기록 → L1 지도 → 기준값 JSON 분류. open3d 없으면 skip.
import json, os, sys

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("open3d")
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import synth_seq as ss  # noqa: E402
import map_seq as ms  # noqa: E402
import obstacles_map as om  # noqa: E402
from test_lidar_map_seq import make  # noqa: E402

CRIT = os.path.join(HERE, "config", "obstacle_criteria_v0.json")


@pytest.fixture(scope="module")
def mapped(tmp_path_factory):
    d = tmp_path_factory.mktemp("l2")
    seq, out = str(d / "seq"), str(d / "map")
    make(seq, 40)
    ms.run(seq, out)
    return out


def nearest(obs, x, y):
    return min(obs, key=lambda o: (o["centroid_xy_m"][0] - x) ** 2 + (o["centroid_xy_m"][1] - y) ** 2)


def test_boxes_classified_by_criteria(mapped):
    """상자 6·7·10·30 cm. 6 cm는 측정 여유 1 cm로 기준 7 cm 경계에 걸려 주의·불가 둘 다 허용.
    단차 값은 윗면까지 보이는 가까운 7·10 cm 상자만 확인한다(3 m의 6 cm 상자는 윗면이 안 보여 앞면 높이만 잰다 — 관측된 만큼만 보고)."""
    out = om.run(mapped, CRIT)
    obs = out["obstacles"]
    assert out["criteria_version"] == "v0.1" and out["type"] == "terrain_map"
    want = {0.10: {"block"}, 0.07: {"block"}, 0.30: {"block"}, 0.06: {"caution", "block"}}
    for x0, x1, y0, y1, h in ss.BOXES:
        o = nearest(obs, (x0 + x1) / 2, (y0 + y1) / 2)
        d = np.hypot(o["centroid_xy_m"][0] - (x0 + x1) / 2, o["centroid_xy_m"][1] - (y0 + y1) / 2)
        assert d < 0.35 and o["level"] in want[h], (h, o)
        if h in (0.07, 0.10):
            assert abs(o["step_m"] - h) < 0.015, (h, o["step_m"])


def test_box_footprints_never_pass(mapped):
    """경로 허가 층에서 상자 안 칸은 하나도 통과가 아니다(소수 누출도 실패)."""
    om.run(mapped, CRIT)
    g = np.load(os.path.join(mapped, "terrain_grid.npz"))
    lv, x0, y0, c = g["level"], float(g["x0"]), float(g["y0"]), float(g["cell"])
    gx, gy = x0 + (np.arange(lv.shape[1]) + 0.5) * c, y0 + (np.arange(lv.shape[0]) + 0.5) * c
    for bx0, bx1, by0, by1, h in ss.BOXES:
        sub = lv[np.ix_((gy > by0) & (gy < by1), (gx > bx0) & (gx < bx1))]
        assert not (sub == om.LEVEL["pass"]).any(), (h, np.unique(sub, return_counts=True))


def test_no_obstacle_on_open_floor(mapped):
    out = om.run(mapped, CRIT)
    for o in out["obstacles"]:
        x, y = o["centroid_xy_m"]
        near_box = any(x0 - 0.35 < x < x1 + 0.35 and y0 - 0.35 < y < y1 + 0.35 for x0, x1, y0, y1, _ in ss.BOXES)
        near_wall = x > ss.WALLS_X[1] - 0.6 or x < ss.WALLS_X[0] + 0.6 or y > ss.WALLS_Y[1] - 0.6 or y < ss.WALLS_Y[0] + 0.6
        assert near_box or near_wall, o


def test_criteria_file_drives_classification(mapped, tmp_path):
    """음성 대조: 기준값 파일의 턱 기준을 9/13 cm로 바꾸면 6 cm는 통과, 7 cm는 통과, 10 cm는 주의가 돼야 한다(측정 여유 1 cm 포함)."""
    c = json.load(open(CRIT, encoding="utf-8"))
    c["rules"]["step_up_m"].update(caution_from=0.09, block_from=0.13)
    p = tmp_path / "crit.json"
    p.write_text(json.dumps(c), encoding="utf-8")
    obs = om.run(mapped, str(p))["obstacles"]
    for x0, x1, y0, y1, h in ss.BOXES:
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        near = [o for o in obs if np.hypot(o["centroid_xy_m"][0] - cx, o["centroid_xy_m"][1] - cy) < 0.35]
        if h in (0.06, 0.07):
            assert not near, (h, near)
        if h == 0.10:
            assert near and near[0]["level"] == "caution", (h, near)


def test_profile_flows_to_map_and_eval_rejects_mismatch(mapped):
    """v0.2 사슬: L1 지도 → L2(--profile B) → terrain_map.json에 criteria_profile=B, 같은 프로필 평가는 받고 A 평가는 거부(Codex 9차 #9)."""
    import eval_obstacles as ev
    from criteria import load_criteria
    v2 = os.path.join(HERE, "config", "obstacle_criteria_v0.2.json")
    out = om.run(mapped, v2, profile="B_stop20")
    assert out["criteria_version"] == "v0.2" and out["criteria_profile"] == "B_stop20"
    tm = json.load(open(os.path.join(mapped, "terrain_map.json"), encoding="utf-8"))
    assert tm["criteria_profile"] == "B_stop20"
    ev.Map(mapped, load_criteria(v2, "B_stop20"))
    with pytest.raises(SystemExit):
        ev.Map(mapped, load_criteria(v2))
    om.run(mapped, v2)                                    # 기본(A)으로 되돌려 다른 시험과 섞이지 않게
