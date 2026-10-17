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
    out = om.run(mapped, CRIT)
    obs = out["obstacles"]
    assert out["criteria_version"] == "v0.1" and out["type"] == "terrain_map"
    want = {0.10: "block", 0.07: "block", 0.30: "block", 0.06: "caution"}
    for x0, x1, y0, y1, h in ss.BOXES:
        o = nearest(obs, (x0 + x1) / 2, (y0 + y1) / 2)
        d = np.hypot(o["centroid_xy_m"][0] - (x0 + x1) / 2, o["centroid_xy_m"][1] - (y0 + y1) / 2)
        assert d < 0.35 and o["level"] == want[h], (h, o)
        if h < 0.2:
            assert abs(o["step_m"] - h) < 0.015, (h, o["step_m"])


def test_no_obstacle_on_open_floor(mapped):
    out = om.run(mapped, CRIT)
    for o in out["obstacles"]:
        x, y = o["centroid_xy_m"]
        near_box = any(x0 - 0.35 < x < x1 + 0.35 and y0 - 0.35 < y < y1 + 0.35 for x0, x1, y0, y1, _ in ss.BOXES)
        near_wall = x > ss.WALLS_X[1] - 0.6 or x < ss.WALLS_X[0] + 0.6 or y > ss.WALLS_Y[1] - 0.6 or y < ss.WALLS_Y[0] + 0.6
        assert near_box or near_wall, o


def test_criteria_file_drives_classification(mapped, tmp_path):
    """음성 대조: 기준값 파일의 턱 기준을 8/12 cm로 바꾸면 6 cm는 통과, 7 cm는 통과, 10 cm는 주의가 돼야 한다."""
    c = json.load(open(CRIT, encoding="utf-8"))
    c["rules"]["step_up_m"].update(caution_from=0.08, block_from=0.12)
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
