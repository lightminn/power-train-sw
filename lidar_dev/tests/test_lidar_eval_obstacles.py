# L2 평가 도구(eval_obstacles) 시험: 손으로 만든 terrain_map으로 각 지표를 확인하고, 합성 기록 → L1 → L2 → 평가 사슬을 한 번 돈다.
import json, os, sys

import pytest

np = pytest.importorskip("numpy")
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_obstacles as ev  # noqa: E402

CRIT_PATH = os.path.join(HERE, "config", "obstacle_criteria_v0.json")
CRIT = json.load(open(CRIT_PATH, encoding="utf-8"))
CELL, X0, Y0, NX, NY = 0.05, 0.0, -1.0, 60, 40
GX, GY = X0 + (np.arange(NX) + 0.5) * CELL, Y0 + (np.arange(NY) + 0.5) * CELL
BOX = {"id": "box1", "kind": "step_up", "height_m": 0.07, "x_m": 1.5, "y_m": 0.0, "size_m": [0.30, 0.40]}
REGION = {"x_m": [0.4, 2.5], "y_m": [-0.9, 0.9]}


def obstacle(i, x, y, level="block", step=0.072, half=0.15):
    return {"id": i, "level": level, "cells": 48, "centroid_xy_m": [x, y], "step_m": step,
            "poly_xy_m": [[x - half, y - half], [x + half, y - half], [x + half, y + half], [x - half, y + half]]}


def footprint(o):
    sx, sy = o["size_m"]
    return ((GX > o["x_m"] - sx / 2) & (GX < o["x_m"] + sx / 2))[None, :] & ((GY > o["y_m"] - sy / 2) & (GY < o["y_m"] + sy / 2))[:, None]


def mk(d, obstacles, box_level=3, edit=None, aligned=True, version=None):
    """바닥 전부 통과·관측, 상자 발자국은 box_level인 지도 폴더."""
    os.makedirs(d, exist_ok=True)
    lv = np.ones((NY, NX), np.int64)
    lv[footprint(BOX)] = box_level
    g = {"level": lv, "level_local": lv.copy(), "observed": np.ones((NY, NX), bool)}
    if edit:
        edit(g)
    np.savez_compressed(os.path.join(d, "terrain_grid.npz"), x0=X0, y0=Y0, cell=CELL, **g)
    json.dump({"type": "terrain_map", "alignment_valid": aligned, "criteria_version": version or CRIT["criteria_version"],
               "obstacles": obstacles}, open(os.path.join(d, "terrain_map.json"), "w"))
    return str(d)


def scene(tmp_path, obstacles, objects=None, **kw):
    gt = {"scene": "t", "objects": objects or [BOX], "eval_region": REGION}
    return ev.eval_scene(mk(tmp_path / "m", obstacles, **kw), gt, CRIT)


def test_perfect_detection(tmp_path):
    s = scene(tmp_path, [obstacle(0, 1.5, 0.0)])
    r = s["objects"][0]
    assert r["expected"] == "block" and r["got"] == "block" and r["level_ok"] and r["detected"]
    assert r["step_err_m"] == pytest.approx(0.002) and r["footprint_pass_cells"] == 0 and r["footprint_cells"] == 48
    g = s["region"]
    assert g["false_obstacles"] == [] and g["floor_false_risk_cells"] == 0 and g["unknown_frac"] == 0


def test_missed_box_and_footprint_leak(tmp_path):
    """음성 대조: 장애물이 없고 발자국이 통과면 놓침 + 누출 48칸."""
    r = scene(tmp_path, [], box_level=1)["objects"][0]
    assert r["got"] == "pass" and not r["level_ok"] and r["detected"] is False and r["footprint_pass_cells"] == 48


def test_false_obstacle_counted_only_inside_region(tmp_path):
    s = scene(tmp_path, [obstacle(0, 1.5, 0.0), obstacle(1, 1.0, 0.7, "caution", 0.055), obstacle(2, 2.9, 0.95)])
    assert [o["id"] for o in s["region"]["false_obstacles"]] == [1]


def test_pieces_near_box_are_not_false(tmp_path):
    """상자 둘레(+0.15 m)에서 갈라진 조각은 오검출이 아니라 그 상자의 조각."""
    s = scene(tmp_path, [obstacle(0, 1.5, 0.0), obstacle(1, 1.5, 0.30, "caution", 0.03, half=0.05)])
    assert s["region"]["false_obstacles"] == [] and s["objects"][0]["pieces"] == 2 and s["objects"][0]["got"] == "block"


def test_border_height_accepts_neighbor_level(tmp_path):
    """6.5 cm(기준 7 cm에서 1.5 cm 안) → 주의 기대, 불가도 맞음. 4 cm는 경계 밖이라 주의면 과경보."""
    r = scene(tmp_path, [obstacle(0, 1.5, 0.0)], objects=[{**BOX, "height_m": 0.065}])["objects"][0]
    assert r["expected"] == "caution" and r["got"] == "block" and r["level_ok"] and r["border"]
    s = scene(tmp_path, [obstacle(0, 1.5, 0.0, "caution", 0.04)], objects=[{**BOX, "height_m": 0.03}])
    assert s["objects"][0]["expected"] == "pass" and not s["objects"][0]["level_ok"]
    assert ev.summarize([s])["over_warning_on_pass_objects"] == 1


def test_floor_false_risk_and_unknown(tmp_path):
    def edit(g):
        g["level_local"][5:7, 15:18] = 2   # 바닥 x 0.75–0.9, y −0.75–−0.65에 가짜 위험 6칸
        g["level"][:, 45:50] = 0          # x 2.25–2.5 unknown
    g = scene(tmp_path, [obstacle(0, 1.5, 0.0)], edit=edit)["region"]
    assert g["floor_false_risk_cells"] == 6
    assert g["unknown_frac"] == pytest.approx(5 / 42, abs=0.01)   # 구역 x 0.4–2.5 = 42칸 중 5열


def test_repeat_measurement_summary(tmp_path):
    gt = {"scene": "a", "objects": [BOX]}
    s1 = ev.eval_scene(mk(tmp_path / "a", [obstacle(0, 1.5, 0.0, step=0.070)]), gt, CRIT)
    s2 = ev.eval_scene(mk(tmp_path / "b", [obstacle(0, 1.52, 0.03, step=0.074)]), {**gt, "scene": "b"}, CRIT)
    sm = ev.summarize([s1, s2])
    assert sm["repeat_step_m"]["box1"]["n"] == 2 and sm["repeat_step_m"]["box1"]["mean"] == pytest.approx(0.072)
    assert sm["repeat_step_m"]["box1"]["sd"] == pytest.approx(0.0028, abs=1e-4)
    assert sm["detection"] == {"n": 2, "detected": 2} and sm["false_obstacles"]["scenes_with_region"] == 0


def test_refuses_unaligned_or_other_criteria(tmp_path):
    with pytest.raises(SystemExit):
        ev.eval_scene(mk(tmp_path / "u", [], aligned=False), {"objects": [BOX]}, CRIT)
    with pytest.raises(SystemExit):
        ev.eval_scene(mk(tmp_path / "v", [], version="v9"), {"objects": [BOX]}, CRIT)


def test_synthetic_chain_end_to_end(tmp_path):
    """합성 기록 → map_seq(L1) → obstacles_map(L2) → 평가. 상자 4개 검출·단계 일치, 가까운 7·10 cm 단차 오차 < 1.5 cm,
    발자국 누출 0, 벽에서 0.6 m 안쪽 구역에 오검출 없음."""
    pytest.importorskip("cv2")
    pytest.importorskip("open3d")
    import synth_seq as ss
    import map_seq as ms
    import obstacles_map as om
    from test_lidar_map_seq import make
    seq, out = str(tmp_path / "seq"), str(tmp_path / "map")
    make(seq, 40)
    ms.run(seq, out)
    om.run(out, CRIT_PATH)
    truth = tmp_path / "truth.json"
    truth.write_text(json.dumps({"boxes": ss.BOXES}))
    reg = {"x_m": [0.4, ss.WALLS_X[1] - 0.6], "y_m": [ss.WALLS_Y[0] + 0.6, ss.WALLS_Y[1] - 0.6]}
    s = ev.eval_scene(out, ev.gt_from_synth_truth(str(truth), reg), CRIT)
    sm = ev.summarize([s])
    assert sm["detection"] == {"n": 4, "detected": 4} and sm["level_ok"]["ok"] == 4, s["objects"]
    for r in s["objects"]:
        if r["height_m"] in (0.07, 0.10):
            assert abs(r["step_err_m"]) < 0.015, r
    assert sm["footprint_pass_cells"] == 0 and s["region"]["false_obstacles"] == [], s["region"]
