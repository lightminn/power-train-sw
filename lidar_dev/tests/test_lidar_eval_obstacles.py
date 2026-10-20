# L2 평가 도구(eval_obstacles v2) 시험: 손으로 만든 terrain_map으로 지표마다 확인(Codex 7차 반례 포함, 합계까지 단언)하고,
# 합성 기록 → L1 → L2 → 평가 사슬을 한 번 돈다.
import json, os, sys

import pytest

np = pytest.importorskip("numpy")
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_obstacles as ev  # noqa: E402

CRIT_PATH = os.path.join(HERE, "config", "obstacle_criteria_v0.json")
CRIT = json.load(open(CRIT_PATH, encoding="utf-8"))
CELL, X0, Y0, NX, NY = 0.05, 0.0, -1.0, 60, 40       # 지도 x 0–3 m, y −1–1 m
GX, GY = X0 + (np.arange(NX) + 0.5) * CELL, Y0 + (np.arange(NY) + 0.5) * CELL
BOX = {"id": "box1", "kind": "step_up", "height_m": 0.07, "x_m": 1.5, "y_m": 0.0, "size_m": [0.30, 0.40]}
REGION = {"x_m": [0.4, 2.5], "y_m": [-0.9, 0.9]}


def rect(x_lo, x_hi, y_lo, y_hi):
    return ((GX > x_lo) & (GX < x_hi))[None, :] & ((GY > y_lo) & (GY < y_hi))[:, None]


def foot(o, grow=0.0):
    sx, sy = o["size_m"]
    return rect(o["x_m"] - sx / 2 - grow, o["x_m"] + sx / 2 + grow, o["y_m"] - sy / 2 - grow, o["y_m"] + sy / 2 + grow)


def ob(i, mask, level="block", step=0.072):
    """장애물 하나(목록 항목, 그 칸 마스크)."""
    js, is_ = np.nonzero(mask)
    return {"id": i, "level": level, "cells": int(mask.sum()), "step_m": step,
            "centroid_xy_m": [float(GX[is_].mean()), float(GY[js].mean())]}, mask


def ring(o):
    """상자 둘레 바닥 한 칸 띠 + 상자 = L2가 실제로 만드는 상자 장애물 모양."""
    return foot(o, 0.05)


def mk(d, obs=(), objects=(BOX,), edit=None, aligned=True, version=None, base=1, profile=None):
    """바닥 전부 base 단계·관측, objects 발자국은 경로 허가 불가. obs = [(항목, 마스크)]."""
    os.makedirs(d, exist_ok=True)
    lv = np.full((NY, NX), base, np.int64)
    for o in objects:
        lv[foot(o)] = 3
    oid = np.full((NY, NX), -1, np.int64)
    for item, m in obs:
        oid[m] = item["id"]
    g = {"level": lv, "level_local": lv.copy(), "observed": lv > 0, "obstacle_id": oid}
    if edit:
        edit(g)
    np.savez_compressed(os.path.join(d, "terrain_grid.npz"), x0=X0, y0=Y0, cell=CELL, **g)
    json.dump({"type": "terrain_map", "alignment_valid": aligned, "criteria_version": version or CRIT["criteria_version"],
               "criteria_profile": profile, "obstacles": [item for item, _ in obs]}, open(os.path.join(d, "terrain_map.json"), "w"))
    return str(d)


def run(tmp_path, obs=(), objects=(BOX,), region=REGION, name="m", grid_objects=None, **kw):
    gt = {"scene": name, "objects": list(objects), **({"eval_region": region} if region else {})}
    s = ev.eval_scene(mk(tmp_path / name, obs, objects if grid_objects is None else grid_objects, **kw), gt, CRIT)
    return s, ev.summarize([s])


def test_perfect_detection(tmp_path):
    s, sm = run(tmp_path, [ob(0, ring(BOX))])
    r = s["objects"][0]
    assert r["got"] == "block" and r["level_strict"] and r["detected"] and r["direction_ok"] and r["step_err_m"] == pytest.approx(0.002)
    assert r["leak"]["status"] == "checked" and r["leak"]["pass_cells"] == 0
    assert sm["detection"] == {"n": 1, "detected": 1, "merged_only": 0} and sm["level"]["strict_ok"] == 1
    assert sm["leak"]["checked"] == 1 and sm["leak"]["pass_cells"] == 0 and sm["false_obstacles"]["n"] == 0
    assert sm["floor_false_risk_cells"]["n"] == 0 and sm["unknown"]["pooled_frac"] == 0 and sm["step_err_m"]["n"] == 1


def test_missed_box_and_leak_reach_summary(tmp_path):
    """음성 대조: 장애물이 없고 상자 칸이 통과면 놓침·단계 틀림·누출이 합계까지 올라간다."""
    s, sm = run(tmp_path, [], grid_objects=[])
    r = s["objects"][0]
    assert r["got"] == "pass" and r["level_ok"] is False and r["detected"] is False
    assert sm["detection"]["detected"] == 0 and sm["level"]["accepted_ok"] == 0
    assert sm["leak"]["pass_cells"] == r["leak"]["pass_cells"] > 0 and sm["step_err_m"]["n"] == 0


def test_one_obstacle_cannot_detect_two_boxes(tmp_path):
    """Codex #1: 정답 두 개(1.3·1.6 m)에 장애물 하나 → 검출 1/2. 둘을 덮는 장애물 하나 → 하나는 검출, 하나는 병합."""
    a = {**BOX, "id": "a", "x_m": 1.3, "size_m": [0.2, 0.2]}
    b = {**BOX, "id": "b", "x_m": 1.6, "size_m": [0.2, 0.2]}
    _, sm = run(tmp_path, [ob(0, foot(a, 0.05))], objects=[a, b], name="one")
    assert sm["detection"] == {"n": 2, "detected": 1, "merged_only": 0}
    s, sm = run(tmp_path, [ob(0, rect(1.15, 1.75, -0.15, 0.15))], objects=[a, b], name="both")
    assert sm["detection"] == {"n": 2, "detected": 1, "merged_only": 1} and s["merged_obstacles"] == [0]


def test_far_obstacle_not_matched_by_polygon_or_level(tmp_path):
    """Codex #2: 칸이 1.5 m 떨어진 장애물(다각형이 커도)은 정답 것이 아니고 오검출. 먼 '불가' 조각이 가까운 주의 조각을 가리지 않는다."""
    far, _ = ob(0, rect(0.45, 0.6, 0.5, 0.65))
    far["poly_xy_m"] = [[0, -1], [3, -1], [3, 1], [0, 1]]
    s, sm = run(tmp_path, [(far, rect(0.45, 0.6, 0.5, 0.65))], grid_objects=[])
    assert s["objects"][0]["detected"] is False and sm["false_obstacles"]["n"] == 1
    near = ob(1, ring(BOX), "caution", 0.055)
    blk = ob(2, rect(1.5, 1.6, 0.52, 0.62), "block", 0.07)   # 정답 가운데에서 약 0.55 m
    s, sm = run(tmp_path, [near, blk], name="two")
    r = s["objects"][0]
    assert r["step_m"] == 0.055 and r["step_err_m"] == pytest.approx(-0.015) and [o["id"] for o in s["region"]["false_obstacles"]] == [2]


def test_unknown_footprint_is_unevaluable(tmp_path):
    """Codex #3: 격자 전부 unknown·장애물 없음 → 3 cm 정답은 '통과 맞음'이 아니라 평가 불가."""
    low = {**BOX, "height_m": 0.03}
    s, sm = run(tmp_path, [], objects=[low], grid_objects=[], base=0)
    r = s["objects"][0]
    assert r["got"] == "unknown" and r["level_ok"] is None and not r["evaluable"]
    assert sm["level"]["evaluable"] == 0 and sm["level"]["unevaluable"] == 1 and sm["level"]["accepted_ok"] == 0
    assert sm["unknown"]["pooled_frac"] == 1.0


def test_step_down_uses_path_level_and_sign(tmp_path):
    """Codex #4: 20 cm 내려가기 — 경로 층 주의가 기대 주의와 맞음(목록 단계와 무관). 부호가 틀리면 오차를 내지 않음."""
    pit = {"id": "pit", "kind": "step_down", "height_m": 0.20, "x_m": 1.5, "y_m": 0.0, "size_m": [0.4, 0.6]}

    def caution_pit(g):
        g["level"][foot(pit)] = 2
    s, _ = run(tmp_path, [ob(0, foot(pit, 0.05), "block", -0.21)], objects=[pit], grid_objects=[], edit=caution_pit, name="dn")
    r = s["objects"][0]
    assert r["expected"] == "caution" and r["got"] == "caution" and r["level_strict"] and r["direction_ok"] and r["step_err_m"] == pytest.approx(0.01)
    s, sm = run(tmp_path, [ob(0, foot(pit, 0.05), "block", 0.20)], objects=[pit], grid_objects=[], edit=caution_pit, name="up")
    assert s["objects"][0]["direction_ok"] is False and "step_err_m" not in s["objects"][0]
    assert sm["direction"] == {"n": 1, "ok": 0} and sm["step_err_m"]["n"] == 0


def test_wall_expected_block(tmp_path):
    wall = {"id": "w", "kind": "wall", "height_m": 1.0, "x_m": 2.0, "y_m": 0.0, "size_m": [0.1, 1.0]}
    s, _ = run(tmp_path, [ob(0, foot(wall, 0.05), "block", 0.9)], objects=[wall])
    r = s["objects"][0]
    assert r["expected"] == "block" and r["got"] == "block" and r["detected"] and "step_err_m" not in r


def test_border_allows_only_nearest_threshold():
    """Codex #5: 6 cm는 5·7 cm 경계 같은 거리 → 나쁜 쪽 경계만 {caution, block}. 5.5 cm {pass, caution}, 3 cm·10 cm는 허용 없음."""
    lv = lambda h: ev.expected_level({"kind": "step_up", "height_m": h}, CRIT["rules"], 0.015)
    assert lv(0.06) == ("caution", {"caution", "block"})
    assert lv(0.055) == ("caution", {"pass", "caution"})
    assert lv(0.03) == ("pass", {"pass"}) and lv(0.10) == ("block", {"block"})


def test_strict_and_accepted_counted_separately(tmp_path):
    s, sm = run(tmp_path, [ob(0, ring(BOX))], objects=[{**BOX, "height_m": 0.065}])
    assert s["objects"][0]["level_strict"] is False and s["objects"][0]["level_ok"] is True
    assert sm["level"]["strict_ok"] == 0 and sm["level"]["accepted_ok"] == 1 and sm["level"]["border_cases"] == 1


def test_leak_not_checked_is_not_zero(tmp_path):
    """Codex #6: 크기 없음·너무 작음은 '누출 0'이 아니라 검사 못 함으로 센다."""
    nosize = {k: v for k, v in BOX.items() if k != "size_m"}
    tiny = {**BOX, "id": "tiny", "x_m": 1.0, "size_m": [0.02, 0.02]}
    _, sm = run(tmp_path, [ob(0, ring(BOX))], objects=[nosize, tiny], grid_objects=[BOX])
    assert sm["leak"]["objects"] == 2 and sm["leak"]["checked"] == 0
    assert sm["leak"]["not_checked"] == {"no_size": 1, "too_small": 1, "outside_map": 0, "unobserved": 0}
    _, sm = run(tmp_path, [ob(0, ring(BOX))], grid_objects=[], base=0, name="dark")   # 안쪽이 전부 unknown
    assert sm["leak"]["checked"] == 0 and sm["leak"]["not_checked"]["unobserved"] == 1


def test_floor_false_risk_reaches_summary(tmp_path):
    """바닥(상자 둘레 밖)의 국소 위험 칸은 합계까지 센다. 상자 바로 옆 띠(L2가 상자 경계로 칠하는 칸)는 세지 않는다."""
    def edit(g):
        g["level_local"][5:7, 15:18] = 2      # 바닥 x 0.75–0.9, y −0.75–−0.65에 가짜 위험 6칸
        g["level_local"][foot(BOX, 0.05)] = 3  # 상자 + 둘레 한 칸
    s, sm = run(tmp_path, [ob(0, ring(BOX))], edit=edit)
    assert s["region"]["floor_false_risk_cells"] == 6 and sm["floor_false_risk_cells"]["n"] == 6


def test_region_outside_map_counts_as_unknown(tmp_path):
    """Codex #7: 지도는 x 0–3 m인데 구역이 0–30 m면 나머지 90%는 unknown."""
    s, sm = run(tmp_path, [ob(0, ring(BOX))], region={"x_m": [0.0, 30.0], "y_m": [-0.9, 0.9]})
    g = s["region"]
    assert g["outside_map_cells"] == g["cells"] - 60 * 36 and sm["unknown"]["pooled_frac"] == pytest.approx(0.9, abs=0.01)


def test_false_obstacle_straddling_region_edge(tmp_path):
    """Codex #8: 가운데는 구역(x ≤ 2.5) 밖 2.6 m지만 칸이 2.4 m까지 들어온 장애물은 오검출."""
    s, _ = run(tmp_path, [ob(0, ring(BOX)), ob(1, rect(2.4, 2.8, 0.5, 0.7), "caution", 0.05)])
    assert [o["id"] for o in s["region"]["false_obstacles"]] == [1]


def test_unknown_pooled_vs_scene_mean_and_empty_region(tmp_path):
    """Codex #10: 1칸 0% + 많은 칸 100% → 칸 비율은 약 100%, 장면 평균 50%. 빈 구역은 예외 없이 따로 센다."""
    s1 = ev.eval_scene(mk(tmp_path / "a", [ob(0, ring(BOX))]), {"objects": [BOX], "eval_region": {"x_m": [0.5, 0.55], "y_m": [0.5, 0.55]}}, CRIT)
    s2 = ev.eval_scene(mk(tmp_path / "b", [ob(0, ring(BOX))], base=0), {"objects": [BOX], "eval_region": {"x_m": [0.4, 0.9], "y_m": [0.4, 0.9]}}, CRIT)
    s3 = ev.eval_scene(mk(tmp_path / "c", [ob(0, ring(BOX))]), {"objects": [BOX], "eval_region": {"x_m": [0.51, 0.52], "y_m": [0.51, 0.52]}}, CRIT)
    u = ev.summarize([s1, s2, s3])["unknown"]
    assert s1["region"]["cells"] == 1 and s3["region"]["cells"] == 0
    assert u["pooled_frac"] == pytest.approx(100 / 101, abs=1e-3) and u["scene_mean_frac"] == pytest.approx(0.5) and u["empty_regions"] == 1


def test_repeat_reports_attempts_and_misses(tmp_path):
    """Codex #11: 같은 상자 세 번 중 한 번 놓침 → 시도 3·측정 2·누락 1."""
    sc = [ev.eval_scene(mk(tmp_path / "a", [ob(0, ring(BOX), step=0.070)]), {"scene": "a", "objects": [BOX]}, CRIT),
          ev.eval_scene(mk(tmp_path / "b", [ob(0, ring(BOX), step=0.074)]), {"scene": "b", "objects": [BOX]}, CRIT),
          ev.eval_scene(mk(tmp_path / "c", [], objects=[]), {"scene": "c", "objects": [BOX]}, CRIT)]
    rp = ev.summarize(sc)["repeat_step_m"]["box1"]
    assert (rp["attempts"], rp["measured"], rp["missed"]) == (3, 2, 1)
    assert rp["mean"] == pytest.approx(0.072) and rp["sd"] == pytest.approx(0.0028, abs=1e-4)


def test_sensitivity_moves_ground_truth(tmp_path):
    """Codex #9: 좌표 오차 민감도. 5 cm·3°에서는 검출이 그대로, 1 m 옮기면 검출이 0까지 떨어진다(정답이 실제로 움직이는지 음성 대조)."""
    M = ev.Map(mk(tmp_path / "m", [ob(0, ring(BOX))]), CRIT)
    gt = {"objects": [BOX], "eval_region": REGION}
    small = ev.sensitivity([M], [gt], CRIT, 0.05, 3.0)
    assert small["combos"] == 27 and small["detected"] == [1, 1]
    big = ev.sensitivity([M], [gt], CRIT, 1.0, 0.0)
    assert big["detected"][0] == 0 and big["false_obstacles"][1] >= 1


def test_refuses_unaligned_old_or_other_criteria(tmp_path):
    with pytest.raises(SystemExit):
        ev.eval_scene(mk(tmp_path / "u", aligned=False), {"objects": [BOX]}, CRIT)
    with pytest.raises(SystemExit):
        ev.eval_scene(mk(tmp_path / "v", version="v9"), {"objects": [BOX]}, CRIT)

    def drop_oid(g):
        del g["obstacle_id"]
    with pytest.raises(SystemExit):
        ev.eval_scene(mk(tmp_path / "w", edit=drop_oid), {"objects": [BOX]}, CRIT)


def test_synthetic_chain_end_to_end(tmp_path):
    """합성 기록 → map_seq(L1) → obstacles_map(L2) → 평가. 상자 4개 검출, 경로 층에서 평가된 상자는 모두 허용 일치
    (3 m의 6 cm 상자는 윗면이 안 보여 발자국이 unknown → 평가 불가로 남는 게 맞음), 가까운 7·10 cm 단차 오차 < 1.5 cm,
    검사한 안쪽 칸에 누출 0(윗면이 안 보이는 상자는 검사 못 함), 벽에서 0.6 m 안쪽 구역에 오검출·바닥 가짜 위험 없음."""
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
    assert sm["detection"] == {"n": 4, "detected": 4, "merged_only": 0}, s["objects"]
    assert sm["level"]["evaluable"] >= 3 and sm["level"]["accepted_ok"] == sm["level"]["evaluable"], s["objects"]
    for r in s["objects"]:
        if r["height_m"] in (0.07, 0.10):
            assert abs(r["step_err_m"]) < 0.015, r
    lk = sm["leak"]   # 윗면이 안 보이는 상자(30 cm·먼 6 cm)는 안쪽이 전부 unknown → 검사 못 함으로 따로
    assert lk["checked"] >= 2 and lk["checked"] + lk["not_checked"]["unobserved"] == 4 and lk["pass_cells"] == 0, s["objects"]
    assert sm["false_obstacles"]["n"] == 0 and sm["floor_false_risk_cells"]["n"] == 0, s["region"]


def _npz(tmp_path, cells, name="map.npz"):
    """cells: {(gx, gy): (code, height_cm)} → 광민 map.npz 형식. 나머지 칸은 정상·높이 0."""
    import numpy as np
    tiles = sorted({(gx // 16, gy // 16) for gx, gy in cells} | {(0, -1), (0, 0), (1, -1), (1, 0)})
    H = np.zeros((len(tiles), 256), np.int16)
    C = np.zeros((len(tiles), 256), np.uint8)
    for (gx, gy), (code, hcm) in cells.items():
        ti = tiles.index((gx // 16, gy // 16))
        k = (gy % 16) * 16 + gx % 16
        C[ti, k], H[ti, k] = code, hcm
    p = str(tmp_path / name)
    np.savez(p, epoch=np.int64(0), tiles=np.array(tiles, np.int64), versions=np.zeros(len(tiles), np.int64), heights_cm=H, codes=C)
    return p


def test_terrain_map_npz_detects_box_and_reads_coordinates(tmp_path):
    """광민 map.npz(10-20): 칸 좌표 변환·장애물 묶음·단차가 맞고, 같은 판정으로 상자를 검출한다."""
    crit = CRIT
    box = {"id": "b", "kind": "step_up", "height_m": 0.10, "x_m": 1.0, "y_m": 0.10, "size_m": [0.20, 0.20]}
    cells = {(gx, gy): (1, 10) for gx in range(18, 22) for gy in range(0, 4)}       # x 0.90–1.10, y 0.00–0.20
    cells.update({(gx, gy): (4, 0) for gx in range(26, 28) for gy in range(-4, -2)})  # 관측 부족
    M = ev.TerrainMapNpz(_npz(tmp_path, cells))
    assert abs(M.x0 - 0.0) < 1e-9 and abs(M.y0 - (-0.8)) < 1e-9 and M.c == 0.05
    assert len(M.obstacles) == 1
    o = M.obstacles[0]
    assert o["step_m"] == 0.10 and abs(o["centroid_xy_m"][0] - 1.0) < 1e-6 and abs(o["centroid_xy_m"][1] - 0.10) < 1e-6
    r = ev.eval_scene(M, {"scene": "npz", "objects": [box], "eval_region": {"x_m": [0.4, 1.4], "y_m": [-0.5, 0.5]}}, crit)
    row = r["objects"][0]
    assert row["detected"] and row["got"] == "block"


def test_terrain_map_npz_rejects_wrong_file(tmp_path):
    import numpy as np
    p = str(tmp_path / "x.npz")
    np.savez(p, a=np.zeros(3))
    with pytest.raises(SystemExit):
        ev.TerrainMapNpz(p)


def test_profile_mismatch_rejected(tmp_path):
    """v0.2: A 프로필로 만든 지도를 B 기준으로 평가하지 않는다(Codex 9차 #2). 같은 프로필이면 통과."""
    from criteria import load_criteria
    v2 = os.path.join(os.path.dirname(CRIT_PATH), "obstacle_criteria_v0.2.json")
    a, b = load_criteria(v2), load_criteria(v2, "B_stop20")
    d = mk(tmp_path / "a", version="v0.2", profile="A_no_stop")
    ev.Map(d, a)
    with pytest.raises(SystemExit):
        ev.Map(d, b)
