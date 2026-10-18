# eval_obstacles.py — L2 결과(terrain_map.json·terrain_grid.npz)를 줄자 실측표(정답 JSON)와 맞대어 평가한다(10-23 계획 A5, 10-19).
# 장면마다: 정답 물체의 검출·단계 일치·단차 오차, 평가 구역 안 오검출(정답 물체와 안 맞는 장애물), 바닥 가짜 위험 칸, unknown 비율,
#   정답 발자국 안 '통과' 칸(경로 허가 층 누출). 여러 장면에 같은 id의 물체가 있으면 단차 반복 측정 평균·표준편차.
# 정답 JSON(장면 하나):
#   {"scene": "s2_boxes_a",
#    "objects": [{"id": "box1", "kind": "step_up", "height_m": 0.07, "x_m": 1.5, "y_m": 0.0, "size_m": [0.30, 0.40]}, ...],
#    "eval_region": {"x_m": [0.4, 2.5], "y_m": [-1.0, 1.0]}}
#   좌표는 L2 지도 좌표와 같다: 원점 = 시작 카메라 바로 아래 바닥, x = 시작 시 카메라 앞, y = 왼쪽(+). x_m·y_m은 물체 가운데,
#   size_m = [x 방향 길이, y 방향 폭](없으면 발자국 검사 생략). kind = step_up(솟은 물체·단) | step_down(꺼진 곳) | wall(넘지 못하는 높은 것).
#   eval_region = 오검출·바닥 가짜 위험·unknown을 세는 구역(벽·가구처럼 정답에 안 적은 것은 구역 밖에 두기).
# 기대 단계는 같은 기준값 JSON으로 정답 높이에서 계산한다. 정답 높이가 경계 ± border_m(줄자 오차 + L2 측정 여유 1 cm) 안이면 이웃 단계도 맞음으로 센다.
# 사용: python eval_obstacles.py <기준값 json> <지도 폴더> <정답 json> [<지도 폴더> <정답 json> ...] [--out eval.json] [--match 0.35]
import argparse, json, math, os, statistics, sys

import numpy as np

LEVEL = {"unknown": 0, "pass": 1, "caution": 2, "block": 3}
NAME = {v: k for k, v in LEVEL.items()}


def expected_level(obj, rules, border_m):
    """정답 물체 → (기대 단계 이름, 허용 단계 집합). wall은 늘 불가."""
    if obj["kind"] == "wall":
        return "block", {"block"}
    r = rules["step_up_m" if obj["kind"] == "step_up" else "step_down_m"]
    h = abs(float(obj["height_m"]))
    lv = "block" if h >= r["block_from"] else "caution" if h >= r["caution_from"] else "pass"
    ok = {lv}
    for thr, lo, hi in ((r["caution_from"], "pass", "caution"), (r["block_from"], "caution", "block")):
        if abs(h - thr) <= border_m:
            ok |= {lo, hi}
    return lv, ok


def _in_poly(x, y, poly):
    """점이 다각형(꼭짓점 목록) 안인지 — 광선 교차."""
    inside, n = False, len(poly)
    for k in range(n):
        (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _near_footprint(o, obj, pad):
    """장애물 가운데가 정답 발자국(+pad) 안이거나 다각형이 정답 가운데를 품으면 그 물체 몫."""
    cx, cy = o["centroid_xy_m"]
    sx, sy = obj.get("size_m") or (0.0, 0.0)
    if abs(cx - obj["x_m"]) <= sx / 2 + pad and abs(cy - obj["y_m"]) <= sy / 2 + pad:
        return True
    return bool(o.get("poly_xy_m")) and _in_poly(obj["x_m"], obj["y_m"], o["poly_xy_m"])


def _region_mask(gx, gy, reg):
    return ((gx >= reg["x_m"][0]) & (gx <= reg["x_m"][1]))[None, :] & ((gy >= reg["y_m"][0]) & (gy <= reg["y_m"][1]))[:, None]


def eval_scene(map_dir, gt, crit, match_m=0.35, border_m=0.015, pad_m=0.15):
    tm = json.load(open(os.path.join(map_dir, "terrain_map.json"), encoding="utf-8"))
    if tm.get("type") != "terrain_map" or not tm.get("alignment_valid"):
        raise SystemExit(f"{map_dir}: 정렬된 terrain_map이 아님 — 평가하지 않는다")
    if tm.get("criteria_version") != crit.get("criteria_version"):
        raise SystemExit(f"{map_dir}: 지도 기준값 {tm.get('criteria_version')} ≠ 평가 기준값 {crit.get('criteria_version')}")
    g = np.load(os.path.join(map_dir, "terrain_grid.npz"))
    lv, loc, obs_m = g["level"], g["level_local"], g["observed"].astype(bool)
    c = float(g["cell"])
    gx, gy = float(g["x0"]) + (np.arange(lv.shape[1]) + 0.5) * c, float(g["y0"]) + (np.arange(lv.shape[0]) + 0.5) * c
    obstacles = tm["obstacles"]
    owner = {}   # 장애물 id → 정답 물체 id
    rows = []
    for obj in gt["objects"]:
        want, ok = expected_level(obj, crit["rules"], border_m)
        mine = [o for o in obstacles if _near_footprint(o, obj, pad_m)
                or math.hypot(o["centroid_xy_m"][0] - obj["x_m"], o["centroid_xy_m"][1] - obj["y_m"]) <= match_m]
        for o in mine:
            owner.setdefault(o["id"], obj["id"])
        best = max(mine, key=lambda o: (LEVEL[o["level"]], -math.hypot(o["centroid_xy_m"][0] - obj["x_m"], o["centroid_xy_m"][1] - obj["y_m"])), default=None)
        got = best["level"] if best else "pass"
        r = {"id": obj["id"], "kind": obj["kind"], "height_m": obj["height_m"], "expected": want, "accepted": sorted(ok),
             "got": got, "level_ok": got in ok, "pieces": len(mine), "border": len(ok) > 1,
             "detected": None if want == "pass" else LEVEL[got] >= LEVEL["caution"],
             "step_m": best.get("step_m") if best else None, "dist_m": None if not best else round(math.hypot(
                 best["centroid_xy_m"][0] - obj["x_m"], best["centroid_xy_m"][1] - obj["y_m"]), 3)}
        if r["step_m"] is not None and obj["kind"] != "wall":
            r["step_err_m"] = round(abs(r["step_m"]) - abs(float(obj["height_m"])), 4)
        if obj.get("size_m"):   # 발자국 안 경로 허가 '통과' 칸 = 누출(상자 위로 길이 열림)
            sx, sy = obj["size_m"]
            fp = _region_mask(gx, gy, {"x_m": [obj["x_m"] - sx / 2, obj["x_m"] + sx / 2], "y_m": [obj["y_m"] - sy / 2, obj["y_m"] + sy / 2]})
            r["footprint_cells"] = int(fp.sum())
            r["footprint_pass_cells"] = int((fp & (lv == LEVEL["pass"])).sum()) if want != "pass" else None
        rows.append(r)
    reg = gt.get("eval_region")
    out = {"scene": gt.get("scene", os.path.basename(map_dir.rstrip("/"))), "map_dir": map_dir, "objects": rows}
    if reg:
        R = _region_mask(gx, gy, reg)
        floor = R.copy()
        for obj in gt["objects"]:   # 정답 물체 둘레(+pad)를 뺀 바닥
            sx, sy = obj.get("size_m") or (2 * match_m, 2 * match_m)
            floor &= ~_region_mask(gx, gy, {"x_m": [obj["x_m"] - sx / 2 - pad_m, obj["x_m"] + sx / 2 + pad_m],
                                            "y_m": [obj["y_m"] - sy / 2 - pad_m, obj["y_m"] + sy / 2 + pad_m]})
        fps = [o for o in obstacles if o["id"] not in owner and reg["x_m"][0] <= o["centroid_xy_m"][0] <= reg["x_m"][1]
               and reg["y_m"][0] <= o["centroid_xy_m"][1] <= reg["y_m"][1]]
        out["region"] = {"cells": int(R.sum()), "unknown_frac": round(float((lv[R] == 0).mean()), 4) if R.any() else None,
                         "unobserved_frac": round(float((~obs_m[R]).mean()), 4) if R.any() else None,
                         "floor_cells": int(floor.sum()),
                         "floor_false_risk_cells": int((floor & obs_m & (loc >= LEVEL["caution"])).sum()),
                         "false_obstacles": [{"id": o["id"], "level": o["level"], "centroid_xy_m": o["centroid_xy_m"], "step_m": o.get("step_m"),
                                              "cells": o["cells"]} for o in fps]}
    return out


def summarize(scenes):
    """장면들을 합친 수치 — 분모를 같이 남긴다."""
    objs = [r for s in scenes for r in s["objects"]]
    det = [r for r in objs if r["detected"] is not None]
    over = [r for r in objs if r["expected"] == "pass" and LEVEL[r["got"]] >= LEVEL["caution"] and not r["level_ok"]]
    errs = [r["step_err_m"] for r in objs if "step_err_m" in r]
    regs = [s["region"] for s in scenes if "region" in s]
    rep = {}
    for r in objs:
        if r.get("step_m") is not None:
            rep.setdefault(r["id"], []).append(abs(r["step_m"]))
    return {"scenes": len(scenes), "objects": len(objs),
            "detection": {"n": len(det), "detected": sum(r["detected"] for r in det)},
            "level_ok": {"n": len(objs), "ok": sum(r["level_ok"] for r in objs), "border_cases": sum(r["border"] for r in objs)},
            "over_warning_on_pass_objects": len(over),
            "step_err_m": {"n": len(errs), "mean": round(statistics.mean(errs), 4) if errs else None,
                           "max_abs": round(max(map(abs, errs)), 4) if errs else None},
            "footprint_pass_cells": sum(r.get("footprint_pass_cells") or 0 for r in objs),
            "false_obstacles": {"scenes_with_region": len(regs), "n": sum(len(g["false_obstacles"]) for g in regs)},
            "floor_false_risk_cells": {"n": sum(g["floor_false_risk_cells"] for g in regs), "of_floor_cells": sum(g["floor_cells"] for g in regs)},
            "unknown_frac_mean": round(statistics.mean(g["unknown_frac"] for g in regs), 4) if regs else None,
            "repeat_step_m": {k: {"n": len(v), "mean": round(statistics.mean(v), 4), "sd": round(statistics.stdev(v), 4) if len(v) > 1 else None}
                              for k, v in rep.items() if len(v) > 1}}


def gt_from_synth_truth(path, region=None):
    """synth_seq.py의 truth.json(상자 (x0, x1, y0, y1, 높이), 시작 카메라 = 월드 원점·앞 = +x) → 정답 JSON. 시험용."""
    t = json.load(open(path))
    objs = [{"id": f"box{k}_{h * 100:g}cm", "kind": "step_up", "height_m": h, "x_m": (x0 + x1) / 2, "y_m": (y0 + y1) / 2, "size_m": [x1 - x0, y1 - y0]}
            for k, (x0, x1, y0, y1, h) in enumerate(t["boxes"])]
    return {"scene": "synthetic", "objects": objs, **({"eval_region": region} if region else {})}


def main():
    ap = argparse.ArgumentParser(description="L2 결과를 줄자 실측표(정답 JSON)와 비교")
    ap.add_argument("criteria")
    ap.add_argument("pairs", nargs="+", help="<지도 폴더> <정답 json> 쌍")
    ap.add_argument("--out")
    ap.add_argument("--match", type=float, default=0.35, help="정답 가운데와 장애물 가운데 거리 한도(m)")
    ap.add_argument("--border", type=float, default=0.015, help="경계 허용(m): 줄자 오차 + L2 측정 여유")
    a = ap.parse_args()
    if len(a.pairs) % 2:
        ap.error("지도 폴더와 정답 json을 쌍으로 준다")
    crit = json.load(open(a.criteria, encoding="utf-8"))
    scenes = [eval_scene(m, json.load(open(g, encoding="utf-8")), crit, a.match, a.border) for m, g in zip(a.pairs[::2], a.pairs[1::2])]
    res = {"criteria_version": crit.get("criteria_version"), "summary": summarize(scenes), "scenes": scenes}
    for s in scenes:
        print(f"[{s['scene']}]")
        for r in s["objects"]:
            print("  %-12s %-8s h %5.3f  기대 %-7s 결과 %-7s %s%s  단차 %s  조각 %d%s" % (
                r["id"], r["kind"], r["height_m"], r["expected"], r["got"], "맞음" if r["level_ok"] else "틀림", "(경계)" if r["border"] else "",
                "-" if r.get("step_m") is None else "%.3f(%+.3f)" % (r["step_m"], r.get("step_err_m", 0)), r["pieces"],
                "" if r.get("footprint_pass_cells") is None else "  발자국 통과 %d/%d칸" % (r["footprint_pass_cells"], r["footprint_cells"])))
        if "region" in s:
            g = s["region"]
            print("  구역 %d칸: unknown %.1f%%, 바닥 가짜 위험 %d/%d칸, 오검출 %d개" % (
                g["cells"], 100 * (g["unknown_frac"] or 0), g["floor_false_risk_cells"], g["floor_cells"], len(g["false_obstacles"])))
    print(json.dumps(res["summary"], ensure_ascii=False))
    if a.out:
        json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
