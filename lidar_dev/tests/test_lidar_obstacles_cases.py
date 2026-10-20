# L2 분류 핵심(classify_points) 반례 시험: 높이 함수로 만든 점구름(수직 면 포함)을 바로 넣는다. L1·open3d 불필요.
# 교차검토 4·5차의 반례(넓은 단 위, 갈린 바닥, 급경사·경사 위 평탄부, 층층 단, 관계없는 윗면, 단일 칸, 얇은 턱, 도랑 너머 낮은 면, 큰 잡음)와
# 경계값·보류·내려가기를 고정한다. 위험 영역은 최빈 단계가 아니라 '통과 칸 0'으로 확인한다(소수 누출을 놓치지 않게).
# 시작 카메라는 (0, 0)에서 +x를 본다. level = 출발 지지면에서의 경로 허가, level_local = 그 칸의 국소 위험.
import json, math, os, sys

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import obstacles_map as om  # noqa: E402

CRIT = json.load(open(os.path.join(HERE, "config", "obstacle_criteria_v0.json"), encoding="utf-8"))
CAMS = np.array([[0.0, 0.0], [0.2, 0.0]])
SP = 0.015   # 지도 점 간격(TSDF 1.5 cm 복셀과 같은 정도)


def scene(zfun, xr=(0.3, 2.5), yr=(-1.0, 1.0), noise=0.002, holes=None, seed=0):
    """z = zfun(x, y) 표면 점 + 이웃 표본 높이 차 > 1 cm인 곳의 수직 면 점. holes(x, y) → True면 그 점은 관측 안 됨."""
    rs = np.random.default_rng(seed)
    xs, ys = np.arange(xr[0], xr[1], SP), np.arange(yr[0], yr[1], SP)
    X, Y = np.meshgrid(xs, ys)
    Z = zfun(X, Y)
    keep = np.ones_like(Z, bool) if holes is None else ~holes(X, Y)
    pts = [np.stack([X[keep], Y[keep], Z[keep]], 1)]
    for ax in (0, 1):
        a = (slice(None), slice(None, -1)) if ax == 1 else (slice(None, -1), slice(None))
        b = (slice(None), slice(1, None)) if ax == 1 else (slice(1, None), slice(None))
        d = np.abs(Z[b] - Z[a])
        ok = (d > 0.01) & keep[a] & keep[b]
        for xa, ya, xb, yb, za, zb in zip(X[a][ok], Y[a][ok], X[b][ok], Y[b][ok], Z[a][ok], Z[b][ok]):
            zz = np.arange(min(za, zb), max(za, zb), SP)
            pts.append(np.stack([np.full_like(zz, (xa + xb) / 2), np.full_like(zz, (ya + yb) / 2), zz], 1))
    P = np.concatenate(pts)
    return P + rs.normal(0, noise, P.shape)


def run(P, **kw):
    core, g = om.classify_points(P, CRIT, CAMS, 0.05, kw or None, start=(0.0, 0.0, 0.0))
    return core, g


def level_at(core, g, x, y):
    ix = int((x - core["grid"]["x0"]) / 0.05)
    iy = int((y - core["grid"]["y0"]) / 0.05)
    return om.NAME[int(g["level"][iy, ix])]


def levels_in(core, g, xr, yr, layer="level"):
    gx = core["grid"]["x0"] + (np.arange(core["grid"]["nx"]) + 0.5) * 0.05
    gy = core["grid"]["y0"] + (np.arange(core["grid"]["ny"]) + 0.5) * 0.05
    mx = (gx > xr[0]) & (gx < xr[1])
    my = (gy > yr[0]) & (gy < yr[1])
    sub = g[layer][np.ix_(my, mx)]
    return {om.NAME[v]: int((sub == v).sum()) for v in range(4)}


def test_wide_step_top_is_not_ground():
    """10 cm 단 위가 바닥보다 넓어도 단 위는 불가(가장 큰 구역 = 바닥 가정 제거)."""
    core, g = run(scene(lambda x, y: np.where(x > 0.9, 0.10, 0.0)))
    top = levels_in(core, g, (1.05, 2.4), (-0.9, 0.9))
    floor = levels_in(core, g, (0.4, 0.8), (-0.9, 0.9))
    assert top["pass"] == 0 and top["block"] > 0.9 * sum(top.values()), top
    assert floor["pass"] > 0.8 * sum(floor.values()), floor


def test_floor_split_by_box_layers():
    """20 cm 상자가 바닥을 둘로 가르면: 반대편 바닥은 국소로는 통과(주의 아님), 경로 허가는 상자에 가로막혀 통과 아님. 상자는 장애물."""
    core, g = run(scene(lambda x, y: np.where((x > 1.2) & (x < 1.5), 0.20, 0.0)))
    far_l = levels_in(core, g, (1.7, 2.4), (-0.9, 0.9), "level_local")
    far = levels_in(core, g, (1.7, 2.4), (-0.9, 0.9))
    box = levels_in(core, g, (1.27, 1.43), (-0.9, 0.9))
    assert far_l["pass"] > 0.8 * sum(far_l.values()) and far_l["caution"] == 0, far_l
    assert far["pass"] == 0, far
    assert box["pass"] == 0 and box["block"] > 0.9 * sum(box.values()), box
    assert any(o["level"] == "block" and 1.2 < o["centroid_xy_m"][0] < 1.5 for o in core["obstacles"]), core["obstacles"]


@pytest.mark.parametrize("deg,allowed", [(35, {"block", "unknown"}), (27, {"block"}), (20, {"caution"})])
def test_slope_never_leaks_as_pass(deg, allowed):
    """경사면: 연결 규칙에 끊기는 35°도 통과가 되면 안 된다. 27°는 불가, 20°는 주의(두 층 모두)."""
    t = math.tan(math.radians(deg))
    core, g = run(scene(lambda x, y: np.clip((x - 1.0) * t, 0, None) * (x < 1.8) + (x >= 1.8) * 0.8 * t))
    for layer in ("level", "level_local"):
        mid = levels_in(core, g, (1.25, 1.6), (-0.8, 0.8), layer)
        got = {k for k, v in mid.items() if v}
        assert "pass" not in got and got <= allowed, (deg, layer, mid)


def test_plateau_behind_steep_slope_is_not_permitted():
    """5차 반례: 27° 경사 위 평탄부 — 국소로는 평평하지만 경로 허가는 경사(불가)를 지나야 하므로 통과 아님. 경사 모서리 칸도 통과 아님."""
    core, g = run(scene(lambda x, y: np.clip(np.tan(np.deg2rad(27)) * (x - 1.0), 0, 0.30)))
    ramp = levels_in(core, g, (1.0, 1.59), (-0.9, 0.9))
    top = levels_in(core, g, (1.75, 2.4), (-0.9, 0.9))
    assert ramp["pass"] == 0, ramp
    assert top["pass"] == 0, top
    assert levels_in(core, g, (1.8, 2.3), (-0.8, 0.8), "level_local")["pass"] > 0


def test_tiered_steps_take_local_boundary():
    """0 → 10 cm → 20 cm: 가운데 단도 바닥에서 10 cm 오르기라 불가(둘레 중앙값 상쇄 금지)."""
    core, g = run(scene(lambda x, y: np.where(x > 1.6, 0.20, np.where(x > 1.0, 0.10, 0.0))))
    for xr in ((1.15, 1.45), (1.75, 2.4)):
        s = levels_in(core, g, xr, (-0.9, 0.9))
        assert s["block"] > 0.9 * sum(s.values()), (xr, s)


def test_climbable_staircase_of_small_steps_passes():
    """4 cm씩 오르는 단은 한 단씩은 통과 기준 안(측정 여유 1 cm 포함 5 cm 미만 → 3.5 cm로 시험)."""
    core, g = run(scene(lambda x, y: 0.035 * np.floor(np.clip(x - 0.9, 0, None) / 0.4)))
    s = levels_in(core, g, (1.0, 2.2), (-0.8, 0.8))
    assert s["block"] == 0 and s["pass"] > 0.6 * sum(s.values()), s


def test_unrelated_top_is_unknown_not_pass():
    """관측 안 된 띠로 떨어진 30 cm 윗면(수직 면 없음)은 출발 지지면에서 관측된 길이 없어 통과 아님."""
    P = scene(lambda x, y: np.where(x > 1.6, 0.30, 0.0), holes=lambda x, y: (x > 1.3) & (x < 1.6))
    core, g = run(P)
    s = levels_in(core, g, (1.7, 2.4), (-0.9, 0.9))
    assert s["pass"] == 0, s


def test_single_cell_post_is_listed_with_valid_polygon():
    """5 cm × 5 cm, 10 cm 기둥: 격자에서 불가이고 장애물 목록에도 남으며 다각형이 칸을 덮는다."""
    core, g = run(scene(lambda x, y: np.where((np.abs(x - 1.425) < 0.025) & (np.abs(y - 0.025) < 0.025), 0.10, 0.0)))
    obs = [o for o in core["obstacles"] if abs(o["centroid_xy_m"][0] - 1.425) < 0.15 and abs(o["centroid_xy_m"][1] - 0.025) < 0.15]
    assert obs and obs[0]["level"] == "block", core["obstacles"]
    poly = np.array(obs[0]["poly_xy_m"])
    assert len(poly) >= 3
    x0, x1, y0, y1 = poly[:, 0].min(), poly[:, 0].max(), poly[:, 1].min(), poly[:, 1].max()
    tol = SP + 1e-6   # 지도 점 간격만큼의 격자 양자화는 허용
    assert x0 <= 1.40 + tol and x1 >= 1.45 - tol and y0 <= 0.0 + tol and y1 >= 0.05 - tol, poly


@pytest.mark.parametrize("hgt,want", [(0.035, "pass"), (0.045, "caution"), (0.05, "caution"), (0.065, "block"), (0.10, "block")])
def test_step_thresholds_with_margin(hgt, want):
    """기준 5/7 cm + 측정 여유 1 cm: 4.5 cm는 주의(경계 근처는 나쁜 쪽), 6.5 cm는 불가."""
    core, g = run(scene(lambda x, y: np.where((x > 1.2) & (x < 1.8) & (np.abs(y) < 0.4), hgt, 0.0)))
    s = levels_in(core, g, (1.3, 1.7), (-0.3, 0.3))
    assert max((v, k) for k, v in s.items())[1] == want, (hgt, s)


def test_no_reference_floor_withholds():
    """z = 0 바닥이 없으면(전부 20 cm 높이 면) 분류 보류 — 통과 칸 0."""
    core, g = run(scene(lambda x, y: np.full_like(x, 0.20)))
    assert core["info"]["withheld"] and core["cell_counts"]["pass"] == 0 and core["cell_counts"]["block"] == 0


@pytest.mark.parametrize("drop,want", [(0.10, "pass"), (0.20, "caution"), (0.35, "block")])
def test_step_down_seen_directly_uses_down_rule(drop, want):
    """수직 면이 보이는 내려가기: 기준 바닥에서 바깥으로 내려가므로 step_down 15/30 cm 규칙."""
    core, g = run(scene(lambda x, y: np.where(x > 1.2, -drop, 0.0)))
    s = levels_in(core, g, (1.4, 2.3), (-0.8, 0.8))
    assert max((v, k) for k, v in s.items())[1] == want, (drop, s)


@pytest.mark.parametrize("noise,unk_max", [(0.003, 0.0), (0.010, 0.05)])
def test_flat_noisy_floor_has_no_obstacles(noise, unk_max):
    """평평한 바닥 σ 3·10 mm: 주의·불가 0(가짜 장애물 없음), 미확인은 관측 칸의 5% 이하(5차: σ10 mm에서 모서리 과검출 확인)."""
    core, g = run(scene(lambda x, y: np.zeros_like(x), noise=noise))
    c = core["cell_counts"]
    assert c["caution"] == 0 and c["block"] == 0 and not core["obstacles"], c
    obs = core["info"]["observed_cells"]
    assert levels_in(core, g, (0.4, 2.4), (-0.9, 0.9))["unknown"] <= unk_max * obs + 1, (c, core["info"])


def test_thin_full_width_bar_is_blocked():
    """5차 반례(v2 회귀): 높이 10 cm·폭 2.5 cm 전폭 턱 — 칸 안 돌출이 양옆 바닥으로 설명되지 않으므로 불가, 건너편 경로 허가 없음, 장애물 목록에 있음."""
    core, g = run(scene(lambda x, y: np.where(np.abs(x - 1.425) < 0.0125, 0.10, 0.0)))
    bar = levels_in(core, g, (1.36, 1.49), (-0.9, 0.9), "level_local")
    beyond = levels_in(core, g, (1.6, 2.4), (-0.9, 0.9))
    assert bar["block"] > 0 and bar["block"] >= 0.8 * 36, bar    # 36칸 = 띠 길이 1.8 m / 5 cm
    assert beyond["pass"] == 0, beyond
    assert any(o["level"] == "block" and abs(o["centroid_xy_m"][0] - 1.425) < 0.1 for o in core["obstacles"]), core["obstacles"]


def test_low_surface_beyond_trench_is_not_reference():
    """5차 반례: z 0 → −10 cm(도랑) → +2.5 cm. 건너편 면은 z ≈ 0이지만 출발 지지면이 아니며, 12.5 cm 오르기 경계 때문에 통과 아님."""
    core, g = run(scene(lambda x, y: np.where(x < 0.9, 0.0, np.where(x < 1.4, -0.10, 0.025))))
    trench = levels_in(core, g, (1.0, 1.3), (-0.9, 0.9))
    far = levels_in(core, g, (1.5, 2.4), (-0.9, 0.9))
    assert trench["pass"] > 0.8 * sum(trench.values()), trench    # 10 cm 내려가기는 통과
    assert far["pass"] == 0, far


@pytest.mark.parametrize("hgt,want", [(1.2, "pass"), (0.95, "caution"), (0.7, "block")])
def test_overhang_uses_overhead_rule(hgt, want):
    """책상·선반 아랫면(바닥 위 판): 돌출이 아니라 머리 위 틈 — 1.2 m 통과, 0.95 m 주의, 0.7 m 불가(기준값 overhead 1.00/0.90 m)."""
    P = scene(lambda x, y: np.zeros_like(x))
    rs = np.random.default_rng(4)
    X, Y = np.meshgrid(np.arange(1.2, 1.8, SP), np.arange(-0.4, 0.4, SP))
    plate = np.stack([X.ravel(), Y.ravel(), np.full(X.size, hgt)], 1) + rs.normal(0, 0.002, (X.size, 3))
    core, g = run(np.concatenate([P, plate]))
    s = levels_in(core, g, (1.3, 1.7), (-0.3, 0.3))
    assert max((v, k) for k, v in s.items())[1] == want and (want == "pass" or s["pass"] == 0), (hgt, s)
    reasons = set(g["reason_local"][g["observed"]].astype(str))
    assert "protrusion" not in reasons, reasons


def test_box_right_in_front_does_not_move_start():
    """6차 반례: 바로 앞(0.5–0.65 m) 10 cm 상자 — 상자 뒤 바닥을 출발 지지면으로 잡으면 안 된다(뒤쪽은 상자에 가로막힘)."""
    core, g = run(scene(lambda x, y: np.where((x > 0.5) & (x < 0.65), 0.10, 0.0)))
    s = levels_in(core, g, (0.85, 2.4), (-0.9, 0.9))
    assert s["pass"] == 0, (s, core["info"])


def test_coincident_height_does_not_erase_protrusion():
    """6차 반례: 10 cm 돌출 점 3개 옆(약 10 cm)에 우연히 같은 높이의 작은 판 — 돌출 칸은 국소 불가, 경로 허가도 통과 아님."""
    def z(x, y):
        return np.where((x >= 1.10) & (x < 1.15) & (y >= 0.10) & (y < 0.15), 0.10, 0.0)
    P = np.concatenate([scene(z, noise=0.0), np.array([[1.02, 0.025, 0.10], [1.025, 0.025, 0.10], [1.03, 0.025, 0.10]])])
    core, g = run(P)
    ix = int((1.025 - core["grid"]["x0"]) / 0.05)
    iy = int((0.025 - core["grid"]["y0"]) / 0.05)
    assert g["level_local"][iy, ix] == om.LEVEL["block"], (g["level_local"][iy, ix], g["reason_local"][iy, ix])
    assert g["level"][iy, ix] != om.LEVEL["pass"], (g["level"][iy, ix], g["reason"][iy, ix])


def test_unevaluated_bottleneck_keeps_beyond_unknown():
    """6차 반례: 0.9–1.7 m 구간을 폭 5 cm 띠만 남긴 평지 — 띠는 표면 평가 불가라 지나갈 수 없고, 뒤쪽은 주의가 아니라 unknown."""
    P = scene(lambda x, y: np.zeros_like(x), noise=0.0, holes=lambda x, y: (x > 0.9) & (x < 1.7) & ~((y >= 0) & (y < 0.05)))
    core, g = run(P)
    s = levels_in(core, g, (1.8, 2.4), (-0.9, 0.9))
    assert s["caution"] == 0 and s["pass"] == 0 and s["block"] == 0, s
    reasons = set(g["reason"][g["observed"]].astype(str))
    assert "via_overhead" not in reasons and "overhead" not in reasons, reasons


# ---------- 기준값 v0.2(10-20, 바퀴별 제어 기준 A·B 프로필) ----------
from criteria import load_criteria  # noqa: E402

V2 = os.path.join(HERE, "config", "obstacle_criteria_v0.2.json")


def test_criteria_v02_profiles_and_v01_compat():
    """v0.2 최상위 rules = 기본 프로필(A). B는 profile로. v0.1 파일은 그대로 읽힘(프로필 없음)."""
    a, b, v1 = load_criteria(V2), load_criteria(V2, "B_stop20"), load_criteria(os.path.join(HERE, "config", "obstacle_criteria_v0.json"))
    assert a["criteria_profile"] == "A_no_stop" and a["rules"]["step_up_m"]["block_from"] == pytest.approx(0.09)
    assert b["criteria_profile"] == "B_stop20" and b["rules"]["step_up_m"]["block_from"] == pytest.approx(0.10)
    assert v1["criteria_profile"] is None and v1["rules"]["step_up_m"]["block_from"] == pytest.approx(0.07)
    with pytest.raises(SystemExit):
        load_criteria(V2, "C")


@pytest.mark.parametrize("h,want", [(0.065, {"v0.1": "block", "A_no_stop": "caution", "B_stop20": "pass"}),
                                    (0.085, {"v0.1": "block", "A_no_stop": "block", "B_stop20": "caution"})])
def test_box_level_follows_profile(h, want):
    """같은 상자를 v0.1·A·B로 분류. L2는 단차에 측정 여유 1 cm(ALG meas_margin_m)를 더해 판정한다:
    6.5 cm(재면 약 6.4 + 1) = v0.1 불가(≥ 7)·A 주의(7–9)·B 통과(< 8, 장애물 없음), 8.5 cm(약 8.3 + 1) = A 불가(≥ 9)·B 주의(8–10)."""
    P = scene(lambda x, y: np.where((x > 1.2) & (x < 1.6) & (np.abs(y) < 0.3), h, 0.0))
    crits = {"v0.1": CRIT, "A_no_stop": load_criteria(V2), "B_stop20": load_criteria(V2, "B_stop20")}
    for name, crit in crits.items():
        core, g = om.classify_points(P, crit, CAMS, 0.05, None, start=(0.0, 0.0, 0.0))
        obs = [o for o in core["obstacles"] if 1.2 < o["centroid_xy_m"][0] < 1.6 and abs(o["centroid_xy_m"][1]) < 0.3]
        got = [(o["level"], o["step_m"], o["centroid_xy_m"]) for o in core["obstacles"]]
        if want[name] == "pass":
            assert not obs, (h, name, got)
        else:
            assert len(obs) == 1 and obs[0]["level"] == want[name], (h, name, got)
