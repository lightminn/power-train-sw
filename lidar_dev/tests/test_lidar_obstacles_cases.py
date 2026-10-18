# L2 분류 핵심(classify_points) 반례 시험: 높이 함수로 만든 점구름(수직 면 포함)을 바로 넣는다. L1·open3d 불필요.
# 교차검토 4차의 반례(넓은 단 위, 갈린 바닥, 급경사, 층층 단, 관계없는 윗면, 단일 칸)와 경계값·보류·내려가기를 고정한다.
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
    core, g = om.classify_points(P, CRIT, CAMS, 0.05, kw or None)
    return core, g


def level_at(core, g, x, y):
    ix = int((x - core["grid"]["x0"]) / 0.05)
    iy = int((y - core["grid"]["y0"]) / 0.05)
    return om.NAME[int(g["level"][iy, ix])]


def levels_in(core, g, xr, yr):
    gx = core["grid"]["x0"] + (np.arange(core["grid"]["nx"]) + 0.5) * 0.05
    gy = core["grid"]["y0"] + (np.arange(core["grid"]["ny"]) + 0.5) * 0.05
    mx = (gx > xr[0]) & (gx < xr[1])
    my = (gy > yr[0]) & (gy < yr[1])
    sub = g["level"][np.ix_(my, mx)]
    return {om.NAME[v]: int((sub == v).sum()) for v in range(4)}


def test_wide_step_top_is_not_ground():
    """10 cm 단 위가 바닥보다 넓어도 단 위는 불가(가장 큰 구역 = 바닥 가정 제거)."""
    core, g = run(scene(lambda x, y: np.where(x > 0.9, 0.10, 0.0)))
    top = levels_in(core, g, (1.05, 2.4), (-0.9, 0.9))
    floor = levels_in(core, g, (0.4, 0.8), (-0.9, 0.9))
    assert top["pass"] == 0 and top["block"] > 0.9 * sum(top.values()), top
    assert floor["pass"] > 0.8 * sum(floor.values()), floor


def test_floor_split_by_box_stays_ground():
    """20 cm 상자가 바닥을 둘로 갈라도 반대편 바닥은 z = 0 기준 바닥이라 통과."""
    core, g = run(scene(lambda x, y: np.where((x > 1.2) & (x < 1.5), 0.20, 0.0)))
    far = levels_in(core, g, (1.7, 2.4), (-0.9, 0.9))
    box = levels_in(core, g, (1.27, 1.43), (-0.9, 0.9))
    assert far["pass"] > 0.8 * sum(far.values()) and far["caution"] == 0, far
    assert box["block"] == sum(box.values()), box


@pytest.mark.parametrize("deg,allowed", [(35, {"block", "unknown"}), (27, {"block"}), (20, {"caution"})])
def test_slope_never_leaks_as_pass(deg, allowed):
    """경사면: 연결 규칙에 끊기는 35°도 통과가 되면 안 된다. 27°는 불가, 20°는 주의."""
    t = math.tan(math.radians(deg))
    core, g = run(scene(lambda x, y: np.clip((x - 1.0) * t, 0, None) * (x < 1.8) + (x >= 1.8) * 0.8 * t))
    mid = levels_in(core, g, (1.25, 1.6), (-0.8, 0.8))
    got = {k for k, v in mid.items() if v}
    assert "pass" not in got and got <= allowed, (deg, mid)


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
    """관측 안 된 띠로 떨어진 30 cm 윗면(수직 면 없음)은 기준 바닥과 관계가 없어 unknown."""
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


def test_flat_noisy_floor_has_no_obstacles():
    """σ 3 mm 평평한 바닥: 주의·불가 0(가짜 장애물 없음)."""
    core, g = run(scene(lambda x, y: np.zeros_like(x), noise=0.003))
    assert core["cell_counts"]["caution"] == 0 and core["cell_counts"]["block"] == 0 and not core["obstacles"], core["cell_counts"]

