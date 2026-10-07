"""`wrist_metrics` 유닛테스트 — 카메라도 ROS 도 없이 도는 유일한 검증 층이다."""
from robot_arm_perception.wrist_metrics import (
    apparent_size_to_distance_m,
    border_contacts,
    clamp_correction,
    classify_grasp,
    distance_to_apparent_size_px,
    fit_distance_curve,
    GRASP_FILL_MAX,
    GRASP_REFERENCE,
    mask_fill_ratio,
    normalized_centroid,
    pixel_to_meter_scale,
    robust_stats,
    size_area_consistent,
    suggest_band,
    summarize,
    usable,
)

W, H = 640, 480


def test_fill_ratio_is_fraction_of_frame():
    assert mask_fill_ratio(W * H // 4, W, H) == 0.25
    assert mask_fill_ratio(0, W, H) == 0.0
    # 마스크가 프레임보다 클 수는 없다 — 상류가 이상해도 1.0 을 넘겨 내보내지 않는다.
    assert mask_fill_ratio(W * H * 3, W, H) == 1.0
    assert mask_fill_ratio(100, 0, 0) == 0.0


def test_centroid_is_centered_and_signed():
    assert normalized_centroid(W / 2, H / 2, W, H) == (0.0, 0.0)
    u, v = normalized_centroid(W, H, W, H)
    assert (u, v) == (1.0, 1.0)          # 우하단 끝
    u, v = normalized_centroid(0, 0, W, H)
    assert (u, v) == (-1.0, -1.0)        # 좌상단 끝
    u, _ = normalized_centroid(W * 0.75, H / 2, W, H)
    assert u == 0.5                      # 오른쪽으로 절반


def test_distance_round_trip():
    # f_px=600, 95mm 박스가 300mm 거리 → 190px
    size = distance_to_apparent_size_px(0.30, 600.0, 0.095)
    assert abs(size - 190.0) < 1e-6
    back = apparent_size_to_distance_m(size, 600.0, 0.095)
    assert abs(back - 0.30) < 1e-9


def test_distance_is_monotonic_decreasing_in_size():
    near = apparent_size_to_distance_m(400.0, 600.0, 0.095)
    far = apparent_size_to_distance_m(100.0, 600.0, 0.095)
    assert near < far                    # 크게 보이면 가깝다


def test_distance_refuses_untrustworthy_input():
    # 너무 작은 겉보기 크기 / 캘리브 안 된 f_px 는 0 이 아니라 None 이어야 한다.
    assert apparent_size_to_distance_m(3.0, 600.0, 0.095) is None
    assert apparent_size_to_distance_m(200.0, 0.0, 0.095) is None
    assert apparent_size_to_distance_m(200.0, 600.0, 0.0) is None
    assert apparent_size_to_distance_m(None, 600.0, 0.095) is None


def test_border_contacts_counts_edges():
    assert border_contacts(100, 100, 300, 300, W, H) == 0
    assert border_contacts(0, 100, 300, 300, W, H) == 1
    # 화면을 가득 채운 근접 프레임 — 손목 캠에서는 정상 상황이라 버리지 않고 4를 센다.
    assert border_contacts(0, 0, W - 1, H - 1, W, H) == 4


def test_occlusion_detected_when_mask_lags_bbox():
    # bbox 200x200 을 거의 채우는 마스크 → 정상
    assert size_area_consistent(200, 200, 36000) is True
    # 같은 bbox 인데 마스크만 급감 → 핑거가 가린 상황
    assert size_area_consistent(200, 200, 8000) is False
    assert size_area_consistent(0, 0, 100) is False


def test_summarize_suppresses_distance_when_clipped():
    full = summarize(mask_pixels=W * H // 2, cx_px=W / 2, cy_px=H / 2,
                     bbox=(0, 0, W - 1, H - 1), frame_w=W, frame_h=H,
                     f_px=600.0, real_size_m=0.095)
    # 네 변에 닿은 프레임의 겉보기 크기는 '거리' 가 아니라 '잘림' 이다.
    assert full['border_contacts'] == 4
    assert full['distance_m'] is None
    assert full['m_per_px'] is None

    clean = summarize(mask_pixels=30000, cx_px=W / 2, cy_px=H / 2,
                      bbox=(220, 140, 420, 340), frame_w=W, frame_h=H,
                      f_px=600.0, real_size_m=0.095)
    assert clean['distance_m'] is not None
    assert abs(clean['distance_m'] - 600.0 * 0.095 / 200.0) < 1e-9


def test_summarize_measures_distance_from_width_not_height():
    """세로는 비스듬히 내려다본 단축 때문에 f_px 가 2.7배 다르다 — 가로만 거리 신호다."""
    out = summarize(mask_pixels=20000, cx_px=W / 2, cy_px=H / 2,
                    bbox=(170, 190, 470, 290), frame_w=W, frame_h=H,   # 300x100
                    f_px=412.0, real_size_m=0.095)
    assert out['bbox_w'] == 300.0 and out['bbox_h'] == 100.0
    assert abs(out['distance_m'] - 412.0 * 0.095 / 300.0) < 1e-9


def test_summarize_works_before_calibration():
    """f_px 실측 전에도 나머지 지표는 나와야 한다 — 관측을 먼저 시작할 수 있어야 한다."""
    out = summarize(mask_pixels=30000, cx_px=400, cy_px=300,
                    bbox=(220, 140, 420, 340), frame_w=W, frame_h=H)
    assert out['distance_m'] is None
    assert out['fill'] > 0.0
    assert out['u'] == 0.25
    assert out['bbox_h'] == 200.0


def test_pixel_to_meter_scale():
    assert pixel_to_meter_scale(0.30, 600.0) == 0.0005
    assert pixel_to_meter_scale(None, 600.0) is None
    assert pixel_to_meter_scale(0.30, 0.0) is None


# ── 실측 표본 → 임계값 ──────────────────────────


def test_robust_stats_ignores_an_outlier():
    """케이블이 붙은 프레임 한 장이 임계값을 통째로 밀면 안 된다."""
    clean = [100.0, 101.0, 99.0, 100.0, 100.0]
    dirty = clean + [400.0]
    assert robust_stats(clean)[0] == 100.0
    assert robust_stats(dirty)[0] == 100.0        # 중앙값은 안 흔들린다
    assert robust_stats([])[0] is None


def test_suggest_band_reports_separation():
    grasp = [0.42, 0.43, 0.41, 0.42, 0.44]
    empty = [0.02, 0.03, 0.01, 0.02]
    band = suggest_band(grasp, empty)
    assert band['lo'] < 0.42 < band['hi']
    assert band['separated'] is True
    assert band['intruders'] == 0
    assert band['margin'] > 0.3                   # 두 상태가 멀찍이 갈린다


def test_suggest_band_refuses_when_states_overlap():
    """겹치면 '분리 안 됨'이라고 말해야 한다 — 억지 임계값이 제일 위험하다."""
    grasp = [0.40, 0.42, 0.38, 0.41]
    slipped = [0.39, 0.41, 0.40]
    band = suggest_band(grasp, slipped)
    assert band['separated'] is False
    assert band['intruders'] > 0
    assert band['margin'] == 0.0
    # 비교 대상이 아예 없으면 '갈린다'고 주장할 근거도 없다.
    assert suggest_band(grasp)['separated'] is False


def test_fit_distance_curve_recovers_f_px_and_offset():
    """자로 잰 기준점이 렌즈 주점과 다르면 그 차이는 절편으로 빠져야 한다."""
    f_px, size, offset = 412.0, 0.095, 0.030
    samples = [(f_px * size / w + offset, w) for w in (400.0, 300.0, 200.0, 150.0)]
    fit = fit_distance_curve(samples, real_size_m=size)
    assert abs(fit['f_px'] - f_px) < 1e-6
    assert abs(fit['offset_m'] - offset) < 1e-9
    assert max(abs(r) for r in fit['residuals']) < 1e-9


def test_fit_distance_curve_needs_two_distinct_points():
    assert fit_distance_curve([(0.30, 200.0)], 0.095)['f_px'] is None
    assert fit_distance_curve([(0.30, 200.0), (0.40, 200.0)], 0.095)['f_px'] is None


def test_fit_distance_curve_refuses_a_degenerate_fit():
    """겉보기 크기가 거의 같은 점들은 '형태만 멀쩡한 쓰레기'를 만든다.

    실제로 상자를 안 옮긴 채 두 거리를 입력했더니 f_px=47198 / 오프셋 -20.9m 가
    나왔고, 값 자체는 계산되므로 화면에 그대로 찍혔다. plausible 로 막는다.
    """
    fit = fit_distance_curve([(0.250, 212.0), (0.300, 211.5)], 0.095)
    assert fit['f_px'] is not None          # 계산은 된다
    assert fit['plausible'] is False        # 하지만 써서는 안 된다
    assert '벌리' in fit['reason']

    good = fit_distance_curve([(0.250, 400.0), (0.500, 200.0)], 0.095)
    assert good['plausible'] is True and good['reason'] is None
    assert good['width_span'] == 2.0


def test_usable_keeps_the_edge_trim_that_always_happens():
    """실기에서는 케이블이 없어도 가장자리에서 2~3% 는 늘 잘려 나간다.

    절대값(`trimmed_px > 0`)으로 거르면 **쓸 만한 표본이 0개**가 된다 — 2026-08-13
    실기 20프레임이 전부 그랬다. 이 테스트가 그 회귀를 막는다.
    """
    frame = {'detected': True, 'occluded': False, 'border_contacts': 0,
             'mask_px': 15400, 'trimmed_px': 342}          # 실측값 그대로
    assert usable(frame) is True
    # 덩어리의 절반이 잘려 나갔으면 그건 무언가 크게 붙어 있었던 프레임이다.
    assert usable({**frame, 'trimmed_px': 15000}) is False


def test_usable_drops_unmeasurable_frames():
    base = {'detected': True, 'occluded': False, 'border_contacts': 0,
            'mask_px': 15000, 'trimmed_px': 0}
    assert usable({**base, 'detected': False}) is False
    assert usable({**base, 'occluded': True}) is False
    # 두 변 이상 잘린 프레임: 거리 실측에는 못 쓰지만 파지 밴드에서는 정상이다.
    clipped = {**base, 'border_contacts': 3}
    assert usable(clipped) is False
    assert usable(clipped, allow_clipped=True) is True


# ── 파지 검증 (2026-08-14 실측 기준) ──────────────


def test_classify_grasp_uses_the_measured_band():
    """실측값 그대로 고정한다 — 정상 0.0483~0.0511 / 실패 0.0714~0.0724."""
    for fill in (0.0483, 0.0495, 0.0511):
        assert classify_grasp({'detected': True, 'fill': fill}) == 'gripped'
    for fill in (0.0714, 0.0724):
        assert classify_grasp({'detected': True, 'fill': fill}) == 'misgripped'


def test_classify_grasp_never_calls_a_miss_a_failure():
    """미검출은 '실패' 가 아니라 '판정 불가' 다.

    멀쩡히 물고 있는데도 프레임이 비는 일이 실기에서 실제로 있었다(어떤 롤에서 53%).
    미검출을 실패로 읽으면 성공한 파지를 놓고 재시도를 반복한다.
    """
    assert classify_grasp({'detected': False, 'fill': 0.0}) == 'unknown'
    assert classify_grasp({'detected': True}) == 'unknown'


def test_grasp_threshold_sits_between_the_measured_states():
    assert GRASP_REFERENCE['fill'] < GRASP_FILL_MAX < 0.0714
    # 기준 자세의 산포(σ) 대비 임계가 충분히 떨어져 있어야 한다.
    margin_sigma = (GRASP_FILL_MAX - GRASP_REFERENCE['fill']) / GRASP_REFERENCE['fill_sigma']
    assert margin_sigma > 5.0


def test_clamp_correction_preserves_sign():
    assert clamp_correction(0.05, 0.03) == 0.03
    assert clamp_correction(-0.05, 0.03) == -0.03
    assert clamp_correction(0.01, 0.03) == 0.01
    assert clamp_correction(None, 0.03) == 0.0
    assert clamp_correction(float('nan'), 0.03) == 0.0
