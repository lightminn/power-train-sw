"""손목 카메라 지표 계산 (ROS 비의존, 2026-08-13 추가).

## 무엇을 재고, 왜 이 지표인가

손목 캠은 **그리퍼에 고정**돼 있다. 그래서 "박스가 화면에 보이는 정도"가 곧
"그리퍼와 박스의 상대 자세"다. 여기서 뽑는 값은 둘:

1. **근접도** — 겉보기 크기. 카메라가 박스에 가까워질수록 화면에서 커진다.
   depth 가 없어도 크기↔거리는 단조 관계라 하강 종료 시점을 판단할 수 있다.
2. **파지 검증** — 제대로 물었으면 박스는 **항상 같은 자리, 같은 크기**로 보인다.

⚠️ **파지 검증에 bbox 폭/높이·PCA 주축각을 쓰면 안 된다.** `arm_joint_5`(손목 롤)
축이 카메라→그리퍼 시선과 거의 나란해서(URDF 고정관절 체인 계산: 카메라→tip 101mm,
평면 밖 성분 14mm), 물린 박스는 손목 롤에 대해 **이미지 평면 안에서 제자리 회전**을
한다. 정사각형이 회전하면 축정렬 bbox 면적은 최대 √2배까지 변한다 — 회전 불변인
**마스크 픽셀 수(fill)와 중심(centroid)** 만이 파지 검증에 쓸 수 있는 양이다.
반대로 근접 판정(하강 중)은 회전 교란이 없는 구간이라 bbox 를 써도 된다 —
단 **가로 폭만** 쓴다(아래).

⚠️ **거리 신호는 bbox 가로 폭 하나뿐이다.** 2026-08-13 실측에서 같은 프레임의 같은
상자로 뽑은 `f_px` 가 **가로 412 / 세로 150** 으로 2.7배 어긋났다. 손목 캠이 상자를
비스듬히 내려다봐 세로만 단축되기 때문이고, 카메라 자세가 그대로인 한 이 비율도
그대로다. 세로를 쓰면 거리가 2.7배 가깝게 나온다 — 하강을 너무 일찍 멈추는 쪽이다.

## ROS 비의존인 이유

이 저장소는 판정 로직을 순수 함수로 떼어 하드웨어 없이 pytest 로 고정하는 관례를
쓴다(`perception_quality.py`·`contract.py`·`robot_arm_gui/state_store.py`). 여기 있는
함수는 전부 숫자만 받고 숫자만 돌려주므로 카메라도 ROS 도 없이 검증된다.

## 좌표 규약

중심 좌표는 **중앙이 0, 화면 가장자리가 ±1** 인 정규화 좌표다(u=우측+, v=아래+,
OpenCV 픽셀 축과 같은 방향). 해상도를 바꿔도 실측 임계값이 그대로 살아남고, 부호가
그대로 "어느 쪽으로 어긋났나"를 뜻해 보정량 계산에 바로 들어간다.
"""
import math

#: 겉보기 크기가 이보다 작으면 거리 환산을 신뢰하지 않는다(픽셀 양자화 오차가 커진다).
MIN_APPARENT_PX = 8.0

#: 마스크가 bbox 를 채우는 비율이 이보다 낮으면 무언가가 박스를 가리고 있다고 본다.
#: 정면에서 본 상자는 bbox 를 거의 다 채운다 — 그리퍼 핑거가 끼어들면 여기가 먼저 준다.
DEFAULT_MIN_MASK_FILL_OF_BBOX = 0.55


def mask_fill_ratio(mask_pixels: int, frame_w: int, frame_h: int) -> float:
    """마스크 픽셀 수 → 화면 점유율(0~1). 손목 롤에 대해 불변인 값이다."""
    area = float(frame_w) * float(frame_h)
    if area <= 0.0:
        return 0.0
    return max(0.0, min(1.0, float(mask_pixels) / area))


def normalized_centroid(cx_px: float, cy_px: float,
                        frame_w: int, frame_h: int) -> tuple:
    """픽셀 중심 → 중앙 기준 정규화 좌표 `(u, v)`, 각각 -1~1.

    화면 정중앙이 `(0, 0)`, 오른쪽 끝이 `u=+1`, 아래쪽 끝이 `v=+1`.
    """
    if frame_w <= 0 or frame_h <= 0:
        return (0.0, 0.0)
    u = (float(cx_px) - frame_w / 2.0) / (frame_w / 2.0)
    v = (float(cy_px) - frame_h / 2.0) / (frame_h / 2.0)
    return (max(-1.0, min(1.0, u)), max(-1.0, min(1.0, v)))


def apparent_size_to_distance_m(size_px: float, f_px: float,
                                real_size_m: float) -> float:
    """겉보기 크기(px) → 거리(m). 핀홀 모델 `d = f_px * 실치수 / 픽셀치수`.

    `f_px` 는 체스보드 캘리브 없이 1회 실측으로 얻는다 — 알려진 거리에 알려진 크기의
    물체를 두고 `f_px = 픽셀치수 * 거리 / 실치수`. 값을 신뢰할 수 없는 입력이면
    `None` 을 돌려준다(0 이나 음수로 답해서 하류가 그걸 유효한 거리로 쓰는 게 최악이다).
    """
    if size_px is None or size_px < MIN_APPARENT_PX:
        return None
    if f_px <= 0.0 or real_size_m <= 0.0:
        return None
    return float(f_px) * float(real_size_m) / float(size_px)


def distance_to_apparent_size_px(distance_m: float, f_px: float,
                                 real_size_m: float) -> float:
    """위 식의 역함수 — 실측 곡선을 검산하거나 임계 픽셀값을 뽑을 때 쓴다."""
    if distance_m is None or distance_m <= 0.0:
        return None
    if f_px <= 0.0 or real_size_m <= 0.0:
        return None
    return float(f_px) * float(real_size_m) / float(distance_m)


def border_contacts(x1: float, y1: float, x2: float, y2: float,
                    frame_w: int, frame_h: int, margin: int = 2) -> int:
    """bbox 가 화면 네 변 중 몇 곳에 닿았는지.

    ⚠️ 전방 캠용 `perception_quality.is_usable_bbox` 는 접촉이 3면 이상이면 검출을
    **버리는데**, 손목 캠에서는 파지 직전에 박스가 화면을 가득 채워 4면에 닿는 게
    **정상**이다. 그래서 여기서는 버리지 않고 개수만 세어, 겉보기 크기가 '거리 때문에'
    작아진 건지 '잘려서' 작아진 건지 구분하는 근거로만 쓴다.
    """
    touched = 0
    if x1 <= margin:
        touched += 1
    if y1 <= margin:
        touched += 1
    if x2 >= frame_w - 1 - margin:
        touched += 1
    if y2 >= frame_h - 1 - margin:
        touched += 1
    return touched


def size_area_consistent(bbox_w_px: float, bbox_h_px: float, mask_pixels: int,
                         min_fill_of_bbox: float = DEFAULT_MIN_MASK_FILL_OF_BBOX) -> bool:
    """겉보기 크기와 마스크 면적이 서로 말이 되는가 — 가림(occlusion) 검출.

    그리퍼 핑거가 박스를 가리면 **마스크 면적만 줄고 bbox 는 그대로**다. 두 값이
    어긋나는 순간을 잡아 그 프레임의 측정을 통째로 버리기 위한 게이트다.
    """
    box_area = float(bbox_w_px) * float(bbox_h_px)
    if box_area <= 0.0:
        return False
    return (float(mask_pixels) / box_area) >= float(min_fill_of_bbox)


def pixel_to_meter_scale(distance_m: float, f_px: float) -> float:
    """그 거리에서 1픽셀이 몇 미터인가 — 중심 오차를 보정량으로 바꿀 때 쓴다.

    ⚠️ 이 스케일이 성립하는 건 광축에 수직한 평면 위에서다. 비스듬히 놓인 면에는
    그대로 적용되지 않는다.
    """
    if distance_m is None or distance_m <= 0.0 or f_px <= 0.0:
        return None
    return float(distance_m) / float(f_px)


def summarize(mask_pixels: int, cx_px: float, cy_px: float,
              bbox: tuple, frame_w: int, frame_h: int,
              f_px: float = 0.0, real_size_m: float = 0.0) -> dict:
    """한 검출 → 지표 한 벌. 노드는 이 dict 를 그대로 직렬화해 발행한다.

    `f_px`/`real_size_m` 이 아직 실측되지 않았으면(기본 0) 거리 항목만 `None` 이 되고
    나머지 지표는 그대로 나온다 — 캘리브 전에도 관측을 시작할 수 있어야 한다.
    """
    x1, y1, x2, y2 = bbox
    bbox_w = max(0.0, float(x2) - float(x1))
    bbox_h = max(0.0, float(y2) - float(y1))
    u, v = normalized_centroid(cx_px, cy_px, frame_w, frame_h)
    touched = border_contacts(x1, y1, x2, y2, frame_w, frame_h)
    # ⚠️ **거리는 가로(bbox_w)로만 낸다.** 2026-08-13 실측에서 같은 상자·같은 프레임으로
    # 뽑은 f_px 가 가로 412 / 세로 150 으로 2.7배 어긋났다 — 손목 캠이 상자 윗면을
    # 비스듬히 내려다봐 **세로만 단축(foreshortening)** 되기 때문이다. 세로를 쓰면 거리가
    # 2.7배 가깝게 나오고, 그 오차가 하강 종료 판단에 그대로 들어간다.
    distance = apparent_size_to_distance_m(bbox_w, f_px, real_size_m)
    return {
        'fill': mask_fill_ratio(mask_pixels, frame_w, frame_h),
        'u': u,
        'v': v,
        'bbox_w': bbox_w,
        'bbox_h': bbox_h,
        'mask_px': int(mask_pixels),
        'border_contacts': touched,
        # 잘린 프레임의 겉보기 크기는 거리 신호가 아니다 — 거리를 아예 내지 않는다.
        'distance_m': None if touched >= 2 else distance,
        'occluded': not size_area_consistent(bbox_w, bbox_h, mask_pixels),
        'm_per_px': pixel_to_meter_scale(distance, f_px) if touched < 2 else None,
    }


# ── 실측 표본 → 임계값 (measure_wrist_*.py 가 쓰는 계산부) ──────────────
#
# 스크립트가 아니라 여기 두는 이유: 임계값을 **어떻게 뽑았는가**가 임계값 자체만큼
# 중요한데, 스크립트 안에 있으면 아무도 검증하지 않는다. 이 저장소는 판정 로직을 순수
# 함수로 떼어 pytest 로 고정하는 관례를 쓴다.

#: MAD → 표준편차 환산 계수(정규분포 가정). 평균·표준편차 대신 중앙값·MAD 를 쓰는 건
#: 실기 표본에 **케이블이 붙은 프레임 같은 이상치가 반드시 섞이기** 때문이다.
MAD_TO_SIGMA = 1.4826

#: 거리 곡선을 풀려면 점들의 겉보기 크기가 최소 이 배수만큼은 벌어져야 한다.
#: 20% 는 파지 구간(~210px)에서 40px 차이 — 실측 σ(1.5px)의 25배라 잡음에 안 묻힌다.
MIN_WIDTH_SPAN = 1.2

#: 자로 잰 기준점과 렌즈 주점의 차이가 이걸 넘으면 계산이 아니라 입력을 의심한다.
MAX_PLAUSIBLE_OFFSET_M = 0.3


def robust_stats(values) -> tuple:
    """표본 → `(중앙값, MAD 기반 σ)`. 표본이 없으면 `(None, None)`."""
    data = sorted(float(v) for v in values if v is not None)
    if not data:
        return (None, None)
    n = len(data)
    median = data[n // 2] if n % 2 else 0.5 * (data[n // 2 - 1] + data[n // 2])
    deviations = sorted(abs(v - median) for v in data)
    mad = (deviations[n // 2] if n % 2
           else 0.5 * (deviations[n // 2 - 1] + deviations[n // 2]))
    return (median, mad * MAD_TO_SIGMA)


def suggest_band(good, other=(), k: float = 3.0, min_sigma: float = 0.0) -> dict:
    """정상 표본 → 판정 밴드 `[lo, hi]`, 그리고 **다른 상태와 갈리는지**.

    `separated=False` 면 그 지표로는 두 상태를 구분할 수 없다는 뜻이다 — 이 경우
    임계값을 억지로 정하지 말고 **관측 조건을 바꾸는 게** 맞다. FSM 결합의 전제가
    이 값이므로, 판단을 사람에게 넘기려고 `intruders`(밴드 안에 들어온 다른 상태 표본
    개수)와 `margin`(밴드 경계와 가장 가까운 다른 상태 표본의 거리)까지 같이 낸다.
    """
    center, sigma = robust_stats(good)
    if center is None:
        return {'center': None, 'lo': None, 'hi': None, 'sigma': None,
                'n_good': 0, 'n_other': len(list(other)),
                'intruders': 0, 'margin': None, 'separated': False}
    sigma = max(float(sigma), float(min_sigma))
    lo, hi = center - k * sigma, center + k * sigma
    others = [float(v) for v in other if v is not None]
    intruders = sum(1 for v in others if lo <= v <= hi)
    margin = min((min(abs(v - lo), abs(v - hi)) for v in others), default=None)
    return {
        'center': center, 'lo': lo, 'hi': hi, 'sigma': sigma,
        'n_good': len(list(good)), 'n_other': len(others),
        'intruders': intruders,
        'margin': 0.0 if intruders else margin,
        'separated': bool(others) and intruders == 0,
    }


def fit_distance_curve(samples, real_size_m: float = 0.0) -> dict:
    """`[(거리 m, 겉보기 가로폭 px)]` → `f_px` 와 **측정 기준점 오프셋**.

    핀홀 모델은 `d = f_px * S / w` 지만, 실기에서 자가 재는 거리는 렌즈 주점이 아니라
    **케이스 앞면 같은 아무 데나**에서 시작한다. 그 차이는 `d = a/w + b` 의 `b` 로
    빠지므로, 점 하나로 `f_px = w*d/S` 를 내면 `b` 가 통째로 `f_px` 오차가 된다
    (가까울수록 크게 틀린다 — 하필 파지 구간이다). 그래서 여러 점을 1/w 에 대한
    직선으로 맞춘다: 기울기 `a = f_px * S`, 절편 `b` = 기준점 오프셋(m).

    반환의 `residuals` 가 크면 모델이 아니라 **표본**을 의심할 것(잘린 프레임·케이블).

    ⚠️ **`plausible` 을 확인하지 않고 `f_px` 를 쓰면 안 된다.** 겉보기 크기가 거의 같은
    점들만 주면 직선의 기울기가 잡음으로 결정돼 **말이 되는 형태의 쓰레기 값**이 나온다
    (실제로 같은 자리에서 두 점을 재 봤더니 `f_px=47198`, 오프셋 `-20.9m` 이 나왔고,
    스크립트는 그걸 그대로 '적용하세요'라고 출력했다). 거리 범위를 충분히 벌리는 것이
    이 실측의 유일한 조건이다.
    """
    points = [(float(d), float(w)) for d, w in samples if d and w and w > 0.0]
    empty = {'slope': None, 'offset_m': None, 'f_px': None, 'residuals': [],
             'n': len(points), 'width_span': None,
             'plausible': False, 'reason': '점이 2개 미만입니다'}
    if len(points) < 2:
        return empty
    xs = [1.0 / w for _, w in points]
    ys = [d for d, _ in points]
    n = float(len(points))
    mean_x, mean_y = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx <= 0.0:                       # 전부 같은 겉보기 크기 — 기울기를 못 푼다
        return empty
    a = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sxx
    b = mean_y - a * mean_x
    widths = [w for _, w in points]
    span = max(widths) / min(widths)
    reason = None
    if span < MIN_WIDTH_SPAN:
        reason = (f'점들의 겉보기 크기 차이가 {(span - 1) * 100:.0f}% 뿐입니다 '
                  f'(최소 {(MIN_WIDTH_SPAN - 1) * 100:.0f}%) — 거리 범위를 더 벌리세요')
    elif abs(b) > MAX_PLAUSIBLE_OFFSET_M:
        reason = (f'기준점 오프셋이 {b * 1000:.0f}mm 로 비현실적입니다 — '
                  '거리를 잘못 입력했거나 표본이 흔들렸습니다')
    elif a <= 0.0:
        reason = '거리와 겉보기 크기가 반비례하지 않습니다 — 표본이 뒤섞였습니다'
    return {
        'slope': a,
        'offset_m': b,
        'f_px': (a / float(real_size_m)) if real_size_m else None,
        'residuals': [y - (a * x + b) for x, y in zip(xs, ys)],
        'n': len(points),
        'width_span': span,
        'plausible': reason is None,
        'reason': reason,
    }


# ── 파지 검증 기준 (2026-08-14 실기 확정) ──────────────
#
# 실측 절차와 원본 수치는 `scripts/measure_wrist_grasp_band.py` 를 쓴 세션 기록에 있다.
# 여기 상수는 그 결과의 단일 출처다 — FSM 2단계가 이 값을 그대로 쓴다.

#: **파지 시점의 손목 롤은 이 관절각으로 고정한다.** 선택이 아니라 전제다.
#:
#: ⚠️ 롤이 바뀌면 그리퍼 몸체가 큐브를 가리는 양이 달라져 `fill` 이 **2.5배까지** 변한다
#: (실측 4개 롤: 0.0203 / 0.0268 / 0.0345 / 0.0495). 심지어 어떤 롤에서는 큐브가 조
#: 안쪽으로 깊이 들어가 **검출률이 53% 로 떨어진다** — 멀쩡히 물고 있는데 절반의
#: 프레임이 "아무것도 안 보인다"고 답한다. 롤을 고정하지 않으면 정상 범위가 실패값
#: (0.0714)까지 넓어져 판정 자체가 성립하지 않는다.
#:
#: 롤 고정은 거리 신호도 같이 구한다 — 가려진 롤에서는 실루엣 가로폭이 줄어 실제 160mm 를
#: **220mm 로** 보고했다. 이 기준 자세에서는 155mm 로 실제와 맞는다.
GRASP_ROLL_JOINT = 'arm_joint_5'
GRASP_ROLL_RAD = 0.0

#: 기준 자세에서 정상 파지의 서명 (40프레임 실측, 검출률 100%).
#: ⚠️ `u`/`v`/`bbox_w` 는 **게이트로 쓰면 안 된다** — 실패 표본 60/60 이 정상 밴드 안에
#: 들어와 전혀 갈리지 않았다. 진단·기록용으로만 남긴다.
GRASP_REFERENCE = {
    'fill': 0.0495, 'fill_sigma': 0.0009,
    'u': -0.055, 'v': 0.556, 'bbox_w': 221.0, 'distance_m': 0.155,
}

#: `fill` 이 이 값 이상이면 **제대로 물리지 않은 것**이다.
#:
#: 근거가 되는 물리는 단순하다 — 제대로 물리면 큐브가 조 안쪽에 들어가 그리퍼 몸체에
#: 가려지고, 실패하면 밖으로 나와 온전히 보인다. 그래서 **실패 쪽이 오히려 크게 보인다.**
#: 실측: 정상 0.0483~0.0511 / 실패 2종(모서리만 물림·빠져나옴) 0.0714·0.0724.
#: 임계는 그 사이 중앙이고, 정상 상단에서 13σ·실패 하단에서 22σ 떨어져 있다.
GRASP_FILL_MAX = 0.06


def classify_grasp(sample, fill_max: float = GRASP_FILL_MAX) -> str:
    """한 프레임 지표 → `'gripped'` | `'misgripped'` | `'unknown'`.

    ⚠️ **미검출을 실패로 판정하지 않는다.** 기준 자세에서는 검출률이 100% 였지만,
    조명이나 자세가 조금만 틀어져도 프레임이 비는 일이 실제로 있었다(어떤 롤에서 53%).
    미검출을 실패로 읽으면 멀쩡한 파지를 놓고 재시도를 반복하게 되므로, 판정 불가는
    판정 불가로 돌려주고 **여러 프레임을 모아 결정하는 건 호출부(FSM)의 몫**으로 둔다.
    """
    if not sample.get('detected'):
        return 'unknown'
    fill = sample.get('fill')
    if fill is None:
        return 'unknown'
    return 'misgripped' if float(fill) >= float(fill_max) else 'gripped'


# ── 실측 표본 채택 필터 ────────────────────────────
#
# 스크립트가 아니라 여기 있는 이유: 이 필터가 **표본을 버리는** 쪽이라
# 틀리면 조용히 '쓸 표본 0개'가 된다(실제로 한 번 그렇게 짰다, 아래 주석 참고).
# 버리는 규칙일수록 테스트로 고정해야 한다.

#: 잘라낸 픽셀이 덩어리의 이 비율을 넘으면 "무언가 크게 붙어 있었다"고 본다.
#: ⚠️ **절대값(`trimmed_px > 0`)으로 판정하면 안 된다** — 2026-08-13 실기 20프레임에서
#: 케이블이 없는데도 전 프레임이 139~514px(덩어리의 2~3%)를 잘라냈다. 상자 가장자리가
#: 픽셀 단위로 깔끔하지 않아 마지막 한두 열이 늘 얇게 잡히기 때문이다. 절대값으로
#: 거르면 **쓸 만한 표본이 0개**가 된다(실제로 그렇게 짰다가 걸렸다).
DEFAULT_MAX_TRIM_RATIO = 0.10


def usable(sample, allow_clipped=False, max_trim_ratio=DEFAULT_MAX_TRIM_RATIO):
    """거리·밴드 실측에 쓸 수 있는 표본인가.

    - 미검출: 제외(단, 파지 밴드 실측의 '빈 그리퍼' 조건은 미검출 자체가 신호이므로
      그쪽은 이 함수를 거치지 않고 따로 센다).
    - `occluded`: 그리퍼 핑거가 상자를 가린 프레임 — 마스크 면적만 줄어 지표가 거짓말한다.
    - `border_contacts >= 2`: 잘린 프레임. 겉보기 크기가 거리 신호가 아니게 된다.
      (파지 거리에서는 정상이므로 밴드 실측은 `allow_clipped=True` 로 부른다.)
    - 잘라낸 비율이 큰 프레임: 케이블 같은 게 붙었다 떨어져 나간 것이라 캘리브에 안 쓴다.
      운용 중에는 그대로 써도 된다 — 잘라낸 값이 맞는 값이다.
    """
    if not sample.get('detected'):
        return False
    if sample.get('occluded'):
        return False
    if not allow_clipped and int(sample.get('border_contacts', 0)) >= 2:
        return False
    trimmed = float(sample.get('trimmed_px', 0))
    total = float(sample.get('mask_px', 0)) + trimmed
    if total > 0.0 and (trimmed / total) > float(max_trim_ratio):
        return False
    return True


def clamp_correction(value_m: float, limit_m: float) -> float:
    """보정량을 한계 안으로 자른다. 3단계(능동 보정)에서 쓸 안전 클램프.

    한계를 파라미터가 아니라 상수로 박으면 실기에서 못 줄인다 — 호출부가 파라미터로
    받은 값을 넘기게 하고, 여기서는 부호를 보존한 채 크기만 자른다.
    """
    if value_m is None or math.isnan(value_m):
        return 0.0
    limit = abs(float(limit_m))
    return max(-limit, min(limit, float(value_m)))
