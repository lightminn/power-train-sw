"""손목 캠 근접용 색상 마스크 (ROS 비의존, 2026-08-13 추가).

## 왜 YOLO 가 아니라 색인가 — 실측 근거

2026-08-13 손목 캠 실기 확인: 그리퍼에 물린 구조물자 상자(95mm 큐브, 빨강/파랑)가
**conf 0.20 까지 낮춰도 `box-segmentation` 으로 검출되지 않았다.** 같은 프레임의 배경
택배 상자는 0.97 로 잡혔다. 전방 D435i 는 같은 상자를 0.9대로 잡으므로 클래스 문제가
아니라, **근접·잘림·초점이 학습 분포 밖**인 것이다(파지 거리에서 상자는 화면 하단을
가득 채우고 아랫부분이 잘린다).

파지 검증에 필요한 건 "이게 무슨 물체인가"가 아니라 **"아까 그 자리에 그만큼 보이나"**
이므로, 학습 분포에 의존하지 않는 색상 마스크가 이 구간에서는 더 맞다. 부수 효과로
**GPU 를 전혀 쓰지 않아** 전방 캠 추론 대역을 한 톨도 뺏지 않는다.

## ROI 가 필수인 이유

같은 실기 프레임에서 배경 선반의 택배 상자도 파랑/분홍이었다. 색만으로 고르면 배경이
이긴다(실제로 confidence 최고 선택이 배경 상자를 파지 대상으로 보고했다). 그리퍼는
화면에서 **항상 같은 자리**(하단 중앙)에 있으므로 ROI 로 자르는 게 정답이다.

## 실측 HSV (2026-08-13, 손목 캠 640x480, 실내 형광등)

    파랑  H 104~117   S 99~246   V 54~255
    빨강  H 171~178   S 96~197   V 93~206

아래 기본 밴드는 여기에 여유를 준 값이다. **조명이 바뀌면 다시 재야 한다** —
`scripts/` 의 실측 도구가 아니라 이 상수를 고치는 게 단일 출처다.
"""
import cv2
import numpy as np

#: (H_low, S_low, V_low), (H_high, S_high, V_high) — OpenCV HSV(H 0~179).
#: 빨강은 H 가 0 에서 감기므로 두 밴드로 나눈다.
RED_LOW = ((0, 90, 60), (8, 255, 255))
RED_HIGH = ((166, 90, 60), (179, 255, 255))
BLUE = ((98, 90, 50), (128, 255, 255))

#: 그리퍼가 보이는 화면 영역 (x0, x1, y0, y1), 화면 크기에 대한 비율.
#: 손목 캠은 그리퍼를 아래로 내려다보므로 하단 중앙이다 — 카메라를 재장착하면 다시 잡을 것.
DEFAULT_ROI = (0.10, 0.90, 0.45, 1.00)

#: 이보다 작은 덩어리는 케이블·반사광 같은 잡음으로 본다(픽셀).
DEFAULT_MIN_BLOB_PX = 1500

#: 이보다 가는 구조물은 대상이 아니라고 본다(픽셀). **면적이 아니라 두께 기준**이다.
#: 2026-08-13 실측에서 그리퍼 옆 **빨간 케이블이 상자 마스크에 간헐적으로 달라붙어**
#: 가로 픽셀 sd 가 31px(13%)까지 벌어졌다. 케이블은 면적이 작지 않다(길다) — 그래서
#: `min_blob_px` 로는 안 걸리고, 상자와 **붙어 있으면** 같은 연결요소가 돼 더더욱 안 걸린다.
#: 열림 연산(침식→팽창)은 커널보다 가는 것만 골라 지우므로 이 상황의 정확한 도구다.
#: ⚠️ 하강 중 멀리서 상자가 작게 보일 때를 생각해 **상자 최소 폭보다 훨씬 작게** 유지할 것.
DEFAULT_THIN_REJECT_PX = 15

#: 가장자리 열/행의 마스크 두께가 중앙값의 이 비율보다 얇으면 상자가 아니라 붙어 나온
#: 잔가지로 보고 잘라낸다. 열림으로 못 끊은 굵은 부착물의 마지막 방어선이다.
DEFAULT_TRIM_FRAC = 0.25


def roi_rect(frame_w: int, frame_h: int, roi=DEFAULT_ROI) -> tuple:
    """정규화 ROI → 픽셀 사각형 `(x0, y0, x1, y1)`. 항상 프레임 안으로 자른다."""
    x0 = max(0, min(frame_w - 1, int(roi[0] * frame_w)))
    x1 = max(x0 + 1, min(frame_w, int(roi[1] * frame_w)))
    y0 = max(0, min(frame_h - 1, int(roi[2] * frame_h)))
    y1 = max(y0 + 1, min(frame_h, int(roi[3] * frame_h)))
    return (x0, y0, x1, y1)


def color_mask(bgr, red_low=RED_LOW, red_high=RED_HIGH, blue=BLUE):
    """BGR 프레임 → 빨강∪파랑 이진 마스크(uint8 0/255).

    형태학 연산으로 검정 밴드(상자를 가로지르는 띠)가 만든 구멍을 메우고 작은 점을 턴다.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(red_low[0], np.uint8), np.array(red_low[1], np.uint8))
    mask |= cv2.inRange(hsv, np.array(red_high[0], np.uint8), np.array(red_high[1], np.uint8))
    mask |= cv2.inRange(hsv, np.array(blue[0], np.uint8), np.array(blue[1], np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def strip_thin_structures(mask, thin_px=DEFAULT_THIN_REJECT_PX):
    """커널보다 **가는** 구조물을 지운다(열림 연산). 케이블 제거의 본체.

    상자는 어느 방향으로도 두꺼워 열림에 살아남고, 케이블은 두께가 커널보다 얇아 침식
    단계에서 통째로 사라진다 — 팽창 단계에서 상자만 원래 크기로 되돌아온다. 사각형은
    모서리만 둥글어질 뿐 **bbox 폭·높이가 보존**되므로 거리 신호를 해치지 않는다.

    ⚠️ 색으로는 케이블과 상자를 못 가른다(같은 빨강이다). 가를 수 있는 건 **모양**뿐이고,
    그중에서도 두께가 가장 안정적인 근거다 — 케이블은 구부러져도 굵어지지 않는다.
    """
    size = int(thin_px)
    if size <= 1:
        return mask
    size |= 1                                     # 커널은 홀수여야 중심이 잡힌다
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def _core_span(profile, trim_frac):
    """1차원 두께 프로파일 → 몸통이 차지하는 연속 구간 `(lo, hi)` (hi 는 배타적).

    가장 두꺼운 지점에서 좌우로 뻗어 나가다 **몸통 두께의 `trim_frac` 배**보다 얇아지면
    멈춘다.

    ⚠️ 기준을 중앙값으로 잡으면 안 된다 — 잔가지가 상자보다 **길면**(열 개수가 많으면)
    중앙값 자체가 잔가지 두께로 끌려가 문턱이 무너진다(실제로 그렇게 짜서 24px 부착물이
    통과했다). 최댓값은 반대로 한 열짜리 튐에 흔들린다. 그래서 **90 백분위수**를 쓴다 —
    잔가지가 열의 절반을 넘어도 상자 열이 상위 10% 안에만 있으면 버틴다.
    """
    strong = profile[profile > 0]
    if strong.size == 0:
        return None
    threshold = max(1.0, float(trim_frac) * float(np.percentile(strong, 90)))
    seed = int(np.argmax(profile))
    lo = seed
    while lo - 1 >= 0 and profile[lo - 1] >= threshold:
        lo -= 1
    hi = seed
    while hi + 1 < profile.size and profile[hi + 1] >= threshold:
        hi += 1
    return lo, hi + 1


def largest_blob(mask, rect=None, min_area=DEFAULT_MIN_BLOB_PX,
                 thin_px=DEFAULT_THIN_REJECT_PX, trim_frac=DEFAULT_TRIM_FRAC):
    """ROI 안에서 가장 큰 연결 덩어리. 없으면 `None`.

    ⚠️ **ROI 밖은 아예 지운다** — 배경의 같은 색 상자가 이기는 것을 막는 게 이 함수의
    존재 이유다. 반환 좌표는 항상 **전체 프레임 기준**이다(호출부가 다시 더할 일 없게).

    ROI 통과 뒤 두 단계로 잔가지를 턴다: (1) 열림으로 가는 구조물 제거,
    (2) 남은 덩어리의 가장자리에서 얇은 열·행 잘라내기. 둘 다 **모양** 기준이라
    상자와 같은 색인 케이블에도 듣는다.
    """
    if rect is not None:
        x0, y0, x1, y1 = rect
        gated = np.zeros_like(mask)
        gated[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
        mask = gated

    mask = strip_thin_structures(mask, thin_px)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    component = (labels == index)

    cols = _core_span(component.sum(axis=0), trim_frac)
    rows = _core_span(component.sum(axis=1), trim_frac)
    if cols is None or rows is None:
        return None
    core = np.zeros_like(component)
    core[rows[0]:rows[1], cols[0]:cols[1]] = component[rows[0]:rows[1], cols[0]:cols[1]]

    area = int(core.sum())
    if area < int(min_area):
        return None
    ys, xs = np.nonzero(core)
    return {
        'area': area,
        'centroid': (float(xs.mean()), float(ys.mean())),
        'bbox': (float(xs.min()), float(ys.min()),
                 float(xs.max() + 1), float(ys.max() + 1)),
        # 잘라낸 양 — 0 이 아니면 무언가가 상자에 붙어 있었다는 뜻이다. 실측 스크립트가
        # "이 프레임은 케이블이 붙었었다"를 사후에 구분하는 근거로 쓴다.
        'trimmed_px': int(component.sum()) - area,
    }


def find_box(bgr, roi=DEFAULT_ROI, min_area=DEFAULT_MIN_BLOB_PX,
             thin_px=DEFAULT_THIN_REJECT_PX, trim_frac=DEFAULT_TRIM_FRAC, **bands):
    """프레임 한 장 → 파지 대상 후보 하나(없으면 `None`) + 마스크.

    반환: `(blob_or_None, mask)` — 마스크는 **ROI·가는구조물 제거를 거친 뒤**의 것이라
    디버그 화면이 곧 '판정 근거'가 된다(원본 마스크를 보여주면 왜 이 bbox 가 나왔는지
    화면에서 설명이 안 된다).
    """
    h, w = bgr.shape[:2]
    rect = roi_rect(w, h, roi)
    mask = color_mask(bgr, **bands)
    x0, y0, x1, y1 = rect
    gated = np.zeros_like(mask)
    gated[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
    gated = strip_thin_structures(gated, thin_px)
    # 이미 게이트·열림을 거친 마스크라 largest_blob 안에서 한 번 더 해도 결과는 같다
    # (열림은 멱등, ROI 는 이미 반영) — 두 함수를 각각 단독으로도 쓸 수 있게 둔 것이다.
    blob = largest_blob(gated, rect, min_area, thin_px, trim_frac)
    return blob, gated
