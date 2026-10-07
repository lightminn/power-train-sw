"""`wrist_color_mask` 유닛테스트 — 합성 프레임으로 돌아 카메라가 필요 없다."""
import numpy as np

from robot_arm_perception.wrist_color_mask import (
    DEFAULT_ROI,
    color_mask,
    find_box,
    largest_blob,
    roi_rect,
    strip_thin_structures,
)

W, H = 640, 480

BLUE_BGR = (200, 60, 20)     # 실측 H≈111 대역
RED_BGR = (40, 40, 200)      # 실측 H≈175 대역


def _frame(patches):
    """`[(x0, y0, x1, y1, color)]` 를 그린 검은 프레임."""
    img = np.zeros((H, W, 3), np.uint8)
    for x0, y0, x1, y1, color in patches:
        img[y0:y1, x0:x1] = color
    return img


def test_roi_rect_is_clamped_to_frame():
    assert roi_rect(W, H, (0.0, 1.0, 0.0, 1.0)) == (0, 0, W, H)
    x0, y0, x1, y1 = roi_rect(W, H, DEFAULT_ROI)
    assert 0 <= x0 < x1 <= W and 0 <= y0 < y1 <= H
    # 말도 안 되는 값이 들어와도 프레임 밖으로 나가지 않는다.
    x0, y0, x1, y1 = roi_rect(W, H, (-5.0, 9.0, -1.0, 4.0))
    assert (x0, y0, x1, y1) == (0, 0, W, H)


def test_color_mask_catches_both_bands():
    img = _frame([(100, 300, 200, 400, BLUE_BGR), (300, 300, 400, 400, RED_BGR)])
    mask = color_mask(img)
    assert mask[350, 150] > 0        # 파랑
    assert mask[350, 350] > 0        # 빨강
    assert mask[100, 100] == 0       # 검은 배경


def test_largest_blob_ignores_outside_roi():
    """배경(상단)의 더 큰 같은 색 물체가 있어도 ROI 안의 것이 선택돼야 한다."""
    img = _frame([
        (50, 20, 400, 220, BLUE_BGR),      # 배경: 훨씬 큼, ROI 밖(상단)
        (260, 380, 380, 460, RED_BGR),     # 그리퍼에 물린 상자: ROI 안(하단 중앙)
    ])
    blob, _ = find_box(img)
    assert blob is not None
    cx, cy = blob['centroid']
    assert 250 < cx < 390 and 370 < cy < 470      # 하단 것을 골랐다
    assert blob['area'] < 350 * 200               # 배경 덩어리가 아니다


def test_small_blobs_are_rejected_as_noise():
    img = _frame([(300, 400, 310, 410, RED_BGR)])   # 10x10 = 100px
    blob, _ = find_box(img)
    assert blob is None


def test_returns_none_on_empty_frame():
    blob, mask = find_box(np.zeros((H, W, 3), np.uint8))
    assert blob is None
    assert int(mask.sum()) == 0


def test_blob_coordinates_are_full_frame():
    """ROI 로 잘라도 좌표는 전체 프레임 기준이어야 한다(호출부가 다시 더하지 않게)."""
    img = _frame([(260, 380, 380, 460, BLUE_BGR)])
    blob, _ = find_box(img)
    x1, y1, x2, y2 = blob['bbox']
    assert 250 <= x1 <= 270 and 370 <= y1 <= 390
    assert 370 <= x2 <= 390 and 450 <= y2 <= 470


# ── 케이블 배제 (2026-08-13 실측 대응) ──────────────
#
# 그리퍼 옆 빨간 케이블이 상자 마스크에 간헐적으로 달라붙어 가로 픽셀 sd 가 31px(13%)
# 까지 벌어졌다. 케이블은 상자와 **같은 색**이라 색으로는 못 가른다 — 아래 테스트는
# 모양(두께) 기준 제거가 실제로 bbox 를 안정시키는지를 고정한다.

BOX = (250, 330, 390, 470)              # ROI 안, 140x140 (파지 거리의 대략적 크기)


def _box_frame(extra=()):
    return _frame([(BOX[0], BOX[1], BOX[2], BOX[3], BLUE_BGR)] + list(extra))


def test_thin_cable_does_not_widen_bbox():
    """상자 옆에 붙은 8px 두께 케이블이 bbox 를 넓히면 안 된다."""
    clean, _ = find_box(_box_frame())
    # 상자 오른쪽 변에서 화면 끝까지 뻗은 가는 빨간 줄 = 케이블
    cabled, _ = find_box(_box_frame([(BOX[2], 395, 639, 403, RED_BGR)]))
    assert clean is not None and cabled is not None
    clean_w = clean['bbox'][2] - clean['bbox'][0]
    cabled_w = cabled['bbox'][2] - cabled['bbox'][0]
    assert abs(cabled_w - clean_w) <= 4          # 실측 sd 31px 를 한 자리로 끌어내린다
    # 면적도 같이 안정돼야 한다 — fill 이 파지 검증의 주 지표라 여기가 흔들리면 소용없다.
    assert abs(cabled['area'] - clean['area']) < 0.02 * clean['area']


def test_thick_appendage_is_trimmed_at_the_edge():
    """열림으로 못 끊을 만큼 굵은(24px) 부착물은 가장자리 잘라내기가 막는다."""
    blob, _ = find_box(_box_frame([(BOX[2], 390, 620, 414, RED_BGR)]))
    assert blob is not None
    assert blob['bbox'][2] - blob['bbox'][0] <= (BOX[2] - BOX[0]) + 8
    assert blob['trimmed_px'] > 0                # 잘라냈다는 사실이 기록으로 남는다


def test_box_itself_survives_thin_rejection():
    """잔가지를 터는 연산이 상자 자체를 갉아먹으면 거리 신호가 통째로 틀어진다."""
    blob, _ = find_box(_box_frame())
    x1, y1, x2, y2 = blob['bbox']
    assert abs((x2 - x1) - (BOX[2] - BOX[0])) <= 2
    assert abs((y2 - y1) - (BOX[3] - BOX[1])) <= 2
    assert blob['trimmed_px'] == 0


def test_strip_thin_structures_removes_only_thin_things():
    img = _frame([(250, 330, 390, 470, BLUE_BGR), (100, 400, 240, 408, RED_BGR)])
    stripped = strip_thin_structures(color_mask(img))
    assert stripped[400, 300] > 0                # 상자는 남는다
    assert stripped[404, 170] == 0               # 8px 줄은 사라진다


def test_thin_rejection_can_be_disabled():
    """멀리서 상자가 아주 작게 보이는 구간을 위해 끌 수 있어야 한다."""
    img = _frame([(300, 400, 340, 440, BLUE_BGR)])       # 40x40 = 1600px
    assert find_box(img, min_area=1000, thin_px=61)[0] is None
    assert find_box(img, min_area=1000, thin_px=0)[0] is not None


def test_largest_blob_without_roi_takes_the_biggest():
    img = _frame([
        (50, 20, 400, 220, BLUE_BGR),
        (260, 380, 380, 460, RED_BGR),
    ])
    blob = largest_blob(color_mask(img), rect=None)
    assert blob is not None
    assert blob['centroid'][1] < 240        # ROI 를 안 걸면 큰 배경이 이긴다
