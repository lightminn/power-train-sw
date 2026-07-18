"""deproject_box — 전체 프레임 정렬(rs.align) 없이 검출 지점만 depth 에 대응시키는
경로를 검증. 박스 중앙 패치의 유효 depth 중앙값을 쓰므로, 저시야/연막으로 패치가
전부 무효(0)여도 예외 없이 None 을 반환해 "측정불가"로 넘어가야 한다.
"""
import numpy as np
import pytest

import yolo_depth_3d as m
from yolo_depth_3d import DepthCal, deproject_box


class FakeDepthFrame:
    def get_data(self):
        return None  # 실 SDK 호출은 monkeypatch 로 대체되므로 내용은 안 씀


def make_cal(scale: float = 0.001) -> DepthCal:
    """profile 없이 deproject_box 가 실제로 읽는 필드만 채운 DepthCal."""
    cal = DepthCal.__new__(DepthCal)
    cal.di = object()
    cal.ci = object()
    cal.c2d = object()
    cal.d2c = object()
    cal.scale = scale
    cal.dmin, cal.dmax = 0.1, 10.0
    return cal


@pytest.fixture
def cal():
    return make_cal()


def test_projection_outside_depth_frame_returns_none(monkeypatch, cal):
    """연막/반사 등으로 투영이 화면 밖으로 튀는 상황 — 정렬 없이도 안전하게 실패."""
    depth_img = np.full((480, 848), 2000, dtype=np.uint16)
    monkeypatch.setattr(m.rs, "rs2_project_color_pixel_to_depth_pixel",
                         lambda *a, **k: (9999.0, 9999.0))

    result = deproject_box(FakeDepthFrame(), depth_img, (400, 200, 460, 260), cal)

    assert result is None


def test_obscured_patch_all_zero_depth_returns_none(monkeypatch, cal):
    """박스 중심 depth 패치가 전부 0(측정불가) — 연막이 카메라 앞을 가린 상황 재현."""
    depth_img = np.zeros((480, 848), dtype=np.uint16)
    monkeypatch.setattr(m.rs, "rs2_project_color_pixel_to_depth_pixel",
                         lambda *a, **k: (430.0, 230.0))

    result = deproject_box(FakeDepthFrame(), depth_img, (400, 200, 460, 260), cal)

    assert result is None


def test_valid_patch_uses_local_median_not_full_frame(monkeypatch, cal):
    """패치 밖 나머지 프레임은 전혀 참조되지 않는다는 것을 poison 값으로 증명하고,
    반환 depth 가 (0 제외) 패치 중앙값 * scale 인지 확인한다 — 단일픽셀/전역정렬 아님."""
    depth_img = np.full((480, 848), 65535, dtype=np.uint16)  # 프레임 전체를 poison 값으로
    cx, cy, r = 430, 230, 4
    patch_values = [2000, 2000, 2010, 2005, 9000, 0]  # 유효 5개(임계 5 충족)+무효 1개 섞임
    ys, xs = np.meshgrid(range(cy - r, cy + r), range(cx - r, cx + r), indexing="ij")
    flat_y, flat_x = ys.ravel(), xs.ravel()
    for i, v in enumerate(patch_values):
        depth_img[flat_y[i], flat_x[i]] = v
    for i in range(len(patch_values), flat_y.size):
        depth_img[flat_y[i], flat_x[i]] = 0  # 패치 나머지도 무효로 채워 유효표본을 고정

    monkeypatch.setattr(m.rs, "rs2_project_color_pixel_to_depth_pixel",
                         lambda *a, **k: (float(cx), float(cy)))
    captured = {}

    def fake_deproject(di, px, z):
        captured["px"] = px
        captured["z"] = z
        return [0.1, 0.2, z]

    monkeypatch.setattr(m.rs, "rs2_deproject_pixel_to_point", fake_deproject)
    monkeypatch.setattr(m.rs, "rs2_transform_point_to_point",
                         lambda ext, pt: tuple(pt))

    # 패치 반경 r = max(4, min(w,h)//6) 이 위에서 쓴 r=4 와 일치하도록 24px 정사각 박스.
    result = deproject_box(FakeDepthFrame(), depth_img, (cx - 12, cy - 12, cx + 12, cy + 12), cal)

    expected_valid = [v for v in patch_values if v > 0]
    expected_z = float(np.median(expected_valid)) * cal.scale
    assert captured["z"] == pytest.approx(expected_z)
    assert result == pytest.approx((0.1, 0.2, expected_z))


def test_patch_rejects_uint16_samples_above_calibrated_depth_range(monkeypatch, cal):
    depth_img = np.full((32, 32), 65535, dtype=np.uint16)
    monkeypatch.setattr(
        m.rs,
        "rs2_project_color_pixel_to_depth_pixel",
        lambda *a, **k: (16.0, 16.0),
    )

    result = deproject_box(FakeDepthFrame(), depth_img, (4, 4, 28, 28), cal)

    assert result is None


def test_out_of_range_raw_samples_cannot_outvote_five_valid_samples(monkeypatch, cal):
    depth_img = np.full((32, 32), 65535, dtype=np.uint16)
    depth_img[12:13, 12:17] = 2000
    monkeypatch.setattr(
        m.rs,
        "rs2_project_color_pixel_to_depth_pixel",
        lambda *a, **k: (16.0, 16.0),
    )
    monkeypatch.setattr(
        m.rs,
        "rs2_deproject_pixel_to_point",
        lambda _di, _px, z: [0.0, 0.0, z],
    )
    monkeypatch.setattr(
        m.rs,
        "rs2_transform_point_to_point",
        lambda _ext, point: point,
    )

    result = deproject_box(FakeDepthFrame(), depth_img, (4, 4, 28, 28), cal)

    assert result == pytest.approx((0.0, 0.0, 2.0))


def test_transformed_depth_outside_calibrated_range_is_rejected(monkeypatch, cal):
    depth_img = np.full((32, 32), 2000, dtype=np.uint16)
    monkeypatch.setattr(
        m.rs,
        "rs2_project_color_pixel_to_depth_pixel",
        lambda *a, **k: (16.0, 16.0),
    )
    monkeypatch.setattr(
        m.rs,
        "rs2_deproject_pixel_to_point",
        lambda *_args: [0.0, 0.0, 2.0],
    )
    monkeypatch.setattr(
        m.rs,
        "rs2_transform_point_to_point",
        lambda *_args: [0.0, 0.0, 65.535],
    )

    result = deproject_box(FakeDepthFrame(), depth_img, (4, 4, 28, 28), cal)

    assert result is None
