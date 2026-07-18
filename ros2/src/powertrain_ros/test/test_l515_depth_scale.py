"""L515 depth 스케일 단일 출처 회귀 테스트.

Gateway 는 raw L515 Z16 을 16UC1 로 **스케일 변환 없이** 발행한다. 따라서 같은
`/l515/depth/image_rect_raw` 를 구독하는 모든 소비자는 동일한 m/unit 을 써야 한다.
L515 는 1/4000 (0.00025) 이고 D400 계열의 0.001 이 아니다 — 틀리면 모든 지형 거리가
정확히 4배로 나온다.

2026-07-18 적대적 코드리뷰: autonomy_controller_node 가 0.001 을 하드코딩해
l515_cloud_node 의 0.00025 와 어긋나 있었다 (지형 거리 4배 과대평가).
"""
import re
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parents[1] / "powertrain_ros"

# L515 = 1/4000 m/unit. 이 값을 바꾸려면 실측 근거가 있어야 한다.
EXPECTED_SCALE = 0.00025


def _literal(path: Path, name: str) -> float:
    """모듈을 import 하지 않고 상수 리터럴만 읽는다 (rclpy 미설치 환경 대응)."""
    source = path.read_text(encoding="utf-8")
    match = re.search(rf"^{name}\s*=\s*([0-9.eE+-]+)", source, re.MULTILINE)
    assert match, f"{path.name} 에서 {name} 상수를 찾지 못했다"
    return float(match.group(1))


def test_l515_cloud_node_uses_l515_scale():
    assert _literal(_PKG / "l515_cloud_node.py", "DEPTH_SCALE_M") == EXPECTED_SCALE


def test_autonomy_controller_uses_l515_scale():
    value = _literal(_PKG / "autonomy_controller_node.py", "L515_DEPTH_SCALE_M")
    assert value == EXPECTED_SCALE


def test_both_consumers_agree():
    """두 노드가 같은 토픽을 읽으므로 스케일이 어긋나면 안 된다."""
    cloud = _literal(_PKG / "l515_cloud_node.py", "DEPTH_SCALE_M")
    autonomy = _literal(_PKG / "autonomy_controller_node.py", "L515_DEPTH_SCALE_M")
    assert cloud == autonomy, (
        f"L515 depth 스케일 불일치: l515_cloud_node={cloud}, "
        f"autonomy_controller_node={autonomy} — 같은 "
        "/l515/depth/image_rect_raw 를 읽으므로 반드시 같아야 한다"
    )


def test_autonomy_controller_has_no_d400_scale_literal():
    """D400 의 0.001 이 TerrainFrame 생성부에 다시 하드코딩되지 않도록 막는다."""
    source = (_PKG / "autonomy_controller_node.py").read_text(encoding="utf-8")
    assert not re.search(r"depth_scale_m\s*=\s*0\.001\b", source), (
        "depth_scale_m=0.001 은 D400 값이다 — L515 는 0.00025"
    )


@pytest.mark.parametrize("raw,expected_m", [(2000, 0.5), (4000, 1.0), (12792, 3.198)])
def test_raw_units_convert_to_expected_metres(raw, expected_m):
    """실측 대조: raw 12792 ≈ 3.2 m (방 크기). 0.001 이면 12.8 m 로 L515 사거리 9 m 초과."""
    assert raw * EXPECTED_SCALE == pytest.approx(expected_m, abs=1e-3)
