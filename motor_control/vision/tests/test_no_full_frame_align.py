"""회귀 가드: 전체 프레임 정렬(rs.align)이 다시 들어오지 않는지 소스 검사.

DepthCal/deproject_box 는 검출별 color→depth 픽셀 투영으로 rs.align 을 대체해
Orin Nano 에서 ~108ms/프레임 걸리던 정렬을 없앴다(설계 docstring 참조). 이 성능
이득은 런타임 동작만으로는 테스트하기 어려우므로, 전체정렬 API 가 다시 쓰이지
않는지를 소스 레벨에서 고정해 저사양 보드 실시간성 회귀를 조기에 잡는다.
"""
import inspect

import yolo_depth_3d as m


def test_rs_align_not_used():
    src = inspect.getsource(m)
    assert "rs.align(" not in src
    assert ".align_to(" not in src
