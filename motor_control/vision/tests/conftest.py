"""무하드웨어 pytest 스위트 공용 설정.

vision/ 스크립트는 Jetson 전용 SDK(pyrealsense2)와 GPU 추론 패키지(ultralytics)를
모듈 최상단에서 import 한다. x86 개발 환경엔 이 SDK 들이 설치돼 있지 않으므로,
테스트 수집 전에 최소 속성만 채운 더미 모듈로 sys.modules 를 선점해 import 자체는
통과시킨다. 실제 계산이 필요한 부분(투영·좌표변환 등)은 각 테스트에서 monkeypatch
로 원하는 반환값을 주입한다 — 이 conftest 는 "import 가 되게만" 만든다.
"""
import sys
import types
from pathlib import Path

# yolo_depth_3d.py 는 직접 실행(sys.path[0]=vision/)을 가정하고 형제 모듈을
# `from yolo_cuda_stream import ...` 식 절대 bare import 로 가져온다. pytest 는
# motor_control/ 을 sys.path 에 넣어주므로(부모 tests/ 에 __init__.py 없음) vision/
# 자체도 명시적으로 얹어줘야 같은 import 가 동작한다.
VISION_DIR = Path(__file__).resolve().parent.parent
if str(VISION_DIR) not in sys.path:
    sys.path.insert(0, str(VISION_DIR))


def _stub_pyrealsense2() -> types.ModuleType:
    mod = types.ModuleType("pyrealsense2")
    # 타입힌트로만 쓰이는 이름들 — 인스턴스화하지 않으므로 빈 클래스면 충분.
    for name in ("pipeline", "config", "composite_frame", "pipeline_profile",
                 "depth_frame", "video_stream_profile"):
        setattr(mod, name, type(name, (), {}))
    mod.stream = types.SimpleNamespace(depth="depth", color="color")
    mod.format = types.SimpleNamespace(z16="z16", bgr8="bgr8")
    # 실 SDK 는 여기서 투영·역투영을 계산한다 — 각 테스트가 monkeypatch 로 대체.
    mod.rs2_project_color_pixel_to_depth_pixel = lambda *a, **k: (0.0, 0.0)
    mod.rs2_deproject_pixel_to_point = lambda *a, **k: [0.0, 0.0, 0.0]
    mod.rs2_transform_point_to_point = lambda *a, **k: [0.0, 0.0, 0.0]
    return mod


def _stub_ultralytics() -> types.ModuleType:
    mod = types.ModuleType("ultralytics")
    mod.YOLO = type("YOLO", (), {})
    return mod


if "pyrealsense2" not in sys.modules:
    sys.modules["pyrealsense2"] = _stub_pyrealsense2()
if "ultralytics" not in sys.modules:
    sys.modules["ultralytics"] = _stub_ultralytics()
