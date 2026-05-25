"""DualSense 텔레옵 데모: 게임패드 입력 → CornerModule.set().

map_input 은 순수 함수로 분리해 단위 테스트한다. 실행 진입점(main)은 Task 9.
"""
from corner_module.config import CornerConfig


def map_input(left_x: float, rt: float, lt: float, cfg: CornerConfig,
              deadzone: float = 0.05):
    """좌스틱 X → 조향각, (RT−LT) → 구동속도. 데드존 적용.

    조향은 좌우 대칭(±steer_max_deg)으로 매핑한다(CornerModule 이 다시 clamp).
    """
    sx = 0.0 if abs(left_x) < deadzone else left_x
    steer_deg = sx * cfg.steer_max_deg

    trig = rt - lt
    if abs(trig) < deadzone:
        trig = 0.0
    drive_vel = trig * cfg.drive_vel_limit
    return steer_deg, drive_vel
