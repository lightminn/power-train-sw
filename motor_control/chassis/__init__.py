"""chassis — 4WS(4륜 조향) 차체 레이어.

kinematics      : 차체 명령(v, ω) → 각 바퀴 (조향각, 구동속도) 순수 계산 (하드웨어·ROS 무관).
chassis_manager : 코너모듈 6개(조향 4 + 고정 2)를 묶어 kinematics 결과를 실제 모터로 분배,
                  estop 전파·안전 interlock·워치독 총괄.
safety_interlock: 모션 홀드와 래치형 비상정지를 중재하는 순수 상태 머신.
telemetry       : ROS와 WP6가 소비할 불변 차체·바퀴 상태 snapshot.
"""

from chassis.telemetry import ChassisSnapshot, WheelSnapshot

__all__ = ["ChassisSnapshot", "WheelSnapshot"]
