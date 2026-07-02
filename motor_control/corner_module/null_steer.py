"""고정 바퀴용 no-op 조향 액추에이터.

로커보기 중간 2바퀴는 조향모터가 없다(구동만). kinematics 는 이 바퀴에
`steer_deg=0` 을 주므로, "항상 0°·fault 없음"인 NullSteer 를 물리면 그 바퀴도
동일한 CornerModule(조향+구동) 인터페이스로 균일하게 다룰 수 있다 — 워치독·
estop·상태머신을 그대로 재사용. 실제로 각을 소비하지 않으니 set_angle 은 무시한다.
"""
from corner_module.actuator import SteerActuator


class NullSteer(SteerActuator):
    def connect(self) -> None:
        pass

    def arm(self) -> None:
        pass

    def disarm(self) -> None:
        pass

    def set_angle(self, deg: float) -> None:
        pass                                        # 조향모터 없음 — 무시

    def tick(self) -> None:
        pass

    def state(self) -> dict:
        # CornerModule.tick() 안전검사가 읽는 키(fault/stale/cur_a)를 무해값으로.
        return {
            "target_deg": 0.0,
            "actual_deg": 0.0,
            "cur_a": 0.0,
            "fault": 0,
            "stale": False,
        }

    def estop(self) -> None:
        pass

    def close(self) -> None:
        pass
