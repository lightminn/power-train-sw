"""차체 모드 결정 — 팔에게 "지금 자세를 고정해야 하나"를 알린다. 순수 계산 코어.

`/chassis_mode` 는 **로봇팔과의 계약 토픽**이다. 팔은 이걸 보고 자세를 락한다:

    DRIVING        정상 주행 — 팔 언락 (움직여도 된다)
    CORNERING      선회 중   ┐
    ROUGH_TERRAIN  험지      ├ 팔 락 (자세 고정) — 그들 LOCK_MODES
    FOLLOW_LEAD    추종 중   ┘
    MISSION_STOP   미션 정차 — 팔이 작업하는 구간

────────────────────────────────────────────────────────────────────────
★ 작성자는 **하나**여야 한다
────────────────────────────────────────────────────────────────────────
`chassis_node` 와 `mission_node` 가 둘 다 `/chassis_mode` 를 쓰면, **팔이 번갈아 받는다** —
우리가 정차 중인데 `DRIVING` 을 보고 **팔이 움직인다**. 그래서 `chassis_node` 가 단독
소유하고, 미션은 `/mission/chassis_mode` 로 **요청**만 한다.

우선순위 (위가 셈):
  ① MISSION_STOP   — 미션 시퀀서가 요청. 팔이 작업 중이므로 절대 안 움직인다.
  ② FOLLOW_LEAD    — 앞 로봇 추종 (WP9, 아직 없음)
  ③ ROUGH_TERRAIN  — IMU 기울임이 크다. 험지에서 팔이 흔들리면 위험하다.
  ④ CORNERING      — 선회 중. 원심력으로 팔이 흔들린다.
  ⑤ DRIVING        — 그 외

⚠️ **아무도 이 모드들을 발행하지 않고 있었다.** 팔은 계속 `DRIVING` 만 받아 코너·험지에서
   자세를 락하지 않았다. 계약은 있었지만 **구현이 비어 있었다.**
"""
from dataclasses import dataclass
import math

MODE_DRIVING = "DRIVING"
MODE_CORNERING = "CORNERING"
MODE_ROUGH_TERRAIN = "ROUGH_TERRAIN"
MODE_FOLLOW_LEAD = "FOLLOW_LEAD"
MODE_MISSION_STOP = "MISSION_STOP"

LOCK_MODES = frozenset({MODE_CORNERING, MODE_ROUGH_TERRAIN, MODE_FOLLOW_LEAD})


@dataclass
class ModeConfig:
    """임계값. 대회 트랙·실차 거동에 맞춰 조정한다."""
    corner_omega: float = 0.25      # |ω| 가 이 이상이면 선회 [rad/s]
    corner_hyst: float = 0.15       # 이 아래로 떨어져야 선회 해제 (히스테리시스)
    rough_tilt_deg: float = 8.0     # roll/pitch 가 이 이상이면 험지
    rough_hyst_deg: float = 5.0     # 이 아래로 떨어져야 해제
    hold_s: float = 0.5             # 락 모드는 최소 이만큼 유지 (덜컥거림 방지)


class ChassisModeSelector:
    """(미션 요청, 운동 상태) → 차체 모드. 단일 작성자가 쓴다."""

    def __init__(self, cfg: ModeConfig = None):
        self.cfg = cfg or ModeConfig()
        self.mode = MODE_DRIVING
        self._t_lock = None          # 락 모드에 들어간 시각

    def update(self, t: float, omega: float = 0.0, roll: float = 0.0,
               pitch: float = 0.0, mission_mode: str = None,
               follow_lead: bool = False) -> str:
        c = self.cfg

        # ① 미션 정차가 최우선 — 팔이 작업 중이면 무조건 안 움직인다
        if mission_mode == MODE_MISSION_STOP:
            self.mode = MODE_MISSION_STOP
            self._t_lock = None
            return self.mode

        # ② 앞 로봇 추종 (WP9)
        if follow_lead:
            self.mode = MODE_FOLLOW_LEAD
            self._t_lock = t
            return self.mode

        tilt = math.degrees(max(abs(roll), abs(pitch)))
        was_locked = self.mode in LOCK_MODES

        # ③ 험지 — 히스테리시스로 경계에서 덜컥거리지 않게
        rough_on = c.rough_tilt_deg if not was_locked else c.rough_hyst_deg
        if tilt >= rough_on:
            self.mode = MODE_ROUGH_TERRAIN
            self._t_lock = t
            return self.mode

        # ④ 선회
        corner_on = c.corner_omega if not was_locked else c.corner_hyst
        if abs(omega) >= corner_on:
            self.mode = MODE_CORNERING
            self._t_lock = t
            return self.mode

        # ⑤ 락 해제 — 최소 유지시간을 지킨다.
        #    팔이 자세를 풀었다 잠갔다 반복하면 오히려 흔들린다.
        if was_locked and self._t_lock is not None and t - self._t_lock < c.hold_s:
            return self.mode

        self.mode = MODE_DRIVING
        self._t_lock = None
        return self.mode
