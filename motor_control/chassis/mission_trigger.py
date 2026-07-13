"""미션 도착 자동 판정 — YOLO 검출을 보고 "여기서 멈춰야 한다"를 결정한다 (WP8).

    /detected_objects (로봇팔 팀 YOLO) → [이것] → MissionSequencer.arrive()

하드웨어·ROS 의존 없음. 인식은 **로봇팔 팀 단일 소스**이고 우리는 결과만 구독한다.

────────────────────────────────────────────────────────────────────────
★ 한 프레임 깜빡임으로 급정거하면 안 된다
────────────────────────────────────────────────────────────────────────
YOLO 는 프레임마다 흔들린다 — 한 번 잘못 잡았다고 로봇이 미션 정차에 들어가면,
팔이 엉뚱한 곳에서 작업을 시작하고 대회가 끝난다. 그래서 **연속 N 프레임** 같은 대상을
봐야 트리거한다(디바운스).

★ **끝난 미션이 다시 트리거되면 안 된다**
   작업을 마치고 재출발할 때 그 물체가 **아직 눈앞에 있다.** 쿨다운 없이는 즉시 다시
   멈춰서 무한 루프에 빠진다. 미션 완료 후 일정 시간 같은 클래스를 무시한다.

★ **거리 게이트** — 미션 지점 **앞에서** 멈춰야 한다
   5 m 앞에서 보였다고 멈추면 팔이 못 닿는다. `stop_distance_m` 안에 들어와야 트리거.

⚠️ `DetectedObject.pose` 의 **좌표계가 계약에 명시돼 있지 않다.** 지금은 카메라(광학)
   기준으로 가정하고 z(전방거리)만 쓴다. 로봇팔 팀과 확정 필요 — base_link 기준이면
   x 를 써야 한다. 잘못 쓰면 **엉뚱한 거리에서 멈춘다.**
"""
from dataclasses import dataclass, field


@dataclass
class TriggerRule:
    """어떤 클래스를 보면 어떤 도착으로 처리할지."""
    class_name: str
    status: str                     # ARRIVED_PICKUP / ARRIVED_DROP
    stop_distance_m: float = 1.0    # 이 안에 들어오면 트리거
    min_confidence: float = 0.6


@dataclass
class TriggerConfig:
    rules: list = field(default_factory=list)   # list[TriggerRule]
    consecutive: int = 5            # 연속 이만큼 봐야 트리거 (디바운스)
    cooldown_s: float = 10.0        # 미션 완료 후 같은 클래스를 이만큼 무시


class MissionTrigger:
    """검출 → 도착 판정. 상태는 연속 카운트와 쿨다운뿐."""

    def __init__(self, cfg: TriggerConfig = None):
        self.cfg = cfg or TriggerConfig()
        self._streak = {}           # class_name → 연속 프레임 수
        self._cooldown_until = {}   # class_name → 이 시각까지 무시

    def on_detections(self, detections, t: float):
        """detections: [(class_name, confidence, distance_m), ...]

        → (status, class_name) 또는 None
        """
        seen = set()
        for rule in self.cfg.rules:
            hit = self._match(detections, rule)
            if hit is None:
                self._streak[rule.class_name] = 0      # 놓치면 처음부터
                continue

            seen.add(rule.class_name)
            if t < self._cooldown_until.get(rule.class_name, 0.0):
                continue                                # 방금 끝낸 미션 — 무시

            n = self._streak.get(rule.class_name, 0) + 1
            self._streak[rule.class_name] = n
            if n >= self.cfg.consecutive:
                self._streak[rule.class_name] = 0
                return rule.status, rule.class_name

        # 이번 프레임에 안 보인 클래스는 연속 카운트를 끊는다
        for name in list(self._streak):
            if name not in seen:
                self._streak[name] = 0
        return None

    def mission_finished(self, class_name: str, t: float):
        """미션이 끝났다 → 그 물체가 아직 눈앞에 있어도 다시 안 멈춘다."""
        self._cooldown_until[class_name] = t + self.cfg.cooldown_s
        self._streak[class_name] = 0

    @staticmethod
    def _match(detections, rule: TriggerRule):
        best = None
        for name, conf, dist in detections:
            if name != rule.class_name:
                continue
            if conf < rule.min_confidence:
                continue
            if dist > rule.stop_distance_m or dist <= 0.0:
                continue
            if best is None or dist < best[2]:
                best = (name, conf, dist)
        return best
