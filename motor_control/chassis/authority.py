"""명령 권한(command authority) — `/cmd_vel` 의 **단일 작성자**를 강제한다 (WP5.2-T).

    /teleop/cmd_vel ───┐
                        ├─→ [CommandAuthority] ─→ (v, ω) 하나만 → chassis
    /autonomy/cmd_vel ─┘

하드웨어·ROS 의존 없음. ROS 어댑터는 `chassis_node` 안에 opt-in으로 내장된다
(`authority_enabled=true`).

────────────────────────────────────────────────────────────────────────
왜 필요한가
────────────────────────────────────────────────────────────────────────
`/cmd_vel` 을 여러 노드가 쓰면 **같은 모터에 상반된 명령**이 간다. ROS 는 이를 막지
않는다 — 마지막에 도착한 메시지가 이긴다. 자율(레인·미션)과 원격(텔레옵)이 동시에
쓰면 로봇이 두 명령 사이에서 진동하거나, **꺼진 줄 알았던 자율이 계속 조종**한다.

────────────────────────────────────────────────────────────────────────
★ zero-confirmed handover — 전환 순간이 가장 위험하다
────────────────────────────────────────────────────────────────────────
모드를 AUTO 로 바꾸는 순간, 레인 추종이 **1초 전에 보낸 전속 명령**이 아직 살아 있으면
로봇이 **즉시 튀어나간다**. 반대로 MANUAL 로 바꿀 때 조종자가 트리거를 당기고 있으면
마찬가지다.

그래서 **전환 후에는 새 소스가 '중립(≈0)'을 한 번 보내기 전까지 아무것도 전달하지
않는다.** 조종자가 트리거에서 손을 떼거나, 자율이 정상 루프를 한 바퀴 돌아야 비로소
권한이 넘어간다. 텔레옵 서버가 이미 쓰는 "재연결 neutral gate" 와 같은 원리다.

⚠️ 이건 **안전 게이트가 아니다.** E-stop·충돌 방지는 `SafetyInterlock` + US-100 이 한다.
   여기는 "누구 말을 들을지"만 정한다.
"""
from dataclasses import dataclass
import math

MANUAL = "MANUAL"
AUTO = "AUTO"
IDLE = "IDLE"                      # 아무도 조종하지 않음 (기본값)
MODES = (IDLE, MANUAL, AUTO)


@dataclass
class AuthorityConfig:
    stale_s: float = 0.3           # 소스가 이 시간 넘게 조용하면 죽은 것으로 본다
    neutral_v: float = 0.02        # 이 이하면 '중립'으로 인정 [m/s]
    neutral_omega: float = 0.05    # [rad/s]


@dataclass(frozen=True)
class Command:
    v: float = 0.0
    omega: float = 0.0
    ok: bool = False               # False = 전달하지 말 것 (chassis 워치독이 알아서 0)
    reason: str = ""


class CommandAuthority:
    """모드에 따라 한 소스만 통과시킨다. 전환 시 zero-confirmed handover."""

    def __init__(self, cfg: AuthorityConfig = None):
        self.cfg = cfg or AuthorityConfig()
        self.mode = IDLE
        self._src = {}                       # name → (v, omega, t)
        self._armed = False                  # 현재 모드의 소스가 중립을 확인했나

    # ── 입력 ─────────────────────────────────────────────────────────────

    def submit(self, source: str, v: float, omega: float, t: float) -> None:
        self._src[source] = (float(v), float(omega), float(t))

    def set_mode(self, mode: str) -> bool:
        """모드 전환. **권한은 바로 안 넘어간다** — 새 소스의 중립 확인이 필요하다."""
        if mode not in MODES:
            return False
        if mode != self.mode:
            self.mode = mode
            self._armed = False              # ★ 전환마다 다시 중립을 확인해야 한다
        return True

    # ── 출력 ─────────────────────────────────────────────────────────────

    def select(self, t: float) -> Command:
        if self.mode == IDLE:
            return Command(reason="IDLE — 아무도 조종하지 않음")

        name = MANUAL_SOURCE if self.mode == MANUAL else AUTO_SOURCE
        entry = self._src.get(name)
        if entry is None:
            return Command(reason=f"{name} 명령 없음")

        v, omega, ts = entry
        if t - ts > self.cfg.stale_s:
            # 오래된 명령을 재생하면 안 된다 — 소스가 죽었는데 계속 달린다.
            return Command(reason=f"{name} stale ({t - ts:.2f}s)")

        if not self._armed:
            if self._is_neutral(v, omega):
                self._armed = True           # ★ 중립 확인 → 이제부터 권한 인계
                return Command(0.0, 0.0, True, f"{name} 중립 확인 — 권한 인계")
            return Command(reason=f"{name} 중립 대기 (v={v:+.2f} ω={omega:+.2f})")

        return Command(v, omega, True, name)

    def _is_neutral(self, v, omega):
        return (abs(v) <= self.cfg.neutral_v
                and abs(omega) <= self.cfg.neutral_omega)


MANUAL_SOURCE = "teleop"
AUTO_SOURCE = "auto"
