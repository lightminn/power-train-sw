# C0 배치 구현 계획 — 후진 한정 extraction 상태 (D2, WP5.1 의미론 개정)

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경 + 젯슨 실기(비회전 FAKE).
> **⚠️ feature-flag `extraction_enabled` 기본 False — 벤치 HIL(실모터 후진) 통과 전
> 실기 활성 금지**(스펙 r6 §6.1 게이트).

**Goal:** 스펙 r6 §6.1 상태표 그대로 — US-100 단독 latch에서 코앞 장애물을 후진으로
벗어날 유일한 탈출 상태. 문 PUSH 구간 생존성(프리모템 시나리오 2).

**Architecture:** `ChassisManager`에 EXTRACTION 모드 추가(§6.1 상태표가 계약).
진입은 콘솔 전용 `extraction_grant`(broker 경유), 명령은 기존 gateway 데드맨 +
authority TELEOP 경로 그대로(추가 소유권 메커니즘 없음 — 게이트웨이가 데드맨·
신선도를 이미 강제), chassis 최종단에서 −0.2≤v≤0·ω=0 clamp. TTL·누적거리·grant
횟수 budget, 타 fault 즉시 차단, 만료·종료 시 **원래 ESTOP latch 보존**.

**Spec:** r6 §6.1. 기준선(A2c 후): 호스트 259 / dev 1063+2skip / ros 466 / 젯슨 466.

## §6.1 상태표 → 계약 (구현·테스트의 권위)

| 항목 | 계약 |
|---|---|
| 진입 | `extraction_grant()` — `extraction_enabled` ∧ `estop_latched` ∧ **활성 estop 원인 == {"us100"} 단독** ∧ grant 잔여(에피소드당 ≤3) 아니면 False. 성공 시 코너 재-arm(인터록 latch는 **유지**), mode="EXTRACTION", TTL 시작 |
| 명령 소유 | 기존 경로 재사용: gateway(데드맨·fresh)→authority TELEOP→`cm.set()`. EXTRACTION 중 set 수락 |
| 최종 clamp | `tick()`에서 kinematics 이전에 `v=min(0,max(-0.2,v))`, `ω=0` 강제(전진 지령은 0) |
| US-100 예외 | EXTRACTION 중 us100 estop 조건이 tick 조기반환·구동 차단을 **하지 않음**(전방 센서, 후진 전용이므로) |
| budget | grant당 TTL 3 s(단조 시계) + 에피소드(래치 1회) 누적 `Σ|v_eff|·dt ≤ 1.0 m` + grant ≤3회. 에피소드 카운터는 `reset_estop()` 성공 시 리셋 |
| 중단 | TTL 만료·budget 소진·**us100 외 estop 원인 발생**·코너 FAULT → 즉시 코너 estop + mode="ESTOP"(latch 그대로) |
| 종료 | 후진으로 거리 확보 → us100 active 해제 → 통상 `reset_estop`→`arm` |
| 워치독 | 기존 chassis 워치독 그대로(명령 stale → 0) |

---

### Task 1: ChassisManager EXTRACTION 코어

**Files:**
- Modify: `motor_control/chassis/chassis_manager.py` (`ChassisConfig.extraction_*` 3필드, `extraction_grant()`, `tick()` 분기, `reset_estop()` 에피소드 리셋, `snapshot()`에 extraction 상태)
- Test: `motor_control/chassis/tests/test_extraction.py` (신규)

**Interfaces:**
- `ChassisConfig`: `extraction_enabled: bool = False`, `extraction_ttl_s: float = 3.0`,
  `extraction_budget_m: float = 1.0`, `extraction_max_grants: int = 3`,
  `extraction_v_limit: float = 0.2` (후진 절대값 상한 m/s).
- `extraction_grant() -> bool` — 상태표 진입 계약. 거부 사유는 반환 False +
  `self._last_extraction_reject`(문자열, snapshot 노출)로.
- `tick()` — mode=="EXTRACTION" 분기: 상태표의 clamp·us100 예외·중단 조건.
  거리 적분은 실제 경과시간(`_now()` 델타) × `|v_eff|`.
- `snapshot()` — `extraction_active: bool`, `extraction_remaining_s`,
  `extraction_budget_left_m`, `extraction_grants_left` (WheelSnapshot 아님 —
  ChassisSnapshot 최상위, 기존 필드 추가 관례).
- interlock은 건드리지 않음(latch 보존이 계약) — us100 예외는 tick의 EXTRACTION
  분기에서 `active_estop_sources`를 검사해 us100 단독일 때만 구동 허용.

- [ ] **Step 1: 실패 테스트** — `test_extraction.py` 12케이스(FakeCorner+주입 시계):
  ①flag off grant 거부 ②비latch 거부 ③us100+타원인 공존 거부 ④정상 grant →
  mode EXTRACTION·코너 armed·latch 유지 ⑤전진 지령 clamp→0 ⑥후진 −0.5→−0.2
  clamp·ω→0 ⑦us100 활성인데도 후진 구동 전달(예외 동작) ⑧TTL 만료 → ESTOP
  복귀·latch 유지·reset_estop 여전히 거부(us100 활성 시) ⑨타 estop 원인 발생
  즉시 차단 ⑩누적 1.0 m 소진 차단 ⑪grant 3회 소진 후 4회차 거부 ⑫reset_estop
  성공 시 에피소드 카운터 리셋.
- [ ] **Step 2: RED**(호스트 /tmp/t3can 스텁) → **Step 3: 구현** → **Step 4: GREEN**
  (chassis+corner 전체 회귀 포함)
- [ ] **Step 5: 커밋** `feat: reverse-only extraction state in ChassisManager (spec r6 §6.1, flag off)`

---

### Task 2: 배선 — chassis_node 서비스 + ops 계약 + 콘솔 PanelAction

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` (`extraction_enabled` 파라미터→cfg, `~/extraction_grant` Trigger 서비스)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/ops_contract.py` (`extraction_grant` console 전용 service → `/chassis_node/extraction_grant`)
- Modify: `operator_console/ops_panel.py` (PanelAction 추가 — STRIP, 강확인 문구 "US-100 단독 latch에서만 · 후진 −0.2 m/s · TTL 3 s")
- Test: `ros2/src/powertrain_ros/test/test_extraction_wiring.py` + `test_ops_contract.py`·`test_ops_panel.py` 케이스 추가

**Interfaces:**
- `~/extraction_grant`(Trigger): `cm.extraction_grant()` 반환·거부 사유를
  response.message로. 파라미터 `extraction_enabled` 기본 False → cfg 전달.
- ops_contract: `ACTIONS["extraction_grant"] = ActionSpec(_CONSOLE, "service",
  ("/chassis_node/extraction_grant",))`.
- 테스트: AST 추출로 서비스 콜백 계약(grant 성공/거부 메시지), contract 표
  검증, 패널 action 존재+STRIP+문구, 소스 계약(파라미터 선언).
- [ ] **Step 1~4: RED→구현→GREEN**(ros 컨테이너 전체 + operator_console)
- [ ] **Step 5: 커밋** `feat: extraction_grant wiring - chassis service, ops action, console panel entry`

---

### Task 3: 문서·3환경·젯슨 실기(비회전) — 리뷰어 주도

- [ ] 핸드오프 §2 C0 행(+WP5.1 의미론 개정 명기 — 스펙 §0 정본 개정 3 이행),
  CLAUDE.md chassis 절에 EXTRACTION 요약 1줄(⚠️ flag off·벤치 HIL 선행),
  기준선 갱신.
- [ ] 3환경 회귀 green.
- [ ] **젯슨 실기(비회전, FAKE)**: 도메인 77에서 FAKE chassis_node
  (`extraction_enabled:=true`) 기동 → rclpy 직접 클라이언트로 ①미latch 상태
  grant 거부 메시지 ②`~/estop`(manual_service latch — us100 아님) 후 grant 거부
  (단독 원인 아님 확인은 us100 fake 불가라 이 두 거부 경로만) ③flag off 재기동
  시 거부 — 확인. 라이브 broker에 extraction_grant action 반영(force-recreate)
  + 콘솔 토큰으로 거부 왕복 1회.
- [ ] **벤치 이월**: 실모터 후진 extraction HIL(“US-100 단독 latch → grant →
  실후진 → latch 해제 → reset→arm” 풀사이클) — 이 통과 전 실기 flag on 금지.
- [ ] 커밋 `docs: C0 chain` + push + 젯슨 pull.

## 완료 기준

- §6.1 상태표 12계약 전부 순수 테스트 통과, 기존 estop/reset/arm 의미론 회귀 0.
- 배선(서비스·ops action·패널) + 젯슨 비회전 거부 경로 실확인.
- 벤치 이월 명기: 실후진 풀사이클 HIL = 실기 활성 게이트.
