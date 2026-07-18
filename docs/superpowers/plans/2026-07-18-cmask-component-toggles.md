# CMASK 구현 계획 — 콘솔 컴포넌트별 on/off + estop 비체결

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경(+pipefail) + 젯슨 실기.

**Goal:** 스펙 `docs/superpowers/specs/2026-07-18-component-mask-design.md` —
구동·조향·US-100·로봇팔 4컴포넌트를 콘솔 Ops 패널에서 on/off하고, OFF 컴포넌트의
무응답·오류가 ESTOP/HOLD를 체결하지 않게 한다(D1 미장착 모드·D2 실기 허용+경고·
D3 무영속·D4 팔은 별도 플래그).

**Architecture:** 순수 `ChassisManager.component_mask` + `CornerModule`
drive/steer enable 플래그(명령·감시 스킵) + chassis_node 단일 초크포인트에서
us100/robot_arm 소스 유입 차단. 표면은 ops `service_setbool` 액션 4종 →
SetBool 서비스 4개, 상태는 `/chassis/safety_state` JSON → broker OpsState →
콘솔(토글 상태+배너), :5005 텔레메트리 미러(관측 전용 콘솔용). **msg 변경 없음.**

**기준선(현재):** 호스트 332 / dev 1163+2skip / ros 512 / 젯슨 512.

## Global Constraints

- 기본 전부 enabled, **영속화 금지**(재시작 = 전부 ON).
- 모터(drive/steer) 토글은 `mode == "IDLE"`에서만 수락, 거부 사유 `"not_idle"`.
- OFF 시 그 컴포넌트의 활성 estop condition/hold 소스 해제하되 **latch된
  ESTOP은 자동 해제 금지**.
- `command_watchdog`·section·qualification·extraction 소스는 마스크 비대상.
- §9-3 관례: chassis_node/teleop 신규 속성 접근은 `getattr(self, "...", None)`
  가드(AST/SimpleNamespace 픽스처 보호). 신규 소켓 금지(콘솔 송신봉인).
- 컴포넌트 키는 정확히 `("drive", "steer", "us100", "robot_arm")`.

---

### Task 1: 순수 코어 — CornerModule 플래그 + ChassisManager 마스크

**Files:**
- Modify: `motor_control/corner_module/corner_module.py`,
  `motor_control/chassis/chassis_manager.py`
- Test: `motor_control/chassis/tests/test_component_mask.py`(신규),
  `motor_control/corner_module/tests/` 기존 파일에 케이스 추가

**Interfaces (Produces):**

```python
# CornerModule
def set_drive_enabled(self, enabled: bool) -> None
def set_steer_enabled(self, enabled: bool) -> None
# 기본 True. disabled 측: tick()에서 명령 송신·fault/stale 판정·estop 발화
# 전부 스킵, arm()/estop()/reset_fault() 대상 제외, state()["drive_enabled"]
# / ["steer_enabled"] 노출.

# ChassisManager
COMPONENTS = ("drive", "steer", "us100", "robot_arm")
@property
def component_mask(self) -> dict[str, bool]          # 복사본
def set_component_enabled(self, component: str, enabled: bool,
                          detail: str = "") -> tuple[bool, str]
# (False, "unknown_component") | (False, "not_idle")(drive/steer이고
# mode != "IDLE") | (True, ""). drive/steer는 6코너에 전파.
# us100 disable: self._interlock.set_estop_condition("us100", False) 및
#   us100발 hold 해제(소스명 "us100" 프리픽스 일괄).
# robot_arm disable: set_motion_hold("robot_arm", False).
# snapshot()·safety_snapshot() 반환에 component_mask 포함.
```

- [ ] RED: ①기본 전부 True ②unknown 거부 ③ARMED에서 drive 토글 거부(not_idle)
  ④IDLE에서 drive OFF→코너 fake 드라이브에 명령 0회·drive fault 주입에도
  corner FAULT/estop 없음·steer는 정상 ⑤us100 OFF→기존 us100 estop condition
  해제되나 latch는 유지(reset 별도) ⑥robot_arm OFF→robot_arm hold 해제
  ⑦ON 복귀 후 arm()이 해당 액추에이터 재편입 ⑧snapshot 마스크 노출.
- [ ] 구현→GREEN(호스트+dev 대상 스위트)→커밋
  `feat: component mask core (corner enable flags + chassis mask)`

---

### Task 2: 노드·ops 배선 — 서비스 4개 + 소스 초크포인트 + 상태 전파

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`,
  `ros2/src/powertrain_ros/powertrain_ros/ops_contract.py`,
  `ros2/src/powertrain_ros/powertrain_ros/ops_broker_node.py`(OpsState
  component_mask 필드 — semantic, 상태 소스 파싱)
- Test: `ros2/src/powertrain_ros/test/test_component_mask_wiring.py`(신규),
  `test_ops_contract.py`·`test_ops_state_sources.py` 케이스 추가

**Interfaces:**
- chassis_node SetBool 서비스 4개: `~/component_enable_drive`·`_steer`·
  `_us100`·`_robot_arm` → `cm.set_component_enabled(...)`; 거부 시
  `success=False, message=사유`; 성공 시 저널 `COMPONENT_MASK`
  (component·enabled) 기록.
- **초크포인트**: us100 유래 유입(`_on_safety_verdict`의 estop/hold 공급 +
  0.75 s 신선도 estop 경로)과 robot_arm 유래 유입(`_on_arm_status` 신선도·
  `robot_arm` hold 공급)을 각각 단일 함수 경유로 정리하고, 함수 초입에서
  `cm.component_mask`를 참조해 OFF면 미공급. 산재 if 금지.
- `/chassis/safety_state` JSON에 `"component_mask": {...}` 추가.
- ops_contract: `"drive_enable"`·`"steer_enable"`·`"us100_enable"`·
  `"robot_arm_enable"` = `ActionSpec(_CONSOLE, "service_setbool",
  ("/chassis_node/component_enable_<name>",))`.
- broker OpsState: `component_mask` 필드(semantic — 변경 시 revision 증가),
  safety_state 소스에서 파싱, 부재 시 전부 True.
- [ ] RED(ros 컨테이너): ①서비스 왕복+IDLE 게이트 거부 메시지 ②us100 OFF에서
  verdict stale/NO_RESPONSE 주입에도 RUN 유지, ON 복귀 시 estop 재발동
  ③robot_arm OFF에서 hold 미발생 ④ops 액션 4종 계약(role·kind·target)
  ⑤OpsState component_mask semantic revision.
- [ ] 구현→GREEN(ros 전체 회귀)→커밋
  `feat: component mask wiring (services, source chokepoints, ops actions)`

---

### Task 3: 콘솔·텔레메트리 — 토글 4행 + 배너 + :5005 미러

**Files:**
- Modify: `operator_console/ops_panel.py`(PANEL_ACTIONS에 needs_bool 토글
  4행 — 라벨 `Drive motors on/off` 등, `us100_enable` confirm_text에 "충돌
  안전 센서를 끕니다 — 접근 시 자동 정지 없음" 명시),
  `operator_console/app.py`(ops 상태의 component_mask로 토글 현재값 표시 +
  상단 배너: OFF 존재 시 주황 `MASK: DRIVE·US-100 OFF`; us100 OFF면 SAFETY
  배너를 주황 `SAFETY DISABLED (US-100 OFF)`로),
  `operator_console/telemetry.py`(optional `component_mask` 파싱 — 부재 None),
  `ros2/src/powertrain_ros/powertrain_ros/chassis_telemetry_sender_node.py`
  (`/chassis/safety_state` String 구독(1 s 신선도) → payload
  `"component_mask"` 미러)
- Test: `operator_console/tests/test_ops_panel.py`·`test_app.py` 케이스 추가,
  `ros2/src/powertrain_ros/test/test_chassis_telemetry_worker.py` 인코딩 케이스
- [ ] RED: ①파서 하위호환(필드 부재→None) ②needs_bool 토글이 True/False 양방향
  제출 가능(기존 arm_lock_override는 단방향인지 확인 후 동일 패턴 확장 —
  현재값 반전 제출) ③us100 위험 문구 존재 ④배너 문자열 케이스.
- [ ] 구현→GREEN(호스트 operator_console+tests, dev 전체)→커밋
  `feat(console): component toggles, mask banner, telemetry mirror`

---

### Task 4: 문서·3환경·젯슨 실기 — 리뷰어 주도

- [ ] 핸드오프 §2 CMASK 행 + 기준선 갱신, 프로젝트 CLAUDE.md operator_console/
  chassis 줄에 마스크 1줄, Notion GUI 페이지 §12.4 매뉴얼에 토글 절 추가
  (쓰기 후 재조회).
- [ ] 3환경 green(+캠페인 무관 — 시뮬 비접촉 확인) → 커밋·push.
- [ ] 젯슨: pull → `--force-recreate powertrain_ros powertrain_control` →
  parity → 실기(비회전): ops로 us100 OFF → US-100 케이블 상태 무관 RUN 유지
  확인 → ON → estop 재발동 확인, drive OFF(IDLE) 수락·ARMED 거부 왕복,
  safety_state·:5005에 마스크 표기. 좀비/포트 관례(§9-0/9-5) 준수.
- [ ] 메모리 갱신.

## 완료 기준

- 4컴포넌트 콘솔 토글 + OFF 시 해당 소스 estop/hold 비체결(스펙 의미론 표),
  latch 비자동해제·IDLE 게이트·무영속 계약 테스트로 봉인.
- 3환경+젯슨 parity green, 실기 us100 OFF/ON 시나리오 통과.
