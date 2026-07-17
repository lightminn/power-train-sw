# A2b 배치 구현 계획 — 콘솔 운용 패널 + 햅틱 arbiter

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development 또는 executing-plans. 체크박스 추적.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 리뷰어 3환경 + **젯슨 실기 검증**(goal 개정: 항상 포함).

**Goal:** 스펙 r6 §3.3(콘솔 게이트 명령 패널 + 헌장 개정)·§3.5(햅틱 arbiter·Tier 1/2) 구현.

**Architecture:** 명령 흐름은 순수 코어로 — `operator_console/ops_panel.py`(2단 확인 상태기계·revision 재검증·제스처 규칙)와 `operator_console/ops_client.py`(A2a `ops_channel_client` 재사용 스레드 래퍼) 위에 GTK는 얇게. 햅틱은 `motor_control/laptop/haptic_arbiter.py`(우선순위 단일 중재·stale 방어, 순수) + `dualsense_output.py`(pydualsense 격리 스레드, 예외 시 출력만 자동 비활성). broker 계약에 `arm_lock_override`(SetBool) 추가.

**Spec:** r6 §3.1(action 표 확장)·§3.3·§3.5. 기준선(A2a 후): 호스트 240 / dev 998+2skip / ros 456 / 젯슨 456.

## Global Constraints

- A2a 계획의 Global Constraints 전부 승계(경로·명령·금지·토큰 실값 금지).
- GTK 실행 코드는 pure-core에서 분리(기존 `test_app.py`가 순수 함수만 검증하는 관례 유지). GTK/GLib import는 테스트에서 요구하지 않는다.
- pydualsense는 선택 의존성 — import 실패 시 출력만 비활성, 입력·주행 경로 무영향(§3.5).
- 콘솔 실 GUI 육안 확인·햅틱/트리거 체감은 **벤치 이월**(사용자 물리 세션). 젯슨 실기 검증 = 콘솔 토큰 핸드셰이크(role=console) + 패널 대상 action 왕복(스택 미가동 시 거부/timeout 의미론 확인).

---

### Task 1: 계약 확장 — `arm_lock_override`(SetBool kind)

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/ops_contract.py` (ACTIONS)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/ops_broker_node.py` (`service_setbool` 실행)
- Test: `ros2/src/powertrain_ros/test/test_ops_contract.py`·`test_ops_broker_node.py`에 추가

**Interfaces:**
- Produces: `ACTIONS["arm_lock_override"] = ActionSpec(_CONSOLE, "service_setbool", ("/chassis_node/arm_lock_override",))`. broker `_execute`가 kind `service_setbool`이면 `SetBool.Request(data=bool(params["data"]))`로 호출(`params["data"]` 부재/비불리언 → 즉시 FINAL_REJECTED). 나머지 의미론(PENDING/late/journal)은 service와 동일.

- [ ] **Step 1: 실패 테스트** — `test_ops_contract.py`에:

```python
def test_arm_lock_override_is_console_only_setbool():
    spec = oc.ACTIONS["arm_lock_override"]
    assert spec.roles == frozenset({oc.ROLE_CONSOLE})
    assert spec.kind == "service_setbool"
    assert spec.target == ("/chassis_node/arm_lock_override",)
```

`test_ops_broker_node.py`에 (FakeServices에 SetBool 서버 `/chassis_node/arm_lock_override` 추가):

```python
def test_arm_lock_override_round_trip_and_param_validation(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    try:
        sock, reader = _client(port, "tok-console-test")
        _hello(reader, [node, targets])
        sock.sendall(_request(
            "tok-console-test", "arm_lock_override",
            params={"data": True},
        ))
        replies = reader.read_until(
            [node, targets],
            lambda lines: _has_request(lines, "r-1", "FINAL_SUCCESS"),
        )
        assert "/chassis_node/arm_lock_override" in targets.calls

        sock.sendall(_request(
            "tok-console-test", "arm_lock_override", request_id="r-2",
            sequence=1, params={},
        ))
        replies = reader.read_until(
            [node, targets],
            lambda lines: _has_request(lines, "r-2", "FINAL_REJECTED"),
        )
        sock.close()
    finally:
        node.close(); node.destroy_node(); targets.destroy_node()
```

- [ ] **Step 2: RED 확인** (호스트: contract / ros 컨테이너: node)
- [ ] **Step 3: 구현** — 계약 1줄 + broker `_execute`/`_start_service_call`에 kind 분기(`SetBool` import, params 검증은 실행 전 `_execute`에서).
- [ ] **Step 4: GREEN** — ros 컨테이너 전체
- [ ] **Step 5: 커밋** `feat: arm_lock_override console action (SetBool kind)`

---

### Task 2: `laptop` 패키지화 + 콘솔 ops 클라이언트 스레드 래퍼

**Files:**
- Create: `motor_control/laptop/__init__.py` (빈 파일 — 패키지화)
- Create: `operator_console/ops_client.py`
- Test: `operator_console/tests/test_ops_client.py`

**Interfaces:**
- Produces: `operator_console.ops_client.ConsoleOpsClient` —
  `__init__(host, port, token, *, submit_sink, state_sink, schedule=GLib.idle_add 주입, client_factory=laptop.ops_channel_client.OpsChannelClient)`.
  전용 스레드: nonblocking reconnect 루프(1 s 백오프), bounded send queue
  (maxlen 16, 초과 drop-oldest+카운터), `pump()` 폴링으로 ACK/push 수신 →
  `schedule(callback)` 로 UI 스레드 핸드오프(테스트에선 동기 호출 주입),
  `submit(action, params=None, expected_state_revision=None) -> request_id`,
  `latest_state()`, `close()`(join). laptop 모듈 import는
  `sys.path` 삽입(`MOTOR_CONTROL_PATH` env, 기본 상대 경로 유도) 후
  `from laptop import ops_channel_client`.
- 테스트는 `client_factory`에 가짜(재전송·응답 시퀀스 스크립트)를 주입해
  스레드 없이 결정론 검증(스레드 루프 본체는 `run_once()`로 분리).

- [ ] **Step 1: 실패 테스트** — 5케이스: ①submit→가짜 클라이언트로 전달·request_id 반환 ②응답 ACK가 schedule 경유 submit_sink 콜백 도착 ③push가 state_sink 도착+latest_state 갱신 ④send queue 상한 drop-oldest ⑤close가 join(플래그) — 코드는 가짜 factory·동기 schedule로 작성.
- [ ] **Step 2: RED** (호스트 `pytest operator_console/tests -q`)
- [ ] **Step 3: 구현**
- [ ] **Step 4: GREEN** + 기존 콘솔 테스트 회귀
- [ ] **Step 5: 커밋** `feat: console ops client thread wrapper reusing the laptop channel`

---

### Task 3: `ops_panel.py` — 2단 확인 상태기계 (순수)

**Files:**
- Create: `operator_console/ops_panel.py`
- Test: `operator_console/tests/test_ops_panel.py`

**Interfaces:**
- Produces:

```python
GESTURE_STRIP = "confirm_strip"          # 1단→확인 스트립 클릭
GESTURE_HOLD = "hold_to_confirm"         # arm: 1.5 s 유지
HOLD_CONFIRM_S = 1.5
PANEL_ACTIONS: tuple[PanelAction, ...]   # (action, label, gesture, needs_bool?)
  # clear_transient_hold·authority_manual/auto/idle·estop_reset(STRIP)·
  # arm(HOLD)·disarm(STRIP)·arm_lock_override(STRIP, 강확인 문구)·미션 6종·
  # operator_hold/resume — estop_reset과 arm 사이 spacer 마커 포함
class ConfirmFlow:
    def __init__(self, *, clock, state_provider): ...
    def begin(self, action) -> ConfirmState      # 1단: 원인·상태 스냅샷 캡처
    def confirm(self, action, *, held_s=None) -> dict | None
        # 2단: revision 재검증(불일치→None+reset, TOCTOU) ·
        # HOLD 제스처는 held_s>=1.5 요구 · 성공 시 submit kwargs 반환
    def reset(self)
```

- state_provider는 `ConsoleOpsClient.latest_state`. begin 시 revision 스냅샷,
  confirm 시 현재 revision과 비교(다르면 None — UI는 1단으로 복귀).
  state가 None(수신 전)이면 begin 거부.

- [ ] **Step 1: 실패 테스트** — 6케이스: ①begin은 상태 없으면 거부 ②STRIP 정상 흐름이 submit kwargs(`expected_state_revision` 포함) 반환 ③revision 변화 시 confirm→None ④HOLD 제스처 held_s 미달 거부·충족 수락 ⑤estop_reset과 arm은 PANEL_ACTIONS에서 spacer로 분리·제스처 상이 ⑥arm_lock_override는 params {"data": ...} 요구.
- [ ] **Step 2: RED** → **Step 3: 구현** → **Step 4: GREEN**
- [ ] **Step 5: 커밋** `feat: console two-step confirm flow with revision revalidation`

---

### Task 4: GTK 배선 + 헌장 개정 + 송신 계약 테스트

**Files:**
- Modify: `operator_console/app.py` (OpsPanel Frame, side box 편입, 배너, argparse `--ops-host/--ops-port/--ops-token-file`, 독스트링)
- Modify: `operator_console/README.md`, `operator_console/__init__.py`, `operator_console/telemetry.py`(독스트링 1줄)
- Test: `operator_console/tests/test_send_surface_contract.py`

**Interfaces:**
- `OpsPanel(Gtk.Frame)`: PANEL_ACTIONS로 버튼 생성(“인라인” — 팝업/다이얼로그
  금지), 1단 클릭 → 원인·상태 라벨 + 확인 스트립 노출, ConfirmFlow 소비,
  arm 버튼은 press/release 시각으로 held_s 계산. estop_reset·arm 사이
  비명령 스페이서 위젯. ACK/거부는 EventLog(event_sink)로.
- 배너: `"OBSERVE: RX-ONLY  |  OPS: TOKEN-GATED  |  "` 로 교체(기존
  READ-ONLY 문구 대체). 독스트링·README는 "관측은 수신 전용, 조작은 게이트된
  ops 채널 경유만" 헌장으로 개정.
- **송신 표면 계약 테스트**(no-send의 실물 대체): 소스 스캔 —
  `operator_console/*.py`에서 outbound 소켓 생성·send는 `ops_client.py`(와
  그것이 위임하는 laptop 채널)에서만; `app.py`·`telemetry.py`·`udp_source.py`·
  `metadata.py`·`pipelines.py`에 `connect(`/`sendall(`/`sendto(` 부재 assert
  (SRT는 GStreamer 파이프라인 문자열이라 소켓 API 아님 — 제외 규칙 주석).
- 토큰 파일 기본 `~/.config/powertrain/ops_console.token`, 부재 시 패널 비활성
  라벨("ops token absent — panel disabled") + 관측은 정상.

- [ ] **Step 1: 실패 테스트** — `test_send_surface_contract.py`(소스 스캔 3 assert: ①금지 파일에 소켓 송신 API 없음 ②배너에 "READ-ONLY CONSOLE" 문자열 부재·"TOKEN-GATED" 존재 ③app.py가 ConfirmFlow·ConsoleOpsClient 사용) + `test_app.py`에 순수 함수 추가분(있다면).
- [ ] **Step 2: RED** → **Step 3: 구현** → **Step 4: GREEN**(호스트 + dev 컨테이너 operator_console)
- [ ] **Step 5: 커밋** `feat: console gated ops panel; charter revised to token-gated command surface`

---

### Task 5: 햅틱 arbiter (순수) + Tier 1 패턴

**Files:**
- Create: `motor_control/laptop/haptic_arbiter.py`
- Test: `motor_control/laptop/tests/test_haptic_arbiter.py`

**Interfaces:**
- Produces:

```python
PRIORITY = ("estop", "authority", "link_loss", "proximity", "bypass")
STALE_S = 0.5
@dataclass(frozen=True)
class Rumble: low: float; high: float; duration_ms: int
class HapticArbiter:
    def __init__(self, *, clock): ...
    def feed_ops_state(self, state: dict, received_s: float)
    def feed_event(self, kind: str, detail: str = "")   # chord_progress/ack/nack
    def decide(self) -> Rumble | None
        # 우선순위 최상 1개만. ops-state age>STALE_S → link_loss 패턴 강제
        # (stale 데이터로 긍정 패턴 금지 — 레드팀 3c). estop_latched → estop
        # 패턴. authority 전이 감지(직전 값 대비) → 1회 펄스. proximity =
        # safety_distance_mm<400 반비례 강도. bypass 활성 → 저강도 상시.
    def lightbar(self) -> tuple[int,int,int] | None     # Tier2 소비용 상태색
```

- [ ] **Step 1: 실패 테스트** — 7케이스: ①stale(>0.5 s)이면 다른 조건 불문 link_loss ②estop이 authority보다 우선 ③전이 1회 펄스(같은 상태 반복 없음) ④proximity 강도 반비례 ⑤bypass 단독 시 저강도 ⑥chord_progress/ack 이벤트 패턴 ⑦lightbar 상태색 매핑(AUTONOMY 파랑/TELEOP 흰/hold 노랑/E-stop 빨강).
- [ ] **Step 2~4: RED→구현→GREEN** (호스트 laptop tests)
- [ ] **Step 5: 커밋** `feat: haptic arbiter - single-pattern priority with stale-state defense`

---

### Task 6: pydualsense 출력 통합 (격리 스레드, feature-flag)

**Files:**
- Create: `motor_control/laptop/dualsense_output.py`
- Modify: `motor_control/laptop/remote_operation_client.py` (`--haptics/--no-haptics` 기본 on, `--trigger-fx` 기본 off)
- Test: `motor_control/laptop/tests/test_dualsense_output.py`

**Interfaces:**
- `DualSenseOutput(arbiter, *, backend_factory=None, clock)`: 격리 스레드
  `run_once()` 20 Hz — arbiter.decide() → backend.rumble/lightbar,
  `--trigger-fx` 시 무권한 잠김(강저항+펄스 프로파일 — 풀강성 금지)·
  슬립 플러터 자리(C1 전 비활성 주석). **backend 예외 1회 → 출력 영구
  비활성 + 경고 1줄, 입력·프로세스 무영향(bare-except)**. backend_factory
  기본 = pydualsense lazy import(부재 시 None → 비활성).
- 클라이언트 통합: 메인 루프에서 `arbiter.feed_ops_state(ops.latest_state())`·
  chord 이벤트 feed. pygame-free import 계약 유지(모듈 최상위 pydualsense
  import 금지 — factory 내부 lazy).

- [ ] **Step 1: 실패 테스트** — 5케이스(가짜 backend): ①arbiter 결정이 backend 호출로 전달 ②backend 예외 후 영구 비활성·이후 호출 없음 ③pydualsense 부재(factory→None) 시 조용히 비활성 ④trigger-fx off면 트리거 API 미호출 ⑤module import가 pydualsense 없이 성공.
- [ ] **Step 2~4: RED→구현→GREEN** + `test_remote_operation_client.py` pygame-free 계약 유지 확인
- [ ] **Step 5: 커밋** `feat: isolated DualSense output thread driven by the haptic arbiter`

---

### Task 7: 문서·3환경·젯슨 실기 (리뷰어 주도)

- [ ] 관측성 계획 :329-334 + Task 6 acceptance 개정(게이트드 ops 채널 문구), 마스터플랜 콘솔 read-only 언급 개정, 프로젝트 CLAUDE.md operator_console 줄 갱신(§3.3 헌장), 핸드오프 §2 A2b 행+기준선.
- [ ] 3환경 회귀 전부 green (신규 수 반영).
- [ ] **젯슨 실기**: pull→parity→콘솔 토큰으로 :9001 핸드셰이크(role=console)→`arm_lock_override` params 검증 거부 왕복(스택 미가동 상태 의미론)→`powertrain_control` 재기동 불필요(콘솔은 노트북측). 콘솔 GUI 육안·햅틱 체감은 벤치 이월 목록에 기록.
- [ ] 커밋 `docs: A2b chain + charter revision` + push + 젯슨 pull.

## 완료 기준

- 패널 명령 전부 ConfirmFlow(STRIP/HOLD·revision 재검증) 경유, estop_reset/arm 분리 제스처+스페이서.
- 송신 표면 계약 테스트로 "ops 클라이언트 외 제어 송신 없음" 봉인(헌장 개정의 실물 증거).
- 햅틱: 단일 패턴 중재·stale 방어·백엔드 장애 격리 — 전부 순수 테스트.
- 벤치 이월: 콘솔 GUI 육안(2단 확인 UX), 햅틱/트리거 체감, chord 실감(누적).
