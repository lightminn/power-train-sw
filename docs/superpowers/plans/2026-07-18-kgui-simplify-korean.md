# KGUI 구현 계획 — 콘솔 간소화 + 한국어화

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경(+pipefail) +
> **완료 선언 규칙(CLAUDE.md): runtime_smoke 실기동 게이트 + 실전 경로 E2E 필수.**

**Goal:** 스펙 `docs/superpowers/specs/2026-07-18-console-simplify-korean-design.md`
— Ops 패널을 광운대 수준(비상정지·경고 초기화·시동·모듈 4스위치)으로 줄이고
나머지는 "고급" 접기, 콘솔 전면 한국어화(상태코드 병기), 텔레메트리 요약+상세
접기, **콘솔 비상정지 버튼 신설**(무확인 즉시).

**Architecture:** 순수 `labels.py` 한국어 상수 + `ops_panel.py` 액션 표 재구성
(`advanced` 플래그·`GESTURE_IMMEDIATE`) + chassis `~/estop` Trigger 서비스 +
broker OpsState `chassis_mode`(semantic) + telemetry 순수 요약 함수 + Gtk
Expander 배치. **msg 변경 없음.**

**기준선(현재):** 호스트 115(operator_console+tests) / ros 533 / dev 1192+2skip / 젯슨 상시 chassis healthy.

## Global Constraints

- 모듈 4종 기본값 **전부 켜짐**(CMASK 무영속 계약 불변 — UI가 바꾸지 않음).
- 비상정지 = 무확인 즉시(확인창 금지). 해제(Reset)는 기존 2단 유지.
- 한국어 + 상태코드 병기: `비상정지(ESTOP)`, `대기(IDLE)`, `정상수신(LIVE)`,
  `지연(STALE)`, `미수신(UNAVAILABLE)`. 저널/저널테일 원문 영문 유지.
- 송신봉인 계약(test_send_surface_contract) 무저촉, §9-3 getattr 관례.
- 기존 액션 이름·프로토콜(ops_contract 액션 키) 불변 — 라벨만 한국어.

---

### Task 1 (ros): chassis `~/estop` 서비스 + ops `estop` 액션 + OpsState `chassis_mode`

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`(서비스 등록
  `~/estop`, 기존 `_srv_reset_estop`/`arm` 등록부 옆), `ops_contract.py`
  (ACTIONS에 `"estop": ActionSpec(_CONSOLE, "service", ("/chassis_node/estop",))`),
  `ops_broker_core.py`(OpsState에 `chassis_mode: str = "UNKNOWN"`),
  `ops_broker_node.py`(safety_state 파싱에 `mode` → `_fields["chassis_mode"]`,
  `_SEMANTIC_FIELDS`에 추가, `_ops_state`·push JSON에 포함)
- Test: `test_ops_contract.py`·`test_ops_state_sources.py`·
  `test_component_mask_wiring.py`(또는 신규 소형 파일) 케이스 추가

**Interfaces (Produces):**
- Trigger `~/estop`: 항상 `success=True`(이미 latch면 멱등), `message="mode=<현재모드>"`.
  핸들러는 `self.cm.estop("console", "operator emergency stop")` 후 snapshot
  모드 반환. §9-3 getattr 가드.
- OpsState.chassis_mode: safety_state JSON `"mode"` 문자열, 부재 시 "UNKNOWN",
  semantic(변경 시 revision 증가). push JSON 키 `"chassis_mode"`.
- [ ] RED: ①estop 서비스 왕복(RUN→ESTOP latch, 재호출 멱등) ②ops 계약
  (role console·kind service·target) ③OpsState chassis_mode 파싱·기본값·
  semantic revision ④push JSON에 포함.
- [ ] 구현→GREEN(ros 컨테이너 전체)→커밋
  `feat: console estop service + chassis_mode in ops state`

---

### Task 2 (console): labels.py + Ops 패널 재구성(기본/고급·즉시 비상정지·모드 게이트)

**Files:**
- Create: `operator_console/labels.py`
- Modify: `operator_console/ops_panel.py`, `operator_console/app.py`(OpsPanel)
- Test: `operator_console/tests/test_labels.py`(신규),
  `test_ops_panel.py` 갱신(라벨 한국어·기본/고급 분류·IMMEDIATE)

**Interfaces (Produces):**

```python
# labels.py (순수 — Gtk import 금지)
COMPONENT_KOREAN = {"drive": "구동 모터", "steer": "조향 모터",
                    "us100": "US-100 안전", "robot_arm": "로봇팔"}
ON_LABEL, OFF_LABEL = "켜짐", "꺼짐"
def mode_korean(mode: str) -> str        # "IDLE"→"대기(IDLE)", "ESTOP"→"비상정지(ESTOP)",
                                         # "ARMED"→"주행(ARMED)", 미지→"<원문>"
def freshness_korean(state: str) -> str  # LIVE→정상수신(LIVE)/STALE→지연(STALE)/
                                         # UNAVAILABLE→미수신(UNAVAILABLE)/WAITING→대기중(WAITING)
def ack_korean(status: str, detail: str) -> str
    # FINAL_SUCCESS→"성공", FINAL_REJECTED→"거부 — <사유 한국어>",
    # OUTCOME_UNKNOWN→"결과 미확정 — 재시도 가능". 사유 매핑:
    # not_idle→"대기(IDLE) 상태에서만 가능", busy: mutation in flight→"다른 명령 처리 중",
    # service unavailable→"대상 노드 없음", 그 외 원문 병기.
```

- ops_panel.py: `PanelAction.advanced: bool = False`,
  `GESTURE_IMMEDIATE = "immediate"`(확인 없이 클릭 즉시 submit — ConfirmFlow
  우회하되 상태 revision 검증은 유지). PANEL_ACTIONS 재구성(한국어 라벨):
  기본 = `estop`("비상정지 (ESTOP)", IMMEDIATE) · `estop_reset`("경고 초기화") ·
  `arm`("시동 — 1.5초 홀드", HOLD) · `disarm`("시동 해제") · 컴포넌트 4행
  (`COMPONENT_KOREAN` + 현재 [켜짐/꺼짐]); 고급(advanced=True) =
  authority 3종("권한: 수동/자동/대기") · `extraction_grant`("구조 탈출 허가 —
  후진 0.2 m/s·3초") · `arm_lock_override`("로봇팔 잠금 해제") ·
  `clear_transient_hold`("일시 정지 해제"). 확인문구 전부 한국어(US-100
  끄기 위험 문구 유지).
- app.py OpsPanel: 기본 버튼들 위, `Gtk.Expander(label="고급")` 안에 advanced
  행들. 비상정지 버튼 빨강(`Gtk.Button` + css class 또는 markup label) + 높이
  여유. 상태줄 `모드: <mode_korean> · 최근: <ack_korean>`. **모터 토글
  (drive/steer)은 chassis_mode != "IDLE"이면 비활성 + 라벨에 "· 대기에서만"**.
  us100/robot_arm 토글은 항상 활성. IMMEDIATE 액션은 클릭 → 즉시
  `client.submit`.
- [ ] RED: labels 헬퍼 전 케이스·액션 표 분류(기본 7행/고급 5행 상수 검증)·
  IMMEDIATE estop 무확인 제출·모터 토글 모드 게이트(순수부)·ack_korean 매핑.
- [ ] 구현→GREEN(호스트 operator_console)→커밋
  `feat(console): korean labels + simplified ops panel with immediate estop`

---

### Task 3 (console): 배너·패널 한국어화 + 텔레메트리 요약/상세 접기 + 스모크 반영

**Files:**
- Modify: `operator_console/telemetry.py`(요약 순수 함수 3종),
  `operator_console/app.py`(배너·Link 행·패널 제목 한국어, 각 텔레메트리
  패널 요약 1줄 + `Gtk.Expander("상세")`로 기존 grid 이동),
  `operator_console/arm_telemetry.py`(표시 문자열은 app 쪽이므로 수정 없으면
  무변경), `operator_console/runtime_smoke.py`(chassis payload에
  `"mode": "IDLE"` 추가 — 요약 경로 실행)
- Test: `operator_console/tests/test_app.py`·`test_arm_telemetry.py` 케이스

**Interfaces:**

```python
# telemetry.py 추가 (순수)
def power_summary(snapshot) -> str    # "47.6 V · 80% · 정상" / 부재 "미수신(UNAVAILABLE)"
def chassis_summary(snapshot) -> str  # "모드 대기(IDLE) · 안전 정상 · 바퀴 6/6"
                                      # (safety_estop_required→"비상정지(ESTOP)",
                                      #  바퀴 fault/stale 있으면 "바퀴 5/6 ⚠")
def arm_summary(snapshot) -> str      # "모터 2 · 최고 45 ℃ 정상" / CRIT면 "⚠ 65 ℃"
```

- 배너: `관측: 수신 전용 | 조작: 토큰 인증 | L515 …` + 안전 배너
  `안전 정상(CLEAR)`/`비상정지(ESTOP) · <사유>`/`안전 해제됨(US-100 꺼짐)` +
  `꺼짐: 구동·US-100`(주황). 패널 제목: "로봇 상태"/"차대"/"로봇팔"/"조작
  (토큰 인증)"/"이벤트 기록". Link 행 `freshness_korean` 사용.
- chassis_summary의 모드 원천: `/chassis/safety_state`는 :5005 payload에
  없으므로 snapshot.component_mask처럼 **:5005에 이미 있는 필드만** 사용 —
  모드는 drive_state 문자열(`"IDLE/OK"` 식)에서 앞부분을 취해 mode_korean
  매핑(없으면 "미수신"). (ops 채널 모드는 Ops 패널 몫 — 이중화 불필요.)
- [ ] RED: 요약 3종(정상·부재·경고 케이스), 배너 문자열(mask_banner_text
  한국어 갱신 — 기존 테스트 기대값 교체), freshness 표기.
- [ ] 구현→GREEN(호스트 operator_console+tests **runtime_smoke 포함 통과**)→커밋
  `feat(console): korean banners + telemetry summaries with detail expanders`

---

### Task 4: 검증·배포·문서 — 리뷰어 주도

- [ ] 3환경 green + **runtime_smoke PASS**(한국어 UI 실기동) + 젯슨 배포
  (`--force-recreate powertrain_control powertrain_chassis` — broker/chassis
  코드 반영).
- [ ] **비상정지 실전 E2E**(ops 클라이언트): `estop` → safety_state
  mode=ESTOP·latch 확인 → `estop_reset` → IDLE 복구. 콘솔 육안은 사용자 몫으로
  안내(기본 화면 구성·한국어).
- [ ] 문서: Notion GUI 페이지 §6 유저 매뉴얼을 한국어 버튼 기준으로 갱신
  (버튼명 표), 핸드오프 §2 행, README 1줄, 메모리.
- [ ] 커밋·push·젯슨 pull.

## 완료 기준

- 기본 화면 = 비상정지·경고 초기화·시동/해제·모듈 4스위치(기본 전부 켜짐)만,
  고급은 접힘. 전면 한국어(상태코드 병기).
- 비상정지 버튼 실전 E2E PASS + runtime_smoke PASS + 3환경 green.
