# 로봇팔 명령 ops 계약 설계와 순수 검증 골격

작성일: 2026-09-10
브랜치: `claude/arm-console-ui-skeleton`
작업 트리: `/home/jo/ros2_ws/power-train-sw/.claude/worktrees/claude-arm-ui`
관련: `docs/reports/2026-09-10-claude-arm-ui.md` (UI 골격), `docs/plans/2026-09-10-arm-console-ui.md` (Codex 소유)

> **이 문서가 보고하는 범위는 계약 설계와 순수 검증 골격까지다.**
> 로봇팔 실제 명령은 **발행되지 않으며, 현재 구조상 발행할 수 없다.** ROS·CAN·USB·소켓
> 연결, 모터 구동, 보정 계산·저장은 구현하지 않았다. `extreme-robot`은 수정하지 않았다.

---

## 1. 조사 결과 — 기존 ops 구조와 그것이 강제한 설계

`ops_contract.py` · `ops_broker_core.py` · `ops_client.py` · `ops_channel_client.py`를
읽고 확인한 사실 세 가지가 이번 설계를 사실상 결정했다.

### 1.1 `ACTIONS`는 닫힌 dict다

`ops_contract.decode_request()`는 `payload["action"] not in ACTIONS`이면 요청을 거부한다.
따라서 **액션 등록 없이는 어떤 팔 명령도 채널을 통과할 수 없다.**

이번 작업은 `ops_contract.py`를 **수정하지 않았다.** 결과적으로 콘솔은 팔 명령을
만들고 검증할 수는 있지만 전송할 수는 없다. 이것은 제약이 아니라 이번 단계에서
원하는 안전 속성이며, 테스트로 고정했다
(`test_arm_actions_are_not_registered_in_the_ops_contract`).

로컬에서 먼저 거부하는 이유가 있다. 등록되지 않은 액션을 그냥 보내면 네트워크를 한 번
돌아 `unknown action`으로 거부되는데, 운용자 자리에서는 그것이 **팔이 안전상의 이유로
거부한 것**과 구분되지 않는다. `ContractGatedTransport`가 로컬에서 "미등록"이라는
사유로 거부하면 "아직 연결 안 함"과 "팔이 거부함"이 서로 다른 문장으로 남는다.

### 1.2 브로커는 변경 명령을 직렬화한다

`ops_broker_core.handle_line()`은 `if self._pending_orders:` 이면
`busy: mutation in flight`로 거부한다. 즉 **동시에 하나의 변경 명령만** 처리된다.

이 사실이 누름 조그 설계를 강제했다. 20 Hz로 조그를 그대로 흘려보내면 대부분 거부되거나,
`ConsoleOpsClient`의 16칸 송신 큐(`SEND_QUEUE_MAXLEN`)에 적재되어 **손을 뗀 뒤에도
살아 있는 백로그**가 된다. 그래서 조그는 **최신값 방식**이다 — 동시에 outstanding인
조그 요청은 최대 1건이고, 새 값은 미전송 이전 값을 뒤따르지 않고 **대체**한다.

### 1.3 기존 계약의 어휘를 그대로 쓴다

- 응답 상태 `PENDING / FINAL_SUCCESS / FINAL_REJECTED / OUTCOME_UNKNOWN`을 그대로 쓴다.
  `operator_console.labels.ack_korean()`이 팔 ACK도 차체 ACK와 동일하게 렌더한다.
  **`OUTCOME_UNKNOWN`은 성공으로도 실패로도 접지 않는다.**
- `expected_state_revision`을 낙관적 동시성 가드로 그대로 쓴다.
- 액션 이름은 전부 `robot_arm_` 접두사다. 기존 어휘에서 `arm`은 **차체 시동**,
  `robot_arm_enable`은 컴포넌트 토글이다. 접두사 없는 `arm_stop`은 그 오해가 가장
  비싼 순간에 차체 명령으로 읽힌다.

---

## 2. 확정한 콘솔 측 계약

### 2.1 액션 17개

| 의도 | 액션 | 주요 params |
|---|---|---|
| 도구 변경 요청 | `robot_arm_tool_change` | `requested_kind`, `observed_tool_generation` |
| 제어권 요청 | `robot_arm_mode_request` | `mode` (MANUAL\|FSM) |
| 제어권 반납 | `robot_arm_mode_release` | — |
| 조그 시작·최신값 갱신 | `robot_arm_jog` | `axis`, `direction`, `speed_level?`, `expires_at_s`, `sequence` |
| 조그 명시 종료 | `robot_arm_jog_stop` | `axis`, `reason`, `sequence` |
| 정지 | `robot_arm_stop` | — |
| 복귀 | `robot_arm_resume_hold` | — |
| home | `robot_arm_home` | — |
| 자세 저장/이동/삭제 | `robot_arm_pose_save` / `_move` / `_delete` | `name` |
| 그리퍼 open/close/stop | `robot_arm_tool_command` | `target`, `command`, 도구 신원 |
| 그리퍼 누름 조그 | `robot_arm_tool_jog` | `target`, `direction`, 도구 신원, 수명 |
| 그리퍼 조그 종료 | `robot_arm_tool_jog_stop` | `target`, `reason`, `sequence`, 도구 신원 |
| 팔·도구 보정 | `robot_arm_calibration` | `scope`, `step`, `action`, 도구 신원(tool scope) |
| 보정 조그 | `robot_arm_calibration_jog` | `target`, `direction`, 도구 신원, 수명 |
| 보정 조그 종료 | `robot_arm_calibration_jog_stop` | `target`, `reason`, `sequence`, 도구 신원 |

`validate_params()`는 **알 수 없는 필드를 무시하지 않고 거부**한다. 조용히 버려진
오타 필드는 작성자가 쓴 것과 다른 뜻의 명령이 되기 때문이다.

### 2.2 조그 수명 계약 (deadman)

- 모든 조그 요청은 `expires_at_s`를 싣는다. **백엔드는 그 시각을 넘기면 스스로
  정지해야 한다.** 콘솔의 명시적 stop은 빠른 경로이지 안전망이 아니다.
- 콘솔도 같은 데드라인으로 hold를 만료시킨다(`JogRegistry.expire`). 양쪽이 모두
  멈추는 것이 정상이고, 한쪽만 멈추는 것이 이 설계가 막으려는 결함이다.
- 기본값 TTL 0.6 s / 갱신 0.2 s는 **잠정값**이며 생성자 인자다. 실제 값은 ops 왕복
  실측과 rate limit에서 나와야 한다(§4 `jog_rate_and_serialization`).

### 2.3 정지 의도는 전송보다 오래 남는다

운용자가 아닌 다른 원인으로 hold가 끝나면 — 도구 교체, 제어권 상실, 통신 단절, 탭
이탈, 캘리브레이션 시작 — 정지는 **pending**으로 기록되고 전송이 확인될 때까지 남는다.
보내지 못한 정지는 "보낼 필요가 없었던 정지"가 아니므로, 재연결 후 다시 시도한다.

정지 사유는 명령에 함께 실린다: `explicit` · `expired` · `authority_lost` ·
`link_lost` · `tool_changed` · `focus_lost` · `calibration_started`.
같은 채널에 정지가 두 번 쌓이면 **첫 사유가 이긴다** — 실제로 모션을 끝낸 것은 그쪽이다.

정지는 **직렬화된 슬롯을 갱신보다 먼저** 차지한다. 브로커가 한 번에 하나만 처리하는데
갱신이 그 자리를 가져가면 모션을 끝내는 바로 그 명령이 밀린다.

---

## 3. 안전 조건 — 코드와 테스트로 강제한 것

`policy.authorize()`가 유일한 판정 지점이고, 순수 함수이며, 위반한 명령은 **객체 자체가
만들어지지 않는다.** UI의 `arm_ui.contracts.gate`는 첫 번째 관문(표시)이고 이것이 두
번째 관문(생성)이다.

| 조건 | 규칙 | 테스트 |
|---|---|---|
| capability 부재 | 명령 **생성 불가**. 비활성이 아니라 거부 | `test_a_missing_capability_makes_the_command_unbuildable`, 액션별 파라미터화 |
| 정지 우선 | stop은 capability·제어권·통신 없이도 항상 허용 | `test_a_stop_is_allowed_with_no_capability_no_grant_and_no_link` |
| 신선도 | LIVE가 아니면 stop 외 전부 거부. ops revision 없으면 거부 | `test_a_non_live_link_blocks_every_non_stop_command` |
| 요청 ≠ 승인 | `granted_mode == MANUAL`만 모션을 연다. `requested_mode`로는 열리지 않는다 | `test_no_motion_without_an_approved_manual_grant`, `test_a_requested_but_ungranted_mode_projects_to_no_grant` |
| 제어권 요청은 열어둠 | 막으면 승인을 영영 못 받는다 | `test_requesting_the_grant_stays_available_without_one` |
| 캘리브레이션 중 | 일반 조그·도구 변경 차단, 보정 명령만 허용 | `test_a_calibration_session_blocks_general_jog_and_tool_change` |
| 도구 교체 중 | 교체 요청 외 전부 차단 | `test_a_tool_change_in_flight_blocks_everything_but_the_change` |
| 도구 신원 | `tool_id`·`tool_generation` 불일치 명령 거부 | `test_a_command_for_a_different_tool_generation_is_refused` |
| 교체된 도구의 정지 | 세대 불일치여도 **허용** — 움직이는 쪽을 멈춰야 한다 | `test_a_stop_for_a_superseded_tool_is_still_allowed` |
| 외부 사건 → 정지 | 교체·권한 상실·단절·탭 이탈·보정 시작 각각이 hold를 끝낸다 | `test_a_context_change_ends_holds_with_the_matching_reason` (6 케이스) |
| 미전송 정지 잔존 | 단절 중 정지는 pending으로 남고 재연결 후 전달된다 | `test_a_stop_that_cannot_be_delivered_stays_pending_and_is_retried` |
| press→pump 사이 권한 상실 | 갱신을 보내지 않고 정지로 전환 | `test_a_grant_withdrawn_between_press_and_pump_stops_instead_of_sending` |
| 결과 미확정 | `OUTCOME_UNKNOWN`도 슬롯을 푼다 — 응답 1건 유실이 hold를 영구 고착시키면 안 된다 | `test_an_unknown_outcome_frees_the_slot_rather_than_wedging_the_hold` |
| UI ↔ 정책 어휘 일치 | capability·모드·타깃·보정 토큰이 동일 문자열 | `test_capability_tokens_match_the_arm_ui_contract_exactly` 외 3건 |
| 표시 ↔ 생성 일치 | 비활성 위젯과 생성 불가 명령이 일치 | `test_the_ui_disables_exactly_what_the_policy_would_refuse` (GTK) |

**기본값은 전부 거부다.** `ArmCommandContext()`는 통신 없음·승인 없음·capability 없음이며,
그 상태에서 허용되는 것은 정지뿐이다(`test_default_context_permits_nothing_except_stops`).

---

## 4. 미확정 — 팔 측 어댑터가 구현해야 할 계약

이 문서는 **콘솔이 무엇을 요청할 수 있는지**만 확정한다. 아래는 확정하지 않았고,
추정하지도 않았다. 코드에서는 `contract.BACKEND_REQUIREMENTS` 9건으로 존재하며
전부 `resolved=False`다.

| key | 내용 |
|---|---|
| `action_registration` | `ops_contract.ACTIONS`에 `robot_arm_*` 액션과 각각의 ROS 대상 등록. **등록 전까지 전송 불가** |
| `grant_report` | MANUAL/FSM 승인 상태 회신 경로. **요청 성공 ACK는 승인이 아니다** |
| `jog_deadman` | `expires_at_s` 경과 시 백엔드 자체 정지. 콘솔 stop 도착과 무관 |
| `jog_rate_and_serialization` | 조그 갱신 주기·rate limit·최신값 병합 정책 실측 확정 |
| `tool_generation_authority` | 활성 도구 세대의 정본과 증가 규칙. 백엔드도 불일치를 거부해야 함 |
| `single_ownership` | 키보드 텔레옵 경로와 MoveIt/팔 FSM 경로의 모터 단일 소유권 |
| `stop_semantics` | stop(토크 해제)·resume(현재 자세 유지)·home(저장 자세) 의미 보존 확인 |
| `calibration_persistence` | 임시 적용 / 가동범위 저장 / 도구 YAML 저장의 구분 |
| `token_role` | 팔 명령에 필요한 역할과 토큰 발급 정책 |

ROS 토픽·서비스 이름은 이 패키지 어디에도 없다. 테스트가 이를 강제한다
(`test_the_package_never_names_a_ros_topic_or_service`).

의도적으로 **연결하지 않은 콜백 2개**도 같은 이유다:

- `select_axis` — 축 선택은 화면 상태 변경이며 ops 의미가 없다.
- `change_speed` — 속도가 팔 측에서 어떻게 표현되는지 확정되지 않았다. 추측해서 묶으면
  아무도 합의하지 않은 명령이 전선에 오른다. 조그 갱신에 `speed_level`을 실을 자리는
  계약에 있으나 비어 있다.

둘 다 콜백이 `None`이라 UI에서 자동으로 비활성이며, 이것이 "아직 연결 안 됨"의 정직한
표시다.

---

## 5. 구조

```
operator_console/arm_ops/
├── contract.py    액션·파라미터 스키마·미확정 백엔드 요구사항. ROS 이름 없음
├── policy.py      순수 판정: 이 맥락에서 이 명령이 존재할 수 있는가
├── jog.py         누름 조그 수명: TTL·최신값 병합·잔존 정지
├── session.py     오케스트레이터. ArmOpsRequest를 만드는 유일한 곳
├── transport.py   유일한 출구. 현재 닫혀 있음
└── adapter.py     ArmUiCallbacks 연결. arm_ui.contracts만 import
```

**의존 방향**: `adapter → session → {policy, jog, contract}` / `session → transport`.
`contract`·`policy`·`jog`·`session`·`transport`는 GTK를 import하지 않으며, 표시장치
없이 import·테스트된다. `arm_ui` 패키지 `__init__`이 GTK 탭을 끌어오므로 어댑터
export만 지연 로딩한다(`test_the_pure_core_imports_without_gtk`).

**전송 기본값은 거부다.** `NullArmTransport`가 production 기본값이고 전부 거부한다.
받아들이는 가짜(`RecordingArmTransport`)는 **패키지 루트에서 접근 불가**하다 — "보내지
않는 전송"을 원하는 통합이 실수로 집어들면 동작하는 명령 채널처럼 보이기 때문이다
(`test_the_recording_transport_is_not_reachable_from_the_package_root`).

---

## 6. 통합 접점 (Codex / 후속 연결)

`app.py`는 **수정하지 않았다.** 연결 시 필요한 것:

```python
from operator_console.arm_ops import (
    ArmCommandSession, ArmOpsCallbackAdapter, ContractGatedTransport,
    registered_actions_from,
)

transport = ContractGatedTransport(
    submit_fn=ops_client.submit,                       # 기존 ConsoleOpsClient.submit
    registered_actions=registered_actions_from(ops_contract),
)
session = ArmCommandSession(clock=time.monotonic, transport=transport)
adapter = ArmOpsCallbackAdapter(session, outcome_sink=show_ack)

manual = ArmManualTab(adapter.callbacks())
calibration = ArmCalibrationTab(adapter.callbacks())

adapter.update_state(state, state_revision=ops_revision)  # 상태 갱신마다
adapter.pump()                                            # 타이머마다 (조그 갱신·정지)
adapter.blur()                                            # 탭 이탈·포커스 상실
adapter.on_disconnect()                                   # ops 연결 끊김
```

`registered_actions`를 주입식으로 둔 이유는 `operator_console`이 "무엇을 보낼 수
있는가"를 답하려고 ROS 패키지에 의존하면 안 되기 때문이다. 기본값은 빈 집합이고,
그것은 오늘 사실이며 동시에 안전한 가정이다.

---

## 7. 검증 결과

실행: 호스트, `/usr/bin/python3`. GTK 테스트는 `broadwayd` 헤드리스 백엔드
(이 호스트에 Xvfb 미설치).

```bash
/usr/bin/python3 -m pytest operator_console/tests/arm_ops -q          # 순수만
broadwayd :7 & GDK_BACKEND=broadway BROADWAY_DISPLAY=:7 \
    /usr/bin/python3 -m pytest operator_console/tests/arm_ops -q      # GTK 포함
```

| 스위트 | 결과 |
|---|---|
| `operator_console/tests/arm_ops` (표시장치 포함) | **197 passed** |
| `operator_console/tests/arm_ops` (표시장치 없음) | 189 passed, GTK 8건 수집 생략 |
| `operator_console` 전체 | §7.1 참조 |

### 7.1 구현 중 실제로 잡은 결함

1. **`arm_ops` import가 GTK를 끌어왔다.** `adapter.py`가 `arm_ui.contracts`를
   import하면 `arm_ui/__init__`이 먼저 실행되어 GTK 탭 모듈이 딸려 온다. 순수 계층이
   표시장치를 요구하게 되므로 어댑터 export를 지연 로딩으로 바꿨다.
2. **필드 길이 상한과 바이트 상한이 모순됐다.** 80자 한글 `tool_id`는 JSON에서
   문자당 6바이트로 이스케이프되어 512바이트 상한을 넘었다 — 계약상 합법인 명령이
   생성 불가였다. 상한을 1200으로 올리고, **액션별 최대 길이 파라미터가 예산 안에
   들어가는지**를 테스트로 고정했다.

### 7.2 미검증 (완료로 표현하지 않는다)

- **실물 로봇팔·ROS·모터와의 연결은 전혀 검증하지 않았다.** 명령은 발행되지 않았다.
- ops 브로커와의 실제 왕복은 검증하지 않았다. 액션이 미등록이므로 왕복 자체가 불가능하다.
- 조그 TTL·갱신 주기의 실측 근거가 없다. 현재 값은 잠정이다.
- rate limit(`_rate_ok`)과 조그 갱신 주기의 상호작용은 실측 미확인이다.
- `OUTCOME_UNKNOWN` 이후의 재시도 정책은 슬롯 해제까지만 정의했고, 재전송 여부는
  기존 ops 멱등 재전송에 맡긴다 — 실연결에서 확인해야 한다.
- 기존 주행·영상·복구 회귀는 콘솔 스위트 통과로만 확인했다. 실기 회귀는 별도다.

---

## 8. 실연결 전제

이 골격이 실제 명령을 보내려면 순서대로 필요하다.

1. `BACKEND_REQUIREMENTS`의 `action_registration` 해소 — `ops_contract.ACTIONS`에
   `robot_arm_*` 등록과 ROS 대상 지정. 이 시점에
   `test_arm_actions_are_not_registered_in_the_ops_contract`가 실패하며, 그것은
   **삭제할 것이 아니라 이 문서의 §8을 갱신하라는 신호**다.
2. `grant_report` 해소 — 승인 상태 회신 경로. 없으면 모션은 계속 열리지 않는다.
3. `jog_deadman` 확인 — 백엔드가 만료 시 스스로 멈추는지 실측.
4. `single_ownership` 확정 — 텔레옵 경로와 FSM 경로의 모터 소유권.
5. `jog_rate_and_serialization` 실측 후 TTL·갱신 주기 확정.
6. 그 다음에야 `ContractGatedTransport`에 실제 `registered_actions`를 주입한다.

1~5 이전에 6을 하면, 미확정 의미의 명령이 실제 팔에 도달한다.
