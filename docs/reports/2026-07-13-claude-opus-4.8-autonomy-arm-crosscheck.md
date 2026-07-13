# 2026 국방로봇 자율주행 계획 ↔ 로봇팔 계약 독립 교차검토

> 검토일: 2026-07-13
> 검토자: Claude Opus 4.8 (독립 수석 리뷰, 안전중요 이동로봇·ROS2 관점)
> 대상: `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md` (이하 **계획**)
> 대조 근거: `ros2/src/powertrain_ros/powertrain_ros/contract.py`,
> `ros2/src/robot_arm_msgs/msg/*.msg` (5종), `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`,
> `ros2/README.md`, `AGENTS.md` 최신 CURRENT STATE OVERRIDE 및 robot-arm team resources,
> `motor_control/chassis/teleop_server.py`
> 범위: **2026 국방로봇경진대회만.** 극한로봇대회·CAD·전장·기구는 검토 대상 아님.
> 이 문서는 판정 보고서이며, 기존 파일은 일절 수정하지 않았다.

---

## 0. 최종 판정

**조건부 반려 (Conditional Reject) — S0 2건을 계획 문구에서 먼저 고친 뒤 승인.**

계획의 **안전 철학과 시스템 설계 방향은 대체로 타당하다.** 특히 `DONE` 단독 재출발 금지,
실제 wheel 정지 확인 후 `MISSION_STOP` 송신, `mission_id` 멱등성, fail-open 금지,
motion hold와 latched E-stop의 분리, 단일 소유권(L515 Gateway / `ChassisManager` / D435i)
원칙은 이 등급의 로봇 프로젝트에서 흔히 빠지는 함정을 정확히 피하고 있다.

그러나 계획은 **"팔 잠금 계약이 자율·원격 양쪽에 적용된다"는 안전 주장을 하면서,
그 주장이 성립하지 않는 현재 코드 두 곳을 인지하지 못했다.**

1. **원격 경로는 `chassis_node`·`/cmd_vel`·`command_authority`를 통째로 우회한다.**
   `teleop_server.py`가 `ChassisManager`를 직접 생성해 `can0`를 독립 소유한다. 계획 §7과
   §4.2·§10의 "원격도 동일한 팔 잠금·파지 계약을 통과한다"는 **현재 아키텍처에서 물리적으로
   불가능**하며, 계획에 이를 바꾸는 WP가 없다.
2. **살아 있는 계약이 fail-open이다.** `chassis_node`는 launch 파라미터에 박힌
   `DRIVING`(= 팔 언락)을 상태와 무관하게 계속 발행하고, 팔 측은 `LOCK_MODES` **거부목록**
   방식이라 "모르는 값 = 언락"이다. 계획은 `DRIVING` 의미 폐기를 선언했지만
   **대체 `ChassisMode` 어휘와 팔 측 fail-closed 반전을 지정하지 않았다.**

이 둘은 각각 "운반 중 원격 전환 시 팔 잠금 미검사 주행"과 "주행 중 팔 언락"이라는
**물자 낙하·팔 파손·규정 감점에 직결되는 경로**다. 계획 문구 수정은 소규모(각 3~6줄)이므로
반려가 아니라 **조건부 반려**로 판정한다.

msg 5종 스키마 자체는 **변경 없이 계획을 구현할 수 있다** (§2 참조). 다만 스키마가
"가능"한 것과 계약이 "완결"된 것은 다르며, 아래 S1 4건이 완결을 막고 있다.

| 등급 | 건수 | 의미 |
|---|---:|---|
| S0 | 2 | 안전 주장이 현재 코드에서 거짓. 계획 승인 전 문구 수정 필수 |
| S1 | 4 | 계약 구멍으로 구현 불가 또는 재현 가능한 fail-open/데드락 |
| S2 | 6 | 중대하나 우회 가능. 7/19 계약 확정 전 해소 |
| S3 | 5 | 개발 용이성·운영 품질 개선 |

---

## 1. S0 — 승인 전 반드시 고칠 계획 문구

### S0-1. 원격 경로가 안전 게이트를 우회한다 (계획의 안전 주장이 거짓)

**근거 (파일:행)**

- `motor_control/chassis/teleop_server.py:265` — `from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners`
- `motor_control/chassis/teleop_server.py:293-294` — `cm = ChassisManager(corners, cfg)` / `cm.connect()`
- `motor_control/chassis/teleop_server.py:127` — `manager.set(v_cmd, w_cmd)` (텔레옵이 직접 모터 명령)
- `ros2/README.md:163` — launch preflight의 `CONTROL_RE`가 `[t]eleop`를 **경쟁 CAN 소유자**로 분류해 검출 시 ABORT
- `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:117` — `/cmd_vel` 구독자는 `chassis_node` 하나뿐
- 리포지토리 전체에서 `/cmd_vel` **발행자는 0개** (`grep -rn "create_publisher.*Twist"` → 결과 없음)
- 계획:112 — "`command_authority`만 최종 자율/원격 속도 명령을 `/cmd_vel`에 쓴다."
- 계획:172 — "자율과 원격 모두 같은 팔 잠금·파지 계약을 통과해야 한다."
- 계획:454 — "자율 실패 시 zero-confirmed handover 후 원격으로 전환."
- 계획:455 — "원격 중에도 US-100과 모터 E-stop은 동일하게 적용."
- 계획:608 — 완료조건 "자율→원격 전환에서도 동일한 팔 잠금 조건을 우회하지 않음."

**무엇이 틀렸나**

`teleop_server`는 ROS 노드가 아니다. 자체 소켓으로 DualSense 입력을 받아 자기 `ChassisManager`로
`can0`를 연다. `can0`는 단일 소유자만 가질 수 있으므로 `chassis_node`와 **동시에 실행될 수 없다**.
즉 현재의 "자율→원격 전환"은 handover가 아니라 **프로세스 교체**다.

**실패 시나리오 (2구간, 구호물자 운반 중 자율 실패)**

1. 로봇이 `CARRYING_LOCKED` 상태로 물자를 들고 주행 중 terrain confidence가 무너져 자율 실패.
2. 운영자가 원격 전환 → `chassis_node` 종료 필요 → `ChassisManager.estop()`가 cleanup에서 호출되고
   (`chassis_node.py:396`) CAN 소켓이 닫힌다.
3. `teleop_server`를 새로 기동 → `can0` 재획득, `ChassisManager` 재생성.
4. **새 프로세스는 `/arm_status`를 구독하지 않는다.** `CARRYING_LOCKED` heartbeat도,
   `GRIP_LOST`도 보지 않는다. 계획 §4.2의 "0이 아닌 주행 명령은 fresh한 lock heartbeat가 있을 때만"이
   원격에서는 **집행되는 코드가 존재하지 않는다**.
5. 운반 중 `GRIP_LOST`가 발생해도 원격 로봇은 정상 주행을 계속한다 → 물자 낙하(감점) 또는
   팔이 물자를 끌며 주행 → 팔 파손.
6. 부수적으로: E-stop latch 상태·`mission_id` 카운터·supervisor FSM이 프로세스 교체로 소실되고,
   재기동에 수 초가 걸려 20분 구간 예산을 잠식한다. "zero-confirmed handover"의 zero 확인은
   교체 중 명령 소스가 없는 구간을 보장하지 못한다(모터는 마지막 명령을 유지할 수 있다 —
   ODrive 런어웨이는 `AGENTS.md`에 이미 실측 기록된 실패 모드다).

**최소 수정 (계획 문구)**

§7 첫머리와 §5(공통 기반 작업)에 다음을 추가한다.

> **WP6-T. 원격 경로의 ROS 통합 (신규, WP6-C와 동일 우선순위).**
> 현재 `motor_control/chassis/teleop_server.py`는 자체 `ChassisManager`로 `can0`를 직접 소유해
> `chassis_node`·`/cmd_vel`·`command_authority`·`/arm_status`를 모두 우회한다. 이 상태에서는
> §4.2와 §10의 "원격도 동일한 팔 잠금·파지 계약을 통과한다"가 성립하지 않는다.
> DualSense 텔레옵을 ROS 노드로 전환해 `/teleop/cmd_vel`만 발행하게 하고, `/cmd_vel`의 유일한
> 작성자를 `command_authority`로, `can0`의 유일한 소유자를 `chassis_node`로 고정한다.
> 자율↔원격 전환은 프로세스 교체가 아니라 `command_authority` 내부의 작성자 전환으로만 수행한다.
> 레거시 standalone `teleop_server`는 `chassis_node`가 정지한 경우에만 쓰는 **최후 수단 진단 모드**로
> 강등하고, 이 모드에서는 팔 잠금 계약이 집행되지 않으므로 **팔이 절차적으로 접힘·고정 확인된
> 경우에만** 사용한다고 명시한다.

§10 완료조건(계획:608)은 다음으로 교체한다.

> - 자율↔원격 전환이 `command_authority` 내부 작성자 전환으로 수행되며, 원격 명령도
>   `chassis_node`의 US-100·모터·명령 freshness·팔 잠금 게이트를 동일하게 통과함을
>   프로세스 교체 없이 실증.

---

### S0-2. `ChassisMode` 계약이 fail-open이고, 계획이 대체 어휘를 정의하지 않았다

**근거 (파일:행)**

- `ros2/src/powertrain_ros/powertrain_ros/contract.py:17` — `MODE_DRIVING = "DRIVING"  # 정상 주행 = 팔 언락`
- `ros2/src/powertrain_ros/powertrain_ros/contract.py:22` — `LOCK_MODES = {MODE_CORNERING, MODE_ROUGH_TERRAIN, MODE_FOLLOW_LEAD}`
  → **거부목록(deny-list)**. 팔 입장에서 "이 집합에 없는 모든 문자열 = 언락".
- `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:67` — `self.declare_parameter("mode", contract.MODE_DRIVING)`
- `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:304-308` — `_publish_mode()`가 **launch 파라미터를 그대로** 발행.
  E-stop이든 motion hold든 IDLE이든 값이 바뀌지 않는다.
- `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:175` — `self.create_timer(0.5, self._publish_mode)` → **2 Hz**
- 계획:117-119 — "기존 `DRIVING=팔 언락` 의미는 폐기하고, 모든 주행 모드에서 팔은 접힘·고정 상태여야 한다."
- 계획:151 — "`ChassisMode`와 작업 중 `ArmStatus` heartbeat는 10 Hz, freshness timeout은 `age > 0.5 s`"

**무엇이 틀렸나**

계획은 `DRIVING`의 **의미를 폐기**한다고만 썼고, ① 새 `ChassisMode.mode` 값 목록,
② 팔 측 판정 로직을 **허용목록(allow-list)으로 반전**하라는 요구, ③ 미지 문자열·stale·미수신의
fail-closed 처리, ④ 우리 측이 지금 당장 `DRIVING` 발행을 중단해야 한다는 지시가 **모두 없다**.
그 결과 계획을 승인해도 **살아 있는 fail-open이 그대로 남는다.**

또한 `_publish_mode`가 정적 파라미터를 2 Hz로 뿌리는 현재 구현은, 계획이 요구하는
10 Hz heartbeat + `age > 0.5 s` freshness와 **정면 충돌한다** (2 Hz면 주기가 0.5 s여서
스케줄링 지터만으로도 상시 stale 판정).

**실패 시나리오 A — 주행 중 팔 언락 (현재 코드, 오늘 재현 가능)**

1. `ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=200` → `mode` 파라미터 기본값 `DRIVING`.
2. `chassis_node`가 2 Hz로 `ChassisMode{mode:"DRIVING"}`를 계속 발행 (`chassis_node.py:307`).
3. 팔 FSM은 `DRIVING ∉ LOCK_MODES` → **언락**. 팔은 자세 고정을 해제한다.
4. 로봇이 험지·뱅크를 주행 → 언락된 3축 팔이 관성으로 흔들린다. L515 하향 ROI 가림(계획:306-308이
   우려한 바로 그 상황) + 팔 관절 부하 + 최악의 경우 팔이 트랙 밖으로 나가 접촉 감점.
5. E-stop이 latch돼도 `_publish_mode`는 여전히 `DRIVING`을 발행한다. 팔은 차체가 멈춘 줄 모른다.

**실패 시나리오 B — 계약 어휘 공백 (계획대로 구현한 직후)**

1. 계획대로 "모든 주행 모드에서 팔 잠금"을 구현하려 하는데, 주행 중 무슨 `mode` 문자열을 보낼지
   계획에 없다. 개발자가 임시로 `DRIVE_LOCKED`를 발행한다.
2. 팔 팀 코드는 `DRIVE_LOCKED ∉ LOCK_MODES` → **언락**. 새 어휘를 도입할수록 fail-open이 심해진다.
3. 반대로 `ChassisMode`가 아예 끊겨도 팔은 마지막 값을 유지하거나 기본 언락으로 남는다.

**최소 수정 (계획 문구)**

§4.1(계획:117-119)의 해당 항목을 다음으로 교체·확장한다.

> - `ChassisMode.mode`의 확정 어휘는 `DRIVE_LOCKED`, `MISSION_STOP`, `HOLD`, `ESTOP` 네 가지다.
>   `DRIVE_LOCKED`는 모든 주행(직진·코너·험지·추종)에서 사용하며 팔은 접힘·고정을 유지한다.
>   기존 `DRIVING`, `CORNERING`, `ROUGH_TERRAIN`, `FOLLOW_LEAD`는 **폐기하고 발행을 즉시 중단**한다.
>   `HOLD`는 자동복구 motion hold, `ESTOP`은 latched 정지이며 둘 다 **작업 허가가 아니다**.
>   팔은 `MISSION_STOP`에서만 잠금을 풀 수 있다.
> - 팔 측 판정은 **허용목록**이어야 한다. `MISSION_STOP`이 fresh하게 수신된 경우에만 잠금 해제가
>   허용되고, 그 외 모든 값·미지 문자열·`age > 0.5 s` stale·토픽 미수신은 **잠금 유지(fail-closed)**다.
>   팔 노드 기동 직후의 기본 상태도 잠금이다. 이 반전은 로봇팔 팀 저장소의 `LOCK_MODES` 거부목록
>   로직을 대체하며, 7/19 계약 확정의 필수 항목이다.
> - `ChassisMode`는 실제 차체 상태에서 파생해야 하며 launch 파라미터 상수를 재발행해서는 안 된다.
>   현재 `chassis_node.py:67`의 `mode` 파라미터와 `chassis_node.py:304-308`의 정적 발행,
>   `chassis_node.py:175`의 2 Hz 타이머는 10 Hz 상태 파생 발행으로 교체한다.

§4.2 QoS 문단(계획:151)에는 "`ChassisMode`는 `chassis_node`의 실제 안전 상태에서 파생해 10 Hz로
발행한다"를 명시한다.

---

## 2. msg 5종 스키마 구현 가능성 판정

**결론: 5종 wire schema를 바꾸지 않고 계획의 핸드셰이크를 구현할 수 있다. 단 조건 3개가 붙는다.**

계획:125의 "5종 wire schema는 유지하고 문자열 어휘와 전이 규칙을 확장한다"는 판단은 **옳다.**
`robot_arm_msgs`는 로봇팔 팀 소유이고(`ros2/src/robot_arm_msgs/VENDORED.md`), ROS2 타입은
패키지명 + 구조 해시로 매칭되므로 스키마 변경은 양 팀 동시 재빌드를 요구하는 고비용 작업이다.
스키마를 건드리지 않기로 한 것은 통합 리스크를 크게 낮춘다.

| 계획이 요구하는 것 | 담을 필드 | 판정 |
|---|---|---|
| 팔 잠금 heartbeat (`STOWED_LOCKED`/`CARRYING_LOCKED`) | `ArmStatus.status` (string) | ✅ 가능 |
| 파지 상실 (`GRIP_LOST`) | `ArmStatus.status` | ✅ 가능 (단 S1-4) |
| `mission_id` 상관 | `ArmStatus.mission_id` (int32) | ✅ 가능 (단 S1-1) |
| `ArrivalStatus` 멱등 재발행 | `ArrivalStatus.mission_id` + `status` | ✅ 가능 |
| freshness (stamp/age) | `Header header` (3종 모두 보유) | ✅ 가능 |
| 작업 허가 의도 (`MISSION_STOP`) | `ChassisMode.mode` | ✅ 가능 |
| 선도 로봇 3D 추종 (WP7) | `DetectedObject.pose` + `DetectedObjectArray.header` | ⚠️ **좌표계 미정의 (S1-3)** |
| 마커 5개 중복 억제 (WP8 3구간) | `DetectedObject.class_id`/`class_name` | ⚠️ 어휘 미확정 (S2-4) |

**조건 1 — `ChassisMode`에는 `mission_id`가 없다** (`ChassisMode.msg:1-2`: `header` + `mode`만).
따라서 `MISSION_STOP`은 미션과 상관될 수 없고, 미션 상관은 `ArrivalStatus`/`ArmStatus`의
`mission_id`로만 이뤄진다. 계획의 FSM(계획:156-169)은 이 구조와 양립하지만, **그래서 S1-2의
토픽 간 순서 문제가 발생한다.**

**조건 2 — `DetectedObject`에는 `track_id`도, per-object stamp도, 3D 크기도, 공분산도 없다**
(`DetectedObject.msg:1-5`: `class_id`, `class_name`, `confidence`, `pose`, `bbox`(2D RoI)).
WP7의 "위치·크기·class·시간 연속성 gate"(계획:402)는 **파워트레인이 2D bbox + 3D pose로
자체 연관(association)을 수행**해야 성립한다. 이는 구현 가능하지만, 계획은 이 부담이
파워트레인 측에 있다는 것을 명시해야 한다.

**조건 3 — 상태 enum이 자유 문자열이라 오타·버전 불일치가 조용히 통과한다.**
스키마를 바꾸지 않으면서 버전을 강제하는 실용적 수단은 `Header.frame_id`를 계약 버전
슬롯으로 재사용하는 것이다(예: `ChassisMode.header.frame_id = "arm_contract/v2"`). 양측이
불일치 시 fail-closed하면 스키마 변경 없이 silent drift를 막을 수 있다. **권고**(S2-4 참조).

---

## 3. S1 — 계약 구멍 (7/19 확정 전 필수)

### S1-1. 잠금 heartbeat의 `mission_id` 의미가 정의되지 않아 영구 motion hold 또는 stale 수락이 발생한다

**근거**

- `ArmStatus.msg:2-3` — `int32 mission_id` / `string status` (둘이 항상 같이 온다)
- 계획:117-118 — "0이 아닌 주행 명령은 로봇팔의 `STOWED_LOCKED` 또는 `CARRYING_LOCKED`
  heartbeat가 fresh할 때만 허용한다."
- 계획:144 — "이전 ID의 지연 메시지는 무시하고" (mission_id 필터)
- 계획:133 — "`STOWED_LOCKED`: … **주행 중 주기적으로 발행**."

**무엇이 틀렸나**

주행 중 발행되는 `STOWED_LOCKED`/`CARRYING_LOCKED` heartbeat도 `ArmStatus`이므로 `mission_id`를
반드시 담는다. 그런데 **주행 중에는 활성 미션이 없다.** 계획은 이때 어떤 `mission_id`를 넣는지,
그리고 파워트레인의 "이전 ID 무시" 필터가 이 heartbeat에 적용되는지를 **한 줄도 정하지 않았다.**

**실패 시나리오 (데드락 — 2구간 픽업 직후)**

1. 픽업 미션 `mission_id = 7` 완료. 팔이 `DONE(7) → STOWING(7) → CARRYING_LOCKED(7)` 발행.
2. 파워트레인이 재출발. 팔은 운반 주행 중 `CARRYING_LOCKED(7)`을 10 Hz로 계속 heartbeat.
3. 하역 지점 도착. 파워트레인이 **새 미션 `mission_id = 8`**을 할당하고 `ARRIVED_DROP(8)` 발행.
4. 파워트레인의 "현재 mission_id와 다른 ArmStatus는 지연 메시지로 무시"(계획:144) 필터가
   `CARRYING_LOCKED(7)` heartbeat를 **버린다**.
5. 하역 작업 중 무언가 실패해 재출발을 시도 → fresh한 잠금 heartbeat가 없다고 판정 → **영구 motion hold.**
   20분 구간 시간이 소진되고 완주 점수(2구간 5+20점 계열)까지 잃는다.

**반대 방향 실패 시나리오 (stale 수락)**

필터를 느슨하게(“status가 lock 계열이면 mission_id 무시”) 만들면, 팔이 **오래된 미션의
`CARRYING_LOCKED`를 재생·재전송**하는 상황에서 파워트레인이 이를 주행 허가로 오인한다.
Reliable + Keep Last 10 QoS는 재연결 시 최대 10개 샘플을 순서대로 밀어낼 수 있으므로,
수신 시각이 아니라 **`header.stamp` 기준 freshness**로만 판정해야 이 경로가 막힌다.

**최소 수정 (계획 문구)** — §4.2 `mission_id` 문단(계획:144-149) 끝에 추가:

> 주행 허가 heartbeat(`STOWED_LOCKED`/`CARRYING_LOCKED`)와 미션 진행 상태(`WORK_READY`,
> `EXECUTING`, `DONE` 등)는 `mission_id` 규칙이 다르다.
> - 미션 진행 상태는 파워트레인이 발행한 **현재 활성 `mission_id`와 정확히 일치할 때만** 수락한다.
> - 주행 허가 heartbeat는 미션과 무관한 팔의 물리 자세 선언이므로 `mission_id = 0`(미션 없음)을
>   싣고, 파워트레인은 **`mission_id`를 무시하고 `status`와 `header.stamp` freshness만으로** 판정한다.
> - 두 판정 모두 수신 시각이 아니라 `header.stamp` 기준 `age`로만 수행한다. Reliable/Keep Last 10
>   재연결 시 밀려오는 과거 샘플이 fresh로 오인되어서는 안 된다.
> - `CARRYING_LOCKED`는 운반 중임을 뜻하므로, 파워트레인은 **직전 픽업 미션이 성공적으로 종료된
>   경우에만** 이를 운반 주행 허가로 수락한다(파지 이력 없는 `CARRYING_LOCKED`는 거부).

---

### S1-2. DDS는 토픽 간 순서를 보장하지 않는데 계획은 순서에 의존한다

**근거**

- 계획:426-427 — "먼저 `/cmd_vel=0`을 만들고 `/wheel_states`가 정지 임계값 아래에서 안정적으로
  유지된 뒤에만 `MISSION_STOP`과 `ArrivalStatus`를 **순서대로** 보낸다."
- 계획:156-169 — FSM이 `MISSION_STOP → ARRIVED_*(mission_id) → WORK_READY(mission_id)` 순서를 전제
- `contract.py:9` — 기존 미결 항목 "락 해제 순서: … `DRIVING`을 먼저 보내 언락 후 `ARRIVED_*` 발행"
- `AGENTS.md` robot-arm team resources — 미결 목록에 "`MISSION_STOP`→`ArrivalStatus` 순서"가 여전히 명시
- `chassis_node.py:136-145` — `/chassis_mode`와 `/arrival_status`는 **서로 다른 DDS 토픽**(별개 publisher)

**무엇이 틀렸나**

DDS/RTPS는 **동일 publisher의 동일 토픽 내부**에서만 순서를 보장한다. `/chassis_mode`와
`/arrival_status`는 별개 토픽이므로 **수신 순서가 뒤집힐 수 있다.** 계획의 FSM은 순서를
전제하지만, 순서는 미들웨어가 주지 않는다. (이 항목은 팔 팀과의 미결 목록에도 이미 올라 있는데,
계획은 "순서대로 보낸다"고만 쓰고 **순서 역전 시의 동작을 정의하지 않았다.**)

**실패 시나리오 (2구간 물자 픽업)**

1. 파워트레인: wheel 정지 확인 → `ChassisMode{MISSION_STOP}` 발행 → 1 ms 뒤 `ArrivalStatus{7, ARRIVED_PICKUP}` 발행.
2. Jetson이 YOLO + terrain + x264로 포화 상태 → DDS 전달 지터로 팔이 **`ARRIVED_PICKUP(7)`을 먼저**,
   `MISSION_STOP`을 나중에 받는다.
3. 팔 FSM은 "도착했으니 작업 시작" 조건만 보고 **잠금을 풀고 작업 자세로 전환**한다.
   이때 팔이 참조하는 마지막 `ChassisMode`는 아직 `DRIVE_LOCKED`(또는 현재 코드에서는 `DRIVING`)다.
4. 계획:120-121이 명시적으로 금지한 상황 — "팔은 그 전에 잠금을 풀거나 작업 자세로 전환하지 않는다" —
   이 **정확히 발생한다.** 만약 이 시점에 정지 판정이 잘못돼 차체가 미세하게 굴러가면
   팔이 펴진 채 이동하게 된다.

**최소 수정 (계획 문구)** — §8 해당 문단(계획:426-430)의 첫 문장을 교체:

> DDS는 서로 다른 토픽 간 전달 순서를 보장하지 않는다. 따라서 핸드셰이크는 **순서가 아니라
> 논리곱 조건**으로 정의한다. 팔은 (a) `age ≤ 0.5 s`인 `ChassisMode{MISSION_STOP}`과
> (b) 동일 `mission_id`의 `ArrivalStatus`를 **동시에 만족할 때만** 작업 자세 전환을 시작한다.
> 둘 중 하나라도 없거나 stale하면 잠금을 유지한다. 파워트레인은 `MISSION_STOP`을 10 Hz로,
> `ArrivalStatus`를 2 Hz로 **반복 발행**하므로 순서 역전은 다음 주기에 자동으로 수렴한다.
> 파워트레인은 작업 중 `MISSION_STOP` 발행을 중단해서는 안 되며, 재출발을 결정한 순간
> `DRIVE_LOCKED`로 전환하는 것이 팔에 대한 "잠금하라"는 유일한 신호다.

---

### S1-3. `/detected_objects`의 좌표계와 D435i extrinsic 소유자가 정의되지 않았다 (WP7 60점 직결)

**근거**

- `DetectedObject.msg:4` — `geometry_msgs/Pose pose` (프레임 정보 없음)
- `DetectedObjectArray.msg:1` — `std_msgs/Header header` (배열 전체에 `frame_id` 하나)
- 계획:55 — "로봇팔 인식: D435i와 `/detected_objects`, `/arm_status`; 파워트레인은 D435i를 열지 않는다."
- 계획:397-403 — WP7이 "`/detected_objects`의 선도 로봇 3D 위치로 거리와 좌우 오차 계산",
  "목표 간격 2.0 m, 허용범위 1.5~2.5 m"를 요구
- 계획:69-74 — 실측 대상 목록에 **`base_link→l515_link`는 있으나 D435i extrinsic은 없다**
- 계획 전문에 TF, `tf2`, `d435i_link` 언급 **0회**

**무엇이 틀렸나**

WP7(5구간 간격유지 35 + 재추종 25 = **60점**)과 WP8의 정차 위치 판단은 전적으로
`/detected_objects`의 3D pose에 의존한다. 그런데 그 pose가 **어느 좌표계인지**(D435i color
optical frame? `base_link`? 팔 base?), **누가 D435i→`base_link` 정적 변환을 실측·발행하는지**가
계획 어디에도 없다. 로봇팔 팀은 D435i를 팔 위 또는 차체 위에 장착하며, 팔이 움직이면
extrinsic이 변할 수도 있다(3축 팔, 그리퍼 재검토 중 — `AGENTS.md`).

**실패 시나리오 (5구간)**

1. 팔 팀이 `pose`를 D435i optical frame(z=전방, x=우, y=하) 기준으로 발행한다.
2. 파워트레인이 이를 `base_link`(x=전방, y=좌, z=상) 기준으로 해석한다.
3. **축이 통째로 뒤바뀐다.** "전방 2.0 m"가 "하방 2.0 m"로 읽힌다. 거리 PID가 발산하거나
   상시 1.5 m 이하로 판정해 전진 금지(계획:403)에 걸려 **로봇이 출발조차 하지 않는다.**
4. 더 나쁜 경우: 부호만 틀려 로봇이 선도 로봇 쪽으로 가속 → **접촉 감점**(계획:22).
5. 이 버그는 HIL에서만 드러나고, 발견 시점이 8월 초(계획:556 WP7 일정)라 복구 여유가 없다.

**최소 수정 (계획 문구)** — §4.1 소유권 원칙(계획:105-121)에 추가하고, WP7(§6)에 완료조건 추가:

> - `/detected_objects`의 `DetectedObjectArray.header.frame_id`는 **`base_link`로 고정**한다.
>   `DetectedObject.pose`는 배열 header의 frame 기준 3D 좌표이며, 로봇팔 팀이 D435i extrinsic을
>   소유·적용해 `base_link` 좌표로 변환한 뒤 발행한다. 파워트레인은 변환을 수행하지 않는다.
>   (대안으로 팔이 optical frame으로 발행하고 `tf2` 정적 변환 `base_link→d435i_link`를
>   **로봇팔 팀이 발행**하는 방식도 허용하되, 둘 중 하나를 7/19에 확정한다.)
> - D435i가 팔 링크에 장착되어 팔 자세에 따라 extrinsic이 변하는 경우, 팔이 동적 TF를 발행하거나
>   **주행 중 팔 자세를 `STOWED_LOCKED`/`CARRYING_LOCKED` 두 가지로 고정**해 두 자세의 정적
>   extrinsic만 사용한다. 후자를 기본 후보로 한다.
> - 축 규약(REP-103, x=전방/y=좌/z=상, 단위 m)과 부호를 계약 시험에 포함하고, 알려진 실측
>   거리·좌우 오프셋의 목표물로 HIL 1회 검증한 뒤에만 WP7을 완료로 판정한다.

---

### S1-4. `GRIP_LOST`의 물리적 근거가 없다 (3축 팔 + 그리퍼 재검토 중)

**근거**

- 계획:136 — "`GRIP_LOST`: 운반 중 파지 상실. 파워트레인은 제어된 motion hold를 수행"
- 계획:134-135 — "`CARRYING_LOCKED`: 물자를 파지한 팔이 운반 자세로 접혀 잠겼고 **파지 상태가 정상임**"
- 계획:606 — 완료조건 "운반 중 `CARRYING_LOCKED` heartbeat 단절 또는 `GRIP_LOST`에서 제한거리 안에 motion hold."
- `AGENTS.md` robot-arm team resources — "팔은 6/24 6축→**3축** 대격변(**그리퍼 재검토 중**)"
- `ArmStatus.msg:3` — `string status` 단일 필드. 파지 신뢰도·힘·개폐량을 실을 슬롯이 없다.

**무엇이 틀렸나**

계획은 `GRIP_LOST`라는 신호가 **존재한다고 가정**하고 그 위에 안전 논증(운반 중 motion hold)을
쌓았다. 그러나 로봇팔의 그리퍼는 **재설계 중**이고, 파지 상실을 검출할 센서(그리퍼 전류, 접촉
스위치, 위치 피드백, 또는 D435i 시각 확인)가 있는지 계획 어디에도 근거가 없다. 3축 팔의 저가
서보 그리퍼는 흔히 **개폐 위치 명령만 있고 파지력 피드백이 없다.**

파지 검출이 불가능하면 `GRIP_LOST`는 **영원히 발행되지 않는 상태**가 되고, 계획:606의 완료조건은
"발생하지 않는 이벤트에 대한 시험"이 되어 **거짓 안전(false assurance)**을 만든다.

**실패 시나리오 (2구간 운반)**

1. 그리퍼에 힘/접촉 피드백이 없다. 팔은 "닫힘 명령을 보냈으니 파지 중"으로 간주해
   `CARRYING_LOCKED`를 계속 heartbeat한다.
2. 사구·자갈 구간의 진동으로 물자가 그리퍼에서 미끄러져 떨어진다.
3. 팔은 이를 인지하지 못하고 `CARRYING_LOCKED`를 계속 발행한다. `GRIP_LOST`는 발행되지 않는다.
4. 파워트레인은 운반 profile(계획:378-379의 보수적 속도)로 **정상 주행을 계속**한다.
5. 하역 지점에서 빈 그리퍼로 `ARRIVED_DROP`을 수행. **물자 낙하 감점 + 물자 점수 0 + 낙하 지점
   불명**으로 회수 불가.

**최소 수정 (계획 문구)** — §4.2 `GRIP_LOST` 정의(계획:136)를 다음으로 교체:

> - `GRIP_LOST`: 운반 중 파지 상실이 **검출된** 상태. 파워트레인은 제어된 motion hold를 수행하고
>   운영자에게 알린다.
>
> **`GRIP_LOST`의 검출 근거는 7/19에 로봇팔 팀이 명시해야 한다** (그리퍼 전류·접촉 스위치·
> 위치 피드백·D435i 시각 확인 중 무엇인지). 검출 수단이 없다고 확인되면 `GRIP_LOST`를
> 안전 논증에서 제거하고 다음 보수적 정책으로 대체한다.
> - 운반 profile의 속도·가속·조향 slew·허용 bank angle을 낙하 위험 기준으로 더 낮춘다.
> - 팔이 파지 확증을 제공하지 못하는 한, 운반 주행 구간을 **최단 경로·최저속**으로 제한하고
>   운반 중 험지 통과를 회피 대상으로 둔다.
> - 하역 직전 `WORK_READY` 응답에 파지 여부 확증을 요구하고, 확증이 없으면 하역 시도 대신
>   운영자 통보 후 정지한다.
> - 계획:606의 완료조건은 "검출 수단이 실재함을 HIL에서 실증한 경우에만" 적용하고,
>   검출 수단이 없으면 완료조건에서 삭제한다. **발생하지 않는 이벤트를 시험 통과로 기록하지 않는다.**

---

## 4. S2 — 중대하나 우회 가능 (7/19 계약 확정 전 해소)

### S2-1. `mission_id` 영속 카운터의 저장 위치가 tmpfs일 위험

- 계획:146-149 — "파워트레인은 **host 영속 상태**의 양수 `int32` 카운터를 publish 전에 원자적으로 증가·저장한다."
- `AGENTS.md:5` — `/run/powertrain`은 **systemd-tmpfiles가 재부팅 후 재생성**하는 tmpfs다.
  `powertrain_ros`가 bind-mount하는 유일한 host 런타임 경로가 여기다.

**실패 시나리오**: 개발자가 "host 영속 상태"를 이미 bind-mount된 `/run/powertrain`으로 해석한다.
대회 당일 Jetson을 재부팅한다. 카운터가 0으로 초기화된다. 팔은 이전 run의 `mission_id = 12`를
기억하고 있고(팔 노드는 재시작되지 않았거나 로그 기반 복구), 파워트레인이 보낸
`ArrivalStatus{1, ARRIVED_PICKUP}`을 **"지연된 이전 ID"로 판정해 무시**한다(계획:144).
→ 핸드셰이크 데드락, 2구간 물자 점수 상실.

**최소 수정**: 계획:146-149에 "**저장 경로는 tmpfs가 아닌 host 영속 볼륨(예: `/var/lib/powertrain/mission_id`)**이며,
`write → fsync → rename` 원자 갱신을 사용하고 compose에 별도 bind-mount로 명시한다.
`/run/powertrain`은 tmpfs이므로 사용하지 않는다."를 추가.

### S2-2. 운영자 skip이 팔 stow 없이는 불가능한데 계획이 그 경로를 정의하지 않았다

- 계획:171-172 — "timeout은 자동 출발 조건이 아니다. 제한된 재시도 또는 **운영자 skip**을 기다리며 정지 상태를 유지한다."
- 계획:117-118 — 0이 아닌 주행 명령은 fresh한 `STOWED_LOCKED`/`CARRYING_LOCKED`가 있을 때만 허용.

**모순**: 팔이 펴진 채 고장나면(예: `FAILED` in `EXECUTING`) 잠금 heartbeat가 나오지 않는다.
운영자가 "skip"을 눌러도 **주행 허가 조건을 만족할 방법이 없다** → 로봇은 그 자리에서 20분을
소진하고 해당 구간의 완주 점수(10~20점)까지 전부 잃는다. 물자 5점을 포기하려는 skip이
완주 점수까지 잃게 만드는 구조다.

**최소 수정**: §4.2 timeout 문단(계획:171-172)에 추가:

> 운영자 skip은 팔이 접힘·고정된 경우에만 유효하다. 팔이 펴진 채 실패한 경우의 유일한 경로는
> **운영자 원격 팔 조작 → 팔이 `STOWED_LOCKED` 발행 → 주행 재개**다. 팔이 어떤 이유로도
> `STOWED_LOCKED`를 발행할 수 없으면, 운영자가 육안으로 접힘을 확인하고 명시적으로 서명한
> **`operator_attested_stow` 오버라이드**로만 주행할 수 있으며, 이 모드는 최고속도를 운반
> profile 이하로 강제하고 TUI에 영구 경고를 표시한다. 이 오버라이드는 대회 모드에서도 유지한다
> (§9의 "검증되지 않은 자동 recovery 비활성"과 별개다).

### S2-3. `ChassisMode` 10 Hz 요구와 현재 2 Hz 타이머의 충돌 (S0-2에 병합되나 별도 추적)

`chassis_node.py:175`의 `create_timer(0.5, ...)`는 2 Hz다. 계획:151은 10 Hz + `age > 0.5 s`를 요구한다.
**주기와 timeout이 같으면 지터만으로 상시 stale**이 된다. 최소 수정은 S0-2에 포함.

### S2-4. msg 구조 해시 불일치 시 조용히 아무것도 수신되지 않는다 — 계약 preflight 부재

- `ros2/src/robot_arm_msgs/VENDORED.md` — "ROS2 타입은 wire에서 **패키지명 + 구조 해시**로 매칭"
- `ros2/scripts/sync_check_msgs.sh` — 로컬 체크아웃 대비 diff만 수행. 런타임 검증 아님.
- 계획:126 — "구현 전 양 팀 저장소가 동일한 어휘·QoS·timeout을 사용하도록 계약 시험을 둔다." (선언만)

**실패 시나리오**: 팔 팀이 `ArmStatus.msg`에 필드 하나를 추가하고 자기 쪽만 재빌드한다.
타입 해시가 달라져 **publisher와 subscriber가 아예 연결되지 않는다.** `ros2 topic echo`는 조용하고,
에러 로그도 없다. 파워트레인은 "팔 heartbeat 없음 → motion hold"로 fail-closed한다(안전은 지켜진다).
그러나 **원인 진단에 몇 시간이 걸리고**, 대회 당일이라면 치명적이다.

**최소 수정**: 계획 §8.1 계약 시험 목록(계획:468-470)에 추가:

> - **arm contract preflight (arm 직전 실행, fail-closed)**: `/arm_status`, `/detected_objects`,
>   `/chassis_mode`, `/arrival_status` 4개 토픽 각각에 대해 endpoint의 타입 해시 일치와
>   상대측 endpoint 존재(publisher/subscriber count ≥ 1)를 확인한다. 하나라도 실패하면
>   자율 arm을 금지하고 TUI에 원인을 표시한다. 스키마 변경이 조용한 무연결로 나타나지 않게 한다.
> - `Header.frame_id`에 계약 버전 문자열(예: `arm_contract/v2`)을 실어 양측이 불일치 시
>   fail-closed하게 한다. 5종 wire schema를 바꾸지 않고 버전을 강제하는 유일한 슬롯이다.
> - `class_name`/`class_id` 어휘(신호등 적/녹, 마커 5종, 선도 로봇, 구호물자, 마네킹)를
>   계약 파일에 열거하고, **미지 class는 무시하되 진단에 기록**한다(미지 class를 이벤트로 승격 금지).

### S2-5. 본선 일자 근거 불일치

- 계획:586 — "### 9월 5~6일: 본선"
- `AGENTS.md` robot-arm team resources — "일정: 7/19 설계문서 확정 → 7/31 국방 서류 → **9/13 국방 본선** → 10/2 극한 본선."

두 문서가 **8일 차이**로 충돌한다. 계획의 동결 일정(§9 "8월 31일~9월 4일: 출전 동결")이 잘못된
날짜에 맞춰져 있을 수 있다. 규정 PDF(`docs/국방로봇_규정.pdf`, SHA-256이 계획:5-6에 고정)로
확정하고 계획 §9 전체를 재정렬해야 한다. **어느 쪽이 맞는지 이 검토에서는 단정하지 않는다 —
규정 원문 확인이 필요하다.**

### S2-6. 팔 팀과의 미결 항목이 여전히 열려 있는데 계획이 이를 해결된 것처럼 서술한다

`AGENTS.md` robot-arm team resources의 미결 목록: "status enum(대문자 스네이크 잠정),
mission_id 관리 주체, `MISSION_STOP`→`ArrivalStatus` 순서, **`ROS_DOMAIN_ID`**, 라이다 역할 분담".
`contract.py:7-9`도 미결 2건을 명시한다.

계획 §4.2는 이 어휘를 **확정된 것처럼** 서술하지만, 로봇팔 팀의 합의 기록은 없다.
특히 `ROS_DOMAIN_ID`는 `ros2/README.md:53`이 "기본 `ROS_DOMAIN_ID=0`"이라고만 적고 있어,
팔 컨테이너가 다른 도메인을 쓰면 **모든 계약 토픽이 조용히 연결되지 않는다**(S2-4와 동일한
증상, 동일한 preflight로 검출).

**최소 수정**: 계획 §9의 7/12~19 항목(계획:542-543)에 "`ROS_DOMAIN_ID`, RMW 구현체,
`use_sim_time` 정책"을 확정 목록에 추가하고, "본 §4.2의 어휘는 **로봇팔 팀 서면 합의 전까지
파워트레인 측 제안이며 확정이 아니다**"를 명시한다.

---

## 5. S3 — 개발 용이성·운영 품질

| # | 항목 | 근거 | 권고 |
|---|---|---|---|
| S3-1 | `/chassis_state`가 `ChassisMode` 스키마를 자유 문자열 진단으로 재사용 | `chassis_node.py:146-150`, `chassis_node.py:315-319` (`mode = "ARMED v=0.10 w=0.00"`) | 계약 msg를 진단에 재사용하지 않는다. `powertrain_msgs`에 진단 전용 타입을 두거나 `diagnostic_msgs` 사용. 현재는 `ChassisMode.mode`의 어휘가 **어디에서도 강제되지 않는다** |
| S3-2 | `/detected_objects` 30 fps에 Reliable/Keep Last 10 적용 시 낭비 | 계획:153 "**해당 계약 토픽**은 Reliable, Keep Last 10, Volatile" (어느 토픽인지 모호) | 계획에 **토픽별 QoS 표**를 명시: 제어/이벤트 4종(`/chassis_mode`, `/arrival_status`, `/arm_status`)은 Reliable·Volatile, heartbeat는 depth 1, 이벤트는 depth 10. `/detected_objects`는 Best Effort·depth 1(센서 스트림). 현 `chassis_node`의 depth 10 기본값은 rclpy 기본과 일치하므로 마이그레이션 비용은 0 |
| S3-3 | `GRIP_LOST` 이후 물자 재파지 절차 미정의 | 계획:141, 계획:432-434 | motion hold 후 무엇을 하는가? 재파지 시도(새 `mission_id`로 `ARRIVED_PICKUP` 재발행) / 물자 포기 후 stow → 완주 우선 중 **하나를 대회 정책으로 사전 결정**하고 §8 2구간 profile에 적는다. 현장 판단에 맡기면 20분을 잃는다 |
| S3-4 | `contract.py`가 계획과 완전히 어긋난 채 방치 | `contract.py:13-38` 전체 — `WORK_READY`/`STOWING`/`STOWED_LOCKED`/`CARRYING_LOCKED`/`GRIP_LOST` **전부 없음**, `MODE_DRIVING` 주석은 "정상 주행 = 팔 언락" | 계획 승인과 **동일 커밋**에서 `contract.py`를 갱신하고, `chassis_node.py:67`의 `MODE_DRIVING` 기본값을 제거한다. 계획만 승인하고 코드를 두면 S0-2의 fail-open이 그대로 남는다 |
| S3-5 | 시뮬레이션 mock 팔 노드의 시간 계약 미기술 | 계획:214-215 (mock `/detected_objects`·`/arm_status`는 외부 ROS2 Humble mock node) | freshness 계약(`age > 0.5 s`)이 sim에서도 성립하려면 `use_sim_time`과 `/clock` 공유가 필요하다. §WP6-S 공통 시뮬레이터 계약에 "mock 팔 노드를 포함한 모든 노드가 동일한 시간원(`/clock` 또는 system clock)을 사용한다"를 1줄 추가 |

---

## 6. 문제없는 부분 (유지·강화 권고)

이 계획은 다음을 **정확히** 짚었다. 리뷰 과정에서 반박을 시도했으나 모두 타당했다.

**안전 설계**

- **`DONE` 단독 재출발 금지**(계획:138-139, 604). `DONE`은 "동작 완료"일 뿐 "주행 안전"이 아니라는
  구분은 정확하다. 픽업은 `CARRYING_LOCKED`, 하역은 `STOWED_LOCKED`를 **별도로** 요구하는 것도 맞다.
- **실제 wheel 정지 확인 후 `MISSION_STOP` 송신**(계획:120-121, 426). "명령을 0으로 보냈다"와
  "바퀴가 멈췄다"를 구분한 것은 이 등급 로봇에서 자주 빠지는 함정을 피한 것이다.
- **motion hold와 latched E-stop의 분리**(계획:114, 142). 특히 "물리 충돌이나 별도 latched 위험이
  확인되지 않은 로봇팔 통신 장애 자체는 E-stop으로 승격하지 않는다"는 **정확한 판단**이다.
  팔 통신 장애를 E-stop으로 만들면 운영자 개입 없이는 복구 불가가 되어 대회 점수를 잃는다.
- **fail-open 전면 금지**(계획:141, 627, 369). "센서 단절 시 마지막 명령을 유지하는 fail-open 동작"을
  §11 "의도적으로 하지 않는 것"에 넣은 것은 옳다.
- **timeout ≠ 자동 재출발**(계획:171, 430). timeout이 암묵적 허가가 되는 것을 명시적으로 막았다.
- **`mission_id` 멱등성과 지연 ID 무시**(계획:144-145). 중복 `ArrivalStatus`가 팔 작업을
  재실행하지 않아야 한다는 요구는 정확하다.
- **재시작 후 암시적 재개 금지**(계획:607). 현재 `chassis_node`가 이미 `reset_estop → IDLE`
  후 **별도 arm 서비스**를 요구하므로(`chassis_node.py:378-388`, `ros2/README.md:353`)
  파워트레인 측은 이미 이 성질을 만족한다.

**아키텍처와 소유권**

- **msg 5종 wire schema 유지 결정**(계획:125). `robot_arm_msgs`는 팔 팀 소유이고 타입 해시 매칭이라
  스키마 변경은 양 팀 동기 재빌드를 강제한다. 어휘만 확장하는 선택은 통합 리스크를 크게 낮춘다.
- **단일 소유권 원칙**(계획:107-112). L515 Gateway 단일 소유, `ChassisManager` 단독 CAN 소유,
  D435i 팔 전용, 의미 인식 중복 모델 금지는 `AGENTS.md`의 CURRENT STATE OVERRIDE와 완전히 일치한다.
- **상시 PointCloud2 금지 + 내부 ROI point cloud**(계획:108-109, 313-314). 8 GB Orin Nano에서
  DDS 직렬화 비용을 피하는 옳은 선택이며 기존 L515 파이프라인 결정과 일관된다.
- **production 코드에 simulator 분기 0개**(계획:102-103, 267). 시뮬레이터가 production을 오염시키지
  않게 하는 강한 계약이다.
- **JAX 실패 시 같은 run 내 자동 NumPy fallback 금지**(계획:354-356). 주행 중 backend 전환은
  타이밍 특성을 바꿔 예측 불가능한 실패를 만든다. 금지가 옳다.
- **QoS 선택(Reliable/Keep Last 10/Volatile)이 rclpy 기본값과 일치**한다. `chassis_node.py:117-155`가
  이미 `depth=10` 기본 QoS를 쓰므로 **마이그레이션 비용이 0**이다(S3-2의 depth 조정만 권고).

**기하와 성능 게이트**

- **트랙 폭 0.9144 m vs 윤거 0.879 m → 좌우 여유 각 18 mm** 인식(계획:63-67)과 "로봇을 점으로
  취급하지 않는다"(계획:76-78)는 이 대회의 핵심 제약을 정면으로 다룬 것이다.
- **footprint erosion 결과가 없으면 임계값을 줄이지 않고 motion hold**(계획:77-78, 369). 옳다.
- **전체부하 backend 채택 gate**(계획:503-514). `MemAvailable` 1.5 GB, chassis 50 Hz jitter,
  YOLO rate 저하 5% 이하, terrain cgroup 격리는 8 GB 통합 메모리 Jetson에서 **정확히 옳은 게이트**다.
- **"측정 전 JAX가 NumPy보다 우월하다고 가정하지 않는다"**(계획:518). 이 절제가 이 계획의 가장
  성숙한 문장이다.

---

## 7. 수정 우선순위

### 즉시 (계획 승인 전 — 계획 문구만 고치면 되는 것)

| 순위 | 항목 | 예상 작업량 |
|---|---|---|
| 1 | **S0-2** `ChassisMode` 어휘 확정(`DRIVE_LOCKED`/`MISSION_STOP`/`HOLD`/`ESTOP`) + 팔 측 **허용목록 반전** + 10 Hz 상태 파생 발행 요구 | 계획 §4.1에 3항목, §4.2에 1줄 |
| 2 | **S0-1** 원격 경로 ROS 통합 WP(WP6-T) 신설 + §10 완료조건 교체 | 계획 §5·§7·§10에 각 1문단 |
| 3 | **S1-1** 잠금 heartbeat의 `mission_id = 0` 규칙 + stamp 기준 freshness | 계획 §4.2에 4줄 |
| 4 | **S1-2** 토픽 간 순서 미보장 → **논리곱 조건**으로 재정의 | 계획 §8에 1문단 교체 |
| 5 | **S1-3** `/detected_objects` frame_id = `base_link` 고정 + D435i extrinsic 소유자 지정 | 계획 §4.1에 3줄, WP7 완료조건 1줄 |
| 6 | **S1-4** `GRIP_LOST` 검출 근거 요구 + 없을 때의 대체 정책 | 계획 §4.2에 1문단 |

**이 6건은 모두 계획 문서 편집만으로 끝나며, 총 1페이지 미만이다.** 코드 변경은 뒤따르지만
계획이 잘못된 안전 주장을 담은 채 승인되는 것을 막는 것이 우선이다.

### 7/19 로봇팔 팀 계약 확정 회의 안건 (S1·S2)

1. `ChassisMode` 신규 어휘 4종과 **팔 측 fail-closed 허용목록 반전** (S0-2) — **최우선 안건**
2. `ArmStatus` 신규 어휘 5종(`WORK_READY`/`STOWING`/`STOWED_LOCKED`/`CARRYING_LOCKED`/`GRIP_LOST`)
3. 잠금 heartbeat의 `mission_id = 0` 규칙 (S1-1)
4. 핸드셰이크 = 순서가 아니라 **논리곱 조건** (S1-2) — 팔 팀 미결 목록의 "순서" 항목을 여기서 종결
5. `/detected_objects` frame_id와 D435i extrinsic 소유자 (S1-3)
6. `GRIP_LOST` 검출 수단의 존재 여부 (S1-4) — **그리퍼 재설계 결과에 종속되므로 조기 확인 필수**
7. `ROS_DOMAIN_ID`, RMW, QoS 표, `class_name` 어휘, 계약 버전 문자열 (S2-4, S2-6)

### 코드 (7/20~31 WP6 구간과 병행)

8. `contract.py` + `chassis_node.py:67/175/304-308` 갱신 (S3-4, S0-2, S2-3)
9. `mission_id` 영속 카운터를 비-tmpfs 경로에 구현 (S2-1)
10. arm contract preflight (타입 해시 + endpoint 존재 확인, fail-closed) (S2-4)
11. 운영자 skip / `operator_attested_stow` 경로 (S2-2)
12. 토픽별 QoS 표 반영, `/chassis_state` 진단 타입 분리 (S3-1, S3-2)

### 별도 확인

13. **본선 일자**(9/5~6 vs 9/13)를 규정 PDF로 확정하고 §9 일정 전체 재정렬 (S2-5)

---

## 8. 검토자 주석

이 검토는 **정적 분석과 문서 대조만** 수행했다. Jetson 실기, 로봇팔 팀 저장소
(`ksp118/extreme-robot`)의 현재 `arm_fsm_node.py` 실제 코드, 실행 중 DDS 그래프는 확인하지 않았다.
특히 다음 두 가지는 **로봇팔 팀 저장소를 직접 확인해 검증해야 한다.**

- `LOCK_MODES` 거부목록이 팔 팀 코드에 **여전히 존재하는지** (`contract.py:22`는 우리 쪽 미러다).
  S0-2의 심각도는 이 확인 결과에 따라 달라진다 — 팔이 이미 허용목록으로 바꿨다면 S1로 강등된다.
- 그리퍼의 파지 검출 수단 존재 여부. S1-4는 이 확인 없이는 해소되지 않는다.

`AGENTS.md`의 "check GitHub before work" 지침(작업 전 우리 GitHub + Jetson 로컬 + 팔 레포
3곳 확인, 2026-07-03 사용자 지시)에 따라, 7/19 계약 확정 전에 위 2건을 실제 코드로 검증할 것을
강하게 권고한다. "합의했다"는 구두 정보는 이 프로젝트에서 이미 한 번(PR #11) 사실과 달랐던 이력이 있다.

---

## 9. 2026-07-13 주 에이전트 후속 대조 및 반영

이 절은 위 Opus 원문을 수정하지 않고 후속 사실 확인과 최종 채택 결정을 기록한다.

- Jetson `~/extreme-robot` HEAD `279d691f...`를 직접 읽어 현행 `DRIVING → unlock`, 취소 모션
  재진입, 상태전이 중 publish, 10 Hz `_tick`, joint-effort 기반이지만 placeholder인 grip threshold를
  확인했다. 기존 dirty/untracked 파일은 수정하지 않았다.
- S0 원격 우회는 수용해 상위 계획에 WP5.2-T를 추가했다. production 원격은
  `/teleop/cmd_vel → command_authority → /cmd_vel → chassis_node`로 통합하며 standalone direct-CAN
  teleop는 진단·복구 전용으로 강등한다.
- S0 mode fail-open은 수용하되 새로운 `DRIVE_LOCKED/HOLD/ESTOP` 4종으로 전면 교체하지 않는다.
  기존 주행 mode를 모두 default-deny 잠금 유지로 재정의하고 `MISSION_STOP` 유일 언락,
  `STOW_REQUEST` 명시적 접기 요청을 추가한다.
- D435i extrinsic은 로봇팔 팀이 TF를 소유·발행하고 파워트레인이 array header frame에서
  `base_link`로 변환하는 ROS 표준 경계로 확정했다.
- `Header.frame_id`를 계약 버전 슬롯으로 재사용하라는 제안은 frame 의미를 훼손하므로 기각했다.
  대신 양 checkout msg SHA-256, endpoint type/QoS와 DDS 왕복 preflight를 사용한다.
- heartbeat는 Reliable/Keep Last 1, arrival event는 Reliable/Keep Last 10,
  `/detected_objects`는 Best Effort/Keep Last 1로 분리했다.
- 순간적인 `WORK_READY/DONE` 수신에 의존하지 않도록 동일 mission ID의 지속 상태를 사용한다.
  픽업 성공은 `CARRYING_LOCKED`, 하역 성공은 `STOWED_LOCKED`가 권위 있는 ACK이며 `DONE`은
  진단일 뿐이다.
- `GRIP_LOST`는 qualified detector가 있을 때만 안전 근거로 사용한다. qualification 실패 시
  최저속·최단경로 운반과 하역 전 재확인 fallback을 적용한다.
- 본선 날짜는 현재 `docs/국방로봇_규정.pdf` 1쪽의 2026년 9월 5~6일로 재확인했다.

최종 반영 정본 후보는 `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`, 세부
실행계획은 `docs/plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md`다.
