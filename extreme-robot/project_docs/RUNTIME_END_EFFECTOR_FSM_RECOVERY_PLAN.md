# 8월 19일 실기 기준선 기반 런타임 엔드이펙터 교체 개발 계획

> 개정: 2026-09-09  
> 현재 기준: `main` PR #56 머지 `ecb2c51`  
> 실기 동작 기준선: PR #49 머지 `656b2cd`와 그 직전 PR #48 `2cdd1aa`  
> 범위: 개발 계획만 작성한다. 기존 제어·캘리브레이션 값을 새로 만들거나 소스 코드를 수정하지 않는다.  
> 요구사항 주의: 로컬에 `docs/requirements/요구사항.md`가 없어 현재 코드, 기존 프로젝트 문서,
> 8월 19일 커밋 메시지와 이번 실행 검증을 근거로 한다.

## 구현 진행 기록 (2026-09-09)

현재 통합 브랜치 `codex/backend-tool-fsm-recovery`에서 다음 항목까지 구현·검증했다.

- `824819b`: 8월 실기 기준 팔 FSM 및 브리지 기동 안전 경로 복구
- `fb3667f`: Claude의 GUI 요청 timeout/cancel/연결단절 복구와 Qt 정상 종료 반영
- `5396f58`: 캘리브레이션 파라미터 런타임 반영 callback 복구
- `617f470`: MoveIt용 `/joint_states`와 런타임 도구용 `/tool/joint_states` 분리
- `8f86437`: `f3229a9` 병합에서 손상된 dual URDF를 `656b2cd` 검증본으로 복구하고 구조 테스트 추가
- `9e51335`: `{3,4}`/`{5}` read-only ping 서명, 연속 확인, 물리 탈거 latch,
  안전한 팔 FSM 상태에서의 자동 profile/FSM 전환 연결

검증 결과는 관련 백엔드 회귀 65건과 GUI 파일별 격리 실행 59건이 통과했다. 실제 launch에서도
`dual → spur` 후 arm-only joint stream과 tool joint stream이 분리됐고 `/compute_fk` 성공 및
`move_group` 생존을 확인했다.

아직 완료로 간주하지 않는 항목은 USB 실기 왕복 시험, cleaner의 물리 ID 확정,
`simulation_ready`/`hardware_ready` 표시 분리다. 현재 dual 구동 정책은 사용자가 실기 정상이라고
확인한 현행 코드를 보존했으며, 이 문서 초안의 ID3-only 회귀 제안으로 되돌리지 않았다.

---

## 1. 개발 원칙

이번 작업은 로봇팔 제어 스택을 새로 만드는 작업이 아니다. **8월 19일에 실기로 확인된 팔,
그리퍼, 카메라, 캘리브레이션, 안전 정책을 그대로 보존하고**, 이후 합쳐진 엔드이펙터 FSM과
런타임 전환 코드의 연결부만 고친다.

목표 동작은 다음과 같다.

1. 로봇 제어 프로세스와 GUI는 실행 중이다.
2. 팔이 `IDLE`/`STOWED`/`STOWED_LOCKED`에서 정지한 것을 확인한다.
3. 기존 엔드이펙터를 제거하고 다른 ID 조합의 도구를 장착한다.
4. 브리지가 등록된 후보 ID를 read-only로 확인해 도구를 판별한다.
5. 기존 `ToolManager`와 도구별 FSM이 새 profile context로 전환된다.
6. GUI가 실제 active 도구와 전환 상태를 표시하고 준비된 조작만 활성화한다.
7. 팔의 기존 캘리브레이션·안전 게이트·카메라·MoveIt/analytic IK 경로는 그대로 동작한다.

여기서 “동작 중 교체”는 스택이 실행 중이라는 뜻이다. **관절이나 도구가 실제로 움직이는
동안에는 교체를 거부한다.**

## 2. 보존할 8월 19일 기준선

통합 기준점은 `656b2cd`이다. 이를 통째로 checkout하거나 현재 코드를 통째로 되돌리지 않고,
아래 동작을 회귀 방지 기준으로 사용한다.

| 기준 커밋 | 검증된 내용 | 이번 작업의 규칙 |
|---|---|---|
| `40e222f` | ID 11을 포함한 5축 구성, 전 축 영점 재측정, 하드스톱 기반 관절 범위, read-only `/joint_states` 20 Hz, 7개 서보 HW error 0 | `JOINT_CONFIG`, center, gear ratio, `joint_limits.py`, stow 자세를 임의 변경하지 않음 |
| `66e7662` | 재캘리브 기준 teleop 리밋 환산, Extended Position 축 보호, 그리퍼 soft limit 복구 및 실기 개폐 | 텔레옵 도메인 변환식과 그리퍼 tick limit을 새 profile 체계로 다시 계산하지 않음 |
| `a28da51` | XM540과 XL430의 address 126 의미 차이를 반영한 전류 안전 정책, 정상 기동 전류 오탐 방지 | 전류 가드 예외와 기본 OFF 정책을 유지하고 도구 FSM 임계값과 혼합하지 않음 |
| `2cdd1aa` 및 포함 이력 | 손목 카메라 관측·거리/파지 캘리브레이션과 기존 카메라 경로 | 카메라 노드, ROI/HSV/거리식, TF 보정값을 런타임 도구 전환 작업에서 수정하지 않음 |
| `667ae9c` | 기존 GUI의 자세 저장·이동·삭제와 freedrive 순서 | 엔드이펙터 GUI 변경이 기존 팔 GUI/teleop 인터페이스를 침범하지 않게 회귀 테스트 |

보존 대상 숫자는 “더 깔끔한 구조”를 이유로 옮기거나 재정의하지 않는다. 변경이 꼭 필요하면
기존 실측값과 단위 변환을 그대로 참조하는 단일 출처를 먼저 확인하고, 별도 실기 증거 없이
값을 바꾸지 않는다.

기준선을 무조건 현재 main에 덮어쓰는 것도 금지한다. 8월 19일 이후 값 중 실제 하드웨어에서
다시 확인된 것이 있다면 그 증거가 우선한다. 단계 A에서 `656b2cd..HEAD`의 설정 차이를
`검증된 후속 변경 / 미검증 후보 / 병합 손상`으로 나누고, 검증된 후속 변경은 그대로 보존한다.

### 2.1 기존 랙피니언 그리퍼의 중요한 보존 조건

`66e7662`에서 실기로 확인된 기존 랙피니언 그리퍼 정책은 **ID 3만 위치 제어하고 ID 4는
torque off/free 상태로 두는 것**이다. 두 독립 위치 제어 루프가 같은 강체 레일을 밀면 서로
겨루고 과전류/트립이 발생했던 근거가 커밋에 남아 있다.

현재 runtime profile의 `dual_motor_gripper`는 ID 3·4를 모두 actuator로 취급한다. 이후의
per-motor endpoint 동기 제어가 실기에서 별도로 검증됐다는 증거가 확인되기 전에는 이를
8월 19일 기준 동작과 동일하다고 간주하지 않는다.

- `identity_ids`: 도구 종류를 식별할 때 버스에서 보여야 하는 ID
- `command_ids`: 실제 goal/torque 명령을 보낼 ID

기존 랙피니언 조립체는 식별 시 ID 3·4가 모두 보일 수 있어도, 1차 복구의 `command_ids`는
검증된 ID 3만 사용하고 ID 4는 torque off를 유지한다. 최신 dual FSM은 별도 실기 승인 profile로
분리하거나, 두 모터 동시 제어가 반복 시험을 통과한 뒤에만 기본값으로 승격한다.

## 3. 현재 구현에서 유지할 부분

다음은 새로 만들지 않고 현재 구현을 유지·보강한다.

- `/tool/change`, `/tool/status`, `/tool/detached` 인터페이스
- `ToolManager`, profile YAML, `SingleMotorGripperFSM`, `DualMotorGripperFSM`, `CleanerFSM`
- `_switch_tool_runtime()`의 profile·ID allowlist·FSM/helper 교체 흐름
- GUI의 공용 OPEN/CLOSE/STOP/ENABLE 버튼과 도구별 패널 재사용 구조
- arm FSM과 bridge/GUI 노드의 장기 실행 context
- 기존 E-stop, read-only, torque-off startup, partial dual fail-closed 정책

현재 mock에서는 `dual → spur → cleaner → dual`의 profile, ID, FSM class 교체가 정상이고,
E-stop, detached, active gripper goal, 미지원 도구 요청도 이전 context를 유지하며 거부한다.
따라서 전환 실행부를 다시 설계하지 않고 입력 트리거와 주변 수명주기만 보강한다.

## 4. 실제로 고쳐야 할 연결부

### 4.1 자동 교체 트리거 부재

`ToolIdentityProvider`는 스텁이고 실제 사용 구현은 parameter 값을 그대로 반환한다.
`_discover_tool_ids()`도 선택된 profile의 ID만 확인하므로 새 도구를 식별하지 않는다.

새 대형 FSM을 만들지 않고 다음처럼 기존 구조를 확장한다.

- `ToolIdentityProvider`의 새 구현으로 등록된 profile ID 서명을 read-only ping한다.
- 감지는 팔이 교체 가능 상태이고 도구 동작/팔 action이 없을 때만 수행한다.
- `{3,4}=기존 랙피니언 조립체`, `{5}=spur`, cleaner는 실제 profile에 ID가 확정된 뒤 등록한다.
- 감지용 `identity_ids`와 구동용 `command_ids`를 구분한다. 도구가 식별됐다는 이유만으로 발견된
  모든 ID에 torque를 켜지 않는다.
- dual은 한쪽만 보이면 `PARTIAL/FAULT`로 두고 절대 single tool로 추정하지 않는다.
- 1회 ping이 아니라 같은 결과가 연속 확인된 뒤 기존 `_switch_tool_runtime()`을 호출한다.
- GUI 콤보박스 요청은 자동 감지 실패 시 진단/수동 선택용으로 남긴다. 수동 선택도 실제 ID
  검증을 우회하지 못한다.

### 4.2 detach 래치와 정상 재장착의 충돌

현재 detach는 정지용 영구 래치이고, 래치 상태에서는 새 도구 전환도 거부한다.

- E-stop 래치는 기존 정책대로 유지한다.
- detach는 `ATTACHED → DETACHED → DETECTING → VALIDATING → READY` 상태로만 확장한다.
- detach 진입 시 기존 도구 STOP/torque off/SyncRead 해제는 현재 코드를 재사용한다.
- 새 ID가 안정적으로 발견되고 profile 검증이 끝난 경우에만 detach를 해제한다.
- 검증 실패 시 이전 도구를 되살리지 않고 `DETACHED/FAULT`로 안전하게 남긴다.

이 상태는 별도 프레임워크로 만들기보다 bridge의 현재 tool 상태와 `/tool/status` 필드를 최소
확장해 구현한다.

### 4.3 MoveIt 고정 모델과 런타임 tool joint 충돌

현재 spur로 전환하면 `/joint_states`에 `gripper_drive_joint`가 추가되지만 실행 중인 MoveIt은
dual-gripper 모델로 고정돼 있다. 그 결과 unknown joint 로그가 반복되고 `/compute_fk` 호출에서
`move_group`이 SIGABRT한다. Claude 2/2, 별도 점검 1회에서 동일하게 재현됐다.

1차 해결은 8월 19일의 검증된 팔 모델과 FK를 유지하는 최소 변경으로 한다.

- MoveIt/analytic IK에 전달하는 RobotState에는 기존 5개 `arm_joint_*`만 포함한다.
- 도구 모터의 raw tick, effort, online, torque, HW error는 기존 `/tool/status`를 사용한다.
- 도구 joint를 `/joint_states`에 유지해야 한다면 MoveIt 입력을 arm-only 토픽으로 분리한다.
- 기존 dual 모델, single 모델, SRDF를 새로 생성하거나 캘리브레이션하지 않는다.

도구별 충돌 형상이 실제 계획에 필요할 때만 2차로 기존 launch-time 모델 선택 자산을 재사용한다.
선택지는 공통 보수 collision envelope 또는 도구 모델 서브스택의 의도된 재기동이며, 새 URDF
재작성은 범위에 넣지 않는다. 1차 완료 조건은 도구 전환 후에도 기존 arm FK와 trajectory가
살아 있는 것이다.

### 4.4 GUI 옵션 패널 영구 비활성화

`pending_tool_change`는 성공 또는 명시적 bridge error에서만 해제된다. 브리지가 응답하지 않으면
패널 전체가 영구 비활성화된다. 브리지 없는 GUI에서 3초 후에도 같은 상태임을 재현했다.

- 요청 ID와 짧은 deadline을 GUI에 둔다.
- 성공, bridge error, timeout, bridge heartbeat 단절, 사용자 취소에서 pending을 해제한다.
- timeout 후 combo는 마지막 active 도구로 돌아간다.
- pending이 풀려도 연결/ready 조건이 false면 운동 버튼은 계속 안전하게 비활성화한다.
- GUI에 `요청값 / 감지값 / active 도구 / 차단 사유`를 구분해 표시한다.

### 4.5 GUI 종료 후 유령 노드

격리 도메인에서 GUI가 SIGTERM 후에도 생존하고 executor thread만 종료되는 것을 재현했다.

- SIGINT/SIGTERM을 Qt `quit()`으로 전달한다.
- `app.exec_()` 종료 후 executor shutdown, node destroy, thread join 순서를 보장한다.
- launch 테스트 종료 후 GUI PID와 동일 node name이 0개인지 검사한다.
- 모든 ROS 통합 테스트에 고유 `ROS_DOMAIN_ID`를 사용해 이전 실행의 publisher가 결과를
  오염시키지 않게 한다.

### 4.6 mock와 실기 준비 상태 혼동

같은 미완성 cleaner profile은 mock 검증 오류 0개, 실기 검증 오류 4개였다. mock의 완화 자체는
시뮬레이션 목적상 유지할 수 있지만 실기 준비 완료로 표시하면 안 된다.

- 기존 mock profile 완화 로직은 유지한다.
- `/tool/status`와 GUI에 `simulation_ready`와 `hardware_ready`를 분리한다.
- mock 화면에 `SIMULATION ONLY`를 고정 표시한다.
- 실기 ID read 실패 시 `online=False`, `motion_allowed=False`뿐 아니라 구체적인 `reason`도
  GUI까지 전달한다.

## 5. 8월 19일 코드의 선택적 복구

현재 main에는 런타임 전환과 별개로 과거 병합 과정에서 손상된 흔적이 있다. 이를 새로 구현하지
않고 `656b2cd`의 검증된 함수 단위로 복구한다.

| 현재 문제 | 복구 방법 |
|---|---|
| `_enable_torque()` 누락, startup 코드가 `_recover_gripper_range()` 안에 잘못 합쳐짐 | `656b2cd`의 torque enable/readback/profile 순서를 기준으로 함수 경계를 복원하고 현재 dual/spur allowlist만 결합 |
| calibration parameter callback 누락 | 8월 19일 callback과 단위/검증 규칙을 그대로 복원 |
| `_do_plan()` 중복, 미정의 `grasp_pose` | 8월 19일 PLAN/frozen-target/IK 흐름을 기준으로 한 함수로 복원한 뒤 tool-ready gate만 앞에 추가 |
| `_do_grasp()` 대기 중 즉시 실패 | 8월 19일 CLOSE 발행·동작 대기·effort 판정 순서를 복원하고 새 tool context 확인만 추가 |
| 기존 gripper/recorded path API 테스트 실패 | 8월 19일 실제 API와 현재 유지 계약을 비교해 복구 또는 명시적 폐기 결정 |
| 현재 dual FSM이 ID 3·4를 모두 구동 | 8월 19일 ID3-only 제어를 기본 호환 profile로 보존하고, 최신 dual 동시 제어는 별도 실기 검증 후 opt-in |

복구는 파일 전체 checkout이나 대규모 cherry-pick으로 하지 않는다. 현재 runtime tool FSM 변경을
보존하기 위해 함수별 diff와 테스트를 이용한 semantic transplant 방식으로 진행한다.

## 6. 구현 순서

### 단계 A — 실기 기준선 보호 테스트

- 별도 worktree에서 `656b2cd`의 함수·설정·토픽을 현재 코드와 비교한다.
- 5축 ID/center/ratio/limit, gripper tick/soft limit, current guard, stow, 카메라/TF 설정을
  변경 금지 목록으로 고정한다.
- `656b2cd..HEAD` 설정 차이를 검증된 후속 변경, 미검증 후보, 병합 손상으로 분류한다.
- 기존 랙피니언 도구에 대해 `identity_ids={3,4}`, `command_ids={3}` 계약을 테스트로 고정하고
  ID 4에 torque/goal write가 발생하지 않는지 검사한다.
- 8월 19일 실기 동작을 hardware adapter fake로 재현하는 characterization test를 먼저 만든다.
- 기존 49개 실패는 `현행 결함`, `fixture 미갱신`, `폐기 API`, `lint`로 분류한다. 전부 최신
  runtime PR의 결함으로 간주하지 않는다.

완료 조건: 이후 PR에서 위 설정이 바뀌면 즉시 실패하는 회귀 테스트가 있다.

### 단계 B — 기존 팔/그리퍼 제어 경로 복구

- 5절의 누락·중복 함수를 `656b2cd` 기준으로 선택 복구한다.
- 복구 후 8월 19일 기존 랙피니언(ID3-only 구동), arm trajectory, analytic FK,
  recorded path 테스트를 먼저 통과시킨다.
- 이 단계에서는 자동 ID 감지나 GUI 기능을 추가하지 않는다.

완료 조건: 8월 19일 제어 경로의 유지 대상 테스트와 현재 tool FSM 테스트가 함께 green이다.

### 단계 C — MoveIt 경계 수정

- FK/MoveIt 입력을 arm joint로 제한한다.
- `dual → spur → cleaner → dual` 각 전환 직후 FK, action server, PID 생존을 검사한다.
- 기존 tool feedback이 `/tool/status`에서 손실되지 않는지 확인한다.

완료 조건: unknown-joint 로그와 `move_group` 종료가 없고, 팔 자세 값은 전환 전후 동일하다.

### 단계 D — 기존 전환부에 자동 감지 연결

- 기존 `_discover_tool_ids()`와 bus lock을 재사용해 후보 profile의 read-only 탐지기를 추가한다.
- 안전 팔 상태와 action inactive gate를 통과한 경우에만 탐지·전환한다.
- partial dual, 복수 후보, 미등록 ID, stale sample을 fail-closed로 처리한다.
- 새 도구 검증 전에는 register/profile/torque write를 하지 않는다.

완료 조건: GUI에서 이름을 선택하지 않아도 ID 변화가 기존 `_switch_tool_runtime()`을 호출하고,
잘못된 조합에서는 write 0회로 거부된다.

### 단계 E — GUI 복구성과 종료 처리

- pending timeout/cancel/heartbeat 처리를 추가한다.
- 자동 감지 상태와 차단 이유, simulation/hardware readiness를 표시한다.
- 기존 자세 저장/이동/삭제와 arm teleop 패널의 동작을 회귀 검증한다.
- SIGINT/SIGTERM 종료 테스트를 추가한다.

완료 조건: 브리지 무응답 후 UI가 복구되고, 유령 GUI node/publisher가 남지 않는다.

### 단계 F — 실하드웨어 단계 검증

1. `read_only:=true`, torque OFF에서 기존 5축 `/joint_states`와 calibration 값이 8월 19일
   기준과 동일한지 확인한다.
2. 기존 랙피니언 도구를 연결해 ID 3 구동, ID 4 torque off 상태로 GUI/FSM 개폐를 재검증한다.
3. 팔은 `STOWED_LOCKED`로 두고 `기존 랙피니언 제거 → ID5 장착` 자동 인식을 검증한다.
4. ID5 제거 후 dual의 ID 3만 연결해 partial dual 차단을 확인한다.
5. 정상 dual을 장착해 왕복 복구를 확인한다.
6. 각 전환 후 `/compute_fk`, arm trajectory, 카메라 토픽, E-stop, current guard를 확인한다.
7. 마지막에만 FSM pick을 실행하고, 도구별 CLOSE/OPEN 또는 cleaner 동작을 검증한다.

실기 시험 중 calibration/limit 값이 예상과 다르면 자동으로 새 값을 저장하지 않는다. 시험을
중단하고 전원 사이클·다회전 카운트·장착 상태부터 확인한다.

## 7. 필수 테스트 매트릭스

| 시나리오 | 기대 결과 |
|---|---|
| 기존 랙피니언에서 OPEN/CLOSE | 8월 19일 endpoint/soft limit과 동일, ID 3만 명령, ID 4 torque off, HW error 없음 |
| arm 5축 피드백 및 stow | 기존 joint 이름·영점·리밋 유지 |
| dual → spur 전환 후 FK | 성공, unknown joint 없음, MoveIt PID 유지 |
| spur → dual 전환 후 arm trajectory | 기존 팔 target/feedback 불변, action 정상 |
| dual 제거 → ID5 장착 | spur 자동 식별, 검증 후 GUI 변경 |
| ID3만 장착 | partial dual fault, torque/register write 없음 |
| 기존 랙피니언을 식별 | ID 3·4 presence로 도구 식별, 구동 allowlist는 ID 3만 포함 |
| 최신 dual 동시 제어 profile 선택 | 별도 실기 승인 전에는 opt-in/validation-required, 기본 동작으로 사용하지 않음 |
| 팔 또는 도구 action 중 교체 | 요청 거부, 현재 context와 기존 action 유지 |
| detach 후 새 도구 검증 실패 | `DETACHED/FAULT`, 이전 도구 자동 재활성화 없음 |
| 브리지 없는 GUI 변경 요청 | timeout 후 pending 해제, 연결 단절 이유 표시 |
| GUI SIGINT/SIGTERM | 제한 시간 안에 PID/node/publisher 소멸 |
| production cleaner profile in mock | `SIMULATION ONLY`, hardware-ready false |
| production cleaner profile in hardware validation | 4개 누락 사유 표시, 동작 불가 |
| 카메라/TF smoke test | 기존 토픽·frame·calibration 값 유지 |
| E-stop 중 새 ID 장착 | 도구를 표시할 수는 있어도 torque/motion은 계속 차단 |
| `dual → spur → dual` 20회 | node/action/PID 유지, timer/subscription 누수 없음 |

자동 검증은 순수 단위 테스트, ROS+Qt 프로세스 내 테스트, 실제 launch 프로세스 테스트로 나눈다.
launch 테스트는 고유 `ROS_DOMAIN_ID`를 사용하고 종료 후 잔존 프로세스도 검사한다.

## 8. PR 분할

1. **PR-A: 8월 19일 기준선 회귀 테스트와 병합 손상 복구**
2. **PR-B: MoveIt arm-only 입력 경계 및 runtime 전환 생존 테스트**
3. **PR-C: 기존 ToolManager에 read-only ID 자동 감지 연결**
4. **PR-D: detach 재장착 흐름과 안전 상태 게이트**
5. **PR-E: GUI pending 복구, 상태 표시, 정상 종료**
6. **PR-F: 실하드웨어 검증 증거와 운영 문서 갱신**

각 PR은 직전 단계 테스트와 8월 19일 보호 테스트가 모두 green이어야 한다. mock 통과만으로
hardware-ready라고 표시하지 않는다.

## 9. 완료 정의

- 8월 19일의 5축 캘리브레이션, 관절 리밋, 그리퍼 endpoint, 전류 안전 정책이 보존됨
- 기존 카메라/TF/파지 관측 경로가 변경 없이 동작함
- 8월 19일 기존 랙피니언의 ID3-only 제어와 팔 제어가 먼저 정상이고 runtime tool FSM 테스트도 green임
- identity ID와 command ID가 분리되어 ID 4가 검증 없이 다시 구동되지 않음
- 물리 ID 변화가 GUI 수동 선택 없이 도구 전환을 트리거함
- 팔이 안전 정지 상태가 아니면 전환이 거부됨
- 검증 전 register/torque write가 없음
- 전환 후 MoveIt/FK/arm action이 살아 있고 unknown tool joint가 들어가지 않음
- 브리지 무응답 후 GUI가 복구되며 차단 이유가 보임
- GUI 종료 후 유령 node/publisher가 남지 않음
- 실제 `dual → spur → dual` 반복 시험과 장애 시험 로그가 `project_docs/`에 남음
- 기존 `MANUAL_GUI_HW_TEST.md`의 “runtime reprovisioning 불가” 설명이 최종 동작과 일치하게 갱신됨
