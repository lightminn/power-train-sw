# 2026 국방로봇경진대회 자율주행 SW 개발계획

> 작성: 2026-07-12  
> 상태: 조건부 승인 정본 후보 — WP5.2 계약 v2·안전 게이트 완료 전 합동 arm 금지
> 규정 근거: `docs/국방로봇_규정.pdf` SHA-256
> `2a55aaf26933a59f4b2a4279f0e9534b1d6b812e88ff878cdaf95f091f6cc0d3`  
> 적용 범위: 파워트레인 SW와 로봇팔 SW 간 통신 계약  
> 명시적 제외: 극한로봇대회, CAD, 기구 제작, 전장 설계, 배터리·방수 구조

> 2026-07-12 로봇팔 Notion의 `파워 트레인 협업 자료` 전체를 재검토해 독립 개발·데이터
> 인터페이스, 팔 작업 중 차체 이동 금지, 주행 중 팔 접힘·고정, 운반 중 파지 유지,
> bounded retry/timeout/skip, 명령권 분리와 센서 가림 요구를 반영했다.

> 2026-07-13 원격 팔 조종 영상 계약을 추가했다. L515 주행 영상 1280×720×30과 D435i 팔 영상
> 848×480×30을 동시에 유지하되, D435i raw RGB와 YOLO 결과를 분리 전송해 추론 지연이 영상
> cadence를 낮추지 않게 한다. 노트북이 최신 검출 결과만 합성하며 stale overlay는 숨긴다.

> 2026-07-13 DualSense 개별관절 원격조종 계약을 추가했다. DRIVE/ARM 모드는 상호배타이며,
> ARM에서는 `joint_1`~`joint_5` 선택 조그와 그리퍼만 허용한다. 기존 `robot_arm_msgs` 5종은
> 변경하지 않는다.

이 문서는 2026 국방로봇경진대회만을 대상으로 하는 파워트레인 SW 실행계획이다. 기존
`2026-07-02-autonomous-driving-kickoff.md`는 개발 이력으로 보존하고, 본 문서가 승인되면
국방대회 자율주행 우선순위와 완료 기준의 새 정본으로 사용한다.

현재 구현은 본 계획의 계약 v2와 호환되지 않는다. 파워트레인 `chassis_node`는 기본
`DRIVING`을 2 Hz로 발행하고, 2026-07-13 Jetson 로봇팔 HEAD는 이를 팔 언락과 취소 모션
재개로 해석한다. 또한 `/cmd_vel`의 팔 상태 게이트, 10 Hz 잠금 heartbeat와 CAN 배타 잠금이
아직 없다. WP5.2 완료 전에는 두 실물 노드를 함께 arm하지 않으며, 혼합 버전은 fail-closed
계약시험 외 production·HIL에 사용하지 않는다.

## 1. 규정에서 직접 도출한 목표

대회는 5개 구간에서 자율주행 1회와 원격주행 1회를 각각 수행한다. 각 구간은 100점이며
자율 70점, 원격 30점으로 총점은 자율 350점과 원격 150점이다. 구간별 제한시간은 20분이고,
트랙 이탈·물자 낙하·선도 로봇 접촉은 감점 대상이다. 규정의 트랙 사진, 곡률, 정지선,
장애물 배열은 예시이며 실제 배치는 달라질 수 있다.

따라서 SW의 최상위 목표는 다음 네 가지다.

1. 미지의 실제 배치에서도 트랙을 이탈하지 않고 저속 완주한다.
2. 의미 객체는 로봇팔 인식 결과를 사용하고, 주행 판단과 정지는 파워트레인이 소유한다.
3. 스모그·가림·험지·센서 단절에서 성능을 낮추더라도 안전하게 계속하거나 정지한다.
4. 자율 실패 시 원격으로 즉시 전환해 30점 경로와 남은 경기시간을 보존한다.

고정 지도, 고정 이동거리, 예시 트랙 좌표에 과적합하지 않는다. Nav2와 완전한 3D SLAM은
현재 점수 획득에 필요한 최소 기능이 아니므로 도입하지 않는다.

## 2. 구간별 SW 득점 목표

| 구간 | 자율 평가 항목 | 직접 담당 SW | 핵심 실패 조건 |
|---|---:|---|---|
| 1. 스모그 정찰 | 스모그 35, 피아식별 20, 완주 15 | terrain path, wheel+IMU, 로봇팔 인식 연동 | RGB/depth 동시 저하, 트랙 이탈 |
| 2. 환경 극복 | 사구·자갈·수중 45, 신호 20, 물자 5 | 험지 속도정책, 신호 이벤트, 팔 핸드셰이크 | 슬립, 빨간불 통과, 물자 작업 중 이동 |
| 3. 장애물 식별 | 비전마커 50, 완주 20 | 인식 이벤트 집계, 정차/서행, 중복 방지 | 5개 중복·누락, 오검출 확정 |
| 4. 겨울 | 빙판 35, 제설 25, 완주 10 | 슬립 감지, 저가속·저감속, 구간 상태머신 | 제자리 헛돌기, 과도한 조향 |
| 5. 정찰 동행 | 간격 유지 35, 재추종 25, 완주 10 | 3D target tracking, 가림 예측, 재획득 | 1.5~2.5 m 이탈, 접촉, 다른 대상 추종 |

배점은 기능 우선순위를 정하는 근거지만 규정에 변경 가능성이 명시돼 있다. 구현은 수치가
바뀌어도 정책 파라미터만 교체할 수 있게 한다.

## 3. 현재 재사용 가능한 기반

- WP1~WP5.1: 단일 `can0` 500 kbps, 10모터 제어, 50 Hz 차체 루프, `/cmd_vel`,
  `/wheel_states`, US-100 안전, latched E-stop과 auto-recovering motion hold.
- ROS2 Humble: `powertrain_ros`와 `robot_arm_msgs`, 로봇팔 그래프와 양방향 DDS 전달.
- L515 Gateway: 단일 SDK 소유, RGB 1280×720 30 fps 목표, raw depth 10 Hz, raw gyro/accel,
  TUI와 SRT의 공동 시작·정지, 재연결과 supervised restart.
- 로봇팔 인식: D435i color+depth 848×480×30, markerless YOLO와 `/detected_objects`,
  `/arm_status`; 파워트레인은 D435i를 열지 않는다. 현행 `/perception/debug_image → SRT` 시험
  노드는 YOLO cadence에 종속되므로 production 원격 영상 경로로 사용하지 않는다.
- 원격 운용: DualSense 텔레옵, 전용 AP, L515·D435i 동시 SRT 영상, D435i 별도 YOLO
  metadata, 진단 TUI.
- 2026-07-13 Jetson WP6 groundwork의 odometry·URDF·RViz·PointCloud 도구는 벤치 시각화 자산이다.
  `/diagnostics/obstacle/*`만 발행하며 production chassis authority에 연결하지 않는다. production
  terrain은 상시 PointCloud2 없이 raw depth ROI 내부 복원 계약을 따른다.

차체 조립 뒤 수행할 `base_link→l515_link` 실측, 최종 `stop_mm`, ODrive 13·14 재확인은
SW 구현 완료와 분리된 차량 커미셔닝 게이트로 유지한다.

### 3.1 차체와 트랙의 기하 입력

현재 `/home/light/urdf_2/urdf_2.urdf`를 영점 자세로 계산한 잠정 외형은 폭 약 0.96 m,
길이 약 1.08 m, 높이 약 0.63 m다. 타이어 접지면에서 상부 프레임까지는 약 0.57~0.60 m이고,
좌우 타이어 중심 간격은 약 0.879 m다. 규정의 트랙 폭 0.9144 m와 비교하면 바퀴 중심의
좌우 여유는 각 약 18 mm뿐이다. 이 값은 메시와 영점 자세 기반 추정이며 production 파라미터가
아니다.

차체 조립 후 다음 값을 실측해 terrain footprint 설정으로 고정한다.

- 실제 최대 외폭과 타이어 접촉면의 좌우 끝.
- 평탄면에서 접지한 여섯 바퀴 중심과 유효 wheel footprint.
- L515 렌즈 중심의 `base_link` 위치와 고정 pitch.
- 허용 가능한 타이어 돌출 여부와 최소 경계 여유.

SW는 로봇을 점으로 취급하지 않는다. 추정된 주행 가능 영역을 wheel footprint와 위치 불확실성만큼
안쪽으로 축소한 뒤 경로를 선택한다. 축소 후 영역이 사라지면 임계값을 억지로 줄이지 않고
motion hold한다.

## 4. 목표 아키텍처

```text
L515 RGB/depth ──> terrain_path_estimator ┐
L515 gyro/accel ─> odometry_estimator ───┼─> autonomy_controller ─> /autonomy/cmd_vel ─┐
/wheel_states ───> odometry_estimator ───┘                                             │
/detected_objects ─> event_adapter ─> segment_supervisor ──────────────────────────────┤
/arm_status ───────> arm_handshake ────────────────────────────────────────────────────┤
teleop adapter ────────────────────────────────────────────────> /teleop/cmd_vel ──────┘

chassis_node 내부 CommandAuthority + arm/safety final gate ─> ChassisManager ─> 10 motors

노트북 DualSense ─> versioned remote input ─> Jetson remote_input_gateway ─┬─> /teleop/cmd_vel
                                                                          └─> /arm/teleop_jog
                                                       robot-arm ArmCommandAuthority ─> Servo/bridge

모든 상태전이·authority·hold/E-stop·mission 결과 ─> append-only mission journal ─> TUI/replay
```

원격 영상은 제어 authority와 분리된 두 독립 채널이다.

```text
L515 Gateway raw RGB 1280×720×30 ───────────────> H.264/SRT :5000 ─┐
D435i 단일 owner raw RGB 848×480×30 ───────────> H.264/SRT :5002 ─┼─> 노트북 dual receiver
                         └─> YOLO latest-only ─> UDP JSON :5003 ──┘      └─> D435i overlay 합성
```

D435i 영상 sender는 YOLO 완료를 기다리지 않는다. YOLO worker는 밀린 frame queue를 처리하지 않고
최신 frame만 소비하며, 결과에 sender session ID, source frame sequence와 capture stamp를 포함한다.
노트북은 source stamp를 자기 clock과 직접 비교하지 않고 local receive monotonic age가 제한 안인
최신 metadata만 raw RGB 위에 합성한다. 결과가 stale하면 overlay만 제거한다. 영상과 metadata가
분리돼도 SRT frame freshness와 remote deadman은 별도 안전 입력으로 유지한다.

실제 로봇과 시뮬레이터는 production 알고리즘을 공유하고 하드웨어 경계만 교체한다.

```text
실차: L515 Gateway + real CAN/ChassisManager ─┐
                                               ├─ 동일 ROS 계약 ─> production autonomy nodes
시뮬: sensor bridge + simulated motor plant ──┘
```

production 코드에는 `if simulator == ...` 분기를 넣지 않는다. simulator 선택은 launch와 bridge
구성으로만 결정한다.

### 4.1 소유권 원칙

- L515 하드웨어 소유자는 Gateway 하나뿐이다. 소비 노드는 SDK를 직접 열지 않는다.
- D435i 하드웨어 소유자는 로봇팔 camera-owner process 하나뿐이다. 파워트레인과 별도 stream
  process는 SDK를 열지 않는다. owner는 한 번의 capture를 raw SRT와 latest-only YOLO 입력으로
  fan-out하며, production sender는 YOLO가 그린 `/perception/debug_image`를 영상 source로 쓰지 않는다.
- D435i raw 영상은 SRT `:5002`, 검출 metadata는 best-effort UDP JSON `:5003`을 사용한다.
  metadata에는 schema version, sender session ID, source frame sequence/capture stamp, bbox, class,
  confidence를 포함한다. 같은 session의 sequence 역행, 16 KiB 초과 payload와 local receive TTL을
  넘긴 결과는 노트북에서 폐기한다. source stamp는 correlation·로그용이며 Jetson과 노트북 clock을
  동기화했다고 가정해 직접 age 계산에 쓰지 않는다. overlay metadata는 command authority가 아니고
  유실돼도 raw 영상 cadence를 막지 않는다.
- ROS 입력은 raw depth image와 CameraInfo를 유지한다. terrain 프로세스 내부에서는 필요한 ROI를
  point cloud로 복원하지만 상시 ROS PointCloud2 토픽은 추가하지 않는다.
- `ChassisManager`를 포함해 실물 `can0`을 여는 모든 entry point는 공통 `RealCanSession`을 사용한다.
  `chassis_node`, legacy teleop, `motor_gui`, calibration·drive-test·demo·single-corner 도구까지
  예외 없이 포함한다. 실물 `can0` 소유권은
  `/run/powertrain/can0.lock`의 `flock`으로 강제하고 획득 실패 프로세스는 모터 연결 전에
  fail-closed 종료한다. fake·MuJoCo·`vcan` profile은 실물 잠금을 획득하지 않는다. 자율 노드는
  `/wheel_states`만 구독한다.
- 로봇팔 팀이 의미 인식의 단일 소스다. 파워트레인은 신호등·마커·마네킹 모델을 중복 실행하지 않는다.
- `CommandAuthority`는 ROS 없는 순수 코어이며 `chassis_node` 프로세스가 직접 소유한다.
  `chassis_node`는 `/autonomy/cmd_vel`과 `/teleop/cmd_vel`만 입력으로 받고, 선택·freshness·zero-confirmed
  handover·팔 interlock을 통과한 결과를 ROS `/cmd_vel` 재발행 없이 `ChassisManager.set()`에 전달한다.
  따라서 외부 `/cmd_vel` publisher 하나가 final gate를 우회하는 구조를 만들지 않는다.
- 노트북의 DualSense 입력은 Jetson의 단일 `remote_input_gateway`까지만 전달한다. Wi-Fi를 넘어 ROS2
  DDS를 직접 노출하지 않으며, version·session ID·단조 sequence를 가진 30 Hz remote-input frame을
  사용한다. Jetson은 자기 monotonic 수신시각으로 freshness를 판정하고 stale·재연결·sequence 역행
  시 drive와 arm 출력을 모두 0으로 만든다. 이 operator-input transport는 기존 `robot_arm_msgs`
  5종 협업 계약과 별도이며 그 wire schema나 토픽명을 바꾸지 않는다.
- `remote_input_gateway`는 `DRIVE`와 `ARM` 중 하나만 활성화한다. 한 frame이나 한 authority tick에서
  nonzero `/teleop/cmd_vel`과 nonzero `/arm/teleop_jog`를 동시에 만들 수 없다. 모드 전환은 client
  표시가 아니라 Jetson authority ACK가 권위이며, 차체 zero-confirmed와 팔 정지를 모두 확인한다.
- 로봇팔 내부 `ArmCommandAuthority`만 자동 FSM trajectory와 원격 `JointJog` 중 하나를 최종
  controller에 전달한다. 기존 `feat/teleop-keyboard`의 5축 `JointJog` 입력·limit 자산은 재사용하되,
  production에서 `/dynamixel/goal_position`을 직접 발행해 FSM/MoveIt을 우회하는 경로는 금지한다.
- mission journal은 명령 authority가 아니다. 제어 경로를 막지 않는 bounded queue를 통해 JSONL로
  기록하며 디스크 지연·용량 한계가 chassis 50 Hz나 safety 판정을 지연시키지 않게 한다.
- ROS graph의 source-topic publisher 수 검사는 진단·preflight로만 사용한다. graph discovery는
  안전 authority가 아니며, 실제 최종 작성자는 `chassis_node` 내부 코어와 CAN 잠금으로 보장한다.
- `chassis_node`가 US-100, 모터 상태, 명령 freshness를 최종 집행한다.
- 수동 reset이 필요한 상태만 E-stop이다. 자동복구 가능한 인식·경로·명령 상실은 motion hold다.
- 로봇팔은 차체 속도를 직접 명령하지 않는다. 로봇팔의 작업·파지 상태는 supervisor의 주행
  허가 입력이며, 최종 속도 권한은 항상 `chassis_node` 내부 `CommandAuthority`에 남는다.
- 0이 아닌 주행 명령은 로봇팔의 `STOWED_LOCKED` 또는 `CARRYING_LOCKED` heartbeat가
  fresh할 때만 허용한다. 기존 `DRIVING=팔 언락` 의미는 폐기하고, 모든 주행 모드에서 팔은
  접힘·고정 상태여야 한다.
- `MISSION_STOP`은 팔의 유일한 언락·작업 허가 의도다. 그 외 모든 `ChassisMode` 값과 stale은
  팔의 접힘·잠금 유지를 뜻한다. `STOW_REQUEST`는 작업 완료·실패·운영자 skip 뒤 팔을 접고
  잠그라는 명시적 요청이다. 실제 wheel 정지가 확인되기 전에는 `MISSION_STOP`을 송신하지 않으며
  팔은 그 전에 잠금을 풀거나 작업 자세로 전환하지 않는다.

### 4.2 로봇팔 협업 안전 계약

기존 `robot_arm_msgs` 5종의 wire schema는 유지하고 문자열 어휘와 전이 규칙을 확장한다.
구현 전 양 팀 저장소가 동일한 어휘·QoS·timeout을 사용하도록 계약 시험을 둔다.

`ChassisMode.mode`의 계약 v2 어휘는 다음과 같다.

- `MISSION_STOP`: 실제 wheel 정지 뒤에만 유효한 유일한 언락·작업 허가.
- `STOW_REQUEST`: 작업 완료·실패·중단 뒤 안전한 주행 자세로 접고 잠그라는 요청. **payload-aware**
  여야 한다 — 파지 확정 이후에는 release 없이 `CARRYING_LOCKED`로 접힘이 기본이며, release가
  필요한 중단은 별도 명시 절차다. (2026-07-14 팔 v2 코드는 운반 상태에서도 무조건 `RELEASE`로
  전이하므로 이 항목은 합의·수정 대상 잔여다.)
- `DRIVING/CORNERING/ROUGH_TERRAIN/FOLLOW_LEAD`: 모두 팔 잠금 유지. 현행 로봇팔의
  `DRIVING → unlock` 분기는 양 팀 동시 컷오버에서 제거한다.
- 미인식 값과 stale: default-deny 잠금 유지. 마지막 모션 재개 금지.

`ArmStatus.status`는 기존 `IDLE/PERCEIVING/PLANNING/EXECUTING/CARRYING/DONE/FAILED`에
다음을 추가한다.

- `WORK_READY`: 동일 `mission_id`의 도착 이벤트를 수락했고, 차체 정지 상태에서 작업할 준비가 됨.
- `STOWING`: 작업 종료 후 주행 자세로 복귀 중.
- `STOWED_LOCKED`: 팔이 접혀 기계적·제어적으로 주행 가능한 상태. 주행 중 주기적으로 발행.
- `CARRYING_LOCKED`: 물자를 든 팔이 운반 자세로 접혀 잠겼다는 자세 상태. 파지 검출기가
  qualification을 통과한 경우에만 파지 정상까지 의미하며 운반 주행 중 주기적으로 발행.
- `GRIP_LOST`: 운반 중 파지 상실. 파워트레인은 제어된 motion hold를 수행하고 운영자에게 알림.

두 locked heartbeat는 문자열 선언만으로 만들 수 없다. 팔 측 adapter는 각 상태마다 승인된 joint
pose tolerance, 모든 joint velocity threshold, 연속 dwell, 취소·action result 완료, controller fault 0,
torque hold를 동시에 확인한다. `CARRYING_LOCKED`는 grip detector가 qualification을 통과한 경우
여기에 qualified grip predicate를 추가하며, 미통과 시에는 자세·잠금 조건만으로 발행하되
파워트레인이 보수적 운반 profile과 하역 전 재확인을 강제한다(위 status 정의와 동일).
자세·잠금 조건이 하나라도 불충족이면 `STOWING` 또는 실패 상태를 발행하고 locked heartbeat를
만들지 않는다.

2026-07-13 로봇팔 Notion의 현행 내부 FSM은 `CARRY`와 `LOCKED` interrupt를 사용한다.
`STOWED_LOCKED/CARRYING_LOCKED`는 이를 대체하는 팔 내부 상태가 아니라 주행 허가용 wire heartbeat
adapter다. 현행 문서의 `DRIVING→unlock`과 `DONE→재출발`은 v1 현황으로만 취급하고 v2 동시 컷오버
뒤에는 사용하지 않는다. 팔은 fresh `CARRYING_LOCKED` 상태에서도 새 `ARRIVED_DROP`을 멱등 수락해야 한다.

`DONE`은 작업 동작 완료 진단일 뿐 수신 여부나 단독 값이 주행 허가가 아니다. 권위 있는 성공
ACK는 픽업이면 동일 `mission_id`의 fresh `CARRYING_LOCKED`, 하역이면 동일 ID의 fresh
`STOWED_LOCKED`다. 따라서 순간적인 `DONE`이 Keep Last 1 history에서 보이지 않아도 최종 상태로
성공을 확정할 수 있다. 2026-07-14 팔 v2 구현은 `DONE`을 아예 발행하지 않는다 — 본 계약과
합치하며, 파워트레인 시험은 "미래에 `DONE`이 수신돼도 무시"하는 회귀로 유지한다.
기존 `CARRYING`은 파지·운반자세 전환 중 상태로만 사용하고 주행 허가로 사용하지 않는다.
`CARRYING_LOCKED` stale·`FAILED`·`GRIP_LOST`는 fail-open 없이 motion hold로 처리한다.
물리 충돌이나 별도 latched 위험이 확인되지 않은 로봇팔 통신 장애 자체는 E-stop으로 승격하지 않는다.
`GRIP_LOST`는 E-stop이 아닌 supervisor-latched motion hold다. 새 heartbeat만으로 자동 해제하지
않고 bounded regrasp 성공 또는 운영자 확인·승인 뒤에만 해제한다.

`GRIP_LOST`를 안전 근거로 사용하려면 로봇팔 팀이 검출 source를 명시해야 한다. 현재 Jetson 팔
FSM은 그리퍼 joint effort를 읽지만 threshold가 placeholder이므로, 무부하·정상 파지·진동·강제
미끄러짐 반복 HIL에서 miss 0과 허용 false positive를 기록하기 전에는 qualified detector가 아니다.
검출 수단이 qualification을 통과하지 못하면 `GRIP_LOST`를 보장된 안전 신호로 주장하지 않고,
운반 profile을 최저속·최단경로·낮은 가감속/조향 slew로 제한하며 하역 직전 운영자 또는 팔 측
재확인을 요구한다.

필수 실패 계약은 현행 wire 어휘 `FAILED`다. 로봇팔 팀이 합의·구현하면 `IK_FAILURE`,
`TRAJECTORY_FAILURE`, `SELF_COLLISION`, `BASE_COLLISION`, `JOINT_OVERCURRENT`, `GRIP_UNCERTAIN`,
`STOW_FAILURE`, `ACTION_TIMEOUT`을 선택적 진단 어휘로 추가한다. 세분화된 어휘는 안전 gate의
필수 의존성이 아니며 미지원 시 `FAILED`를 사용한다. 파워트레인은 어떤 실패 값도 성공으로
재해석하지 않고 mission journal과 TUI에 보존한다. 실패 후 자세도 작업 종류를 보존한다. 픽업 실패는
release를 확인한 뒤 `STOWED_LOCKED`, 하역 실패·release 불확실은 `CARRYING_LOCKED` 또는
`GRIP_UNCERTAIN/FAILED_HOLD`로 남는다. release 확인 없이 `STOWED_LOCKED`로 축약하지 않는다.
재출발은 작업 종류에 맞는 fresh `STOWED_LOCKED/CARRYING_LOCKED`와 supervisor의 명시적 전이 뒤에만
가능하다.

작업 시도마다 새로운 단조 증가 `mission_id`를 할당한다. 이전 ID의 지연 메시지는 무시하고,
동일 ID의 `ArrivalStatus` 재수신은 팔 작업을 중복 실행하지 않는 멱등 동작이어야 한다. 파워트레인은
`WORK_READY` 또는 명시적 실패를 받을 때까지 `ArrivalStatus`를 제한된 주기로 재발행한다. 프로세스
재시작 뒤 ID 재사용을 막기 위해 파워트레인은 host 영속
`/var/lib/powertrain/mission_id`의 양수 `int32` 카운터를 publish 전에 원자적으로 증가·저장한다.
대회 run 중 wrap은 허용하지 않으며 손상·범위 초과 시 새 작업을 시작하지 않고 motion hold한다.

`WORK_READY/DONE/FAILED`는 mission-scoped 상태이므로 현재 `mission_id` 일치를 요구한다.
픽업 직후 최초 `CARRYING_LOCKED`와 하역 직후 최초 `STOWED_LOCKED`도 해당 작업 ID와 일치해야
한다. 이후 주행 중 잠금 heartbeat는 freshness와 상태를 주행 허가에 사용하되 이전 미션의
`DONE`으로 해석하지 않는다. 활성 미션이 없는 빈 적재 상태의 `STOWED_LOCKED`는 `mission_id=0`
또는 **마지막 완료 ID 유지** 둘 다 유효하다(2026-07-14 팔 v2 구현 = 마지막 완료 ID 유지).
파워트레인은 완료 ACK 판정에서만 ID를 해석하고 상시 heartbeat의 ID는 해석하지 않는다.

`ChassisMode`와 로봇팔의 현재 상태 `ArmStatus` heartbeat는 10 Hz로 발행한다. freshness는
수신시각이 아니라 `header.stamp` 기준 `0 ≤ now-stamp ≤ 0.5 s`로 판정하며 stamp가 0, 미래,
비단조·역행이면
폐기한다. heartbeat 토픽은 Reliable, Keep Last 1, Volatile을 사용한다. 이벤트 토픽
`ArrivalStatus`는 Reliable, Keep Last 10, Volatile로 2 Hz 재발행한다. 같은 ID의
`WORK_READY/PERCEIVING/PLANNING/EXECUTING` 중 하나를 받으면 작업 수락 ACK로 보고 재발행을
멈추며, `FAILED` 또는 2 s 무응답이면 정지 상태를 유지한다. timeout은 heartbeat 주기의 3~5배를
유지하고, 전체부하에서 지연되면 timeout을 줄이지 말고 발행·스케줄링과 부하를 먼저 개선한다.
미인식 ArmStatus는 직전 locked sample을 계속 허가하지 않고 즉시 motion hold하며 raw status와 stamp를
`CONTRACT_VIOLATION`으로 journal/TUI에 기록한다. 다음 recognized locked heartbeat 전에는 해제하지 않는다.
node clock이 뒤로 이동하거나 기준 clock domain이 바뀌면 기존 sample과 dwell을 모두 무효화하고 새로
단조 증가하는 heartbeat가 쌓일 때까지 motion hold한다.

```text
DRIVE (fresh STOWED_LOCKED or CARRYING_LOCKED)
  → STOP_REQUESTED (authority final output=0)
  → qualified per-wheel stop thresholds satisfied for configured dwell
  → MISSION_STOP + ARRIVED_*(mission_id) publish
  → arm waits for both in either DDS arrival order
  → WORK_READY or later accepted state (same mission_id)
  → ARM_WORK including automatic stow to task-specific drive posture
  → optional DONE(mission_id) diagnostic
  → authoritative success ACK:
       pickup: fresh CARRYING_LOCKED(same id)
       drop: fresh STOWED_LOCKED(same id)
  → safety/freshness/command-authority 재검사
  → RESUME
```

timeout은 자동 출발 조건이 아니다. 제한된 재시도 또는 운영자 skip을 기다리며 정지 상태를
유지한다. 자율과 원격 모두 같은 팔 잠금·파지 계약을 통과해야 한다.

실패 경로는 `FAILED/ABORT → motion hold → bounded regrasp 또는 STOW_REQUEST → STOWING →
STOWED_LOCKED → 운영자 skip 승인 → 새 명령으로 RESUME`이다. 팔 상태 회복이나 heartbeat 복귀만으로
이전 `/cmd_vel`을 자동 재개하지 않는다.

팔 노드가 사망하고 ArmStatus가 stale하며 active mission이 취소된 경우에만 별도
`~/arm_lock_override` (`std_srvs/SetBool`)를 허용한다. override는 자율에 노출하지 않고 원격 저속
profile만 허용한다. `GRIP_LOST` latch, 미인식 status, US-100, motor fault, E-stop, watchdog와
command authority는 절대 우회하지 않는다. 팔 heartbeat가 복귀하면 flag는 감사 목적으로
유지하더라도 drive permission은 즉시 0이 된다. 운영자가 fresh `STOWED_LOCKED`를 확인하고 override를
명시적으로 끈 뒤에만 정상 원격 profile을 새로 선택한다. 이전 mission·명령은 자동 재개하지 않는다.
가능하면 별도 `/joint_states` receiver가 pose·velocity locked predicate를 독립 확인해야 override를
승인한다. 이 채널도 없을 때는 운영자 2단계 육안 확인, 짧은 TTL, hold-to-run deadman과 최저 속도
상한을 모두 요구한다. 팔 노드가 죽는 순간 독립 관측에서 팔이 움직이는 중이면 override를 거부한다.

### 4.3 데이터 freshness

각 입력은 값과 함께 stamp, age, validity를 전달한다. 오래된 마지막 값을 정상값처럼 재사용하지
않는다. 영상·depth·인식·odom 중 하나가 끊겨도 서로의 freshness를 독립적으로 판정한다.
`DetectedObjectArray.header.frame_id`는 비어 있으면 안 되며 해당 stamp에서 `base_link`로 TF 변환
가능해야 한다. TF 부재·stale·변환 실패는 fail-open 없이 해당 인식 의존 행동을 motion hold한다.
로봇팔 팀이 D435i extrinsic과 정적·동적 TF 발행을 소유하고, 파워트레인은 array header frame에서
`base_link`로 변환한다. 좌표는 REP-103, 단위 m를 따른다. 로봇팔 카메라 extrinsic 변경은
인터페이스 변경으로 취급해 거리·좌우 오차 acceptance를 재실행한다.

센서와 협업 토픽의 시간 품질도 값의 품질과 별도로 판정한다. 각 입력에 대해 header stamp와
local receive clock의 차이, 동일·역행 stamp 수, RGB/depth/IMU/wheel 간 skew를 기록한다. 알려진
고정 표적을 사용해 `base_link→l515_link`, D435i optical axis·부호, `base_link` 변환 뒤 XYZ 오차와
팔 자세별 extrinsic 반복성을 qualification한다. 시간·TF qualification을 통과하지 않은 입력은
confidence를 올리거나 작업·주행 허가를 만들 수 없다.

### 4.4 Jetson-first compute partition

production 최적화 목표는 개별 kernel 최고속도가 아니라 Orin Nano 8 GB에서 RGB 전송,
로봇팔 YOLO, terrain, ROS, CAN·안전을 동시에 안정적으로 유지하는 것이다.

- GPU 후보: depth deprojection, 좌표변환, elevation scatter, 표면 특징, RGB 대량 전처리와
  신경망 추론.
- CPU 고정: CAN, US-100, 50 Hz chassis loop, E-stop/motion hold, command authority,
  segment FSM, 작은 wheel+IMU odometry, 프로세스 supervision.
- GPU 입력은 프레임당 한 번만 배열화하고 terrain 결과가 완성될 때까지 GPU에 유지하며,
  CPU에는 작은 path/diagnostic 결과만 반환.
- Orin은 CPU/GPU가 같은 물리 DRAM을 공유하므로 외장 GPU식 PCIe `H2D`로 표현하지 않는다.
  실제 측정 대상은 CPU buffer에서 accelerator array로의 materialization/copy, cache coherency,
  synchronization과 memory-bandwidth 비용이다.
- producer가 호환 device buffer를 제공할 때만 DLPack 등 zero-copy를 사용한다. L515 CPU buffer에
  불가능한 zero-copy를 가정하거나 수명·동기화가 불명확한 buffer를 공유하지 않는다.
- terrain accelerator는 별도 프로세스와 cgroup/container에 두어 memory/CPU 한도를 적용하고,
  종료·OOM이 chassis/safety 프로세스를 함께 죽이지 않게 한다.
- CPU affinity/cpuset 후보를 전체부하에서 검증해 chassis/safety가 x264와 NumPy terrain에
  굶지 않게 하되, 검증 없이 realtime priority나 core pinning을 production에 적용하지 않는다.
- Orin Nano에서 사용할 수 없는 `nvv4l2h264enc`를 다시 도입하지 않고 software x264를 유지한다.
- 전체부하 profile은 L515 1280×720×30과 D435i 848×480×30의 software x264 두 sender를 동시에
  포함한다. 둘 중 하나의 해상도·FPS를 조용히 낮춰 부하시험을 통과시키지 않으며, 먼저 bitrate,
  preset, worker thread, 불필요한 frame copy와 L515 depth/overlay submit을 조정한다.

### 4.5 Jetson production service partition

Compose는 책임과 장애 전파 경계를 다음 네 서비스로 고정한다.

- `powertrain_ros`: 기존 L515 Gateway·ROS image. 카메라/SRT 단일 소유 계약을 유지하며 control을
  함께 띄우지 않는다.
- `powertrain_control`: `chassis_node`, US-100, teleop input adapter를 감독한다. `network_mode: host`,
  필요한 `/dev`, `/run/powertrain`, `/var/lib/powertrain` bind와 restart policy를 명시한다. ROS image에
  pygame/SDL 또는 승인된 web-input dependency를 추가해 production teleop이 ROS adapter로
  `/teleop/cmd_vel`만 발행하게 한다. direct-CAN teleop은 진단 profile에만 남긴다.
- `powertrain_autonomy`: terrain와 autonomy controller를 같은 GPU-enabled 프로세스에 둔다.
  내부 결과는 immutable dataclass로 전달하고 외부 출력은 `/autonomy/cmd_vel` 하나뿐이다. L4T R36.5
  aarch64에서 pin한 CUDA/JAX image, cgroup memory limit, healthcheck와 supervised restart를 가진다.
- `powertrain_observability`: journal daemon만 실행한다. host network, `/run/powertrain`,
  `/var/lib/powertrain/runs`, `PYTHONPATH=/workspace`, 명시적 command/entrypoint와 restart policy를 둔다.

서비스별 Compose contract와 install-space import를 자동시험한다. 한 서비스 kill/OOM이 다른 서비스의
owner lock이나 50 Hz control을 함께 종료시키지 않아야 한다.

로봇팔 저장소의 D435i camera owner와 sender는 위 네 powertrain 서비스 밖의 robot-arm 소유
process다. powertrain Compose가 이를 재시작하거나 D435i를 fallback-open하지 않는다. 양쪽 sender의
상태와 노트북 receiver feedback만 observability가 읽으며, 한 encoder의 종료·재시작이 다른 encoder나
camera SDK owner를 함께 죽이지 않아야 한다.

## 5. 공통 기반 작업

### WP5.2. 로봇팔 계약 v2와 차체 안전 게이트

WP6보다 먼저 수행한다. 이 작업은 기존 5개 msg의 wire schema를 바꾸지 않고 문자열 계약,
순수 상태 코어와 최종 차체 게이트를 구현한다.

파일 책임과 변경 경계는 다음과 같다.

- `ros2/src/powertrain_ros/powertrain_ros/contract.py`: 계약 v2 문자열과 상태 집합의 단일 출처.
- `ros2/src/powertrain_ros/powertrain_ros/arm_interlock.py`: ROS 없는 순수 freshness,
  mission correlation, `drive_allowed`와 work-permit 판정.
- `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`: `ArmStatus` 수신·stamp 검증,
  최종 motion hold 집행, `contract_v2_verified` compatibility lock, 10 Hz 동적 `ChassisMode`,
  진단과 제한된 override 서비스.
- `motor_control/chassis/runtime_lock.py`: 공통 `RealCanSession`과 실물 CAN용 `flock` 획득·해제.
  fake·simulation과 분리하며 real CAN을 여는 GUI·교정·시험·demo까지 전 entry point에 적용.
- `motor_control/chassis/teleop_server.py`, `teleop_dualsense.py`: production에서는 입력 adapter로만
  사용하고 `powertrain_control`에서 `/teleop/cmd_vel`을 발행. direct-CAN은 진단 profile로 격리.
- `ros2/src/powertrain_ros/powertrain_ros/wheel_stop.py`와 `config/wheel_stop.yaml`: 6축 HIL로
  qualification한 per-wheel stop threshold와 dwell의 단일 출처. unqualified이면 work permit과
  authority handover를 모두 거부.
- `ros2/src/powertrain_ros/powertrain_ros/mission_supervisor.py`: 이후 WP8에서 영속
  `mission_id`, 정지 확인, 이벤트 재발행·ACK·abort/stow를 단독 소유.

구현 순서는 다음과 같다.

1. 현행 로봇팔 `DRIVING → unlock`을 재현하고 `contract_v2_verified=false`에서 구 팔도 잠그는
   `CORNERING`만 발행해 이전 모션 재개와 nonzero 구동을 막는다.
2. `contract.py`와 순수 `arm_interlock.py`를 테스트 우선으로 구현한다.
3. 우리 `chassis_node`에 default-deny 팔 상태 게이트, stamp freshness, QoS depth 1을 연결한다.
4. 실물 CAN runtime lock과 같은 host bind를 두 production 컨테이너의 모든 실물 entry point에 연결한다.
5. 로봇팔 팀이 `MISSION_STOP` 유일 언락, `STOW_REQUEST`, 10 Hz 현재상태 heartbeat와
   멱등 `ArrivalStatus`를 같은 컷오버로 구현한다.
6. 혼합 버전 fail-closed, mock 계약시험, 실제 DDS 1사이클 순으로 검증한다.

`/chassis_state`는 계약 타입 `ChassisMode`에 임의 진단 문자열을 담지 않도록 별도 진단 타입 또는
`std_msgs/String`으로 분리한다. 계약 어휘 검증과 운용 진단을 섞지 않는다.

파워트레인 단독 실차 시험을 위해 `arm_gate_mode=arm_absent_field` launch profile을 함께 정의한다.
활성 조건은 ① `/arm_status` publisher가 ROS graph에 부재(주기 재확인, publisher 출현 시 다음
tick에 mock 비활성·default-deny 복귀), ② 운영자가 팔의 기계적 접힘·고정을 육안 확인(바퀴 부양
확인과 동급의 명시 절차), ③ journal/로그에 profile·확인자 기록이다. 이 profile은 내부 mock
locked heartbeat로 arm freshness gate만 대체하며 `MISSION_STOP`/`ArrivalStatus` 발행과 팔 작업
허가는 계속 금지되고, US-100·motor·E-stop·command watchdog·authority gate는 그대로 적용된다.
대회 production launch에는 포함하지 않는다. 이 profile이 없으면 팔 팀의 v2 배포 전까지 모든
실차 주행 HIL(오도메트리·terrain·원격 리허설)이 차단된다는 것이 도입 근거다.

완료 기준:

- 현행 구 로봇팔 노드와 새 chassis를 함께 실행해도 source topic의 nonzero 명령이 모터에 도달하지 않음.
- compatibility profile에서 `DRIVING/MISSION_STOP/ArrivalStatus` 발행 0이며 구 팔이 이전 모션을
  재개하지 않음. 이 결과는 혼합 production 승인이 아니라 동시 컷오버 전 deployment gate임.
- fresh `STOWED_LOCKED/CARRYING_LOCKED`가 없으면 50 Hz 다음 tick에서 drive 0.
- `ArmStatus` 과거 버스트·역행 stamp·미인식 status가 주행 허가를 만들지 않음.
- `chassis_node` 실행 중 legacy teleop entry point가 실물 `can0` 잠금 획득에 실패해 모터 연결 전 종료.
- `powertrain_jetson`과 `powertrain_ros`가 같은 host inode의 lock을 두고 경쟁해 두 번째가 실패함.
- ODrive 11~16 전진·후진·좌/우 pivot 후 정지 노이즈를 각각 10회 측정하고, node 13/14 포함
  per-wheel threshold와 dwell을 동결하기 전 work permit이 fail-closed함.
- 계약 v2 mock과 실제 로봇팔에서 10 Hz heartbeat와 default-deny mode 의미가 일치.
- `arm_absent_field`에서 `/arm_status` publisher가 출현하면 다음 tick에 mock이 비활성화되고
  default-deny로 복귀하며, 이 profile에서 `MISSION_STOP`/`ArrivalStatus` 발행이 0회임.

### WP5.2-T. 원격 명령의 공통 authority 통합

현재 `teleop_server.py`와 `teleop_dualsense.py`는 자체 `ChassisManager`로 실물 CAN을 열어
`chassis_node`, US-100, 팔 gate와 향후 `command_authority`를 우회한다. CAN lock만 추가하면
동시 실행은 막지만 자율↔원격 전환이 안전한 handover가 되지는 않는다. 따라서 WP5.2와 같은
우선순위로 원격 입력 경로를 ROS authority에 통합한다.

```text
DualSense/web remote → /teleop/cmd_vel ┐
autonomy controller  → /autonomy/cmd_vel├→ chassis_node 내부 CommandAuthority
operator mode/select → authority FSM ───┘              ↓
                              arm/safety final gate → one ChassisManager/can0
```

- DualSense·web 원격 프로세스는 입력 adapter가 되어 `/teleop/cmd_vel`과 freshness만 발행한다.
- `chassis_node` 내부 `CommandAuthority`만 최종 setpoint를 만들며 ROS `/cmd_vel` writer/subscriber
  경계 없이 zero-confirmed 상태에서 source를 전환한다.
- `chassis_node`는 자율/원격을 구분하지 않고 동일한 US-100·motor·arm gate를 최종 집행한다.
- legacy standalone direct-CAN teleop는 `chassis_node`가 완전히 종료됐고 운영자가 팔 접힘을
  확인한 진단·복구 모드로만 남긴다. production 원격 점수 경로로 사용하지 않는다.
- `/autonomy/cmd_vel`과 `/teleop/cmd_vel` publisher count는 진단한다. source graph 값만으로 최종
  주행 허가를 만들지 않으며, `chassis_node`는 외부 `/cmd_vel`을 구독하지 않는다.
- 반자동 원격 보조는 별도 모터 owner가 아니라 `command_authority` 안의 원격 profile로 둔다.
  운영자가 전진·후진 속도 의도를 계속 소유하고, terrain 중심/heading 보정, bank·clearance 기반
  속도 상한, 선도 로봇 거리 보조, 작업점 자동 정렬과 zero-confirmed stop만 제한적으로 적용한다.
  보조 입력이 stale하거나 confidence가 낮으면 보정만 제거하고 원격 저속 또는 motion hold로
  전이하며, autonomy command로 조용히 승격하지 않는다.
- 반자동 보조는 기본 OFF·operator opt-in이다. 조종기의 전용 `ASSIST_BYPASS` 입력 하나로 다음
  authority tick에서 모든 보정을 0으로 만들고 raw teleop으로 복귀한다. raw teleop도 동일 chassis
  safety gate를 통과한다.

완료 기준:

- 자율↔원격 전환에서 `chassis_node`와 CAN owner 프로세스를 교체하지 않음.
- 전환 전후 nonzero 명령 공백·중첩 0, zero-confirmed handover.
- zero-confirmed 판정은 authority 출력 0, fresh·단조 `WheelStates`, 정확히 6개 unique wheel,
  finite 속도, `healthy=true`, 모든 stale=false, axis/fault=0과 qualified per-wheel threshold를 모두
  요구한다. 불량 sample은 dwell을 리셋하고 median 판정을 사용하지 않는다.
- 원격 명령도 팔 heartbeat stale·`GRIP_LOST`·US-100·motor fault에서 같은 hold/E-stop 전이.
- legacy direct-CAN teleop는 production chassis 실행 중 CAN lock 단계에서 실패.
- 반자동 보조의 개입량·confidence·해제 원인이 TUI와 mission journal에 기록되고, 보조 프로세스
  사망 시 마지막 보정 명령이 유지되지 않음.

#### DualSense DRIVE/ARM 조작 계약

DualSense 하나로 차체와 팔을 조작한다. 아래 표는 구현·HIL을 시작하기 위한 **초기 키매핑 후보**이며,
운전자 사용성 시험 뒤 versioned config로 변경할 수 있다. DRIVE/ARM 상호배타, deadman,
stow-before-drive와 전역 E-stop 의미는 키 위치와 무관한 고정 안전 계약이다.

| 초기 후보 입력 | DRIVE mode | ARM mode |
|---|---|---|
| R2 / L2 | 전진 / 후진 | 그리퍼 열기 / 닫기 |
| 좌스틱 X | 차체 회전 | 사용 안 함 |
| 우스틱 Y | 사용 안 함 | 선택 관절 signed velocity jog |
| D-pad 좌 / 우 | 사용 안 함 | `joint_1`~`joint_5` 순환 선택 |
| L1 hold | 원격 drive deadman | 원격 arm deadman |
| CREATE+OPTIONS 1 s hold | ARM 전환 요청 | DRIVE 전환·stow 요청 |
| ○ | 전역 수동 latched E-stop | 전역 수동 latched E-stop |

- ARM 진입은 fresh D435i raw receiver feedback, fresh remote input, L1 deadman, actual wheel-stop,
  fresh `MISSION_STOP`, fresh·단조 `/joint_states`, controller fault 0, 기존 FSM action cancel과 모든
  joint 정지 확인을 모두 요구한다. 하나라도 깨지면 원격 팔 motion hold이며 차체는
  `MISSION_STOP`을 유지한다.
- ARM mode는 한 번에 한 arm joint만 `control_msgs/msg/JointJog` velocity로 명령한다. 그리퍼는
  같은 authority가 소유한 `FollowJointTrajectory` 경로를 사용한다. 두 trigger가 동시에 눌리거나
  selected joint가 유효하지 않으면 그 출력을 0으로 만든다.
- 로봇팔은 원격 조작 중 기존 `ArmStatus=EXECUTING`을 10 Hz로 발행한다. 새 wire status를 만들지
  않는다. DRIVE 전환 요청 시 jog를 0으로 만들고 remote authority를 해제한 뒤
  `STOW_REQUEST → STOWING → STOWED_LOCKED`를 완료해야 차체 authority를 넘긴다.
- 초기 production에는 `home=all zero` 단축키를 넣지 않는다. 현재 자세와 충돌경로를 모른 채 여러
  관절을 동시에 원점으로 보내지 않고, 접힘은 검증된 `STOW_REQUEST` trajectory만 사용한다.
- 로봇팔 production 경로는 5축 URDF/controller와 `joint_1`~`joint_5` feedback을 먼저 qualify한다.
  `JointJog`는 `ArmCommandAuthority`가 선택한 동안에만 MoveIt Servo의 joint-limit·collision·singularity
  제한을 거쳐 단일 hardware bridge로 전달한다. 현재 확인된 Jetson 환경에는 MoveIt Servo가 설치돼
  있지 않으므로 로봇팔 image dependency, 5축 Servo config와 전체부하 qualification을 명시적
  산출물로 둔다.

추가 완료 기준:

- `DRIVE↔ARM` 전환 중 nonzero chassis/arm 명령 중첩이 0이고, client가 모드를 먼저 표시해도 Jetson
  ACK 전에는 입력 의미가 바뀌지 않음.
- ARM deadman·remote input·D435i feedback 중 하나를 끊으면 다음 arm control tick에 jog가 0이 되고,
  stale 입력이 재연결 뒤 재생되지 않음.
- FSM trajectory와 remote jog가 동시에 hardware bridge에 도달하지 않으며, ARM 종료 뒤 fresh
  `STOWED_LOCKED` 전에는 현재 ARM mapping의 입력이 차체 주행으로 재해석되지 않음.

### WP5.3. 관측성·진단·qualification 기반

자율 기능을 더하기 전에 실패 원인과 복구 근거를 운영자가 한 화면과 한 로그에서 확인할 수 있게
한다. 이 작업은 제어 authority가 아니며 각 production node가 만든 작은 immutable snapshot과
이벤트를 집계한다.

WP5.3은 WP5.2 Task 1·3·4·5가 완료된 뒤 순차 실행한다. 팔 계약·interlock·mission supervisor·
command authority·CAN lock은 WP5.2가 단독 소유하고 WP5.3은 이를 수정하지 않는 관측 adapter다.

- append-only mission journal은 segment/FSM 전이, command owner, motion hold/E-stop source,
  `mission_id`와 팔 ACK, `FAILED/GRIP_LOST/STOW_REQUEST/skip`, 자율↔원격 handover, operator
  override, terrain confidence와 reject reason을 monotonic sequence와 wall/ROS stamp를 함께 JSONL로
  남긴다. 미인식 팔 status는 `CONTRACT_VIOLATION`으로 남긴다. run마다 새 파일을 만들고 크기
  제한·flush 정책·비정상 종료 복구를 시험한다.
- CAN health matrix는 AK 1~4와 ODrive 11~16 각각의 last-seen age, heartbeat/feedback rate,
  axis error·fault·stale, CAN error-passive/bus-off delta, owner PID/process, hold/E-stop source와
  recovery count를 표시한다. 단일 `flock` owner가 제공한 telemetry만 권위 있게 표시한다.
- 독립 채널 진단은 ROS/DDS heartbeat와 RTT, L515·D435i별 SRT submit/sent/drop·receiver feedback,
  D435i metadata age/drop, remote input freshness, command owner, arm heartbeat, CAN telemetry,
  두 camera owner 상태를 한 health snapshot에 합치되 하나의 장애를 다른 채널 장애로 뭉뚱그리지 않는다.
- L515 commissioning 도구는 장착 pitch 20°/25°/30°, ROI, 근거리 사각지대, footprint erosion,
  wheel clearance, depth valid ratio, below-ground 분리, 팔/물자 가림, TF와 known-target XYZ 오차를
  같은 기록 형식으로 비교한다. 최종 선택은 production YAML에 동결하고 대회 run 중 온라인 튜닝을
  금지한다.

완료 기준:

- 강제 종료 뒤 JSONL 마지막 완전 레코드까지 파싱 가능하고 chassis 50 Hz 지표 저하가 없음.
- CAN 10개 노드의 정상·stale·fault·복구를 서로 다른 행과 source로 식별함.
- ROS, L515/D435i SRT, D435i metadata, remote, arm, CAN과 두 camera owner를 하나씩
  kill/restart해 해당 채널만 정확히 열화되고 고아 프로세스·중복 owner 없이 복구됨.
- 운영자가 TUI만 보고 현재 command owner, 정지 원인, 수동 해제 필요 여부와 다음 복구 절차를
  구분할 수 있음.

### WP6-S. Production-parity 시뮬레이션 기반

시뮬레이터 선택의 최우선 기준은 실제 로봇에 배포할 SW를 수정 없이 실행할 수 있는가다.
MuJoCo를 production SW의 자동·폐루프 검증 authority로 사용하고 Isaac Sim은 고충실도
RGB/depth perception challenge set 생성기로 제한한다. 실물 HIL이 최종 authority다.

#### 공통 시뮬레이터 계약

시뮬레이터 adapter는 실제 하드웨어와 같은 ROS 토픽·frame_id·단위·stamp·freshness를 제공한다.

- 발행: L515 color/depth/CameraInfo/gyro/accel. fast mode의 bridge는 `/wheel_states`도 발행하고,
  vcan full-stack mode에서는 실제 `chassis_node`가 `/wheel_states`를 발행. mock
  `/detected_objects`와 `/arm_status`는 외부 ROS2 Humble mock node가 담당.
- 구독: `/autonomy/cmd_vel`의 선택된 최종 source intent, `/chassis_mode`, `/arrival_status`.
- 추가 ground truth는 `/sim/*` namespace에만 발행하고 production 노드는 이를 구독하지 않음.
- terrain, odometry, controller, authority, supervisor, tracking 코드는 실차와 동일 package와
  container image를 사용.
- 실제 launch와 simulation launch의 차이는 hardware adapter와 파라미터 파일로 제한.

시뮬레이터 공통 frame은 RGB, depth mm, CameraInfo, gyro, accel, wheel states, ground-truth pose,
ground-truth track edge와 stamp를 포함한다. simulator-neutral `scenario.yaml`이 SI 단위, frame,
PRNG algorithm/seed, track·sensor·fault parameters와 expected metric을 단독 소유한다. 같은 scenario에서
MuJoCo와 Isaac Sim adapter가 동일한 기하·센서 계약을 생성해야 한다.

검증 범위는 일정에 따라 계층화한다. P0 필수는 분석 fixture, recorded replay, MuJoCo fast mode다.
P1은 hidden seed 폐루프다. P2/stretch는 vcan 10모터 full-stack과 Isaac adapter다. P2 미완료가 P0/P1
통과 기능의 실차 저속 HIL을 무기한 막지 않지만, 미검증 범위를 production claim으로 올리지 않는다.

#### MuJoCo fast autonomy mode

MuJoCo가 `chassis_node`가 선택한 command를 bridge contract로 받아 articulation을 움직이고
`/wheel_states`를 생성한다. L515 입력은
MuJoCo bridge가 발행하고 로봇팔 입력은 외부 ROS2 Humble mock node가 발행한다. 이 모드는 CAN을
우회하지만 production autonomy 전체를 그대로 실행하며 수백 개 hidden seed를 빠르게 평가한다.

- procedural elevated track: 3D 중심선, 폭, 높이, 뱅크, 곡률, 마찰, 낙하 경계.
- seed를 dev, regression, hidden evaluation, stress로 분리.
- 평탄·뱅크·뱅크 전환·아래 바닥·센서 dropout을 자동 생성.
- completion, wheel clearance, edge crossing, false hold, recovery, runtime을 기록.

#### MuJoCo vcan full-stack mode

실제 `chassis_node`, `ChassisManager`, AK/ODrive CAN protocol과 50 Hz 루프까지 검증할 때 사용한다.

```text
production chassis_node
    ↕ Linux vcan0
10-motor CAN emulator
    ↕ joint target/feedback
MuJoCo AK steering ×4 + ODrive wheel ×6 plant
```

CAN emulator는 AK 명령/상태, ODrive 명령/heartbeat/encoder, node timeout, fault, 지연과 frame
drop을 재현한다. 이 모드에서 실제 CAN frame codec, motor health, `/wheel_states`, command watchdog,
motion hold와 E-stop 경계를 검증한다. production 모터 코드를 simulator API 호출로 바꾸지 않는다.

#### Isaac Sim perception challenge mode

Isaac Sim은 로봇 제어 authority나 별도 production 구현이 아니다. RTX 워크스테이션에서 RGB/depth,
조명, 재질, 그림자, 반사, 가림막, 선도 로봇, 신호등·마커·마네킹 장면을 다양화해 같은 센서
계약의 fixture를 만든다. terrain estimator와 로봇팔 인식은 Isaac Sim 밖의 production ROS2 Humble
프로세스로 실행한다.

Isaac Sim 내부 Python 3.12에 production Humble Python 3.10 package를 복제하지 않는다. Isaac
bridge는 표준 sensor message만 외부로 보내고, `robot_arm_msgs` 생성과 소비는 외부 production 또는
mock container가 담당한다. Isaac Sim 버전과 USD asset은 팀원이 검증한 조합으로 고정하고 대회 전
API upgrade를 금지한다.

#### 시뮬레이션 완료 기준

- production autonomy source에 simulator 이름 분기 0개.
- 같은 recorded input에서 실차 launch와 simulation launch의 알고리즘 출력 일치.
- MuJoCo fast mode hidden seed에서 낙하 방향 wheel footprint 진입 0회와 fail-open 0회.
- MuJoCo vcan mode에서 실제 50 Hz chassis loop, 10모터 상태, watchdog/fault 전이 검증.
- Isaac Sim fixture가 production ROS topic adapter를 통해 terrain/인식 node에 입력됨.
- 시뮬레이터 통과를 실물 성능으로 과장하지 않고 실제 L515·차체 HIL 차이를 보고서에 기록.

### WP6-A. Wheel+IMU 상태 추정

목적은 절대 위치 탐색이 아니라 수 초 범위의 연속적인 거리·방향·자세 추정이다.

- `/wheel_states`로 병진 속도와 이동거리 계산.
- L515 gyro-z 적분과 정지 중 bias 추정으로 yaw 계산.
- accel 저역통과로 roll/pitch와 기울기 추정.
- 바퀴 yaw는 IMU 이상 시 보조 입력으로만 사용.
- wheel 명령/측정, IMU, 추정 속도의 불일치로 슬립과 stuck 후보 산출.
- 같은 측의 wheel command/measurement 편차, 좌우 차동과 IMU yaw의 불일치, 한 바퀴만
  회전·정지하는 상태, 명령 대비 encoder 응답비를 virtual-shaft 진단으로 산출한다. 초기 단계는
  terrain profile별 경고와 속도 상한만 적용하고, wheel별 토크 재분배는 별도 HIL을 통과한 이후의
  실험 기능으로 남긴다.
- `/odom`, `/chassis/tilt`, 진단 상태 발행.
- 고정 거리 `odom_m`을 미션 도착의 단독 조건으로 사용하지 않음.

완료 기준:

- 합성 직진·회전·정지·bias·stamp 역행 단위시험.
- 입력 단절 시 stale 전이와 재연결 초기화 시험.
- 평탄면 5 m 직진 오차 ±5% 이내와 제자리 90° 회전 실측.
- 험지에서는 정확도보다 슬립/stuck 검출률을 별도로 기록.
- ODrive 한 축 stale·저응답, 한 바퀴 공회전, IMU yaw 불일치 fault injection에서 해당 wheel과
  원인을 식별하고 검증되지 않은 자동 토크 보상을 하지 않음.

### WP6-B. Bank-aware terrain path estimator

기존 `lane_follower` 개념을 차선이 없고 지면에서 떠 있으며 뱅크가 적용된 펌프트랙용 terrain
perception으로 바꾼다. L515는 전방 장애물의 유무보다 "앞의 어느 3D 표면에 바퀴를 올릴 수
있는가"를 계산하는 센서로 사용한다.

#### 장착과 관측 범위

- L515 렌즈 중심 약 0.60 m, 고정 하향 pitch 25°를 초기 후보로 사용.
- 최종 장착 전 20°·25°·30°를 비교해 근거리 사각지대, 0.5~4 m track coverage,
  선도 로봇 검출을 함께 평가.
- depth FOV 약 70°×55°, RGB FOV 약 69°×42°를 기준으로 차체·브래킷이 광로와 냉각구를
  가리지 않게 배치.
- 장착 비교는 팔 완전 접힘, 작업 준비, 구호물자 운반, 비정상 정지 자세에서 각각 수행한다.
  팔·그리퍼·구호물자가 하향 terrain ROI나 선도 로봇 시야를 가릴 때 confidence 저하와 motion
  hold가 fail-open 없이 동작하는지 기록한다.
- 최종 `base_link→l515_link`는 실측 전 임시값으로 production 완료 판정에 사용하지 않음.

#### 내부 3D 처리

상시 PointCloud2 발행은 depth image 대비 메모리 복사와 DDS 직렬화가 크므로 금지한다. 대신
terrain 프로세스가 depth와 intrinsics로 고정 ROI의 XYZ point cloud를 내부 생성한다.

```text
raw depth + CameraInfo
    → 고정 ROI/stride
    → ROI별 valid ratio + robust median/MAD/percentile reject
    → 내부 XYZ point cloud
    → L515 extrinsic + WP6 roll/pitch로 중력 정렬
    → 5 cm 후보 해상도의 2.5D elevation grid
    → 높이·법선·경사·거칠기·관측 신뢰도
    → 뱅크/오르막/낙하 경계/장애물 분류
    → footprint-safe 중심경로
```

- 넓고 연속적인 좌우 경사는 뱅크로 수락하고 장애물로 오인하지 않음.
- 짧은 거리의 큰 음의 높이 변화, 표면 종료 뒤 아래 바닥, 지지 표면과 연결되지 않은 영역은
  낙하 경계로 분류.
- 단일 전역 평면이 아니라 작은 grid의 국소 표면과 연결성을 사용해 뱅크 진입부와 펌프트랙
  굴곡을 보존.
- 단일 center pixel depth를 거리·높이 근거로 쓰지 않는다. ROI별 valid depth 비율, robust median,
  MAD 또는 percentile 기반 outlier 제거, 프레임 간 temporal consistency, 표면 연결성과 local normal
  일관성을 함께 계산한다. confidence는 관측 개수만 아니라 이 품질 지표와 reject reason을 포함한다.
- Gateway의 고비용 RGB-depth alignment를 다시 켜지 않고 terrain 프로세스의 제한된 depth ROI에서만
  품질 필터와 3D 복원을 수행한다.
- 좌우 낙하 경계 사이를 wheel footprint와 불확실성만큼 erosion한 뒤 중심선과 heading을 계산.
- 카메라 아래 사각지대는 wheel+IMU odometry로 최근 1~2초의 bounded local grid만 이동·누적해
  보완. 장기 지도, loop closure, 전역 SLAM으로 확장하지 않음.
- RGB는 바닥·경계 후보와 진행 방향을 보조하고 depth와 독립 confidence를 가짐.
- 출력은 path offset, heading error, 좌우 wheel clearance, bank angle, longitudinal slope,
  roughness, confidence, stamp.

#### JAX 계산 backend

첫 production authority는 NumPy다. JAX GPU는 NumPy와 fixture 동등성 및 Jetson 전체부하 gate를
모두 통과한 뒤에만 별도 production profile 후보가 된다. 두 backend는 같은 고정 shape 입력과 같은
결과 계약을 사용한다. terrain와 controller는 `powertrain_autonomy`의 한 프로세스 안에서 immutable
dataclass로 연결하고 외부에는 `/autonomy/cmd_vel`만 발행한다.

- JAX 대상: depth deprojection, 좌표 변환, mask, elevation scatter, 표면 특징 계산.
- ROI, stride, grid shape를 고정하고 invalid point는 shape 변경 대신 mask로 처리.
- 시작 시 dummy frame으로 JIT warm-up을 끝내고 warm-up 전 자율 arm을 금지.
- 주행 중 새로운 shape나 dtype으로 재컴파일하지 않음.
- accelerator 결과의 shape, stamp, finite value, range를 CPU 경계에서 검증하고 NaN, device error,
  timeout은 결과 폐기와 motion hold로 변환.
- `XLA_PYTHON_CLIENT_PREALLOCATE=false`를 기본 후보로 검증해 로봇팔 YOLO와 8 GB RAM을 보호.
- JAX/jaxlib/CUDA 조합은 JetPack R36.5 aarch64 컨테이너에서 qualification을 통과한 정확한 버전으로
  함께 pin하고 `jax.devices()`가 의도한 GPU를 반환하는지 preflight에서 확인.
- backend는 preflight에서 configuration으로 한 번 선택하고 주행 중 자동 전환하지 않음. JAX가
  시작 또는 실행 중 실패하면 terrain freshness 상실로 motion hold하고 supervised restart.
  NumPy는 전체부하 승인을 별도로 받은 launch profile일 때만 다음 arm 전에 선택할 수 있으며
  JAX 장애 직후 같은 run에서 자동 fallback하지 않음.

JAX 채택은 kernel 단독 속도가 아니라 전체 시스템 영향으로 결정한다. L515 Gateway, software
x264 두 sender, 전체 ROS/CAN/SRT, 로봇팔 YOLO를 동시에 실행해 terrain p99 30 ms 이하,
depth 10 Hz deadline 준수, L515·D435i RGB SRT receiver 각각 29 fps 이상, 동일 장면 YOLO rate
기준선 대비 지속 저하 5% 이하,
OOM 0을 모두 확인한다. NumPy도 동일한 전체부하 gate를 독립 통과해야 fallback profile 자격을
얻는다. 둘 중 하나가 더 빠르더라도 chassis 50 Hz와 시스템 메모리를 더 침해하면 채택하지 않는다.

완료 기준:

- NumPy와 JAX가 동일 fixture에서 허용오차 안의 elevation/path 결과를 생성.
- 녹화 depth에서 평탄면, 일정 뱅크, 뱅크 전환, 양쪽 낙하 경계, 트랙 아래 바닥을 구분.
- 직선·곡선·부분 가림·낮은 대비에서 confidence가 기대 방향으로 변화.
- depth hole, 비반사·반사 표면, 단일 spike, 아래 바닥 혼입, 연속 프레임 점프에서 robust filter가
  잘못된 주행가능 표면을 만들지 않고 reject reason을 기록.
- footprint erosion 결과가 없거나 입력이 stale하면 속도 명령을 만들지 않음.
- 20°·25°·30° 장착 비교 HIL로 production pitch와 실제 ROI를 기록. 각도를 반복 재현할 수 있는
  브래킷·기준면 fixture는 기구팀 인계 입력이며 SW가 임의 제작하거나 각도 정본을 추정하지 않는다.

### WP6-C. Autonomy controller와 command authority

- terrain path offset/heading을 4WS 속도·yaw-rate 명령으로 변환.
- 곡률, 좌우 wheel clearance, bank angle, slope, confidence, tilt, slip에 따라 속도 제한.
- 자율, 원격, mission hold의 우선순위를 하나의 상태머신으로 집행.
- 모드 전환은 먼저 0속도를 확인한 뒤 새 작성자에게 권한을 넘김.
- `EMPTY_STOWED`와 `CARRYING_LOCKED` 주행 profile을 분리한다. 운반 profile은 최고속도,
  가감속, 조향 slew와 허용 bank angle을 더 보수적으로 제한한다.
- `EMPTY_STOWED`는 fresh한 `STOWED_LOCKED`, `CARRYING_LOCKED` profile은 fresh한 동명 상태가
  없으면 0이 아닌 명령을 차단한다.
- terrain·odom·일반 명령 freshness 상실의 controlled hold는 지형 profile별 감속 한계를 따른다.
  반면 팔이 작업·미잠금 상태이거나 `GRIP_LOST`인 협업 interlock은 다음 50 Hz tick에서 drive를
  즉시 0으로 gate한다. 빙판·사구의 일반 hold는 제한 감속 후 0으로 수렴하고 latched E-stop은 별도
  최대 제동 정책을 사용한다. 실제 제동거리 HIL 전에는 더 낮은 속도 profile을 쓴다.
- 자율 프로세스 사망, stale terrain path, stale odom은 자동복구 motion hold.
- US-100 near/no-response와 모터 fault의 latched E-stop 정책은 변경하지 않음.

완료 기준:

- `chassis_node` 밖에서 `ChassisManager.set()`을 호출하는 production process가 없음.
- `/autonomy/cmd_vel`과 `/teleop/cmd_vel` publisher 수는 진단으로 기록하되 순간 graph 값만으로
  주행을 허가하지 않고, 외부 `/cmd_vel`은 구독하지 않음.
- 자율↔원격 전환 중 비영점 명령이 이어지지 않음.
- 모든 입력 상실 조합에 대해 hold/E-stop 구분 자동시험.

## 6. 구간별 득점 기능

### WP7. 5구간 선도 로봇 추종

기존 후순위 WP9를 공통 기반 직후로 앞당긴다.

- `/detected_objects`의 선도 로봇 3D 위치로 거리와 좌우 오차 계산.
- `DetectedObjectArray.header.frame_id`와 stamp에서 `base_link` TF 변환을 먼저 수행하고,
  frame이 비거나 TF가 stale이면 target 명령을 만들지 않음.
- 목표 간격 2.0 m, 허용범위 1.5~2.5 m를 설정값으로 소유.
- 검출 중에는 거리 PID와 좌우 P/PD 제어.
- 가림 시작 시 마지막 상대속도와 wheel+IMU로 짧게 예측하되 감속.
- 예측 한계를 넘으면 정지하고 재검출을 기다림.
- 재검출은 위치·크기·class·시간 연속성 gate를 통과해야 같은 대상으로 수락.
- 1.5 m 이하 접근 또는 대상 불확실 시 전진 금지.

완료 기준:

- 가짜 target stream으로 간격·가림·오검출·재획득 단위시험.
- 사람/박스 대역 HIL 뒤 실제 UGV 또는 4족 로봇으로 검증.
- 2.0 m ±0.5 m 유지율, 최소거리, 재획득시간, 접촉 0회를 기록.

### WP8. 이벤트 기반 구간 supervisor

하나의 거대한 코스 시퀀서 대신 5개 독립 구간 profile과 공통 상태머신을 사용한다.

공통 상태는 `READY → DRIVE → STOP_REQUESTED → EVENT_HOLD → ARM_WORK → STOW_VERIFY →
RESUME → COMPLETE`로 두고, 각 전이는 인식 이벤트, 실제 wheel 정지, 팔 상태, 운영자 명령,
timeout으로 결정한다. odom 거리는 보조 gate와 진단에만 사용한다.

- 1구간: 스모그 진입/이탈, 로봇팔의 피아식별·LED 완료 결과 2건, 완주. 피아식별과
  LED 구동은 로봇팔 계층이 소유하고 파워트레인은 주행 hold/resume만 담당.
- 2구간: 구호물자 작업 정차, 팔 완료, 빨간불 hold, 초록불 resume, 완주.
- 3구간: 서로 다른 marker 5개 집계, 중복 억제, 실패/성공 기록, 완주. stable instance id가
  없으면 `class_name + base_link 3D 위치 cluster + 최소 재관측 시간 + confidence`를 dedup key로
  사용한다. 가능하면 로봇팔 팀이 `class_id`에 마커 고유 ID를 채우는 계약을 우선한다.
- 4구간: 빙판/제설 mode, stuck recovery 요청, 완주.
- 5구간: follow mode, 가림, 재획득, 완주.

먼저 authority 최종 출력을 0으로 만들고, fresh·단조 header, 정확히 6개 unique wheel, finite 속도,
`healthy=true`, stale=false, fault=0을 만족한 sample에서만 wheel별 정지 dwell을 누적한다. threshold는
전진·후진·피벗 뒤 실측 noise에 대해 `max(측정 상한×1.5, encoder resolution floor)`로 정한다. 한 조건이라도
깨지면 dwell을 0으로 리셋한다. 모두 연속 충족된 뒤에만 `MISSION_STOP`과 `ArrivalStatus`를 발행한다.
threshold map이 unqualified·누락이면 작업 허가를 fail-closed한다. `median-of-6`은 단일 회전 wheel을
숨기므로 사용하지 않는다. DDS 토픽 간 도착 순서는 가정하지
않으며 팔은 fresh `MISSION_STOP`과 동일 `mission_id`의 arrival을 모두 받은 뒤에만
`WORK_READY`를 발행한다. 같은 `mission_id`의 `WORK_READY` 뒤에만 팔
작업을 수락한다. `DONE`은 진단으로만 기록하고, 픽업은 동일 ID의 `CARRYING_LOCKED`, 하역은
동일 ID의 `STOWED_LOCKED`까지 확인한 뒤 재출발한다. `DONE` 단독으로는 재출발하지 않는다. timeout은 자동
재출발이 아니라 motion hold와 운영자 통보로 끝낸다.

구호물자 운반 중에는 fresh한 `CARRYING_LOCKED`를 요구한다. stale하거나 `GRIP_LOST/FAILED`가
오면 제어된 motion hold로 전이한다. 복구 시에도 임의로 이전
명령을 재개하지 않고 supervisor가 상태를 다시 확인한다.

`FAILED/ABORT` 또는 운영자 skip은 `STOW_REQUEST → STOWING → STOWED_LOCKED`를 거친 뒤에만
종결한다. `GRIP_LOST`는 supervisor latch로 남겨 bounded regrasp 성공 또는 운영자 승인 전까지
새 잠금 heartbeat가 와도 재출발하지 않는다.

### WP9. 환경 degradation 정책

- 스모그: RGB/depth 유효도 저하를 측정하고 wheel+IMU 기반 저속 통과 또는 hold.
- 사구·자갈·수중: slip/tilt 기반 속도·가감속 제한, stuck 판정.
- 빙판·WD-40: 조향 변화율과 가속 제한, 헛돌기 시 정지 후 제한된 recovery.
- 센서가 회복되면 즉시 최고속으로 복귀하지 않고 안정 프레임 수를 만족한 뒤 단계 복구.

자동 recovery는 정해진 횟수와 거리·시간 한도를 가진다. 한도를 넘으면 원격 전환 대기 상태로
들어가며 무한 재시도하지 않는다.

## 7. 원격주행·원격 팔 조종 경로 보존

자율 기능이 원격 경로를 약화시키면 안 된다.

- L515 RGB 1280×720×30과 D435i RGB 848×480×30을 동시에 송신한다. 두 채널 모두 노트북
  receiver의 첫 완전한 5초 window에서 실측 29 fps 이상이어야 하며, sender의 caps나 submit rate만으로
  30 fps 달성을 주장하지 않는다.
- L515 SRT는 `:5000`, D435i raw SRT는 `:5002`, D435i YOLO metadata UDP는 `:5003`으로 고정해
  기존 L515와 legacy 좌표 포트 `:5001`에 충돌하지 않게 한다. 노트북은 두 영상을 동시에 표시하고
  D435i 창에만 최신 metadata를 합성한다.
- D435i camera owner의 capture→SRT 경로는 capture→YOLO 경로와 bounded latest-frame fan-out으로
  분리한다. YOLO가 느리거나 실패하면 밀린 frame을 쌓지 않고 최신 frame으로 건너뛰며, raw SRT
  30 fps는 계속 유지한다. `/perception/debug_image`는 BENCH 디버그 출력이지 production sender 입력이 아니다.
- 영상 profile은 운용 중 새 encoder graph를 조립하지 않고 사전 HIL한 정적 구성을 전환한다.
  `NORMAL`은 두 raw RGB stream과 선택된 L515 depth/overlay를 제공한다. `CONGESTED`는 두 raw
  stream의 해상도와 30 fps를 보존하면서 각 bitrate를 qualification된 단계로 낮춘다.
  `EMERGENCY_REMOTE`도 두 raw RGB stream을 남기고 L515 depth/overlay SRT submit을 먼저 중단한다.
  D435i metadata는 작은 best-effort packet이므로 계속 보내되 영상·제어를 block하지 않는다.
  진입·복귀에는 채널별 RTT·loss·send/drop 지표와 hysteresis, 최소 유지시간을 사용한다.
- sender의 submit/sent/drop은 열화 힌트일 뿐 수신 성공의 권위가 아니다. 노트북은 L515와 D435i
  각각의 decode/display fps, frame age, sequence gap, RTT/loss heartbeat를 독립 보고한다. L515
  feedback stale은 원격주행을 motion hold하고, D435i raw feedback stale은 원격 팔 명령을 hold하며
  차체는 `MISSION_STOP`을 유지한다. 비권위 companion stream 장애를 다른 subsystem의 E-stop으로
  잘못 승격하지 않지만, dual-stream readiness와 전체부하 HIL은 실패로 기록한다.
- D435i metadata 유실·stale은 raw 영상이나 팔 deadman을 대신하지 않는다. 노트북은 age 제한을 넘긴
  bbox를 숨기고 `OVERLAY_STALE`을 표시하며 수동 팔 조종은 fresh raw video와 deadman으로 판단한다.
- profile 전환 acceptance는 전환 순간 fps 하나로 판정하지 않는다. orphan/encoder overlap 0,
  채널별 최대 blackout, 첫 IDR 수신시간, 전환 뒤 같은 첫 완전한 5초 window에서 두 receiver 모두
  ≥29 fps를 기록한다.
  blackout 동안 command는 미리 정한 hold 또는 최저속 정책을 따르며 마지막 영상에 의존해 계속 달리지 않는다.
- 최저 qualified bitrate에서도 어느 raw stream이든 receiver 29 fps를 유지하지 못하면 해상도·fps를
  몰래 낮추지 않고 해당 원격 동작을 `REMOTE_VIDEO_UNAVAILABLE`로 막는다. L515가 stale한 원격주행과
  D435i가 stale한 원격 팔 조종은 허용하지 않는다.
- 운영 화면에 영상뿐 아니라 command owner, safety, odom, path confidence, target range,
  segment state, CAN 10-node health matrix, 채널별 freshness와 마지막 mission event를 표시.
- `q`, 클라이언트 단절, 어느 SRT receiver 단절도 camera owner·다른 sender·ROS를 죽이지 않는 계약 유지.
- 자율 실패 시 zero-confirmed handover 후 원격으로 전환.
- 원격 중에도 US-100과 모터 E-stop은 동일하게 적용.
- 네트워크 단절과 조종기 단절은 motion hold로 처리하고 자동 재연결.
- 초기 후보 mapping은 `CREATE+OPTIONS` 1초 hold로 DRIVE/ARM 전환 요청, D-pad로
  `joint_1`~`joint_5` 선택, 우스틱 Y로 저속 조그, R2/L2로 gripper open/close, L1 hold-to-run이다.
  구체적인 물리 버튼·축은 HIL과 운전자 피드백 뒤 versioned config로 변경할 수 있다. 어떤 mapping을
  쓰더라도 Jetson ACK 전 mode 변경 금지와 같은 입력 frame의 DRIVE/ARM 동시 활성 금지는 유지한다.
- 원격 팔 명령은 표준 `JointJog`를 사용하지만 기존 5종 협업 msg에는 필드를 추가하지 않는다.
  자동 FSM과 원격조종은 로봇팔 내부 `ArmCommandAuthority`에서 상호배타이며, direct Dynamixel
  command publisher와 검증되지 않은 `home` 단축키는 production에서 금지한다.
- 팔 노드 사망 시 정상 경로는 motion hold다. active mission 취소, arrival 재발행 중단,
  `GRIP_LOST` latch 없음과 팔 stale을 확인하고 감사 로그가 남는 override를 명시적으로 활성화한
  경우에만 자율 금지·팔 작업 금지·원격 저속 profile로 이동한다. heartbeat 복귀 시 즉시 다시 hold한다.
- 반자동 원격 보조는 운영자의 속도 의도를 대체하지 않는다. terrain centering/heading hold,
  bank·clearance speed cap, lead-distance assist와 작업점 정렬만 제공하며 모든 출력은 같은
  `chassis_node` 내부 `CommandAuthority → final gate` 안전 경로를 통과한다. 기본 OFF이며
  `ASSIST_BYPASS` 한 입력으로
  다음 tick에 raw teleop으로 복귀한다.

## 8. 시험 전략

### 8.1 자동시험

순수 Python 코어를 먼저 만들고 ROS 노드는 변환과 I/O만 담당한다.

- odometry: 단위, bias, integration, stale, reconnect, slip/stuck.
- terrain path: NumPy/JAX 동등성, deprojection, gravity alignment, elevation grid, bank,
  낙하 경계, footprint erosion, confidence, stale.
- tracking: 거리 제어, 가림 예측, 잘못된 대상 거부, 접촉 방지.
- supervisor: 구간별 정상 전이, 중복 이벤트, timeout, mission_id mismatch.
- arm handshake: 실제 정지 전 작업 금지, `ArrivalStatus` 멱등성, 지연된 이전 ID 거부,
  `DONE` 단독 재출발 금지, 두 토픽 순서 역전, 주행허가 heartbeat stale·과거 버스트·stamp 역행,
  `CARRYING_LOCKED/GRIP_LOST`, `FAILED/ABORT/STOW_REQUEST` 전이.
- authority: 동시 작성자 방지, zero-confirmed handover, 프로세스 사망.
- remote input: version/session/sequence, 30 Hz freshness, DRIVE/ARM 상호배타, mode ACK, deadman,
  한 관절 `JointJog`, trigger conflict, stale/reconnect replay 거부.
- arm command authority: FSM cancel ACK, remote Servo 단독권, 5축 limit·collision·singularity 제한,
  stale joint feedback/controller fault hold, remote 종료 뒤 stow-before-drive.
- safety: motion hold와 latched E-stop 경계 회귀시험.
- CAN owner: 실물 잠금 중 두 번째 owner fail-closed, fake·vcan profile 비충돌, 비정상 종료 뒤
  커널 `flock` 자동 해제.
- perception contract: frame 누락, TF stale·실패, extrinsic 변경, marker dedup key.
- observability: journal 순서·비정상 종료 tail, CAN health matrix, 채널별 원인 분리, 로그 writer
  backpressure가 50 Hz loop를 막지 않는지 검증.
- depth quality: valid ratio, robust median, MAD/percentile reject, temporal consistency, surface
  connectivity와 reject reason.

arm 직전 fail-closed preflight는 다음을 확인한다.

- 우리 vendored msg 5종과 Jetson 로봇팔 source msg의 SHA-256 일치.
- `/arm_status`, `/detected_objects`, `/chassis_mode`, `/arrival_status`의 상대 endpoint, type과 QoS.
- 실차 `ROS_DOMAIN_ID=0`, `use_sim_time=false`와 실제 DDS 왕복. RMW 구현은 양 팀 검증 조합으로 고정.
- simulator mock을 포함한 모든 node는 `use_sim_time=true`와 동일 `/clock` 사용.
- `ArmStatus/ChassisMode`는 Reliable/Keep Last 1, `ArrivalStatus`는 Reliable/Keep Last 10,
  `/detected_objects`는 최신 센서 stream이므로 Best Effort/Keep Last 1.
- 신호등·마커·선도 로봇·구호물자·마네킹 class 어휘는 계약 fixture로 고정하고 미지 class는
  이벤트로 승격하지 않고 진단만 기록.

### 8.2 기록 재생

실차 튜닝 전에 RGB/depth/IMU/wheel/detected_objects를 동기 기록한다. 같은 기록을 반복 재생해
알고리즘과 파라미터 변경 전후를 비교한다. 성공 장면만 고르지 않고 가림, blur, stale, 오검출,
슬립 사례를 회귀 fixture로 보존한다. 환경 회귀 세트에는 연막·안개, 그림자·역광, 반사·비반사
표면, 팔·구호물자 가림, depth hole·jump, 선도 로봇 부분가림, marker 중복·오검출, 뱅크 전환과
트랙 아래 바닥 혼입을 포함한다.

### 8.3 시뮬레이션 검증

1. 분석적 fixture로 JAX/NumPy terrain 수학과 경계조건을 검증한다.
2. MuJoCo perception-in-the-loop에서 정답 pose·track edge와 알고리즘 출력을 비교한다.
3. MuJoCo fast mode로 production autonomy 폐루프와 hidden procedural seed를 반복한다.
4. MuJoCo vcan mode로 production chassis/CAN/safety까지 확장한다.
5. Isaac Sim challenge fixture로 RGB/depth 재질·조명·가림 변화에 대한 perception을 검증한다.
6. 실제 L515 기록을 동일 replay adapter에 넣어 sim-to-real 차이를 noise model과 보고서에 반영한다.
7. 같은 환경 열화 fixture를 simulator, recorded replay, 실제 대체 환경 HIL에 공통 ID로 추적해
   어느 단계에서만 통과하는지 구분한다.

시뮬레이션 성공 기준은 예쁜 단일 데모가 아니라 seed 집합의 정량 결과다. 최소 wheel clearance,
track edge overrun, false hold, fail-open, completion, recovery time, estimator runtime을 seed별로
저장한다. dev/regression seed로 튜닝하고 hidden evaluation seed는 완료 판정 때만 실행한다.

### 8.4 Jetson 통합

- `powertrain_ros`, L515 Gateway, 로봇팔 인식, terrain backend 동시 부하에서 CPU/RSS/GPU
  memory, `MemAvailable`, swap I/O, EMC/GR3D 사용률, 온도·클럭과 각 rate 측정.
- NumPy와 JAX 각각 terrain 평균/p99, depth age, CPU buffer→accelerator array
  materialization/copy·동기화 비용, 추가 JIT compile 횟수 측정.
- L515 1280×720×30과 D435i 848×480×30을 동시에 우선 보존하고 L515 depth/overlay와 terrain
  diagnostic cadence는 낮출 수 있다. D435i YOLO가 raw sender를 기다리게 하거나 frame backlog를
  만들 수 없으며, inference rate는 별도 지표로 기록한다.
- 프로세스 강제 종료·카메라 분리·네트워크 단절 뒤 고아 프로세스와 중복 SDK owner 0 확인.
- CAN, L515, D435i 소유권 경계를 침범하지 않는지 확인.
- ROS/DDS, L515 SRT, D435i SRT, D435i metadata, remote input, arm heartbeat, CAN telemetry,
  L515 Gateway와 D435i owner를 하나씩 kill/restart하고
  다른 채널의 health가 거짓 장애로 바뀌지 않는지와 journal의 원인 기록을 확인.
- commissioning에서 확정한 L515 pitch·ROI·TF·quality threshold YAML의 SHA-256을 arm 전 기록하고
  run 도중 변경을 거부.

전체부하 backend 채택 gate는 다음과 같다.

- 30분 연속 실행 중 Linux OOM killer 0, terrain/chassis 비정상 종료 0, sustained swap I/O 0.
- `MemAvailable` 최솟값 1.5 GB 이상. 미달 backend는 kernel 속도와 무관하게 거부.
- shared-DRAM buffer materialization/copy·동기화 p99 5 ms 미만.
- chassis telemetry 60초 3000 samples, complete 5초 window 49.8~50.2 Hz, overrun 0,
  20 ms 대비 tick interval jitter p99 2 ms 이하와 최대 interval 25 ms 이하.
- terrain p99 30 ms 이하, depth deadline 10 Hz, 같은 완전한 5초 window에서 L515와 D435i SRT
  receiver 각각 29 fps 이상.
- 동일 장면 로봇팔 YOLO rate 저하 5% 이하.
- JAX compile은 arm 전 warm-up 1회만 허용하고 arm 뒤 추가 compile 0회.
- terrain process kill, CUDA device error와 terrain cgroup memory-limit 초과를 주입해 terrain만
  종료되고 chassis/safety 50 Hz가 유지되며 motion hold로 전이하는지 확인.

GPU 사용 여부는 통합 메모리 OOM이 CPU safety까지 전파되는 위험, software x264와 NumPy
fallback의 CPU 경합, 10 Hz depth에서 accelerator dispatch가 실익보다 클 가능성을 실제 계측으로
판정한다. 측정 전 JAX가 NumPy보다 우월하다고 가정하지 않는다. 2026-07-13 `agy`의 Gemini 3.1 Pro
High 재검토는 현행 코드의 arm gate·CAN lock·authority·wheel-stop 미구현을 S0/S1로 판정했다. 이는
계획 구조의 폐기가 아니라 WP5.2를 구현 선행조건으로 유지해야 한다는 근거로 반영했다.

2026-07-13 Claude Code `claude-opus-4-8` 교차검토와 로컬·Jetson 재대조에서 현행
`DRIVING → 팔 언락`, 팔 상태 gate 부재, CAN owner lock 부재, heartbeat/freshness 계약 미구현을
확인했다. 검토의 대규모 재설계 제안은 채택하지 않고 기존 5종 wire schema를 유지한 계약 v2,
default-deny gate, `STOW_REQUEST`, stamp 기반 Keep Last 1 heartbeat, supervisor-latched
`GRIP_LOST`, 제한된 operator override와 acceptance test로 반영했다.

같은 날 safety-contract, execution-feasibility, autonomy-scope 독립 리뷰와 Gemini 3.1 Pro High
재검토를 추가로 수행했다. 최종 command authority를 `chassis_node` 내부로 이동, physical locked
predicate와 독립 override proof, 모든 real-CAN opener lock, 완전한 wheel-stop sample 조건,
receiver feedback 기반 영상 판정, Compose 4-service 분리, NumPy-first/JAX qualification,
P0/P1/P2 simulation 순서와 WP5.3↔WP6 의존성 분리를 본 정본에 반영했다.

### 8.5 HIL 순서

1. 구 로봇팔+새 chassis 혼합 버전에서 nonzero 명령 차단과 팔 언락 미발생.
2. mock heartbeat 단절·과거 버스트·역행 stamp·미인식 status의 fail-closed 전이.
3. `powertrain_jetson` 첫 owner 실행 중 `powertrain_ros`의 두 번째 lock이 같은 host inode에서 실패.
4. ODrive 11~16 전진·후진·좌/우 pivot 후 10회 정지 노이즈 측정, HALL 보강 gate와 YAML 동결.
5. 바퀴 부양 상태에서 command authority, 반자동 원격 보조와 장애 전이.
6. qualified per-wheel 정지 조건 충족 전 work permit 미발행, DDS 순서 역전에도 팔 작업 미시작.
7. 로봇팔 mock으로 ACK/stale/중복/재시작/지연 ID/DONE 단독/FAILED/GRIP_LOST/override fault injection.
8. 실제 로봇팔과 `wheel 정지 → MISSION_STOP+ARRIVED_* → work accepted → 작업·자동 stow →
   CARRYING_LOCKED 또는 STOWED_LOCKED → 새 명령 resume` 1사이클.
9. 팔 노드 사망 후 정상 hold와 운영자 override의 원격 저속·자율 금지·명시적 해제.
10. mission journal tail 복구, CAN 10-node health matrix와 채널별 kill/restart 진단.
11. 평탄 저속에서 odometry, virtual-shaft 진단과 NumPy terrain 기준 구현.
12. L515 20°·25°·30° 장착·TF·known-target 비교와 robust depth 품질·낙하 경계 검증.
13. 전체 Jetson 부하에서 JAX/NumPy backend production 선택.
14. 모의 표적 추종, frame/TF 실패와 가림막 재획득.
15. 네트워크 profile별 L515·D435i SRT 열화·복구, D435i metadata stale와 profile hysteresis 시험.
16. 스모그·모래·자갈·수중·빙판 조건별 degradation·motion-hold 제동 시험.
17. 5개 구간 자율/반자동 원격/수동 원격 반복 리허설과 점수표 기록.

## 9. 일정과 개발 순서

작업 의존 순서는 날짜보다 우선한다.

1. WP5.2 Task 1 계약 코어와 override fail-closed 규칙.
2. Task 2 chassis gate와 compatibility-lock mode.
3. Task 3 cross-container CAN runtime lock.
4. Task 4 원격 입력 공통 authority, DRIVE/ARM 단일 gateway와 qualified wheel-stop predicate.
5. Task 5 mission supervisor·override abort·멱등 arrival.
6. Task 6 perception frame 계약.
7. 로봇팔 팀의 default-deny mode, `FAILED`, `STOW_REQUEST`, 10 Hz heartbeat,
   `CARRYING_LOCKED` drop 수락과 멱등 arrival 동시 컷오버.
8. Task 7 mock 계약시험과 실제 DDS 합동 1사이클.
9. WP5.3 Task 1~3과 5의 journal·health·CAN/arm adapter 코어.
10. WP6-S P0 fixture/replay/MuJoCo fast mode와 WP6-A odometry.
11. WP5.3 Task 4 depth/time/TF qualification 뒤 WP6-B NumPy terrain 기준.
12. WP6-C `chassis_node` 내부 authority와 autonomy controller.
13. WP5.3 Task 6 dual-video operator console·remote profile/assist 뒤 WP5.2 Task 7 원격 팔 합동 HIL,
    이어서 WP5.3 Task 7 regression과 Task 8 최종 HIL.
14. JAX qualification profile → WP7 → WP8 → WP9. P2 vcan/Isaac은 stretch로 병행.

1~5가 끝나기 전 실물 합동 arm은 금지하지만, 독립 fake·simulation과 WP6 순수 코어 개발은
병행할 수 있다. 파워트레인 단독 실차 주행 HIL은 §5의 `arm_gate_mode=arm_absent_field` profile
요건(팔 노드 부재 + 운영자 기계적 접힘 확인 + journal 기록)을 만족하는 경우에만 병행할 수 있다.

### 7월 12~19일: 서류와 설계 고정

- 본 계획 승인과 2026 규정 기준 문서 교정.
- SW 아키텍처, 자체 개발 이력, HIL 증거를 제출자료에 반영.
- WP5.2 계약 v2, fail-closed gate, 원격 authority 통합과 CAN lock을 최우선 구현.
- WP5.3 mission journal·CAN matrix·독립 채널 진단의 순수 코어와 TUI 계약 구현.
- WP6-S 공통 simulator contract와 production/hardware 경계 고정.
- WP6-A odometry 설계·순수 코어·합성시험은 실물 합동 arm 없이 병행.
- 로봇팔 팀과 2026 구간 event 이름, `mission_id` 재시작 규칙, `MISSION_STOP`, `WORK_READY`,
  `STOW_REQUEST`, `DONE`, `STOWED_LOCKED/CARRYING_LOCKED`, `GRIP_LOST`, frame/TF, QoS와
  heartbeat timeout, `ROS_DOMAIN_ID=0`, RMW와 `use_sim_time` 계약 확정.

### 7월 20~31일: 공통 자율 기반

- WP6-A wheel+IMU 완료.
- wheel mismatch/virtual-shaft monitoring. 반자동 원격 assist는 WP6-B/C 뒤에 연결.
- WP6-S MuJoCo procedural elevated track, sensor bridge, fast autonomy mode.
- WP6-B NumPy terrain 기준 구현과 JAX 고정 shape kernel.
- L515 장착각·ROI·시간/TF qualification, robust depth 품질, 뱅크와 낙하 경계 녹화 재생 baseline.
- WP6-C command authority와 자율↔원격 handover.
- 실내 평탄 저속 통합시험.

### 8월 1~9일: 고배점 기능

- WP7 선도 로봇 추종과 가림 후 재획득.
- WP8 2·3·5구간 event supervisor.
- MuJoCo vcan 10모터 emulator와 full-stack mode.
- Isaac Sim perception challenge fixture adapter.
- 로봇팔 mock 및 실제 그래프 합동 1사이클.

### 8월 10~16일: 환경 대응

- WP8 1·4구간 profile.
- WP9 스모그·험지 degradation 정책.
- 실제 또는 최대한 가까운 대체 환경 HIL.

### 8월 17~23일: 전 구간 통합

- 자율/원격 전환, 재시작, 부분점수 전략 검증.
- Jetson 전체 부하와 장시간 안정성.
- 구간별 파라미터를 코드에서 YAML로 동결.

### 8월 24~30일: 반복 리허설

- 5개 구간 각각 자율 3회 이상, 원격 3회 이상.
- 성공률, 점수 예상, 이탈, hold, E-stop, 복구시간 기록.
- 실패율이 높은 기능은 단순화하고 새 기능 추가를 중단.

### 8월 31일~9월 4일: 출전 동결

- release tag와 Jetson exact-HEAD 고정.
- 컨테이너 image, YAML, 운영 절차, 복구 명령 백업.
- 대회 모드에서는 검증되지 않은 자동 recovery와 디버그 기능 비활성.

### 9월 5~6일: 본선

2026-06 규정 PDF 1쪽의 `9. 5(토) ~ 9. 6(일)`을 근거로 한다. 기존 문서의 9월 13일 표기는
이전 일정이므로 본 계획에서 사용하지 않는다.

- 구간별 preflight 후 자율/원격 수행.
- 실패 시 부분점수와 남은 시간을 우선해 원격 전환 또는 다음 시도 결정.

## 10. 완료 정의와 중단 기준

기능 완료는 코드 merge가 아니라 다음 다섯 조건을 모두 만족해야 한다.

1. 순수 코어 자동시험과 기존 회귀시험 통과.
2. 해당 production node를 수정하지 않은 MuJoCo hidden-seed simulation 통과.
3. Jetson exact-HEAD 배포와 전체 프로세스 동시 실행.
4. 해당 구간의 정상·센서 단절·프로세스 사망 HIL.
5. 운영자가 로그 없이도 TUI 상태와 절차로 복구 가능.

로봇팔 통합 기능은 추가로 다음을 모두 통과해야 완료다.

- 현행 구 로봇팔과 새 chassis 혼합 버전에서 nonzero 구동과 팔 언락이 fail-closed 차단됨.
- 실제 `can0` 중복 owner가 모터 연결 전에 `flock` 실패로 종료됨.
- 실제 wheel 정지 전에 팔이 작업 자세로 전환하지 않음.
- `DONE`만 수신하고 작업 종류에 맞는 `STOWED_LOCKED/CARRYING_LOCKED`가 없으면 출발하지 않음.
- 이전 `mission_id`의 지연 `DONE`을 무시하고 중복 `ArrivalStatus`가 작업을 재실행하지 않음.
- 토픽 도착 순서가 역전돼도 `MISSION_STOP`과 동일 ID arrival을 모두 받기 전 작업하지 않음.
- 과거 DDS burst·역행 stamp·빈 frame·stale TF가 주행 또는 인식 행동을 허가하지 않음.
- 운반 중 `CARRYING_LOCKED` heartbeat 단절 또는 `GRIP_LOST`에서 profile별 제한거리 안에
  motion hold하고 heartbeat만으로 자동 재출발하지 않음.
- `FAILED/ABORT` 뒤 `STOW_REQUEST → STOWED_LOCKED → 운영자 skip`으로 갇힘 없이 종료됨.
- 팔 노드 재시작 뒤 작업이나 주행이 암시적으로 재개되지 않음.
- 자율→원격 전환에서도 동일한 팔 잠금 조건을 우회하지 않음.
- 자율↔원격 전환이 프로세스·CAN owner 교체 없이 `command_authority` 내부에서 수행되고 원격
  명령도 동일한 chassis gate를 통과함.
- DualSense DRIVE/ARM mode가 Jetson ACK 기반으로 상호배타 전환되고, ARM mode에서
  `joint_1`~`joint_5` 개별 조그와 그리퍼를 모두 제어할 수 있으며 FSM/remote hardware command
  중첩이 0임.
- 원격 팔 deadman·D435i raw feedback·remote input·joint feedback 중 하나가 stale이면 다음 arm
  control tick에 hold하고, DRIVE 복귀는 `STOW_REQUEST → STOWED_LOCKED` 뒤에만 허용됨.
- operator override는 active mission·arrival을 먼저 취소하고 arm stale에서만 유효하며,
  `GRIP_LOST`와 다른 안전 gate를 우회하지 않고 heartbeat 복귀 시 즉시 drive 0이 됨.
- 팔 접힘·운반·비정상 정지 자세의 L515 가림에서 terrain stale가 fail-open으로 이어지지 않음.
- 필수 `FAILED`가 fresh locked posture 없이 재출발을 허가하지 않음. 팔 팀이 선택 진단 8종을
  제공하면 원인별로 기록되지만 미지원이 base 안전 gate를 막지는 않음.
- mission journal과 TUI에서 command owner, hold/E-stop source, mission correlation, CAN 10-node
  health, 채널별 freshness를 동일 run ID로 추적할 수 있음.
- L515 시간·TF·장착·depth 품질 qualification 결과와 production YAML hash가 보존됨.
- 네트워크 profile과 반자동 원격 보조가 두 RGB stream의 30 fps와 공통 chassis safety gate를 우회하지 않음.
- L515 1280×720과 D435i 848×480 receiver가 동시에 29 fps 이상이고, YOLO 지연·metadata 유실이
  D435i raw cadence를 낮추지 않으며 stale overlay가 자동 폐기됨.
- L515 video stale은 원격주행, D435i raw video stale은 원격 팔 조종만 hold하고, 어느 경우에도
  마지막 frame이나 stale bbox를 근거로 명령을 계속하지 않음.

다음 조건이면 기능 확장을 중단하고 단순 fallback을 선택한다.

- L515·D435i 동시 30 fps 원격 영상 목표를 지속적으로 훼손함.
- 8 GB Jetson에서 OOM 또는 제어주기 위반을 유발함.
- 실제 환경 성공률이 원격주행보다 낮고 남은 기간에 개선 근거가 없음.
- 안전 상태와 자동복구 상태를 운영자가 구분할 수 없음.
- 로봇팔 또는 L515 하드웨어 소유권을 중복시킴.
- remote arm 입력이 `ArmCommandAuthority`·MoveIt Servo를 우회해 Dynamixel goal을 직접 씀.
- 계약 v1/v2 혼합 배포가 팔 언락 또는 nonzero 구동을 허용함.

## 11. 의도적으로 하지 않는 것

- 극한로봇대회 전용 벽 통로, 극한 미션 YAML, 일정, 문서.
- Nav2, 사전 정밀지도, 전역 경로계획, loop closure를 포함한 완전한 SLAM.
- 상시 ROS PointCloud2 추가와 고주기 RGB-depth alignment. 내부 ROI point cloud와 제한된
  debug 시각화는 허용.
- 파워트레인의 신호등·마커·마네킹 중복 인식 모델.
- odometry 누적거리만으로 미션 도착 확정.
- 센서 단절 시 마지막 명령을 유지하는 fail-open 동작.
- 무제한 자동 재시도와 검증되지 않은 대회 당일 기능 추가.
- simulator 전용 알고리즘 fork, production 코드의 simulator 이름 분기, Isaac Sim 내부 Python으로
  production package 복제.
- 통신 단절 시 자동 원점 복귀. 현재 위치·경로가 보장되지 않으므로 motion hold와 운영자 복구를 쓴다.
- LLM/VLA/end-to-end 모델을 최종 주행 authority로 사용하거나 검증되지 않은 강화학습 정책을
  실차 제어에 적용하는 것.
- 네트워크 상태에 따른 추론 backend·FP16의 운용 중 자동 전환. encoder와 영상 profile만 사전
  qualification된 정적 구성 사이에서 hysteresis를 두고 전환한다.
- 통신 단절 자체를 무조건 latched E-stop으로 승격하는 것.
- 동일 목적의 behavior tree를 기존 segment FSM 위에 중복 적층하는 것.
- latency 측정과 필요성 증명 없이 PREEMPT_RT, realtime priority 또는 강제 CPU pinning을 도입하는 것.
- 대회 run 중 ROI·TF·안전 threshold를 온라인 학습·자동 튜닝하는 것. qualification 뒤 YAML을 동결한다.

## 12. 문서 정본 관계

- 대회 요구사항: `docs/국방로봇_규정.pdf`.
- 참고 사례: `docs/2025국방로봇_출품작설명서_합본.pdf`. 타 팀의 관측성·센서 품질·통신 열화 대응
  아이디어만 현재 아키텍처에 맞게 선별했으며 2026 요구사항이나 실물 성능 근거로 사용하지 않는다.
- 로봇팔 최신 통합 현황: Notion `로봇팔↔파워트레인 통합 개발 계획 (커스텀 msg 기반)
  (2026.07.13)` (`39b2d27b08d38064bdb0cd764f749d2a`). 현행 FSM·D435i markerless 인식·그리퍼
  실측은 구현 현황으로 사용하되, 페이지의 미합의 v1 unlock/DONE 의미는 WP5.2 v2 동시 컷오버로 대체한다.
- 자율주행 SW 우선순위와 일정: 본 문서.
- WP5.2 구현 절차: `docs/plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md`.
- 관측성·품질·원격 보조 구현 절차:
  `docs/plans/2026-07-13-observability-data-quality-remote-assist-plan.md`.
- 외부 교차검토 원문: `docs/reports/2026-07-13-claude-opus-4.8-autonomy-arm-crosscheck.md`.
- L515 production 운영: `AGENTS.md`의 L515 Gateway ownership와
  `docs/reports/2026-07-12-l515-gateway-performance-hil.md`.
- 차체 안전: `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`와
  `docs/reports/2026-07-10-wp5-control-safety-hil.md`.
- 기존 `docs/plans/2026-07-02-autonomous-driving-kickoff.md`는 2025 규정과 극한로봇 범위를
  포함한 역사 문서이며, 본 문서와 충돌하면 본 문서를 따른다.
