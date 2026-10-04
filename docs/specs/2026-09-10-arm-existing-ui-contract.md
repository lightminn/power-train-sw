# 기존 로봇팔 UI 연결 계약 — 1단계

기준: 파워트레인 `2ffbad6`, 로봇팔 `aed6c2c`. 2026-09-10 로컬 소스 대조.
이 문서는 후속 연결 구현의 계약이며 아직 송신기나 GUI에 적용된 프로토콜이 아니다.
제품 수정은 파워트레인 레포에 한정한다. Claude 소유 `operator_console/arm_ui/`는 수정하지 않는다.

## 기존 경로와 확인된 공백

`arm_console_bridge_node.py`는 팔 ROS 4개 토픽을 읽어 5 Hz로 UDP :5007에 전송한다.
YOLO는 별도 :5003이다. `operator_console/arm_telemetry.py`가 :5007을 해석하고
`app.py::_refresh_end_effector_summary`가 기존 도구 영역과 상세 창을 채운다.

| 기존 화면 | 현재 입력/코드 | 사용할 팔 원천 | 필요한 연결 |
|---|---|---|---|
| 도구 선택 | 사용자가 고른 문자열; gripper_a/b 별칭 | `/tool/status.tool_type` | 실제 타입 매핑, 최초 감지값 우선, 후보 분리 |
| 장착 상태 | end_effector_attached | discovered, detached, physical_tool_detached, actuator online | 버스 감지와 기계 체결을 구분 |
| 도구 ID/인터페이스 | end_effector_id/interface | tool_profile.actuator_ids, actuators | 현재 ID 목록과 관측 출처 표시; 물리 커넥터 규격은 미상 |
| 조종 모드 | `연동 예정` 고정 | `/control/mode_status`, `/tool/status.control_mode` | 상태 필드 추가; 요청 상태와 분리 |
| 관절 부하 | dynamixel.current 최대 절댓값 | `/joint_states.effort` 또는 기존 `/dynamixel/state` | 서로 다른 원천·단위 보존, 팔/도구 분리 |
| 관절각 | joints.position_rad | `/joint_states.position` | velocity 생략 허용 |
| 도구 상세 피드백 | 기존 선택/장착/부하/텔레메트리 행 | `/tool/status.actuators` | 현재 도구만 필터, 교체 시 이전 값 제거 |
| 기존 모터 진단 | dynamixel ID/각도/current/temperature | `/dynamixel/state`가 실제 발행될 때 | 기존 경로 유지; MoveIt 원천에 없는 온도는 미수신 |
| 팔/도구 FSM (후속 탭) | 필드 없음 | `/fsm/state`, `/arm_status`, `/tool/status.fsm_state` | 별도 필드, 원천별 age |

### 직접 실행해 확인한 호환 문제

팔 브리지 `publish_joint_states`는 팔에 name/position/effort를 채우고 velocity는 비운다.
기존 `_joint_payload`는 name/position/velocity의 길이가 같아야 하므로 합법적인 이 메시지를 제외한다.
순수 변환기에 `names=[arm_joint_1], position_rad=[0.2], velocity=[]`를 넣어 실행한 결과
`joints:null`, 콘솔 decode 결과 `joint_names=()`를 확인했다.
3단계에서 빈 velocity를 허용하되 0 속도를 만들어 넣지 않는다. 필요하면 새 선택 필드로
관절 상태를 전달하고 기존 v1 경로와 함께 해석한다.

## 타입 및 ID

| 정본 타입 | 실 프로파일 ID | UI 이름 제안 |
|---|---|---|
| spur_1motor_gripper | 5 | 단일 그리퍼 |
| dual_motor_gripper | 3, 4 | 듀얼 그리퍼 |
| cleaner | 2 | 청소 모듈 |
| environment_sensor | 팔 프로파일 없음 | 환경 센서 모듈; 기존 :5008 유지 |

기존 `그리퍼 1/2`와 `gripper_a/b`에는 기구 종류를 특정하는 근거가 없다.
숫자 대응을 사실로 확정하지 않는다. 연결 구현에서는 실제 타입을 보존하고 기구 이름을
표시하는 방식이 우선이며, 숫자 이름을 유지할 경우 사용자 대응 확인이 필요하다.
mock cleaner ID는 실 프로파일과 다를 수 있으므로 ID를 상수로 감지하지 않고 활성 프로파일을 따른다.
팔 JOINT_CONFIG의 ID는 1축=11, 2축=14, 3축=13, 4축=12, 5축=16이다.
현재 설정의 참고값이지 배포 현장의 관측값은 아니다. 런타임 도구 ID와 합쳐 부하를 집계하지 않는다.

## 값의 의미와 미수신 처리

- 팔 position은 rad이며 UI에서 필요 시 degree로 변환한다. legacy motor position_deg는
  `(tick-2048)*360/4096`의 서보축 값으로, 기어비·영점을 적용한 관절각이 아니다.
- MoveIt effort는 주소 126의 signed raw다. Nm/A/mA로 변환하지 않는다.
  모델별 의미가 확인되기 전 `피드백 raw`로 표시한다. 도구 effort도 동일 원칙을 적용한다.
- 기존 모터 파서는 tick 0..4095만 허용한다. 다회전·음수 tool tick을 기존 motor 배열에 억지로 넣지 않는다.
- 기존 모터 레코드는 온도를 필수로 요구한다. 온도가 없는 tool/JointState 원천을 위해
  0도를 만들어 채우지 않고 별도 선택 필드로 전달한다.
- 모터 ID 감지는 기계적 잠금 확인이 아니다. 현재 팔에 별도 lock 관측값이 없으므로
  발견+online을 `장착 확인/체결`로 단정하지 않는다. 기존 상태 배지 문구는 `감지됨`으로 연결 가능하다.
- `end_effector_id`는 고유 제품 식별자와 모터 ID 목록을 혼용하지 않는다. 상세 행에는 `모터 ID 3, 4`처럼 표시한다.
- control mode는 승인 상태로만 표시한다. `/control/mode_status`가 이벤트성일 수 있어
  주기 `/tool/status.control_mode`를 함께 사용하되 출처를 기록한다. 서로 모순되면 불일치를 노출한다.
- 도구 선택만으로 `/tool/change`를 발행하지 않는다. 기존 popup 열기와 선택 기능은 유지한다.

## 후속 상태 확장 규칙

기존 v1 레코드를 유지하고 `arm_runtime` 선택 객체를 추가하는 방식을 제안한다.
신규 순수 모델/파서에 정의하고 기존 파일에는 최소 연결만 둔다.

- tool_type: 정본 타입 또는 null; candidate_tool_type은 UI 로컬 상태로만 유지.
- actuator_ids, actuators: 활성 도구 목록. position_tick, feedback_raw, online,
  torque_state, operating_mode, hardware_error, model은 관측값이 없으면 null.
- control_mode, arm_fsm_state, arm_contract_status, tool_fsm_state는 독립 필드.
- discovered, detached, physical_tool_detached, motion_allowed, read_only, mock_mode,
  emergency_stop, profile_valid, calibrated 및 백엔드 reason을 전달.
- 원천별 age_s(tool/status, joints, mode_status, arm_fsm, arm_status)를 기록한다.
  전송 시 source age와 수신 후 경과시간을 합쳐 판정하고 미래/음수/NaN은 거부한다.
- 기존 1초 원천 신선도 기준을 우선 사용한다. UDP만 살아 있고 원천이 멈추면 정상으로 표시하지 않는다.
- generation은 bridge 세션과 tool 문맥을 구분하도록 설계한다. 도구 전환 또는 sender 재시작 시
  이전 진단/요청 상태를 폐기한다. 임의 string 타입 변경만으로 실장 완료를 판단하지 않는다.
- 기존 4096 byte 한도를 유지한다. 전체 tool_profile/캘리브레이션 기록을 복사하지 않고
  표시 필드만 화이트리스트로 전송한다. 초과 시 선택 상세를 먼저 생략하고 truncated로 표시한다.
  동일 포트에 두 송신기를 띄우지 않는다.
- 기존 renderer도 새 객체 원천별 freshness를 사용하도록 최소 수정한다. parser만 추가하고
  기존 UDP LIVE 판정으로 오래된 도구 상태를 표시하는 불완전 연결은 허용하지 않는다.

## 최소 수정 접점 및 단계 인계

2단계: 신규 순수 tool 상태 변환 모듈 + 기존 bridge에 read-only 구독/전송 연결,
콘솔 신규 상태 모델 + 기존 선택/상세 콜백에 연결. 기존 영상 metadata 경로는 변경하지 않는다.
3단계: 동일 모델에 mode·joints raw 피드백 연결. velocity 빈 배열과 미수신 온도를 처리한다.
Claude의 신규 탭은 이후 이 모델을 주입받는다. 이번 단계에서 Claude 위젯 API를 임의 확정하지 않는다.
ops 명령, 팔 ROS publisher, SDK, 차체 제어는 1~3단계 관측 연결에 필요하지 않다.

## 검증 현황과 한계

- 로컬 팔 HEAD aed6c2c, 작업 트리 clean 확인. 파워트레인은 지정 커밋 유지.
- Docker 실행 컨테이너 목록은 비어 있었다. 기존 GUI는 앞서 로컬 미연결로 실행한 화면이다.
- `ssh jetson`은 hostname 해석 실패: Jetson 체크아웃/실제 토픽/배포 버전은 확인하지 못했다.
- 원본 main은 앞선 조회 7d7f001을 참고하되 이번 단계에서 최신 동기화를 주장하지 않는다.
- 순수 변환기 실행으로 velocity 생략 시 관절 소실 재현. 실제 로봇 구동·ROS live 연결 검증 아님.
- 후속 실제 연결 전에 접근 가능한 Jetson 주소와 실행 중인 팔 원천의 토픽/타입/QoS를 읽기 전용 확인한다.

## 1단계 완료 판정

기존 위젯↔원천 대응, 단위·신선도·미수신 규칙, 타입 매핑의 미확정 부분과 최소 수정 접점을 정리했다.
기존 UI 연결 코드 구현과 실환경 확인은 아직 완료되지 않았으며 2~4단계에서 수행한다.
