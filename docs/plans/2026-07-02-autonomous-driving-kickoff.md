# 파워트레인 자율주행 개발 착수 계획 (실행판)

> 작성 2026-07-02. 근거: 로봇팔 팀 GitHub(`ksp118/extreme-robot` main + **열린 PR #11·#10**),
> Notion 「극한로봇 Robot arm」(통합 개발 계획 · 기말 이후 계획서 Ver2 · TF 좌표계 초안 · Phase 1·2 문서).
> 방침: **로봇팔 팀이 확정한 인터페이스(메시지 5종)에 우리가 맞춘다.**
> 이 문서는 "무엇을, 어떤 도구로, 어떤 파일에, 어떻게 테스트하는지"까지 내려간 실행 계획이다.
>
> **📌 WP5.1 최신 상태 (2026-07-11): HIL 완료.** 아래
> 2026-07-07 WP5 `/cmd_vel → 10모터` HIL은 기존 차체 경로의 유효한 이력이다. 이후 추가한
> 순수 Python 안전 코어·비블로킹 50 Hz 제어·별도 US-100 ROS 노드·`/safety_verdict`·
> `/wheel_states` 경로는 실행 HEAD `ec452f6474b6fc57437d576298f2bc954649be42`에서
> `motor_control` 198 passed, `motor_gui` 91 passed, Jetson `powertrain_ros` 32/32를 통과했다.
> 실제 US-100 near·전원분리·재연결 latch와 present 8모터 fail-safe, 60초 50 Hz는 통과했다.
> `INVALID_READING` 실측은 환경상 생략했으며 자동시험으로 보완했다. ODrive 13·14는 이번에
> 일시 부재했지만 기존 10모터 실증 이력이 있다. 지상 제동과 최종 `stop_mm`은 차체 조립 후
> 실차 커미셔닝으로 분리하며 WP5.1 완료를 막지 않는다.
> 상태·증거는 HIL 보고서에 기록한다.
>
> **개발 순서**: command authority spec과 L515 경량 color/depth/IMU 파이프라인은 완료했다.
> 다음은 WP6 오도메트리다. PointCloud2는 현재 파이프라인에 없다. WP8 미션 시퀀서와
> `MISSION_STOP`·락 해제 순서·`ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클은
> 별도 미결 작업으로 유지한다.
>
> **2026-07-12 L515 성능 closure**: real-image RGB SRT receiver 29.91 fps, ROS color
> 30.0~30.2 Hz, raw Depth 10.0 Hz, SRT drop 0, SDK frame gap 0으로 완료했다. RGB에서는
> alignment를 억제하고 Depth/overlay SRT는 best effort다. 상세는
> `docs/reports/2026-07-12-l515-gateway-performance-hil.md`다.
>
> **📌 현재 상태 (2026-07-11 Jetson HIL 재확인)**: **WP1~WP5와 L515 공통 입력 완료**. WP1(CAN 구동
> 드라이버)·WP2(키네마틱스)·WP3(ChassisManager)는 10모터 실기 HIL, WP4는 로봇팔 실물
> ROS 그래프와 양방향 DDS 전달, WP5는 `/cmd_vel → ChassisManager → 10모터` 실기 HIL을 통과했다.
> L515는 분리/재연결과 D435i 동시 60초 HIL까지 통과했다. **다음 = WP6(오도메트리)**이며
> WP8은 병렬 후속이다. 남은 크로스팀 블로커는
> `MISSION_STOP`·락 해제 순서 계약과 풀 핸드셰이크 1사이클이다. 센서는
> **L515=파워트레인 RGB/depth/IMU, D435i=로봇팔 전용, US100=독립 안전**으로 확정했고,
> 2026-07-10 Jetson USB에서 L515와 D435i 동시 연결을 확인했다.
>
> **아래 배너는 7/5~7/7의 검증 이력이다. 현재 우선순위는 위 상태를 따른다.**
>
> **📌 진행 이력 (2026-07-05)**: WP1(CAN 구동 드라이버)·WP2(키네마틱스)·WP3(ChassisManager) **완료 + 실기 HIL 통과**.
> WP1 = `DriveOdriveCan` 구현+단위테스트+실기 HIL(커밋 `8453866`). **WP3 실기 4WS HIL 완주(2026-07-05, 실물 육안 확인)**:
> 10모터(AK 조향4 + ODrive 구동6) 단일 can0 협조 — 조향 홈→전진(바퀴 0.8 rev/s)→좌/우선회(애커만+역위상 4WS+차동,
> 조향 꺾임+바퀴 회전 동시 실물 확인)→정지. estop 안전망은 HIL이 잡은 통합버그 2건(status 굶음 `4e5cf1c`·
> arm→첫 tick false-stale `91c71e8`)에서 실제 작동 입증.
> ⚠️ HIL 교훈: 바퀴 지령이 **0.3 rev/s(HALL 코깅존) 미만이면 실물이 정지**한 채 텔레메트리만 그럴듯함 —
> 저속 테스트도 v≥0.4 m/s(바퀴 ≥0.6 rev/s)로, 그리고 **실물 육안 확인을 HIL 통과 조건에 포함**할 것.
> 사전작업: 구동 6축 게인 재튜닝(vel_gain 0.12, `23ae99d`)+전 보드 NVM 통일+CAN 일괄캘리 도구(`d416e60`).
> 로봇팔 **PR #11 main 머지 완료** → **당시 WP4 블로커 해소.** 당시 남은 1순위 블로커 = **D435i 카메라 독점**(§6-④).
> 당시 다음 단계 = **WP4(ROS2 메시지 왕복)** 또는 WP5. 하드웨어 숙제 = node12·16 HALL 접지/필터캡(역방향 피드백 품질).
>
> **📌 진행 상태 추가 (2026-07-07 갱신)**: ① **차체 4WS 텔레옵 유선+무선 완료·실기 검증** —
> `chassis/teleop_dualsense`(유선) + `chassis/teleop_server`↔`laptop/laptop_client_chassis`(무선,
> 상태회신 `서버[ARMED v..]` 표시·무한 재연결; DualSense 축은 `laptop/dualsense_axis_finder.py` 실측
> LX0·RT5·LT2·□3·○1). min_drive 플로어 1.0 rev/s(코깅존 회피)·조향 슬루 4500 erpm.
> ② **CAN "잘 되다 먹통" 완전 규명·종결** — 모터 PWM 노이즈(정지 폐루프 27% ≫ 회전 2%, SVM 에지
> 정렬+그라운드 도메인 비대칭) → bus-off 폭풍 → mttcan TX 웻지. **절연형 트랜시버 교체로 원천
> 해결**(최악 27.9%→0.0%) + 웻지 워치독 상주(compose `canwatchdog`, 보험). 전말·실험 16종:
> `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md`. **수동주행(원격조종) 체인은 이제 안정 —
> 당시 WP4(ROS2) 착수 준비 완료.**
>
> **📌 진행 상태 추가 (2026-07-07 오후): WP4·WP5 완료.** ③ **WP4(ROS2 메시지 왕복) ✅** —
> 분리 아키텍처(우리 `ros2/` 워크스페이스·`powertrain_ros` 컨테이너, robot_arm_msgs 벤더링,
> 통신은 DDS만). 로봇팔 실물 그래프 상대로 양방향 배달 검증(팔→우리 /detected_objects 수신 ·
> 우리→팔 /chassis_mode 도달). 계획 `docs/plans/2026-07-07-wp4-ros2-roundtrip.md`. ④ **WP5
> (chassis_node) ✅ 실기 HIL 통과** — ROS `/cmd_vel`→ChassisManager→10모터: 전진 v=1.0 4축
> 1.71~1.88 rev/s 균일 · 좌회전 애커만 차동(좌 1.07~1.17<우 1.15~1.18, min_rev 1.0 플로어로 압축) ·
> cmd_vel 끊김→0.00(워치독) · ~/arm·~/disarm·~/estop 서비스 동작. **자율주행의 실행 하부(ROS가
> 실제 모터를 굴림)가 성립.** 남은 계약 2건(MISSION_STOP·락 해제 순서)+풀 핸드셰이크는 로봇팔
> 팀과 합동. **다음 = WP6(오도메트리) 또는 WP8(미션 시퀀서).** ⚠️함정: colcon ament_python
> stale egg-info(entry point 누락) + scp본이 git pull 막음 → 신규 파일은 커밋·푸시·pull 로 배포.
>
> **📌 진행 상태 추가 (2026-07-07 저녁): 자율주행 4센서 배치 확정 (설계만 — 구현 0줄).**
> 보유 센서 = US100·D435i·L515(3D LiDAR)·IMU. **자율 구간 실내(조명 통제) 확정** → L515 가용.
> **배치 결정**: ① **L515 = 우리 자율 카메라**(RGB 레인 + depth 벽추종/장애물 + **자체 IMU** 오도메트리
> 보정) — 실내라 100% 가용, ② **D435i = 로봇팔 팀 인식 전용(통째로)**, 우린 `/detected_objects` 구독만
> → **§6-④ D435i 독점(1순위 블로커)을 재발행 협상이 아니라 '센서 분리'로 해소**, ③ **US100 = 독립
> 충돌 안전**(publish-only → `chassis_node` 최종 게이팅). IMU는 별도 부품이 아니라 **L515 내장
> IMU(BMI085) 사용** — L515 드라이버를 우리가 소유하므로 크로스팀 의존 0(D435i 내장 IMU는 로봇팔
> 발행 의존이 생겨 배제). **설계원칙**: 바퀴=거리·IMU=회전 상보(EKF·Nav2 안 씀), 인식=로봇팔 단일소스.
> **범위 = WP5 확장 + WP6 + WP7 + 공통 선행(L515 드라이버 노드)**. 상세 = §5 하단 '센서 드라이버·
> 데이터흐름' + WP5/6/7 인라인. **닫힘: §6-④·⑤ 협의 종결, WP6 'IMU 후순위'·WP7 '카메라 TBD' 제거.**

---

## 1. 한눈에 보기

- 젯슨 1대에 두 팀 소프트웨어 통합 확정. 프로그램끼리는 **ROS2 토픽**(정해진 양식의 쪽지를 우편함으로 주고받는 방식)으로 대화.
- **인식(YOLO)은 로봇팔 팀 전담.** 우리는 결과(신호등·정지선·마커의 이름+3D 위치)를 구독만 한다.
- 우리가 만드는 것: **① CAN 구동 드라이버 완성 → ② 4바퀴 조향 계산기 → ③ 차체 통합 제어 → ④ ROS2 연결 →
  ⑤ 주행거리 추정 → ⑥ 레인 추종 → ⑦ 미션 흐름 관리 → ⑧ 앞 로봇 추종** 순서.
- **진행 상황: 기존 WP1~WP5 완료, WP5.1 Tasks 1~8 소프트웨어 완료·최종 HIL 대기.** HIL 뒤
  command authority → L515 경량 파이프라인 → WP6 순서이며, WP8·크로스팀 핸드셰이크는 남아 있다.
- **실기 실행 게이트**: 결합 launch는 `stop_mm` 명시가 필수이며 기본값이 없다. 시나리오 1~8은
  바퀴 부양 상태에서 임시값을 명시한 HIL 후보, 시나리오 9는 50 kg 실차의 별도 승인 지상주행
  단계다. 9번의 통제 저속 제동 실측으로 승인된 값만 생산 launch에 사용한다.
- 마감 역산: **7/19 설계 확정 → 7/31 국방 서류 → 9/13 국방 본선 → 10/2 극한 본선.**
- 원격조종 점수 33~40% → 기존 텔레옵·스트리밍은 그대로 1급 유지 (자율 스택과 병행, 건드리지 않음).

---

## 2. 로봇팔 팀이 정해놓은 것 (요약)

| 항목 | 내용 |
|---|---|
| 통신 계약 | 커스텀 메시지 5종 `robot_arm_msgs` — **main에 머지됨(PR #11, 2026-07-03)**. 인식 노드 `robot_arm_perception`도 함께 |
| 우리가 받는 것 | `/detected_objects`(모든 인식 결과, 30fps), `/arm_status`(팔 상태, `DONE`=재출발 신호) |
| 우리가 보내는 것 | `/arrival_status`(mission_id+도착상태), `/chassis_mode`(`DRIVING`/`CORNERING`/`ROUGH_TERRAIN`/`MISSION_STOP`/`FOLLOW_LEAD`) |
| 핸드셰이크 | 정차 → `MISSION_STOP`+`ArrivalStatus` 송신 → 팔 작업 → `DONE` 수신 → 재출발 |
| 우리 몫 명시 | Nav2 안 씀·레인 추종 / 4WS 키네마틱스 / 오도메트리 / 레인은 raw 센서 직접 처리 |
| 금지 | 정지선·신호등·마커를 우리가 자체 인식하지 않기 (100% `/detected_objects` 구독 — 단일 인식 소스 원칙) |
| 환경 | ROS2 **Humble**, Docker(호스트 네트워크 공유), 좌표는 REP-103(x=앞, y=왼쪽, z=위) |
| 주의 | 그들 main엔 주행 코드 0줄(전부 우리 백지), 요구사항 문서는 레포에 없음(노션+직접 소통) |

---

## 2-1. 젯슨 현황 (2026-07-02 실측 / 2026-07-03 싱크 갱신)

로봇팔 팀 환경이 **젯슨에 이미 들어와 있다.** WP4(환경 구축)가 예상보다 가볍다. **(2026-07-03: 젯슨 두 레포 모두
GitHub origin/main과 싱크 완료 — `~/power-train-sw` @ `efc0b59`, `~/extreme-robot` @ `0ec4a0a`. 로컬 스크래치는 백업 후 정리.)**

| 항목 | 실측값 | 시사점 |
|---|---|---|
| 보드 | Orin Nano 8GB, JetPack 6(L4T R36.5.0) | 두 팀 스택 동시 구동 시 RAM 8GB가 병목 후보 |
| 디스크 | NVMe 233GB 중 **99GB 여유** (56% 사용) | 컨테이너 추가 여유 충분 |
| 우리 컨테이너 | `powertrain_jetson` (**가동 중**, 20.8GB) | 기존 CAN·텔레옵·비전 스택 정상 |
| 로봇팔 컨테이너 | `ros2_humble` (**7일 전 종료 상태**, 19.5GB) — host 네트워크+privileged, `~/extreme-robot/ros2_ws`→`/root/ros2_ws` 마운트 | 재기동만 하면 ROS2 Humble 사용 가능. privileged라 can0 접근도 가능 |
| 여분 이미지 | `osrf/ros:humble-desktop` (4.8GB) 이미 pull됨 | 우리 전용 ROS2 컨테이너를 만들 경우 베이스로 바로 사용 가능 (빌드 대기 없음) |
| 로봇팔 레포 | `~/extreme-robot` = **main @ `0ec4a0a`** (2026-07-03 싱크) — `robot_arm_msgs`·`robot_arm_perception` 있음 | ✅ 메시지 빌드 준비됨 (PR #11 머지분 반영) |
| 호스트 ROS | 없음 (전부 Docker) | 우리도 Docker 안에서만 ROS2 사용 |
| 센서 | RealSense·LiDAR USB 미연결, can0 DOWN | 모터·센서 전원 인가 후 작업하는 날에만 올라옴 (정상) |
| 기타 | 그들 compose에 WSL/X11 잔재 마운트 (`/mnt/wslg`) | 젯슨에선 무해하나 GUI(rviz2) 쓸 때 DISPLAY 설정 손봐야 할 수 있음 |

---

## 3. 도구 상자 — 어떤 프로그램으로 뭘 만드나

| 도구 | 무엇인가 | 우리 용도 |
|---|---|---|
| **Python 3.10** | (전 코드 공통) | 모든 신규 코드. 기존 자산(corner_module 등)과 동일 언어 |
| **ROS2 Humble / rclpy** | 로봇 프로그램 연결 프레임워크의 파이썬 라이브러리 | 노드(=프로그램 1개) 작성, 토픽 발행/구독 |
| **python-can** | CAN 버스 송수신 라이브러리 | 10모터 제어 (이미 검증된 프레임 그대로) |
| **OpenCV (cv2)** | 영상 처리 라이브러리 | 레인 인식 (이진화·원근보정·중심선 추출) |
| **numpy** | 수치 계산 | 키네마틱스·영상 배열 처리 |
| **pytest** | 파이썬 테스트 러너 | 키네마틱스·시퀀서 무하드웨어 단위테스트 (기존 24개 테스트에 추가) |
| **ros2 bag** | 토픽 녹화·재생기 | 실차 주행 데이터 녹화 → 사무실에서 알고리즘 재생 튜닝 |
| **rviz2** | ROS 3D 시각화 도구 | 오도메트리 궤적·인식 결과 눈으로 확인 |
| **YAML** | 사람이 읽는 설정 파일 | 대회별 미션 목록 정의 (코드 수정 없이 대회 전환) |
| **Docker** | 컨테이너 (이미 사용 중) | ROS2 실행 환경 (아래 WP4) |
| custom pyrealsense2 2.50.0 node | L515 → ROS Image/CameraInfo/Imu 6토픽 | **구현·단독/동시 HIL 완료**. D435i는 로봇팔 몫, PointCloud2 없음 |

**재사용하는 기존 자산** (새로 안 만듦): `corner_module`(조향+구동 협조 제어, HIL 검증) · 10모터 CAN 버스(6/29 8모터 → 7/4 10모터 전수 검증) ·
`safety_us100`(충돌방지) · 텔레옵+SRT 스트리밍 · GL-SFT1200 전용망 · `motor_gui`(진단).

---

## 4. 만들 프로그램 지도 (최종 형태)

```
                    ┌─ ROS2 세계 (신규) ──────────────────────────────┐
 [로봇팔 팀]        │                                                 │
 /detected_objects ─┼▶ mission_sequencer ◀── /odom ── odometry_node   │
 /arm_status ───────┼▶   (미션 흐름 관리)                              │
                    │        │ /cmd_vel (속도·회전 명령)               │
 /arrival_status ◀──┼── lane_follower ──┘  (레인 추종·신호등 반응)      │
 /chassis_mode  ◀───┼──┐     │                                        │
                    │  └─ chassis_node (차체 노드 = ROS2 껍데기)        │
                    └────────┼────────────────────────────────────────┘
                             ▼  (여기서부터 기존 파이썬 세계 — ROS 무관)
                    ChassisManager (6코너 통합, ✅완료 efc0b59)
                       ├─ kinematics.py (4WS 계산기, ✅완료 99a48a2)
                       └─ CornerModule ×6 (조향4 + 고정2, 기존)
                            ├─ SteerAk40 (기존, CAN)
                            └─ DriveOdriveCan (✅완료 8453866, WP1)
                                     ▼
                        can0 (500k) → AK ×4(조향) + ODrive ×6(구동) = 10모터
```

설계 원칙: **ROS2는 껍데기만.** 키네마틱스·차체 제어는 ROS 없는 순수 파이썬으로 만들어 pytest로 검증하고,
ROS2 노드는 그걸 감싸기만 한다 (기존 corner_module 스타일 유지 — ROS 없이도 테스트·텔레옵 가능).

---

## 5. 작업 패키지 (WP) — 이 순서대로 개발

### WP1. CAN 구동 드라이버 완성 — `DriveOdriveCan` — ✅ 완료 (커밋 `8453866`, 실기 HIL 통과)

> **완료(2026-07-05)**: 아래 계획대로 구현하되 실제 구조는 "공유 버스 계층" 대신 **드라이버별 자체
> socketcan 소켓 + CAN 필터**(자기 node만 수신)로 단순화 — SocketCAN 브로드캐스트 특성상 소켓 공유
> 불필요했고, 필터가 다중모터 트래픽 간섭도 차단. `bus` 주입 파라미터로 무하드웨어 단위테스트(9개).
> 실기 HIL: node 11·12 CAN 구동(1.0 rev/s 추종·Iq 실측·stale 정상). 상세는 커밋 로그 참고.

- **무엇**: `motor_control/corner_module/drive_odrive_can.py` — 코너모듈이 구동을 USB로만
  할 수 있으면 4바퀴 동시 제어가 불가능 → CAN 버전을 채워 넣는다. (완료 — 위 콜아웃)
- **어떻게**: 6/29 8모터 검증 때 쓴 프레임을 그대로 클래스에 옮기면 됨 (새 발명 없음):
  - `connect`: 버스 열기 → `Clear_Errors(0x18)` → 제어모드 설정(`0x0B`)
  - `arm`: 현재 위치 읽고(`0x09` RTR) → `input_pos=현재`(점프 방지) → `CLOSED_LOOP(0x07, 8)`
  - `set_velocity`: `Set_Input_Vel(0x0D)` — turns/s 그대로
  - `disarm`/`estop`: 속도 0 → `IDLE(0x07, 1)`
  - `state`: 하트비트(에러·상태) + 엔코더 속도 반환
- **구현 포인트**: 조향(SteerAk40)과 구동이 **한 can0을 같이 쓰므로** 버스 객체 1개를 공유하고 수신 프레임을
  확장ID(AK)/표준ID(ODrive)로 분배하는 얇은 공유 계층 필요 (`can_ak_odrive_demo.py`에 이미 있는 패턴 재사용).
- **도구**: python-can. **하드웨어**: 젯슨+모터 (반나절).
- **테스트**: 노드 11 하나로 1바퀴 회전 → 4노드(11/12/15/16) 동시 → estop 시 즉시 IDLE 확인.
- **완료 기준**: `CornerModule(SteerAk40, DriveOdriveCan)` 조합으로 기존 HIL 테스트 통과.
- **HIL 하드웨어 (기록용 — 07-04/05 실제 사용 구성)**: 젯슨 + CAN 트랜시버(ADM3053 — **외부 5V**·종단 60Ω·TX/RX 스왑주의) + 구동 ODrive 듀얼축 3보드+BL70200 ×6(node 11~16 전부 셋업·캘리 완료) + AK45-36 ×4(id 1~4) + 48V + 브레이크 저항(≈2Ω). ⚠️ **안전 필수**: 물리 E-stop(48V 차단)·**바퀴 지면에서 띄우기**·저속 시작(단 바퀴 지령 ≥0.6 rev/s — 코깅존 위), **단일노드→다노드** 순서.

### WP2. 4WS 키네마틱스 계산기 — ✅ 완료 (커밋 `99a48a2`)

- **무엇**: "전진속도 v, 회전 곡률 κ(=1/회전반경)" 명령을 **코너 4개의 (조향각, 바퀴속도)**로 바꾸는 순수 수학 모듈.
- **파일**: `motor_control/chassis/kinematics.py` (신규 패키지 `chassis/`)
- **수식** (차체 중심 원점, 코너 i 위치 (xᵢ, yᵢ)):
  - 조향각: `δᵢ = atan( xᵢ·κ / (1 − yᵢ·κ) )` — 직진(κ=0)이면 전부 0
  - 바퀴속도: `vᵢ = v · √( (1−yᵢ·κ)² + (xᵢ·κ)² )` — 회전 시 바깥쪽 바퀴가 빨라짐
  - 단위 변환: 바퀴 반지름 0.1 m → `turns/s = vᵢ / (2π×0.1)`
  - 보너스 모드: **제자리 회전**(피벗): `δᵢ = atan2(xᵢ, −yᵢ)`, 속도는 회전방향 부호
- **입력 파라미터**: 축거/윤거(코너 좌표) — **설계팀에서 실측값 받기** (받기 전엔 파라미터로 두고 진행),
  조향각 한계 ±45°(기존 config), 속도 한계 5 turns/s.
- **도구**: numpy, pytest.
- **테스트** (pytest, `motor_control/chassis/tests/test_kinematics.py`):
  직진→4바퀴 0°·등속 / 좌회전→좌우 대칭·안쪽 각도 큼·바깥 속도 큼 / 조향각 한계 초과 시 속도 자동 제한 / 피벗 모드.
- **완료 기준**: 테스트 전부 통과 + 손계산 케이스 3개 일치.
- **완료(2026-07-03)**: 구현은 **(v, ω) 입력** 채택(피벗 v=0까지 한 식으로 통합; κ은 v=0에서 발산). 기하는 설정 표(`default_geometry()`, 잠정 플레이스홀더). **pytest 14 통과.** 상세 = Notion 「4WS 애커만 키네마틱스」.

### WP3. 차체 통합 제어 — `ChassisManager` — ✅ 완료 (커밋 `efc0b59`) + **실기 4WS HIL 통과 (2026-07-05)**

- **무엇**: 코너모듈 **6개**(조향 4 + 고정 2)를 하나의 "차체"로 묶는 클래스. `set(v, ω)` 한 줄이면 전 코너가 움직인다.
- **파일**: `motor_control/chassis/chassis_manager.py`
- **어떻게**:
  - 코너↔모터 매핑(`DEFAULT_WHEEL_MAP`): 앞좌=AK1+ODrive11 · 앞우=AK2+ODrive12 · **중좌=고정+ODrive13⚠️ · 중우=고정+ODrive14⚠️** · 뒤좌=AK3+ODrive15 · 뒤우=AK4+ODrive16 → **6구동/4조향(10모터)**. 중간 13/14는 잠정값(검증셋=11·12·15·16). (**실제 배치는 조립 후 표 숫자만 교체.**)
  - 50Hz 루프에서 kinematics 결과를 각 `CornerModule.set()`에 분배, `tick()` 일괄 호출
  - estop 전파: 1곳이라도 트립하면 4코너 전부 정지 (기존 corner_module 워치독·과전류 트립 재사용)
  - US-100 안전 게이팅 그대로 물림 (stop 판정 → v=0)
- **테스트**: ① `fake.py`(가짜 드라이버)로 무하드웨어 pytest — "set(1.0, 0.5) 호출 시 각 코너에 기대값 도달"
  ② HIL: 바퀴 든 상태에서 직진/좌회전/피벗 명령 → 각도·속도 육안+로그 확인.
- **완료 기준**: 텔레옵을 ChassisManager 경유로 바꿔 4WS 수동주행 성공 (이게 원격주행 업그레이드도 겸함).
- **완료(2026-07-03)**: 코너 **6개**(조향 4 + 고정 2) 통합 · kinematics 분배 · estop 전파 · US-100 게이팅 · 워치독. 상세 = Notion 「차체 통합 제어 (ChassisManager)」.
- **실기 4WS HIL 통과(2026-07-05, 실물 육안 확인)**: `build_real_corners("can0")` 로 10모터 협조 —
  조향 홈 → 전진(6바퀴 0.8 rev/s) → 좌/우선회(**애커만**: 안쪽 앞바퀴 +31.5° > 바깥 +16.5°, **뒤축 역위상** 4WS,
  안/바깥 **차동속도** — 전부 kinematics 계산과 실측 일치) → 정지·홈 복귀. faulted=0.
  HIL이 잡아 고친 통합버그 2건: ① SteerAk40 소켓 무필터 → 다중모터 버스에서 AK status 굶음(`4e5cf1c`),
  ② `state()`가 stale 판정 전 버퍼 드레인 안 함 → 6코너 순차 arm(~1.2s) 뒤 첫 tick false-estop(`91c71e8`).
  ⚠️ 교훈: 바퀴 지령 <0.3 rev/s(HALL 코깅존)면 실물 정지 — 테스트는 v≥0.4 m/s + **실물 육안 확인 필수**.

### WP4. ROS2 환경 가동 + 메시지 왕복 (WP1~3과 병렬 가능) — 젯슨 실측 반영판

- **무엇**: 젯슨에 **이미 있는** ROS2 환경을 살리고, 로봇팔 팀 메시지가 오가는지 확인. (§2-1: 컨테이너·이미지·레포가
  이미 젯슨에 있음 — "구축"이 아니라 "가동+브랜치 갱신"이 실제 작업)
- **어떻게** (젯슨에서, 순서대로):
  1. ✅ **(완료) `robot_arm_msgs` main 머지 + 젯슨 싱크됨** (PR #11 머지, 2026-07-03). 젯슨 `~/extreme-robot`가
     main @ `0ec4a0a`로 싱크돼 `robot_arm_msgs`·`robot_arm_perception` 존재. → 바로 아래 2번부터 진행.
  2. **그들 컨테이너 재기동**: `docker start ros2_humble` (7일 전 종료 상태. host 네트워크+privileged 확인됨)
  3. **메시지 빌드** (그들 컨테이너 안): `cd /root/ros2_ws && colcon build --packages-select robot_arm_msgs && source install/setup.bash`
  4. **우리 쪽 접근 결정** (§6-⑥ 협의): 단기 = 우리도 `ros2_humble`에 들어가 작업 (이미 privileged → can0 접근 가능,
     `pip install python-can`만 추가). 장기 = 우리 전용 컨테이너 분리 — 이미 pull된 `osrf/ros:humble-desktop` 베이스로
     `docker/Dockerfile.jetson-ros2` 작성 (빌드 대기 거의 없음). **권장: 단기로 시작해 P1 끝나기 전 분리 여부 판단.**
  5. **왕복 테스트**: 터미널 2개에서 `ros2 topic pub --once /arrival_status robot_arm_msgs/ArrivalStatus '{mission_id: 1, status: ARRIVED_PICKUP}'`
     ↔ `ros2 topic echo /arrival_status`. 이후 우리 컨테이너↔그들 컨테이너 간(host 네트워크 공유)으로 동일 확인.
- **도구**: Docker, git, colcon, ros2 CLI. **하드웨어**: 젯슨만 (모터·센서 불필요).
- **주의**: RAM 8GB — 두 컨테이너 + YOLO + 스트리밍 동시 구동 부하는 P2에서 실측 (§8 리스크).
- **완료 기준**: 5종 메시지 pub/echo 왕복 성공, `ROS_DOMAIN_ID` 합의값으로 고정.

### WP5. 차체 노드 — `chassis_node` (ROS2 껍데기)

- **무엇**: ROS2 세계와 기존 파이썬 세계를 잇는 유일한 다리.
- **파일**: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` (ament_python 패키지 신규)
- **입출력**:
  - 구독 `/cmd_vel` (`geometry_msgs/Twist` — linear.x=전진속도, angular.z=회전율 → κ=ω/v 변환) → `ChassisManager.set()`
  - 구독 `/arm_status` → DONE 이벤트를 시퀀서에 전달
  - 발행 `/chassis_mode`(현재 `mode` 파라미터 인텐트)·`/chassis_state`(진단),
    `/arrival_status` 훅 + arm/disarm/estop 서비스.
    `/odom`은 WP6 `odometry_node`가 발행한다.
- **기존 WP5 안전 이력**: `/cmd_vel`이 0.5초 끊기면 자동 정지. 이 워치독은 현행
  WP5.1 용어로 자동복구 `MOTION_HOLD`다.
- **테스트**: `ros2 topic pub /cmd_vel` 수동 발행 → 실차 4WS 동작 / 발행 중단 → 0.5초 내 정지.
- **완료 기준**: ROS2 토픽만으로 주행·정지·모드 발행이 된다.

#### WP5.1 제어·안전 보강 — 소프트웨어 완료, 최종 HIL 대기

- **구조**: `motor_control/`의 순수 Python `SafetyInterlock`·`ChassisManager`가 정책과
  최종 E-stop, 단일 `can0` 500 kbps의 AK ×4 + ODrive ×6을 소유한다. 얇은 내부 ROS 노드는
  블로킹 US-100 UART를 별도 5~10 Hz 프로세스로 격리하고, `/safety_verdict` 최신값을 50 Hz
  `chassis_node`로 전달한다. `chassis_node`는 `/wheel_states`를 50 Hz로 발행한다.
- **상태**: `CHECKING`은 자동복구 `MOTION_HOLD`; `VALID`는 유효 거리가 `stop_mm` 미만일 때
  latched `ESTOP`; `INVALID_READING`은 거리 무효지만 0x50 MCU/UART 생존 응답이 있어 정상
  `RUN`; 거리와 0x50이 모두 연속 3회 실패한 `NO_RESPONSE`는 latched `ESTOP`이다. 0x50은
  초음파 송수신부 정상까지 증명하지 않는다.
- **복구**: `/cmd_vel` 0.5초 watchdog과 연결 단절도 자동복구 `MOTION_HOLD`다. 이 명령
  watchdog은 아래 0.75초 safety-topic freshness와 별개다. `ESTOP`은 위험 해소 후 reset해도
  `IDLE`까지만 가며, 별도 arm 전에는 회전하지 않는다.
- **freshness**: 생산 `safety_topic_timeout=0.75 s`, 최초 수신 timeout 1.0초다. 판정 age가
  0.75초를 초과한 다음 50 Hz tick, 명목상 마지막 수신 후 0.75~0.77초에 E-stop한다.
  `safety_required=false`는 BENCH/FAKE 전용이다.
- **검증 경계**: 배포 HEAD `c3610c136357a8c881263926ec18bcd7e3432a5d`에서 로컬
  pure-Python 189/91, 격리 read-only ROS 3패키지 build·31/31, Jetson 동일 HEAD의 3패키지
  build·31/31을 통과했다. 로컬 JUnit은 `.superpowers/sdd/final-motor-control-c3610c1.xml`,
  `.superpowers/sdd/final-motor-gui-c3610c1.xml`, `.superpowers/sdd/final-ros-c3610c1.xml`이고
  Jetson ROS XML은 `/home/zetin/power-train-sw/ros2/build/powertrain_ros/pytest.xml`이다.
  별도 49831bb의 software-only FAKE acceptance도 통과했지만 실기 HIL은 아직 실행하지 않았다.
  HIL 전까지 CAN delta·생산 `stop_mm`을 확정하지 않는다.

### WP6. 오도메트리 (주행거리·자세 추정)

- **무엇**: "지금까지 몇 m 왔나, 어느 방향인가"를 바퀴 회전량으로 추정. 대회 구간 전환의 기준
  (규정상 지형 순서가 고정이라 "N m 지점부터 험지" 식 전환이 가능 — 정밀 위치 불필요).
- **파일**: `ros2/src/powertrain_ros/powertrain_ros/odometry_node.py`
- **어떻게**: `chassis_node`가 발행하는 `/wheel_states`(바퀴별 속도+조향각) + `/l515/imu` 구독 →
  turns/s×2π×0.1=m/s → 평균 적산=거리 → `/odom`(`nav_msgs/Odometry`) 발행.
- **IMU 융합 확정 (2026-07-07)**: IMU를 **L515 내장 IMU(BMI085)로 확정**(더 이상 후순위 아님 — 센서 배치
  확정 배너). 설계원칙 **"바퀴=거리, IMU=회전"** — 병진(거리)은 바퀴 엔코더, 방향(yaw)은 IMU gyro-z 적분
  (정지 시 자이로 바이어스 자동보정; 바퀴 yaw는 로커보기 슬립에 약해 보조로만), roll/pitch는 IMU accel
  중력벡터(저역통과) → `ROUGH_TERRAIN` 판정·`/chassis/tilt` 발행. **상보필터**(robot_localization EKF·Nav2 안 씀).
- **테스트**: 순수함수 pytest(합성 wheel+imu 입력 → 기대 pose) / 줄자 5 m 직선 주행 → `/odom` 거리 오차
  **±5% 이내** / 제자리 90° 회전 → yaw 일치. rviz2로 궤적 눈 확인.

### WP7. 레인 추종 v0 (자율주행의 본체)

- **무엇**: 전방 카메라로 "갈 수 있는 길"을 보고 스스로 조향·속도를 정하는 노드.
- **파일**: `ros2/src/powertrain_ros/powertrain_ros/lane_follower_node.py`
- **알고리즘 v0** (OpenCV, 15Hz면 충분):
  1. 영상 하단 절반만 사용(관심영역) → 흑백 변환·이진화(트랙 vs 배경 — 대회장 바닥 색 보고 임계 결정)
  2. 원근 보정(버드아이 뷰)으로 위에서 본 그림으로 변환
  3. 가로줄마다 주행가능 영역의 **가운데 점** 추출 → 이은 선 = 갈 길
  4. 화면 중앙 대비 오프셋(픽셀) → 조향각 = Kp×오프셋 + Kd×변화율 (PID 제어)
  5. 속도 = 기본속도 × (1 − |조향각|/최대각) — 코너 자동 감속
  6. `/detected_objects`에서 정지선·빨간 신호 발견(+거리 z<임계) → 정지, 초록 → 재출발
  - 트랙이 벽으로 된 통로(극한)면: depth 영상에서 좌/우 벽 거리 비교 → 중앙 유지로 대체 (같은 구조, 입력만 교체)
- **개발 방법 (중요)**: 실차에서 바로 튜닝하지 않는다.
  ① 트랙(또는 유사 환경) 주행 영상을 **먼저 녹화**(ros2 bag/mp4) → ② 사무실에서 재생하며 임계값·PID 튜닝
  (cv2 창에 인식 결과 오버레이) → ③ 저속 실차 검증. 이 사이클을 돌린다.
- **카메라 확정 (2026-07-07)**: **L515 color 로 확정** — 임시 웹캠 단계 생략(센서 배치 확정 배너로 D435i
  독점 해소). 입력 = `/l515/color/image_raw`(레인) +
  `/l515/depth/image_rect_raw`(벽추종·전방 장애물 정지). PointCloud2는 없으며 벽 통로
  구간은 `16UC1` depth image의 좌/우 영역을 직접 비교한다. 실내라 depth 신뢰도 높음.
- **완료 기준**: 직선+곡선 모의 트랙 저속 1회 완주, 빨간 신호 정지·초록 재출발 데모.

### WP8. 미션 시퀀서 (대회 시나리오 관리)

- **무엇**: "1번 미션 지점까지 가서 → 멈추고 → 팔 깨우고 → 끝나면 다음으로"를 관리하는 노드. mission_id의 주인.
- **파일**: `ros2/src/powertrain_ros/powertrain_ros/mission_sequencer_node.py` + `missions/kukbang.yaml`, `missions/geukhan.yaml`
- **YAML 예시** (대회 전환 = 파일 교체):
  ```yaml
  - mission_id: 1
    name: 구호물자 픽업
    trigger: {odom_m: 12.5}          # 또는 {detect: pickup_marker, z_max: 1.0}
    on_arrive: ARRIVED_PICKUP        # /arrival_status로 송신할 문자열
    wait_arm_done: {timeout_s: 300}  # DONE 대기 (타임아웃 시 운영자 판단)
    then: resume                     # 재출발
  ```
- **어떻게**: 상태머신(기존 corner_module 상태머신 스타일) — `DRIVE → ARRIVE(정차+MISSION_STOP+ArrivalStatus)
  → WAIT_ARM(DONE 대기) → RESUME`. 타임아웃·재시도·수동 개입(텔레옵 전환) 훅 포함.
- **테스트**: 하드웨어 없이 — 가짜 `/arm_status` 발행으로 전체 전이 pytest + 로봇팔 팀 arm_fsm(mock)과 맞대고 리허설.
- **완료 기준**: mock 통합 리허설 1회 통과 ("도착→팔→재출발"이 자동으로 돈다).

### WP9. 추종 주행 (국방 ⑤구간 — 앞 로봇 따라가기)

- **무엇**: `/detected_objects`의 앞 로봇 좌표(z=거리, x=좌우)로 간격 유지 주행.
- **어떻게**: 속도 = PID(거리 − 목표간격), 조향 = P(좌우 오프셋). `/chassis_mode=FOLLOW_LEAD` 발행(팔 자세 락).
  목표 간격은 규정 확인 후 결정. 앞 로봇 미검출 시 즉시 정지(안전 기본값).
- **완료 기준**: 사람이 박스 들고 걷는 것 따라가기 데모 → 실로봇 간 테스트.

### 센서 드라이버·데이터흐름 (WP6·7 공통 선행) — 2026-07-07 추가

4센서 중 L515·IMU 공통 입력까지 2026-07-11 구현·HIL 완료했다. 아래 드라이버는
WP6(오도메트리)·WP7(레인)의 완료된 공통 선행작업이다.

- **드라이버 노드 `/l515_camera_node`**: `powertrain_ros`의 custom pyrealsense2 2.50.0
  노드다. color/depth Image+CameraInfo와 gyro/accel Imu 6토픽만 발행한다. serial을 엄격히
  고정하며 D435i fallback과 PointCloud2·IR·confidence·alignment는 없다. 분리/재연결 및
  D435i 동시 60초 HIL을 통과했다.
- **좌표계**: REP-103(x=앞·y=왼쪽·z=위), 트리 `odom → base_link → l515_link`(→ 카메라 내부 IMU 프레임).
  `base_link→l515_link`는 마운트 실측 static TF. 대회는 상대측위라 `map` 없음.

**데이터 흐름:**

```
L515 ─┬─ color ─► lane_follower(WP7) ─┐
      ├─ depth ─► (벽추종·장애물 정지)  │
      └─ imu ──► odometry(WP6) ─► /odom ├─► /cmd_vel ───┐
바퀴 /wheel_states ─┘        + /tilt    │                │
/detected_objects(로봇팔=D435i) ────────┘                ▼
US100 ─► /safety_verdict ───────────► chassis_node(gate+워치독) ─► 10모터
```

**확정 설계 결정 5개** (2026-07-11 최신):
1. L515 드라이버 = custom `pyrealsense2 2.50.0` 노드, color/depth Image+CameraInfo와
   분리 gyro/accel Imu만 발행(PointCloud2 없음)
2. 바퀴속도 출처 = `chassis_node`가 `/wheel_states` 발행 (CAN 단일소유자)
3. 오도메트리 융합 = "바퀴=거리, IMU=회전" 상보필터 (robot_localization EKF 아님)
4. US100 게이트 위치 = `chassis_node` 최종 게이팅 (defense in depth)
5. 레인↔벽추종 전환 = 미션 YAML로 구간 명시 (지형순서 고정이라 자동감지 불필요)

---

## 6. 로봇팔 팀 협의 항목 (P0 주간에 처리)

**통보** (그들 문서 원칙: "미정 스펙은 정해서 통보"):
1. **mission_id는 우리 미션 시퀀서가 정의·관리** — YAML 표를 만들어 공유.
2. 상태 문자열 = **대문자 스네이크** (`ARRIVED_PICKUP`, `DONE` …) — 그들 잠정값 그대로 확정 제안.
3. 깨우는 순서 = `MISSION_STOP` 송신 → 직후 `ArrivalStatus` 송신 — 그들 제안에 동의.

**협의**:
4. ✅ **해소 (2026-07-07) — D435i 독점**: 재발행/통일 협상 대신 **'센서 분리'로 종결** — **D435i는 로봇팔 팀
   전용(통째로), L515가 우리 레인·depth·IMU 카메라.** 우린 `/detected_objects`만 구독. IMU도 **L515 내장(BMI085)**
   사용 → D435i 내장 IMU 협의 불필요. (센서 배치 확정 배너 참조)
5. ✅ **해소 (2026-07-07) — 라이다 기동 주체**: **L515가 우리 라이다 = 우리가 `l515_camera` 드라이버 기동.**
   (연막 구간 역할 분담은 별건으로 잔존 — 자율 구간 실내 확정이라 우선순위 낮음)
6. `ROS_DOMAIN_ID` 값·우리 노드 컨테이너 배치 (WP4).
7. 텔레옵은 ROS 밖 기존 경로 유지 — 자율↔원격 전환 시 "전환 순간 정지" 규칙만 합의.

---

## 7. 일정 매핑 (WP → 주차)

| 기간 | 마일스톤 | 작업 |
|---|---|---|
| ~7/5 (P0) | 쪽지 왕복 | **WP2·WP3 완료 ✅**, WP4(머지 해소→착수 가능), §6 협의 |
| 7/6~7/12 (P1) | ROS→모터 직결 | ~~WP1~~·~~WP3~~(완료+HIL, 7/5), WP4·WP5 (그들 주차계획 "파워트레인 통신 연결 테스트" 주간) |
| 7/13~7/19 (P2) | 레인 v0 + **설계 문서** | WP6, WP7 착수, 7/19 국방 문서에 아키텍처 반영 |
| 7/20~7/31 (P3) | 팔 통합 + **국방 서류** | WP8, 신호등 반응, mock 리허설, 7/31 제출 |
| 8월 (P4) | 실전 기능 + **극한 서류 8/17** | WP7 실트랙 튜닝, WP9, 연막 대응, 자율↔원격 전환 절차 |
| 8/18~ (P5) | 리허설 | 국방 5구간·극한 4구간 리허설 ≥3회, 새 기능 금지 |

**의존 관계**: WP2→WP3→WP5, WP1→WP3, WP4→WP5~9. WP2와 WP4는 지금 병렬로 시작 가능.

**2026-07-11 최신 실행 순서**: command authority spec과 L515 경량 color/depth/IMU
파이프라인 HIL을 완료했다. 다음은 WP6이며, WP8과 크로스팀 핸드셰이크는 계속 미결이다.

---

## 8. 리스크와 대비

| 리스크 | 대비 |
|---|---|
| 단일 젯슨 성능 — **RAM 8GB 실측** (YOLO 30fps + 레인 + 스트리밍 + 컨테이너 2개) | P2에서 부하 실측 → 레인 해상도/주기 하향(15Hz→10Hz), 스왑 확인, 최악엔 역할 분리 재논의 |
| ~~D435i 독점 미해결~~ ✅ 해소 | **'센서 분리'로 종결** (L515=우리 카메라, D435i=로봇팔). §6-④ 참조 |
| ~~RealSense 2대 동시 부하 미검증~~ ✅ 해소 | 60초 동시 HIL: L515 color/depth 29.75/29.45 Hz, `/detected_objects` 약 19.7 Hz, USB error 0, 합산 최대 RAM 약 1.41 GiB |
| 코너 좌표(축거/윤거)·코너↔모터 배치 미확정 | 전부 파라미터/설정 표로 — 실측값 오면 숫자만 교체 |
| 그들 구조 유동적 (3축 리팩터 등) | 메시지 5종 계약에만 의존, 그들 내부 구현에 결합 금지 |
| 레인 추종 실패 구간 | 전 구간 텔레옵 폴백 (원격 점수 확보) — WP3 완료 시점부터 4WS 텔레옵 가능 |
| 국방 ② 수중 구간 방수 | 설계팀에 조기 확인 요청 |
