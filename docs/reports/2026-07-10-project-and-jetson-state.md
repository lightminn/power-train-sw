# 파워트레인 SW 정본 상태 — 레포·Jetson·문서 감사

> 확인일: 2026-07-10 KST. 범위는 파워트레인 SW와 로봇팔 연동 인터페이스이며,
> CAD·전장은 인계 요구사항 외에는 다루지 않는다.
>
> **WP5.1 최신 override (2026-07-10): Tasks 1~8 소프트웨어 완료, 최종 실기 HIL 미실행.** 아래
> WP5 `/cmd_vel → 10모터` HIL은 기존 차체 경로의 역사적 완료 기록이다. 새 비블로킹
> 50 Hz 제어·US-100 상태/liveness·`/safety_verdict`·`/wheel_states`·latched E-stop 경로는
> `docs/reports/2026-07-10-wp5-control-safety-hil.md`가 `NOT RUN`인 동안 실기 완료로 보지 않는다.
> Jetson software-only FAKE는 commit `49831bb42058a177ed9c41d72d0273f4f0a8f535`에서
> 통과했지만 실기 HIL 증거가 아니다.

## 1. 현재 완료 상태

| 영역 | 정본 상태 |
|---|---|
| 모터 버스 | can0 500 kbps, AK45-36 ×4 + ODrive/BL70200 ×6, node 1~4·11~16 |
| CAN 물리층 | ADM3053 절연 트랜시버로 PWM 노이즈 0%, 웻지 워치독은 보험으로 상주 |
| ODrive | pp=10, cpr=60, bandwidth=30, vel_gain=0.12, vel_int=0.2 |
| 차체 | WP1~3 및 10모터 4WS 유·무선 텔레옵 실기 HIL 완료 |
| ROS2 기준선 | WP4 양방향 DDS 전달, 기존 WP5 `/cmd_vel → 10모터` 실기 HIL 완료 |
| WP5.1 제어·안전 | Tasks 1~8 소프트웨어 완료; 로컬 motor_control 189 passed·motor_gui 91 passed·임시 read-only ROS 3패키지 build·powertrain_ros 23 tests passed; Jetson software-only FAKE PASS; 최종 실기 HIL 미실행 |
| 센서 소유권 | L515=파워트레인 RGB/depth/IMU, D435i=로봇팔 전용, US100=독립 안전 |
| 형상 최적화 | v4 계산 기준 50 kg 확정, 86 kg 재최적화 안 함 |

## 2. Jetson 실측

- `~/power-train-sw`: `main`, 미푸시 커밋 없음. 로컬 PC보다 센서배치 문서 1커밋 뒤.
- 미추적 `motor_control/vision/tests/`: D435i 기반 `yolo_depth_3d`의 비동기 인코딩,
  SRT, 좌표 UDP, depth 역투영 회귀 테스트. 팀원 작업이므로 보존한다.
- `~/extreme-robot`: `Gripper_YOLO_FSM` 브랜치, `origin/main` 이후 로봇팔 인식·그리퍼
  작업 커밋과 perception 미커밋 변경이 존재한다. 파워트레인 문서가 그 내부 구현에
  결합하면 안 된다.
- USB: Intel RealSense 515와 Depth Camera D435i 동시 연결 확인.
- 컨테이너: `powertrain_ros`, `powertrain_canwatchdog` 실행 중. 점검 시 chassis/ROS 제어
  프로세스는 없고 can0는 DOWN이었다. `powertrain_jetson`과 로봇팔 `ros2_humble`은
  종료 상태였다.
- ROS 계약: `sync_check_msgs.sh ~/extreme-robot` 통과 — 벤더링된 `robot_arm_msgs` 5종과
  현재 로봇팔 체크아웃 사이 드리프트 없음.
- 자원: NVMe 233 GB 중 58 GB 가용(74% 사용), RAM 7.4 GiB 중 5.3 GiB 가용,
  swap 3.7 GiB 중 24 MiB 사용.
- 미추적 vision 테스트는 세 Jetson 컨테이너 모두 `pytest`가 없어 실행하지 못했다.
  코드·수집 결과로 통과를 추정하지 않으며, 팀원 작업을 커밋하기 전 의존성이 있는
  테스트 환경에서 별도 실행해야 한다.
- WP5.1 FAKE(commit `49831bb42058a177ed9c41d72d0273f4f0a8f535`): startup `ESTOP`, far
  `ARMED/RUN`, near `ESTOP`, far 뒤 latch, reset→`IDLE` 무암시 arm, 별도 arm, publisher-death
  `ESTOP`을 확인했다. 60초 count 3000, mean/min-5s 50.000 Hz, tick p99 0.280 ms, overrun 0,
  max interval 21.453 ms, publisher-death delay 0.753 s다. Jetson ROS XML은
  `/home/zetin/power-train-sw/ros2/build/powertrain_ros/pytest.xml`; FAKE raw log는 미보존이다.

## 3. WP5.1 현재 계약

- 순수 Python `SafetyInterlock`·`ChassisManager`가 제어·안전 정책, 최종 E-stop, 단일
  `can0` 500 kbps의 AK45-36 ×4 + ODrive/BL70200 ×6을 소유한다. 얇은 내부 ROS 노드는
  블로킹 US-100 UART를 별도 5~10 Hz 프로세스로 격리하고, `/safety_verdict`를 전달하며,
  `/wheel_states`를 50 Hz로 발행한다.
- US-100 상태는 `VALID`, `INVALID_READING`, `CHECKING`, `NO_RESPONSE`다.
  `INVALID_READING`은 0x50이 MCU/UART 생존만 증명한 정상 통과 상태다. `CHECKING`,
  `/cmd_vel` 0.5초 watchdog, 연결 단절은 자동복구 `MOTION_HOLD`다. 이 명령 watchdog은
  0.75초 safety-topic freshness와 별개다. 유효 근거리 또는
  거리·0x50 생존 확인이 모두 연속 3회 실패한 `NO_RESPONSE`는 latched `ESTOP`이다.
- `ESTOP` reset은 `IDLE`까지만 복구하며 별도 arm이 필요하다. 생산 safety topic timeout은
  0.75초(`age > threshold` 뒤 다음 50 Hz tick, 명목 0.75~0.77초), startup timeout은
  1.0초다. `safety_required=false`는 BENCH/FAKE 전용이다.
- 0x50은 초음파 송신기·수신기 정상까지 증명하지 않는다. `INVALID_READING` 통과는 HIL과
  운영 절차에서 계속 추적할 잔여 위험이다.
- 결합 실기 launch는 `stop_mm` 명시가 필수이고 생산 기본값이 없다. 승인 전 임시값은 바퀴를
  든 시나리오 1~8의 통제 HIL 후보에만 쓴다. 50 kg 실차 지상주행 시나리오 9는 별도 사용자
  허가가 필요하고, 그 제동 실측으로 승인된 값만 생산 launch에 사용한다.

## 4. 다음 작업

1. WP5.1 시나리오 1~8 부양 HIL과 별도 승인 시나리오 9 지상 제동 HIL, 생산 `stop_mm` 실측.
2. HIL 통과 뒤 단일 `/cmd_vel` command authority spec.
3. L515 경량 color image + depth image + IMU 파이프라인 spec. PointCloud2는 opt-in.
4. 위 인터페이스 확정 뒤 WP6 오도메트리.
5. WP8 미션 시퀀서와 `MISSION_STOP`·락 해제 순서 계약.
6. `ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클.

## 5. 문서 해석 규칙

- 이 문서와 최신 kickoff 계획이 현재 상태의 정본이다.
- WP5.1 HIL 결과·측정률·CAN delta·생산 `stop_mm`·go/no-go는
  `docs/reports/2026-07-10-wp5-control-safety-hil.md`만 정본으로 삼고, 미실행 칸을 추정해
  채우지 않는다.
- 5~6월 specs/plans/reports의 당시 수치는 역사 기록으로 보존한다.
- BL70200 실기 설정은 `bl70200_setup.py`, CAN 캘리는 `can_calibrate_all.py`를 따른다.
- `odrive_calibration.py`의 pp=5 단일축 경로는 레거시이며 사용하지 않는다.
