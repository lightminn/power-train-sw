# 파워트레인 SW 정본 상태 — 레포·Jetson·문서 감사

> 확인일: 2026-07-10 KST. 범위는 파워트레인 SW와 로봇팔 연동 인터페이스이며,
> CAD·전장은 인계 요구사항 외에는 다루지 않는다.

## 1. 현재 완료 상태

| 영역 | 정본 상태 |
|---|---|
| 모터 버스 | can0 500 kbps, AK45-36 ×4 + ODrive/BL70200 ×6, node 1~4·11~16 |
| CAN 물리층 | ADM3053 절연 트랜시버로 PWM 노이즈 0%, 웻지 워치독은 보험으로 상주 |
| ODrive | pp=10, cpr=60, bandwidth=30, vel_gain=0.12, vel_int=0.2 |
| 차체 | WP1~3 및 10모터 4WS 유·무선 텔레옵 실기 HIL 완료 |
| ROS2 | WP4 양방향 DDS 전달, WP5 `/cmd_vel → 10모터` 실기 HIL 완료 |
| 센서 소유권 | L515=파워트레인 RGB/depth/IMU, D435i=로봇팔 전용, US100=독립 안전 |
| 형상 최적화 | v4 계산 기준 50 kg 확정, 86 kg 재최적화 안 함 |

## 2. Jetson 실측

- `~/power-train-sw`: `main`, 미푸시 커밋 없음. 로컬 PC보다 센서배치 문서 1커밋 뒤.
- 미추적 `motor_control/vision/tests/`: D435i 기반 `yolo_depth_3d`의 비동기 인코딩,
  SRT, 좌표 UDP, depth 역투영 회귀 테스트. 팀원 작업이므로 보존한다.
- `~/extreme-robot`: `Gripper_YOLO_FSM` 브랜치, `origin/main` 이후 로봇팔 인식·그리퍼
  작업 커밋과 perception 미커밋 변경이 존재한다. 파워트레인 문서가 그 내부 구현에
  결합하면 안 된다.
- USB: Intel RealSense 515와 Depth Camera 435if 동시 연결 확인.
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

## 3. 다음 작업

1. WP6 오도메트리 또는 WP8 미션 시퀀서.
2. `MISSION_STOP`·락 해제 순서 계약 확정.
3. `ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클.
4. L515 `realsense-ros` 드라이버와 `/wheel_states`·US100 최종 게이트 구현.

## 4. 문서 해석 규칙

- 이 문서와 최신 kickoff 계획이 현재 상태의 정본이다.
- 5~6월 specs/plans/reports의 당시 수치는 역사 기록으로 보존한다.
- BL70200 실기 설정은 `bl70200_setup.py`, CAN 캘리는 `can_calibrate_all.py`를 따른다.
- `odrive_calibration.py`의 pp=5 단일축 경로는 레거시이며 사용하지 않는다.
