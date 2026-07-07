# WP4 — ROS2 가동 + 메시지 왕복 (분리 아키텍처)

착수계획(`docs/plans/2026-07-02-autonomous-driving-kickoff.md`)의 WP4. 로봇팔 팀
(`ksp118/extreme-robot`)과 **분리 개발**하며 ROS2 메시지 계약만 공유, 우리 쪽 pub/sub 이
그들 **실물 FSM** 과 왕복 성립함을 증명하고 계약을 확정한다.

## 결정 사항 (2026-07-07 사용자 확정)

- **분리 아키텍처**: 우리 코드는 우리 레포(`ros2/`)·우리 컨테이너(`powertrain_ros`),
  로봇팔은 그들 것. 공유는 메시지 계약뿐, 통신은 DDS(host net, DOMAIN 0).
- **msg 공유 = 벤더링(A안)**: `.msg` 5개를 `ros2/src/robot_arm_msgs/` 에 복사해 우리가
  직접 빌드. ROS2 타입은 이름+구조해시로 매칭되어 그들 빌드본과 wire 호환. 드리프트는
  `sync_check_msgs.sh` 로 감시. (대안 B=submodule 은 큰 레포 통째라 기각.)

## 조사 결과 (로봇팔 레포 실측, 2026-07-07)

- `ros2_humble` 컨테이너 상시 실행(net=host·privileged·humble·Fast-DDS·DOMAIN 0).
- 살아있는 노드: `/arm_fsm_node` `/perception_node` `/moveit_dynamixel_bridge` `/stream_node`.
- 그들이 이미 **우리 토픽을 구독**: `/arrival_status`·`/chassis_mode`. **발행**: `/arm_status`·`/detected_objects`.
- `perception_node` 는 `pyrealsense2` 로 D435i 를 **직접 오픈** → 단일 점유(WP7 블로커, WP4 무관).

## 계약 (robot_arm_msgs, 5종)

| 방향 | 토픽 / 타입 | 필드 | 값 |
|---|---|---|---|
| 우리→팔 | `/arrival_status` `ArrivalStatus` | header·mission_id·status | `ARRIVED_PICKUP`·`ARRIVED_DROP` |
| 우리→팔 | `/chassis_mode` `ChassisMode` | header·mode | 락 `{CORNERING, ROUGH_TERRAIN, FOLLOW_LEAD}` / 해제 `DRIVING` |
| 팔→우리 | `/arm_status` `ArmStatus` | header·mission_id·status | `IDLE→PERCEIVING→PLANNING→EXECUTING→CARRYING→DONE`·`FAILED` |
| 팔→우리 | `/detected_objects` `DetectedObjectArray` | header·objects[] | class_id·class_name·confidence·Pose·bbox @30fps |

문자열 단일 출처 = `ros2/src/powertrain_ros/powertrain_ros/contract.py`.
그들 `arm_fsm_node.py` 는 이 값들을 "⚠️ 잠정값 — 파워트레인 팀과 합의 후 확정"으로 둠.

### ⚠️ 미결 2건 (우리가 확정해 팀에 전달)

1. **`MISSION_STOP`**: 우리 vocab 에 있으나 그들 `LOCK_MODES` 에 없어 팔이 무시. 미션
   정차 중엔 팔이 움직여야 하므로 **락 아님이 맞음** — 다만 vocab 에 남길지/정보용으로만
   보낼지 확정 필요.
2. **락 해제 순서**: 코너(→팔 락) 상태로 미션 지점 정차 시, 팔은 `locked=True` 인데
   우리가 정차 중 `DRIVING` 을 안 보내면 **락에 걸린 채 집기 불가**. → 미션 정차 진입
   시퀀스에 "`DRIVING`(언락) 먼저 → `ARRIVED_*` 발행" 을 규약으로 못박아야 함.

## 작업

- [x] **4-1. 워크스페이스 스캐폴딩** — `ros2/`(벤더 msgs + `powertrain_ros` 노드),
  `docker/Dockerfile.ros`, compose `powertrain_ros` 서비스, `sync_check_msgs.sh`.
- [ ] **4-2. 빌드·그래프 합류** — 컨테이너에서 `colcon build` → `ros2 interface show` 5종 →
  `ros2 topic echo /arm_status`·`/detected_objects` 로 그들 발행 실수신.
- [ ] **4-3. 왕복 검증(실물 FSM 상대)** — `bringup` 실행: `/chassis_mode` 발행 → 그들 arm_fsm
  LOCKED/언락 전이 로그 확인 / `/arrival_status ARRIVED_PICKUP` → IDLE→PERCEIVE 전이 확인 /
  우리 노드가 `/arm_status` 스트림 수신.
- [ ] **4-4. 계약 확정** — 미결 2건 확정, contract.py·이 문서 갱신, 팀 공유.

## 범위 밖 (다음)

- **WP5**: `powertrain_ros` 노드에 `chassis.ChassisManager` 연결 — ROS `(v,ω)`·핸드셰이크를
  실제 10모터 구동에 물림. 우리 컨테이너에 python-can·can0 이미 준비됨. can0 socketcan
  접근 검증 필요(net=host 라 커널 레벨 존재, python-can AF_CAN 바인딩).
- **WP7**: D435i 단일 점유 해소(원본 재발행 or realsense-ros 통일) — 로봇팔 팀과 협의.
