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
  `docker/Dockerfile.ros`, compose `powertrain_ros` 서비스, `sync_check_msgs.sh`. (커밋 `99f7bb5`)
- [x] **4-2. 빌드·그래프 합류** ✅ (2026-07-07) — `powertrain-sw:ros` 빌드(python-can 4.6.1),
  `colcon build` 2/2, `ros2 interface list` 5종 인식. **그래프 합류: 우리 컨테이너에서 그들
  실물 노드 8개(`/arm_fsm_node`·`/perception_node`…) 전부 관측** — 분리 컨테이너 간 DDS 합류 확인.
- [~] **4-3. 왕복 검증(실물 FSM 상대)** — **양방향 배달 ✅**: 팔→우리 `bringup` 이 `/detected_objects`
  실수신(3Hz, 물체 0=카메라 유휴) · 우리→팔 `/chassis_mode(DRIVING)` 가 그들 ros2_humble 컨테이너에
  도달(echo 확인). **미완(안전상 팀 협의 후)**: `ARRIVED_PICKUP` 은 실제 팔 구동을 유발하므로
  단독 발사 보류 → 로봇팔 팀과 합동으로 `ARRIVED_PICKUP→[PERCEIVE…DONE]→재출발` 풀 핸드셰이크
  1사이클. (`/arm_status` 는 그들 FSM 이 상태전이 때만 발행 → IDLE 정지 중엔 무발행이라 수동관찰 불가)
- [ ] **4-4. 계약 확정** — 미결 2건 확정, contract.py·이 문서 갱신, 팀 공유.

### 검증 로그 (2026-07-07)

```
colcon build         : robot_arm_msgs 15.4s + powertrain_ros 1.9s = 2/2
ros2 node list(우리) : /arm_fsm_node /perception_node /moveit_dynamixel_bridge /stream_node … (그들 8개)
팔→우리              : ← /detected_objects 6~7프레임/2s (최다 0개 물체)
우리→팔              : ros2_humble 에서 echo /chassis_mode → mode: DRIVING, frame_id: base_link
QoS                  : /detected_objects RELIABLE/VOLATILE 양측 일치
```

## 범위 밖 (다음)

- **WP5**: `powertrain_ros` 노드에 `chassis.ChassisManager` 연결 — ROS `(v,ω)`·핸드셰이크를
  실제 10모터 구동에 물림. 우리 컨테이너에 python-can·can0 이미 준비됨. can0 socketcan
  접근 검증 필요(net=host 라 커널 레벨 존재, python-can AF_CAN 바인딩).
- **WP7**: D435i 단일 점유 해소(원본 재발행 or realsense-ros 통일) — 로봇팔 팀과 협의.
