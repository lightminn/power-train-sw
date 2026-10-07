# 로봇팔↔파워트레인 통합 개발 계획

> 이 문서는 Claude Code가 이 저장소에서 작업할 때 참고하는 프로젝트 컨텍스트 파일.
> 인식 소스 단일화(로봇팔 Jetson YOLO) + 위치/라벨 통합 전송을 위한 커스텀 msg 패키지 기반 개발 계획.
> 파워트레인 자율주행 방식(레인 추종, Nav2 미사용) 연동 인터페이스 포함.
> **문서 완료 목표: 7/19**
> 최종 업데이트: 2026-07-04 (커밋 `3bed8bd` — HW-2~6 실하드웨어 테스트: 브릿지 부재 서보 방어, 손목 카메라 TF, 디버그 스트리밍 추가. 이전: 2026-06-25 파워트레인 자율주행 회신 + 비전 파이프라인 실증 스펙 반영)

## 워크스페이스 구조

- ROS2 워크스페이스: `~/ros2_ws`, `~/robot_arm_ws/src`
- 기존 패키지: `robot_arm_description`, `robot_arm_moveit_config`
- 신규 생성 예정: `robot_arm_msgs` (메시지 전용 패키지)

---

## 0. 핵심 결정 사항

- **인식 소스 단일화**: 로봇팔 Jetson의 YOLO 하나만 인식 수행 → 파워트레인은 좌표/라벨만 구독 (컴퓨팅 중복 제거)
- **커스텀 msg 채택**: 위치(Pose) + 라벨(class_name) 같이 보내야 하므로 PoseStamped/PoseArray 대신 커스텀 메시지 패키지 `robot_arm_msgs` 사용
- **픽 대상도 라벨 필요** → 픽 대상/관찰 대상 모두 동일 메시지 타입으로 통일
- **네트워크**: Jetson 와이파이 연결로 진행. 단, 멀티캐스트 차단 가능성 있어 사전 테스트 + Discovery Server 백업 플랜 유지
- **미정 스펙은 우리가 정해서 이후 통보**

## 0-1. 전송 방식 확정 (2026-06-25)

- 로봇팔·파워트레인 모두 **동일한 Jetson 단일 보드**에서 구동 (2026-06-25 재확정). 최종 목표는 Jetson 단독 무선 주행(외부 노트북 비의존).
- 파워트레인이 별도 컨테이너(`powertrain_jetson`)에서 자체 YOLO+Depth 파이프라인을 돌리던 건 **단일 보드로 합치기 전 개발/검증용 환경**이었던 것으로 정리. 단일화 이후엔 컨테이너 자체를 가져가는 게 아니라 그 안의 **검출 로직만** 로봇팔 Jetson의 인식 노드로 포팅(6번 참고).
- 따라서 노드 간 데이터 경로는 전부 **ROS2 토픽**(같은 머신 내 통신). 머신 경계가 없으므로 UDP 전송 레이어는 실주행 경로에서 **사용 안 함**.
- 기존 `yolo_depth_3d.py`의 UDP 송신(`CoordSender`)·`recv_yolo3d.py`는 원래 **개발 중 노트북 모니터링용 디버그 도구**였음. 폐기하지 않고, 인식 노드의 **옵션 디버그 출력 플래그**로 남겨 필요 시에만 사용.
- 인식 노드 재작성 시 `yolo_depth_3d.py`의 검출 로직(YOLO + depth deprojection) 재활용. 좌표는 depth→color 변환 거친 **color optical frame** 기준(REP-103 optical 컨벤션) 유지, `DetectedObjectArray.header.frame_id`에 박음.
- 알고리즘 구체 스펙은 **6번 섹션** 참고.

## 0-3. pose 추정 방식 확정 — markerless (2026-06-29)

- **대회 규정상 타겟 객체에 ArUco 등 마커 부착 금지** 확인 → 마커 기반 pose 추정(ArUco `estimatePoseSingleMarkers`/solvePnP) **폐기**. 손목 보조카메라 + ArUco 6DOF 구상도 중단.
- 대체: **markerless pose**. Translation = YOLO **segmentation** 마스크 centroid의 depth median deproject(6번 알고리즘 그대로). Orientation = 마스크 (u,v) 픽셀 **2D PCA** 주축 각도를 optical Z축 yaw 로 근사 → quaternion. (2D PCA 채택 이유: D435i depth 노이즈/flying pixel에 강건, yaw 1축 coarse 근사 목적에 충분.)
- 카메라 내부파라미터는 RealSense 스트림 프로파일에서 직접 취득 → 별도 calibration yaml 불필요.
- 제거된 것: ArUco 코드, `camera_calibration.yaml`, Phase 1 더미 스켈레톤(`dynamixel_control/perception_node.py`). 복원 필요 시 git 이력 참조.

## 0-2. 파워트레인 자율주행 방식 (2026-06-25 회신 반영)

- **Nav2 안 씀**. 경진대회 트랙이 폭 ~1m 고정 레인 구간(펌프트랙 / 극한 코리도)이라 지도 기반 길찾기 불필요 → "경로 = 레인", 레인 보고 따라가는 **레인 추종** 방식.
- 흐름: 카메라/라이다로 앞쪽 레인+장애물·정지선·신호등 인식 → 방향/속도 결정 → 4WS 조향+구동(CornerModule)으로 전달. 코너·험지는 자동 감속.
- 상황별 모드 전환: 평소(레인 추종) / 신호 빨강·정지 / 신호 초록·출발 / 험지·계단(저속) / 미션 지점(정지·인식) / 선도차 추종 구간(앞 로봇 추적). 모드 전환은 카메라/라이다가 정지선·신호·마커 인식해서 트리거.
- **키네마틱스 + odometry는 파워트레인 단독 구현** — 4WS 애커만 레이어를 기존 CornerModule 위에 얹음, odom은 wheel 회전량+IMU로 단거리만(레인 추종이라 정밀 위치추정 불필요). **로봇팔 쪽 작업 없음.**
- 일정: 7/19까지 방향·설계 확정, 실제 구현은 그 이후 — 본 문서 Phase 5(문서화 목표 7/19)와 일치.
- 라이다·Depth는 양쪽이 "공유"한다고 했음 → 동일 물리 센서인지, 마운트 위치만 다른 별도 센서인지는 **오픈 이슈 5번**에서 확인 필요. 어느 쪽이든 좌표계(TF)만 맞으면 양쪽 다 그대로 받아 씀.

---

## 1. 메시지 인터페이스

새 패키지 `robot_arm_msgs` 생성 → 양쪽 팀이 공통 빌드/의존.

### `DetectedObject.msg` — 객체 하나

```
int32 class_id
string class_name
float32 confidence
geometry_msgs/Pose pose            # markerless: position=depth median, orientation=2D PCA yaw
sensor_msgs/RegionOfInterest bbox  # 2D bbox, 디버깅/시각화용 (옵션)
```

### `DetectedObjectArray.msg` — 한 프레임 인식 결과 전체

```
std_msgs/Header header             # frame_id = optical frame 이름, stamp
DetectedObject[] objects
```

**설계 포인트**

- `header`는 array 레벨에 하나만 (같은 프레임·같은 시각). frame_id에 optical frame 이름 박아서 frame 통일 이슈를 메시지 설계로 흡수
- pose는 stamp 없는 `Pose`. 시각 정보는 array header가 보유
- bbox는 RViz 인식 검증용. 안 쓰면 제거 가능

### `ArrivalStatus.msg` — 파워트레인 → 로봇팔 도착/상태 (권장)

```
std_msgs/Header header
int32 mission_id      # 어느 미션 지점인지
string status         # 예: ARRIVED_PICKUP, ARRIVED_DROP ...
```

> 단순 Bool은 어느 지점/몇 번째 시퀀스인지 구분 불가 → FSM 분기 위해 mission_id + status 필요

### `ChassisMode.msg` — 파워트레인 → 로봇팔 주행 모드 (신규 제안)

```
std_msgs/Header header
string mode    # DRIVING, CORNERING, ROUGH_TERRAIN, MISSION_STOP, FOLLOW_LEAD
```

> `ArrivalStatus`와 역할 분리: `ChassisMode`는 "지금 차체가 물리적으로 어떤 상태인지"(연속적) 신호 — 코너링/험지일 때 로봇팔이 **자세 락**(현재 관절각 유지)해서 흔들림으로부터 보호. `ArrivalStatus`는 "어느 미션 지점에 도착해서 뭘 해야 하는지"(이산 이벤트) 신호. `MISSION_STOP`이 들어오면 로봇팔 FSM이 깨어나고, 그다음 `ArrivalStatus`로 구체 작업을 받는 흐름으로 제안.

### `ArmStatus.msg` — 로봇팔 → 파워트레인 작업 상태 (신규 제안)

```
std_msgs/Header header
int32 mission_id
string status   # PERCEIVING, PLANNING, EXECUTING, DONE
```

> 기존 설계엔 파워트레인 → 로봇팔 방향만 있고 **로봇팔 → 파워트레인 완료 신호가 없었음**. `status=DONE`을 파워트레인이 받아야 재출발(레인 추종 재개) 가능 — 빠뜨리면 파워트레인이 영원히 대기함.

---

## 2. 토픽 구조

인식 소스: **로봇팔 Jetson YOLO 하나**. 거기서 두 토픽으로 분기.

| 토픽 | 타입 | 내용 | 구독자 |
| --- | --- | --- | --- |
| `/detected_objects` | `DetectedObjectArray` | 보이는 거 전부 (신호등/정지선/마커 + 관찰 대상) | 파워트레인(레인 추종 트리거), 로봇팔 FSM |
| `/pick_target` | `DetectedObject` | 선별된 "집을 대상" 하나 | 로봇팔 FSM (MoveIt) |
| `/arrival_status` | `ArrivalStatus` | 도착/미션 상태 | 로봇팔 FSM |
| `/chassis_mode` | `ChassisMode` | 주행 모드(평소/코너/험지/미션정지/추종) | 로봇팔(자세 락 트리거) |
| `/arm_status` | `ArmStatus` | 팔 작업 상태(인식중/계획중/실행중/완료) | 파워트레인(재출발 트리거) |

- `/detected_objects`: 인식 노드가 매 프레임 그대로 publish
- `/pick_target`: 픽 대상 클래스 선별 → 하나만 publish. **QoS `transient_local`(latched)** 로 줘서 "도착 → 집어" 타이밍에 최신 타깃 안 놓치게
- 분리 이유: 로봇팔 FSM이 매번 배열 필터링 안 해도 되고 픽 트리거 명확
- `/chassis_mode`·`/arm_status`는 위 메시지 설계 포인트 참고 (역할 분리 이유 동일하게 적용)
- **제어-플레인 토픽/액션 (Phase 3 구현, 팀 간 인터페이스 아님 — 로봇팔 내부):** `moveit_dynamixel_bridge`가 `/joint_states`(position+**effort=raw 전류**) publish + `/arm_controller`·`/gripper_controller`의 `follow_joint_trajectory` 액션 서버 제공(2026-07-04: 토크 활성화 성공한 ID만 SyncRead 등록해 부재 서보에 방어적). `arm_fsm`은 MoveIt `move_action`(MoveGroup)로 팔, `/gripper_controller`로 그리퍼 구동 — **단, 2026-07-05 HW-7 실측: URDF가 아직 3축(`joint_1~3`)만 반영돼 MoveIt 6DOF IK가 `NO_IK_SOLUTION`(원천 불가)이라, 기본값은 `ik_mode='analytic'`(FK+수치 자코비안 위치-only 3DOF IK, `/arm_controller/joint_trajectory` 직접 publish)이고 URDF 5축 확장 후 `ik_mode='moveit'`로 전환 예정.** 카메라 TF는 `camera_tf.launch.py`(static)가 전방 RGB-D `base_link↔camera_color_optical_frame` + (신규, 2026-07-04) 손목 RGB `base_link↔wrist_camera_link`(홈 포즈 기준 placeholder) 발행. 인식 노드는 원격 모니터링용 `/perception/debug_image`(bbox·마스크·거리 오버레이)도 발행하며, `stream_node`가 이를 H.264/SRT로 PC에 스트리밍(신규, 2026-07-04).

---

## 3. 단계별 개발 계획

### Phase 0 — 인터페이스 합의 (미팅 전~미팅)

- [ ] `robot_arm_msgs` 메시지 5개 확정 (DetectedObject / DetectedObjectArray / ArrivalStatus / ChassisMode / ArmStatus)
- [ ] 노션에 기존 토픽 스펙 있는지 스캔 → 있으면 이름/타입 맞추고 없으면 우리 안으로 통보
- [ ] 도착/상태 토픽 타입 합의 (status 문자열 enum 네이밍까지 — 대소문자 불일치 주의)
- [ ] optical frame 이름 실제값 확정 → array header frame_id에 박을 값
- [ ] `ChassisMode`/`ArmStatus` 신설 합의 — 코너링/험지 자세 락, 팔 작업완료→재출발 트리거용
- [ ] 라이다/Depth 카메라가 차체-팔 간 동일 물리 센서인지 확인 (TF 트리 구조에 영향)

### Phase 1 — 메시지 + 스켈레톤

- [ ] `robot_arm_msgs` colcon build 통과 (양쪽 PC에서)
- [ ] 인식 노드가 더미 데이터로 `/detected_objects`, `/pick_target` publish (타입 검증)
- [ ] 파워트레인/로봇팔 양쪽에서 `ros2 topic echo` 수신 확인 (네트워크 테스트 겸)

### Phase 2 — 인식 파이프라인 실제 연결

- [x] YOLO **segmentation** 추론 → class_id/name/confidence/bbox/mask 채우기 (6번 섹션 참고)
- [x] markerless pose → Pose 채우기: translation=마스크 centroid depth median deproject, orientation=2D PCA yaw (0-3 참고). *빌드+런타임 검증 완료(2026-06-29, test 모드 bus.jpg). translation 실측은 RealSense 하드웨어 필요.*
- [x] 픽 대상 선별 로직 (`/pick_target`): `pick_classes` 화이트리스트 + `pick_min_conf` 통과 + depth 조건(`require_depth`) 만족 객체 중 confidence 최고 하나, transient_local QoS. *빌드+런타임 검증 완료(2026-06-29, test 모드 require_depth=false). → 다음은 Phase 3 FSM.*
- [ ] frame_id가 실제 TF tree에 존재하는지 확인 (`ros2 run tf2_tools view_frames`)
- [ ] (참고) 4WS 키네마틱스 + odom은 파워트레인 단독 구현 — 로봇팔 쪽 구현 항목 없음

### Phase 3 — FSM 통합

- [x] 로봇팔 FSM: 도착 신호 수신 → `/pick_target` 읽기 → MoveIt 픽 시퀀스 *(arm_fsm_node.py, 12상태, MoveIt 단일 경로 '가'. 빌드+mock 전이 검증. 실서보 연동만 남음)*
- [x] 로봇팔 FSM: `/chassis_mode`가 CORNERING/ROUGH_TERRAIN/FOLLOW_LEAD이면 자세 락(`LOCKED`: 진행 모션 취소+홀드), DRIVING 복귀 시 직전 상태로 언락 *(스켈레톤 구현, 복구 정밀화 TODO)*
- [x] 로봇팔 FSM: 픽 완료 시 `/arm_status`(status=DONE) publish *(구현. status enum은 파워트레인 합의 후 확정)*
- [x] **실하드웨어 경로 (브릿지)**: `moveit_dynamixel_bridge`가 `/arm_controller`·`/gripper_controller` 액션 + `/joint_states`(position+effort) 제공 → MoveIt/FSM 실서보 구동 경로 완성. effort로 파지/DROP 판정.
- [x] **TF**: `camera_tf.launch.py`로 `camera_color_optical_frame→base_link` 연결 (MoveIt 목표 변환).
- [ ] 파워트레인: `/detected_objects`에서 신호등/정지선/마커만 필터 → 레인 추종 트리거 *(파워트레인 팀 작업)*
- [ ] 상태 토픽 핸드셰이크 (도착 → 픽 완료 → 다음 미션 신호) 흐름 검증 *(실하드웨어 통합 테스트에서 — HW 인수인계 문서 참고)*
- [ ] **실측 캘리브** (HW 연결 후): 전류 임계(`grasp/drop_effort_thresh`), 그리퍼 틱(`gripper_open/close_tick`), `gripper_ids`, 카메라 오프셋(`cam_*`).

### Phase 4 — 네트워크 + 통합 테스트

- [ ] Jetson 와이파이 + `ROS_DOMAIN_ID` 통일
- [ ] 멀티캐스트 사전 테스트, 안 되면 Fast DDS Discovery Server 전환
- [ ] 두 PC 물려서 end-to-end (도착→인식→픽→다음) 리허설

### Phase 5 — 문서화 (7/19 목표)

- [ ] 메시지/토픽 인터페이스 표 문서화
- [ ] 인식 아키텍처 다이어그램 (단일 YOLO 소스 → 양쪽 컨슈머)
- [ ] 제출 문서에 반영

---

## 4. 미션 시나리오 메모

- **픽**: 한 번에 하나만 집음 → `/pick_target` 단일 객체로 충분
- **관찰만**: 여러 개를 보기만 하는 상황 + 신호등/정지선/마커 → `/detected_objects` 배열로 전부 전달, 집지 않음
- 신호등/정지선/마커 트리거도 결국 단일 YOLO 결과를 파워트레인이 구독하는 구조 → 별도 인식 파이프라인 없음
- **평소 주행/코너/험지**: 팔은 항상 자세 락 — 진동·충격 보호, 제출 문서 안전성 항목으로도 활용 가능
- **선도차 추종 구간(앞 로봇 따라가기)**: `/chassis_mode=FOLLOW_LEAD` 주행, 팔은 별도 동작 없이 자세 락 유지(기본 IDLE)

---

## 5. 오픈 이슈 (미팅에서 확정)

- [ ] 노션 기존 토픽 스펙 존재 여부 → Claude Code로 워크스페이스/리포 스캔
- [ ] optical frame 실제 이름 (URDF vs 브릿지 코드 파라미터 대조) — *현재 `camera_tf.launch.py`·perception·FSM 모두 `camera_color_optical_frame` placeholder 사용. 실 RealSense frame 이름은 HW 연결 후 `tf2_tools view_frames`로 확정.*
- [ ] status 문자열 enum 네이밍 합의
- [ ] 멀티캐스트 가능 여부 (와이파이 환경)
- [ ] `/chassis_mode`(MISSION_STOP) → `/arrival_status`(mission_id+status) 트리거 순서/타이밍 확정 — 제안한 "모드로 깨우고 상태로 구체 작업" 흐름에 파워트레인도 동의하는지
- [ ] 라이다·Depth 센서 공유 방식 확인 (동일 물리 센서 vs 마운트 위치 다른 별도 센서) → `base_link`/`arm_base_link`/camera optical frame TF 관계 결정
- [ ] 자세 락 구현 방식 합의 (현재 각도 유지 명령 vs 별도 안전 자세로 이동) — *FSM은 현재 `LOCKED`에서 "진행 모션 취소 + 현재 각도 홀드"(브릿지 torque 유지)로 구현. 별도 안전 자세 이동으로 바꿀지 합의 필요.*
- [ ] 파워트레인 Orin Nano(카메라·연산)를 로봇팔 Jetson 단일 보드로 합치는 마이그레이션 범위 확정 (카메라 마운트 위치 1개로 통일, `powertrain_jetson` 컨테이너 정리/제거 시점)

---

## 6. 참고 — 파워트레인 비전 파이프라인 실증 스펙 (2026-06-24 실카메라 검증)

0-1에서 결정한 "검출 로직 재활용"의 구체 스펙. 출처: 파워트레인 레포 `motor_control/vision/yolo_depth_3d.py` (컨테이너 `powertrain_jetson`, Jetson Orin Nano — 단일 보드 확정 전 파워트레인 단독 개발/검증 환경).

- **카메라**: RealSense D435i, color+depth 848×480, 30fps
- **추론**: YOLO26n → TensorRT FP16 엔진(최초 1회 빌드 5~10분, 이후 `.engine` 캐시)
- **좌표계**(camera optical frame 기준): X=오른쪽, Y=아래, Z=전방[m]. 방위각 `az=atan2(X,Z)`(우+), 고도각 `el=atan2(-Y,Z)`(상+)
- **depth 추정**: 검출 박스 중앙 1/3 영역 패치의 **중앙값**(median) 사용 — 단일 픽셀은 0/튐이 많아서 배제. 측정 불가 시 `no-depth` 처리(잘못된 좌표 전파 방지)
- **성능 최적화 핵심**: 전체 depth↔color 정렬(align)을 생략하고 검출별 color→depth 투영(2.9ms)만 수행 → Orin Nano 기준 **5.5fps → 30fps**. 좌표 오차는 정렬 대비 평균 11mm로 센서 노이즈 이내
- **실증 결과**: person 등 다중 클래스 동시 검출, d=1.6~1.8m 환경에서 conf 0.91~0.94, frame_age 65~70ms

→ Phase 2 재작성 시 위 알고리즘(박스 중앙 패치 median depth, 정렬 생략, optical frame 컨벤션) 그대로 가져오면 됨. 송신부(SRT 영상/UDP 좌표 분리 전송)는 가져올 필요 없음 — 0-1 결정대로 동일 Jetson 내 ROS2 토픽으로 대체.
