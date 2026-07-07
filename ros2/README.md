# ros2/ — 파워트레인 ROS2 워크스페이스

로봇팔 팀(`ksp118/extreme-robot`)과 **분리 개발**하는 우리 ROS2 층. 각 팀이 자기
노드·자기 컨테이너를 갖고, **공유하는 것은 메시지 계약(`robot_arm_msgs`)뿐**이며 통신은
DDS(host net, DOMAIN 0)로만 한다.

```
로봇팔: ros2_humble 컨테이너 (그들 소관)          우리: powertrain_ros 컨테이너 (이 워크스페이스)
  /arm_fsm_node /perception_node …                 powertrain_ros 노드
  robot_arm_msgs (그들 빌드)                        robot_arm_msgs (벤더링, 우리 빌드)
        └──────────────── DDS (host net) ────────────────┘
              /arm_status ·/detected_objects  ↔  /chassis_mode ·/arrival_status
```

## 구조

```
ros2/
├── src/
│   ├── robot_arm_msgs/      벤더링 사본(정본=ksp118). VENDORED.md 참조. ⚠️ 원본 아님
│   └── powertrain_ros/      우리 노드 (ament_python) — bringup_node(WP4 스켈레톤)
│                            + contract.py(계약 문자열 단일 출처)
└── scripts/
    └── sync_check_msgs.sh   벤더 msg 가 로봇팔 정본과 일치하는지 드리프트 체크
```

**설계원칙: ROS2 는 껍데기.** 제어 로직은 `../motor_control/`(ROS 없는 순수 파이썬,
pytest)에 있고, 노드는 그걸 import 해 토픽에 붙이기만 한다.

## 빌드 · 실행 (Jetson)

```bash
# 0) ssh 접속 직후 — 레포로
cd ~/power-train-sw

# 1) ROS 컨테이너 기동 + 진입 (최초 1회 이미지 빌드: ros:humble-ros-base pull)
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
docker exec -it powertrain_ros bash

# 2) (컨테이너 안) 빌드 + 소스
cd /workspace/ros2
colcon build
source install/setup.bash

# 3) 노드 실행
ros2 run powertrain_ros bringup
#   락 모드로 발행 시:  ros2 run powertrain_ros bringup --ros-args -p mode:=CORNERING
```

✅ 기대: `powertrain_ros 브링업 …` 로그 + 로봇팔 FSM 이 떠 있으면 `← /arm_status …`
수신 로그. `ros2 topic list` 에 `/chassis_mode`·`/arrival_status` 발행 확인.

## 계약

`src/powertrain_ros/powertrain_ros/contract.py` 가 문자열 어휘의 단일 출처.
미결 2건(MISSION_STOP·락 해제 순서)은 `docs/plans/2026-07-07-wp4-ros2-roundtrip.md` 참조.
