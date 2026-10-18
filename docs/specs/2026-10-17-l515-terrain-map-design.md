# L515 주행 경로 2.5D 지형 지도 설계

- 작성일: 2026-10-17 (r1 → Codex 1회차·agy 반영 r2 → 2회차 반영 r3 → 3회차 반영 r4 → 4회차 반영 r5 → 5회차 반영 r6 → 6회차 반영 r7 → 7회차 반영 r8 → 8회차 반영 r9 → 9회차 반영 r10, 2026-10-18 → 기준면 기울기 표시 추가 r11, 2026-10-18 → 콘솔 표시 정리 r12, 2026-10-18 → 바퀴 헛돎 합의 r13, 2026-10-19)
- 브랜치: `feat/l515-terrain-map` (기준 `feat/l515-terrain-overlay` `99e4a46` — draft PR #15 위에 쌓는다)
- 출발점: 2026-10-16 팀 회의 업무 정리 — "LiDAR point cloud 수집 → 3D 맵 디지털 재구축 → 장애물 표시 → 정보 구성".
  10-23까지 광민(사용자)이 맵 재구축 + 장애물 인식을, 선이가 정보 구성(표시 기준)을 맡는다.
- 상태: 설계 — Codex 교차검증 **수렴**(10회차, 2026-10-18), agy 독립 검토 반영. r13 바퀴 합의는 Codex 6회차 수렴·agy 반영(2026-10-19, §12)

## 1. 목적과 범위

L515 depth와 바퀴·IMU 위치 추정으로 **주행한 경로를 따라 누적되는 2.5D 지형 지도**를 Jetson에서 만들고,
칸마다 주행 가능 / 돌출 장애물 / 도랑·낙차 / 관측 부족 / 미관측으로 분류해 운용 콘솔 "지형 지도" 페이지에 위에서 본 음영 지도로 표시한다.

- **운용자 시각 보조 전용이다**(PR #15와 같은 원칙). 주행 제어·정지·속도 제한·`SafetyInterlock`·US-100에 입력을 주지 않는다.
  지도 노드는 ROS 토픽을 **구독만** 하고 아무것도 발행하지 않는다. 위치 추정 코어는 노드 안에서만 돈다(r7, §4.4·§8.2).
- 2026-10-17 사용자 결정:
  - 지도 범위는 **주행 경로 누적 지도**(로봇 주변 이동 창이 아니다).
  - 접근안 **A**: 바퀴·IMU 위치 추정 + 누적 2.5D 격자. **B**(지도 맞춤 위치 보정)는 시간이 남으면 바로 착수할 수 있게 계약·시험 장치를 A에서 미리 만든다(§4.7).
  - 콘솔은 **기본 지도 패널 + 데이터 계약**까지. 장애물 정보의 구분 표시·범례·정보 구성의 다듬기는 선이 파트다.
  - 10-23 완료 기준은 **로봇 실시간 + 녹화 재생 둘 다**.
  - 판정은 원본 칸으로, 표시는 보간 + 음영으로 한다. **모든 수치는 파라미터로 남겨 바꿀 수 있게 한다.**
- 2.5D의 뜻: 바닥 칸마다 값 하나. 아래가 빈 구조(책상 밑, 터널)는 표현하지 않는다(비목표).

### 전제 (적어 두는 가정)

- **판정은 프레임 안에서 한다(r4, §4.5)**: 한 depth 프레임 안에서는 그 프레임의 바닥 평면이 공통 기준이다. 그래서 칸마다 "그 프레임 바닥 대비 높이" `h`로
  돌출(`h ≥ +0.08 m`)·도랑/낙차(`h ≤ −0.08 m`)·주행 가능을 정하고, 지도에는 **분류 결과**를 최근 3회 다수결로 쌓는다.
  - 프레임마다 바닥 평면이 다르므로 서로 다른 프레임의 `h`를 더하거나 평균하지 않는다. r1–r3의 높이 누적(평균·δz 연쇄)은
    단차 윗면 소실, 치운 물체 자리의 바닥 상승, 초기 융합 정지, 계단의 주행 가능 오판 반례가 이어져 폐기했다(§12 ② 1–3회차).
  - 의미: 칸 분류는 **그 칸을 마지막으로 본 위치에서 넘을 수 있는가**다. 넓은 윗면은 아래에서 보면 돌출, 올라서서 보면 주행 가능이며,
    윗면 위에서 본 아래 바닥은 낙차(도랑 코드)로 보인다. 다수결이라 새 관점이 두 번 겹치면 바뀐다.
  - 경사로는 평면 앞쪽이 점점 높아지므로 일정 거리부터 돌출로 보인다(PR #15 오버레이와 같다). 올라서서 보면 주행 가능으로 바뀐다.
  - 표시용 높이(음영)는 칸을 마지막으로 본 프레임 기준 `h`다. 서로 다른 프레임에서 본 이웃 칸 사이에 음영 이음매가 생길 수 있다(표시 전용).
- **위치 오차는 쌓인다.** 바퀴 미끄러짐(로커-보기·험지)과 IMU 방향 오차로 멀리 갈수록 지도가 휘거나 겹친다.
  A는 보정하지 않고, 누적 거리·자세 끊김 횟수·칸 나이를 함께 보여 오해를 줄인다(§4.10). B가 이 오차를 줄인다.
  위치는 지도 전용 **평면 추측 항법**(`terrain_map/odometry.py`)으로 구한다: 방향 = 차체 z축 자이로 적분, 차체 속도 = 자이로 ω와 유효 바퀴의 조향각·위치로 푼 최소제곱(§4.4).
  한두 바퀴만 헛도는 표본은 바퀴 합의(r13, 기본 끔 — 실기 잔차 측정 뒤 켬)로 그 바퀴를 빼고 푼다(§4.4). 여러 바퀴가 같은 비율로 미끄러지는 공통 미끄러짐은 모형에 없고,
  차체가 기울어진 채 선회하면 방향 오차가 생긴다(롤 20°에서 약 6 %, §10).
- **지도는 과거 관측 기록이다.** 칸은 마지막으로 본 순간의 상태이고, 차체 뒤·옆 사각지대는 갱신되지 않는다.
  오래된 칸은 흐리게 그리고, 지도를 실시간 장애물 감지로 쓰지 말라는 문구를 콘솔에 고정한다(agy 4·6).
- **관측 범위**: L515 앞 0.3–3.0 m(기본). 멀수록 평면 기울기 오차가 높이 오차로 커지므로(1° → 3 m에서 5 cm) 다수결과 범위 상한으로 줄인다.
- **조명·재질**: L515는 적외선이라 직사광선·물웅덩이·젖은 면·검은/반사 재질에서 깊이가 사라진다. 깊이가 없는 칸은 "미관측"이며
  **미관측은 평지도 도랑도 아니다**(배경색, 별도 범례). 시연 장소의 조도를 미리 확인한다(agy 1·7).
- **바닥 평면은 로봇의 지지면이어야 한다(r5, §4.2)**: 매 프레임 PR #15 판정기(`TerrainDetector`)의 RANSAC 평면을 쓰되, 평면을 새로 맞춘 프레임(`FRESH`)이고
  **로봇 바로 앞 근거리 띠의 점이 그 평면 위에 있을 때만** 지도에 넣는다. 판정기는 화면 아래 절반의 지배적 평면을 고르므로, 앞에 큰 도랑이 있으면
  도랑 바닥을 평면으로 잡아 도랑 전체를 주행 가능으로 만든다(Codex 4회차, 2.4 m 도랑에서 평면 offset 0.70 m, 도랑 점 43,173개 모두 코드 0).
  평면은 그 프레임에서 보인 바닥에 맞춘 것이라 차체 피치·롤과는 무관하다. 근거리 띠가 바닥이 아니면(바로 앞이 도랑·바위) 그 프레임은 버려진다.
- **장착 기하는 측정 전이다**: 운용 URDF(`jetin_rover.urdf.xacro`)의 L515 값(x 0.30, z 0.35, 기울기 없음)은 자리표시 값이고,
  보관 CAD(`simulation/jetin_rover_description`)는 base_link 기준 depth 원점 (0.348, 0, 0.029) m·20° 하향이다. 기본값은 CAD를 쓰고 실차 측정(§7.3)으로 교체한다.

### 비목표

- 완전 3D(voxel·점군) 지도, 위가 막힌 구조 표현, 루프 클로저·전역 최적화.
- 프레임 사이 일관된 절대 고도·경사로 기울기 지도(확장 여지만 둔다, §11).
- 지도 기반 경로 계획·자율주행 입력, 실시간 충돌 경고.
- 콘솔에서 로봇으로 보내는 지도 명령(초기화 등). 콘솔은 관측 수신 전용 헌장을 지킨다(§4.9).
- 입체 메시 보기(후순위, §11).
- 기존 추정 코어(`state_estimation.py`)의 기울어진 선회 보정(공용 코어라 이번 범위 밖, §10).

## 2. 기존 자산과 재사용

| 자산 | 위치 | 이 설계에서 |
|---|---|---|
| L515 depth·CameraInfo·IMU | Gateway `/l515/depth/image_rect_raw`·`camera_info`, `/l515/accel/sample`·`/l515/gyro/sample` | 그대로 구독. depth 헤더 stamp는 장치 시각을 ROS 시계로 사상한 값(`l515_dashboard/gateway_ros.py` `_stamp`) |
| 바닥 평면 | `terrain_overlay.detector.TerrainDetector` | 재사용. 결과에 점·평면 필드, 설정에 최소 바닥점 수 필드를 더한다(§4.2, 기존 기본값 불변) |
| 바퀴·IMU 위치 추정 | `powertrain_ros` `odometry`·`imu_tilt` 노드, `state_estimation.py` `StateEstimator` | **쓰지 않는다**(r8). 별도 노드는 발행 시각 stamp·미래 IMU 사용(Codex 6회차), 코어를 직접 써도 0.25 s IMU 신선도 창·재연결 시 공백 무시·freeze·정지 확인 전 bias 학습·조향 방식별 기하 때문에 지도 시간축과 맞지 않았다(Codex 7회차). 지도 전용 평면 추측 항법을 순수 모듈로 둔다(§4.4). 기존 코드는 바꾸지 않는다 |
| 바퀴 상태·자이로 | chassis `/wheel_states`(50 Hz, 차체 틱마다 ROS 시각 stamp — `chassis_node.py`, 바퀴 단위 `drive_turns_per_s` + = 전진), Gateway `/l515/gyro/sample` | 지도 노드가 직접 구독해 평면 추측 항법에 넣는다(§4.4). accel·조향 방식은 쓰지 않는다 |
| 세션 키·게이트 | `terrain_overlay.contract` 세션 세 값·`operator_console/terrain.py` `TerrainSessionGate` | 세션 선택 규칙만 공용 모듈로 분리하고, 순서 판정은 스트림별로 둔다(§4.9) |
| WP6 고도 격자 | `powertrain_autonomy/terrain/grid.py` | **쓰지 않는다**. 로봇 중심 고정 크기·짧은 이력용이고 자율주행 코어와 결합을 만들지 않기 위해서다 |
| 녹화 | `ros2 bag`·`rosbag2_py`(Jetson·로컬 시험 이미지 모두 `ros:humble-ros-base`, 2026-10-17 로컬 확인) | 녹화·결정적 재생(§6) |

## 3. 아키텍처

```
[Jetson · powertrain-sw:ros · compose 서비스 powertrain_terrain_map (opt-in 프로필 terrain-map) — 구독만, ROS 발행 없음]
 Gateway ─ depth·camera_info ─────────────┐
 Gateway ─ gyro ──────────────────────────┤
 chassis ─ /wheel_states ──────────────────┤
                                           ▼
   terrain_map_node (얇은 ROS 어댑터)
     └ terrain_map.runtime.MapRuntime (ROS 없음)
         ├ odometry.PlanarOdometry — 워터마크까지 확정한 자이로·바퀴 표본으로 평면 자세 표본
         ├ FrameScheduler — 처리할 depth 선택(실시간·재생 공용 규칙)
         ├ depth 솎기 + TerrainDetector — 바닥 평면 (FRESH + 지지면 검사)
         ├ scan.build_local_scan — 프레임 → base 수평 좌표 칸 + 프레임 안 분류
         ├ pose.PoseTrack — 자세 표본 보간·유효성
         ├ correction.PoseCorrector — A: 그대로 / B: 지도 맞춤
         ├ classmap.ClassMap — 칸별 분류 이력 다수결·전역 version
         └ contract — 상태 패킷·타일 패킷(≤1200 B)
   UDP :5010 ─▶
[운용 PC · operator_console] MapReceiver(:5010) → TileStore(TTL) → render.map_rgba → MapPanel("지형 지도" 페이지)
[노트북 · 재생] terrain_map_replay(bag 결정적 처리 — 같은 런타임) 또는 ros2 bag play + 노드(별도 ROS domain)
```

### 3.1 모듈 경계

| 모듈 | 책임 | 의존 | 실행 위치 |
|---|---|---|---|
| `terrain_map/config.py` | `MapConfig`(모든 수치 파라미터와 범위·상호 검증) | 표준 라이브러리 | 공용 |
| `terrain_map/contract.py` | 포트·스키마 상수, 상태/타일 패킷 인코딩·검증, 칸 바이트 형식, 분류 코드 | **표준 라이브러리만** | Jetson·콘솔 |
| `terrain_map/geometry.py` | 장착 변환, 바닥 평면 위 base 수평 좌표, SE(2) 연산 | numpy | Jetson |
| `terrain_map/scan.py` | depth 점 + 평면 → base 수평 좌표 칸 `LocalScan`(칸 높이·분류) | numpy | Jetson |
| `terrain_map/odometry.py` | `PlanarOdometry` — 자이로·바퀴 표본 워터마크 정렬, 바이어스(정지 시만), 적분, 공백·연속성 판정, 자세 표본 방출 | 표준 라이브러리 | Jetson·재생 |
| `terrain_map/pose.py` | `PoseTrack` — 자세 표본 `(stamp, x, y, yaw, valid)` 버퍼, stamp 보간, 유효성 | 표준 라이브러리 | Jetson |
| `terrain_map/schedule.py` | `FrameScheduler` — stamp 기준 처리 프레임 선택·pending 관리 | 표준 라이브러리 | Jetson·재생 |
| `terrain_map/correction.py` | `PoseCorrector` 프로토콜, `IdentityCorrector`(A), B 시험 장치용 합성 도구 | numpy | Jetson |
| `terrain_map/classmap.py` | `ClassMap` — 칸별 분류 이력·다수결, 표시 높이, 나이, 전역 version, 순환 송신 목록 | numpy | Jetson |
| `terrain_map/runtime.py` | 틱 논리(입력 보관·처리·송신 일정), 시계 주입 | numpy + 위 | Jetson |
| `powertrain_ros/terrain_map_replay.py` | bag을 읽어 같은 런타임(평면 추측 항법 포함)을 결정적으로 돌리는 CLI(오프라인·A/B 비교·콘솔 송신) | `rosbag2_py` + 위 | 노트북·Jetson 컨테이너 |
| `powertrain_ros/terrain_map_node.py` | ROS 어댑터(구독·실행기·UDP). 발행 없음 | rclpy + 위 | Jetson |
| `operator_console/session_gate.py` | 세션 선택 규칙(PR #15에서 분리, 동작 불변) | 표준 라이브러리 | 콘솔 |
| `operator_console/terrain_map.py` | 수신 스레드, `TileStore`, 순수 렌더 함수 `map_rgba` | `terrain_map.contract`, numpy | 콘솔 |
| `operator_console/app.py` `MapPanel` | 그리기(cairo), 따라가기·확대 | GTK/cairo | 콘솔 |

- `terrain_map`은 rclpy·gi·`powertrain_ros`·`operator_console`·`rosbag2_py`를 import하지 않는다. `terrain_overlay`(detector·camera)는 import한다.
  ROS 메시지 해석이 필요한 재생 CLI만 `powertrain_ros` 쪽에 둔다(의존 방향 `powertrain_ros` → `terrain_map`).
- 콘솔 시스템 python에는 numpy 2.5.4가 있고 scipy·cv2는 없다(2026-10-17 확인). 콘솔 쪽 코드는 numpy와 표준 라이브러리만 쓴다.

## 4. 구성 요소 설계

### 4.1 파라미터 (`MapConfig`, 전부 변경 가능)

| 이름 | 기본 | 범위 | 뜻·근거 |
|---|---|---|---|
| `cell_m` | 0.05 | 0.02–0.20 | 판정 칸 크기 |
| `tile_cells` | 16 | 8–16 | 타일 한 변 칸 수(16² × 2 B = 512 B → base64 684자, §5.4) |
| `pixel_stride` | 2 | 1–4 | **판정기 앞에서** depth를 솎는다(내부파라미터도 1/s, §4.2) |
| `ground_min_points` | 100 | 30–200 | 지도용 판정기의 최소 바닥 후보·voxel 수(오버레이 판정기는 기존 200 유지, §4.2) |
| `range_min_m`·`range_max_m` | 0.3·3.0 | 0.2–1.0·1.0–5.0 | 카메라 기준 바닥 전방 거리 사용 구간 |
| `max_abs_height_m` | 1.0 | 0.3–1.2 | 평면 기준 |h|가 이보다 큰 점은 버린다(벽·천장·먼지 구름 일부) |
| `min_points_per_cell` | 5 | 3–50 | 한 프레임에서 칸 값을 정하는 최소 점 수 |
| `cell_trim_fraction` | 0.1 | 0.0–0.4 | 칸 값 = 위아래로 `k = max(1, ⌊f·n⌋)`개씩 버린 순서통계(§4.3) |
| `obstacle_height_m` | 0.08 | 0.02–0.5 | 프레임 안 `h ≥` 이면 돌출. 도현 기준값으로 교체 예정 |
| `ditch_depth_m` | 0.08 | 0.02–0.5 | 프레임 안 `h ≤ −` 이면 도랑·낙차. 같은 이유 |
| `history_length` | 3 | 1–3 | 칸별 분류 이력 길이(다수결, §4.5) |
| `history_max_age_s` | 10 | 1–120 | 이보다 오래된 이력 표는 다수결 전에 만료(§4.5) |
| `support_band_m` | 0.5 | 0.2–1.0 | 지지면 검사 근거리 띠: 카메라 전방 `range_min_m`부터 이 길이까지(§4.2) |
| `support_tol_m` | 0.04 | 0.01–0.1 | 근거리 띠 점 높이 중앙값이 평면에서 이 안이어야 지지면 |
| `support_min_points` | 200 | 20–5000 | 근거리 띠 유효 점 최소 수(솎은 뒤 기준) |
| `support_base_height_m` | 없음(끔) | 0.05–0.80 | 평지에 섰을 때 base_link 원점의 바닥 위 높이(실측값). 설정하면 평면에서 잰 base 원점 높이 `n·o + d`가 이 값 ±`support_height_tol_m` 안이어야 지지면 |
| `support_height_tol_m` | 0.03 | 0.01–0.15 | 위 높이 검사 허용폭 |
| `support_base_variation_m` | 없음 | 0.0–0.1 | 평지·험지 정지 자세에서 실측한 base 높이 변동(±). 높이 검사를 켜려면 필수이며 `variation ≤ tol < min(obstacle_height_m, ditch_depth_m) − variation`이어야 한다(§4.2) |
| `min_observations` | 2 | 1–10 | 이보다 적게 관측된 칸은 "관측 부족" |
| `age_step_s` | 2.0 | 2.0–10 | 칸 나이 5비트 단위(31 = `31·step` 이상). 하한 2.0은 표현 범위 `31·step ≥ 60 s`(콘솔 `stale_display_s` 상한)를 보장한다 |
| `process_hz` | 5 | 1–10 | 처리 목표 주기(stamp 간격 기준, §4.8) |
| `status_hz` | 5 | 1–10 | 상태 패킷·송신 틱 주기 |
| `max_tiles_per_tick` | 12 | 1–64 | 틱당 갱신 타일 송신 상한 |
| `refresh_tiles_per_tick` | 8 | 1–64 | 틱당 순환 재송신 타일 수(0 불가 — TTL 복구에 필요) |
| `max_map_tiles` | 1024 | 64–8192 | 타일 수 상한(0.8 m 타일 → 약 655 m²). 순환 한 바퀴 = 1024/(8×5) ≈ 26 s |
| `depth_pending_max` | 5 | 2–10 | pending depth 수. `≥ ⌈pose_wait_s × 10 Hz⌉ + 1`이어야 한다(설정 검증, §4.8) |
| `pose_wait_s` | 0.4 | 0.1–0.8 | 선택된 depth가 자세를 기다리는 최대 시간(수신 단조시각 기준). `≥ lateness_margin_s + 1/(2·process_hz) + input_lag_budget_s`(설정 검증) |
| `input_lag_budget_s` | 0.25 | 0.0–0.5 | 바퀴·자이로 입력이 depth보다 늦게 도착해도 자세를 기다려 주는 허용 지연(위 검증의 항) |
| `pose_max_gap_s` | 0.06 | 0.02–0.2 | depth stamp를 감싸는 두 자세 표본(바퀴 표본 시각) 간격 상한 |
| `min_valid_wheels` | 4 | 1–6 | 전진량 계산에 쓸 유효 구동 바퀴 수 하한(`drive_stale` 또는 `steer_stale`이면 무효 — `odometry` 노드와 같은 판정) |
| `wheel_radius_m` | 0.10356 | 0.05–0.2 | 바퀴 반지름. 노드가 `chassis.kinematics.default_geometry()`에서 반지름·바퀴 위치 `(name → x, y)`를 읽어 넘긴다(`terrain_map`은 `motor_control`을 import하지 않음) |
| `lateness_margin_s` | 0.05 | 0.0–0.15 | 워터마크 = 두 스트림 최신 stamp 중 작은 쪽 − 이 값. 그 이전 표본만 확정 처리, 그보다 늦게 온 표본은 버리고 셈 |
| `gyro_gap_epoch_s` | 0.06 | 0.02–0.2 | 확정 구간의 자이로 표본 간격이 이를 넘으면 그 사이 회전을 모르므로 연속성 상실 → 새 epoch |
| `wheel_gap_invalid_s`·`wheel_gap_epoch_s` | 0.06·0.30 | — | 바퀴 표본 간격(또는 유효 바퀴 < `min_valid_wheels` 구간)이 앞 값을 넘으면 그 구간 자세 무효(속도 외삽), 뒤 값을 넘으면 새 epoch |
| `ls_max_condition` | 50 | 5–1000 | 최소제곱 `[cos δ, sin δ]` 행렬 조건수 상한. 넘으면 `v_y = 0`으로 두고 `v_x`만 푼다(§4.4) |
| `max_lateral_speed_m_s` | 0.2 | 0.05–1 | 풀이한 `|v_y|`가 이보다 크면 그 표본 무효 |
| `max_slip_wheels`(r13) | 0 | 0–5 | 바퀴 합의에서 한 표본에 뺄 수 있는 바퀴 수 상한(남는 바퀴 ≥ `min_valid_wheels`). **0(기본)이면 합의를 끄고 r12와 같다.** 실기 잔차를 잰 뒤 2로 켠다(§4.4·§7.3) |
| `slip_residual_m_s`(r13) | 0.10 | 0.03–1 | 바퀴 합의의 잔차 허용치. 남은 바퀴의 `|잔차|`가 모두 이하여야 합의로 본다. 시뮬 정상 주행 최대 0.043 m/s(§12 r13). 실기 분포로 다시 정한다(§7.3) |
| `slip_epoch_s`(r13) | 1.0 | 0.1–5 | 바퀴 합의 실패(`WHEEL_SLIP`)가 이보다 오래 이어지면 새 epoch |
| `max_stream_skew_s` | 0.5 | 0.2–2 | 두 스트림 큐의 최신 stamp 차가 이를 넘으면 정체로 보고 새 epoch·큐 비움(큐 상한) |
| `stationary_command_turns_per_s`·`stationary_measured_turns_per_s` | 0.005·0.02 | — | 정지 판정: 모든 유효 바퀴 |명령| < 앞 값 ∧ |측정| < 뒤 값 |
| `bias_stationary_s` | 1.0 | 0.3–5 | 정지가 이 시간 이어지고 그 구간 자이로 표준편차 < `bias_max_std_rad_s`이면 바이어스 갱신 |
| `bias_max_std_rad_s` | 0.01 | 0.001–0.05 | 정지 구간 자이로 흔들림 상한(진동·외력 회전 배제) |
| `bias_alpha` | 0.2 | 0.01–1 | 두 번째 정지 구간부터의 갱신 가중(`b ← b + α(평균 − b)`). 첫 구간은 그 평균을 그대로 쓴다 |
| `stamp_regression_epoch_s` | 1.0 | 0.1–10 | stamp가 이보다 크게 역행하면(시계 점프·bag 반복) 새 epoch |
| `camera_x_m`·`camera_y_m`·`camera_z_m` | 0.348·0.0·0.029 | — | base_link 기준 depth 광학 원점(보관 CAD). 실차 측정으로 교체 |
| `camera_pitch_down_deg`·`camera_roll_deg`·`camera_yaw_deg` | 20·0·0 | 0–35·±20·±30 | 장착 회전. pitch 상한 35°는 판정기 평면 기울기 상한 40°(`MAX_TILT_DEG`)에 여유를 둔 값. 같은 세 값으로 IMU 축도 변환한다(§4.4) |
| `corrector` | `"identity"` | `identity`·`height_match` | §4.7 |
| `match_*` | §4.7 | — | B 상수 |
| 콘솔: `display_upsample`·`hillshade_azimuth_deg`·`hillshade_altitude_deg`·`hillshade_exaggeration`·`stale_display_s`·`max_render_pixels` | 4·315·40·6·30·2,000,000 | 1–8·0–360·5–85·1–20·5–60·— | §4.10. 판정에 쓰지 않는다 |
| 콘솔(r12): `tidy_low_m`·`tidy_min_neighbours`·`weak_alpha` | 0.03·2·0.45 | 0–0.05·0–4·0.2–1 | §4.10 표시 정리. 0·0·1이면 각 규칙이 꺼진다. 판정에 쓰지 않는다 |

- 노드 파라미터와 콘솔 CLI로 바꾼다. 범위 밖이거나 서로 맞지 않으면(패킷 > 1200 B, pending 용량 부족 등. 나이 표현 범위는 두 쪽 범위가 계약으로 묶여 있어(`31·2.0 = 62 ≥ 60`) 각자 범위 검증만 한다) 기동 실패(조용한 클램프 없음).
- 분류 경계는 Jetson에서만 쓴다. 콘솔은 받은 분류 코드를 그대로 그린다.

### 4.2 바닥 평면과 점 (`TerrainDetector` 확장·depth 솎기)

- `TerrainResult`에 필드 셋을 더한다(기본값 `None`, 기존 사용처 불변): `points`(카메라 광학 3D, `(H·W,3) float32`, 이미 계산한 배열 참조),
  `valid`(유효 깊이 마스크), `ground_plane`(`(normal(3,), offset)` — `normal·p + offset` = 바닥 위 높이, 카메라 쪽 +).
- `DetectorConfig`에 `min_ground_points`(기본 200 = 기존 `MIN_GROUND_POINTS`)를 더한다. 바닥 후보 수·voxel 수 두 검사에 같은 값을 쓴다. 오버레이 노드는 기본값 그대로다.
- 지도 노드는 판정기 **앞에서** depth를 `pixel_stride`로 솎는다(`depth[::s, ::s]`). 내부파라미터는 `fx/s, fy/s, cx/s, cy/s`, 크기 `⌈W/s⌉×⌈H/s⌉`의 `CameraModel`.
  최소 바닥점 수는 표본 수에 의존하므로 지도용 판정기는 `ground_min_points`(기본 100)를 쓴다(Codex 확인: 성긴 바닥이 stride 2에서 200이면 `NO_GROUND`, 100이면 `FRESH`).
  stride·최소점 조합은 실기 FRESH 비율(§7.3-3)로 확정한다.
- 지도는 **`status == OK ∧ ground_state == FRESH` ∧ 지지면 검사 통과**인 프레임만 쓴다.
- **지지면 검사**(Codex 4회차 P1-1): base 수평 좌표 계산(§4.3)과 같은 축으로 카메라 전방 거리 `range_min_m ≤ p·f ≤ range_min_m + support_band_m` 인 유효 점을 모아,
  수가 `support_min_points` 이상이고 그 높이 `h`의 중앙값이 `|·| ≤ support_tol_m`이면 통과. 실패하면 `NOT_SUPPORT`로 세고 그 프레임은 지도에 넣지 않는다.
  근거리 띠는 로봇이 곧 밟을 바닥이라, 이 띠와 맞는 평면은 로봇 지지면과 같은 높이의 면이다. 앞쪽 넓은 도랑 바닥이 평면으로 잡히면 근거리 바닥이 +0.2 m라 실패한다.
  - **base 원점 높이 검사**: `support_base_height_m`이 설정돼 있으면, 평면에서 잰 base_link 원점 높이 `n·o + d`(`o = −R·t`, §4.3)가
    그 값 ±`support_height_tol_m` 안이어야 한다. 근거리 띠 검사만으로는 **도랑이 근거리 띠 안에서 시작할 때** 통과해 버린다
    (독립 렌더: 도랑 시작 1.0 m → 근거리 중앙값 +0.20 m로 실패, 0.45 m → 근거리 띠가 도랑 바닥이라 통과, 평면 offset은 둘 다 0.70 m).
    r5의 **카메라 높이**(평면 offset) 고정 비교는 차체 숙임에 따라 카메라 높이가 변해 실패했다(Codex 5회차: 10° 숙임 평지 offset 0.439 → 정상 바닥 거부,
    15° 숙임 + 10 cm 도랑 offset 0.509 → 도랑 통과). base 원점 높이는 평면 법선에서 바로 계산하므로 **평면에 대한 차체 자세와 무관**하다 —
    같은 렌더에서 평지 수평 0.471, 평지 10° 숙임 0.471(통과), 15° 숙임 + 10 cm 도랑 0.571, 수평 + 20 cm 도랑 0.671(둘 다 실패).
    - **보장 조건**(Codex 6회차 P1-2): 실제 base 높이 변동 `var`와 도랑 깊이 `D`는 같은 측정값에 더해진다(평면이 도랑 바닥이면 측정 = 실제 높이 + D).
      그래서 평지는 `|var| ≤ tol`일 때 받아들여지고, 근거리 띠에서 시작하는 도랑은 `D > tol + var`일 때 반드시 거부된다.
      같은 이유로 근거리 띠를 덮는 높이 `E`의 윗면이 평면이 되면 측정 = 실제 높이 − E라, `E > tol + var`일 때 반드시 거부된다(Codex 7회차: 돌출 임계 2 cm 설정에서 5 cm 윗면이 통과).
      설정 검증으로 `support_base_variation_m ≤ support_height_tol_m < min(obstacle_height_m, ditch_depth_m) − support_base_variation_m`을 강제한다 —
      그러면 **분류 임계 이상인 도랑·윗면은 지지면으로 받아들여지지 않는다.** 그보다 작은 높이 차는 어차피 돌출·도랑으로 분류되지 않는 크기다.
      기본 8 cm 임계에서는 `var < 4 cm`여야 켤 수 있고(예: var 3 cm, tol 3–4 cm), 실측 var가 크거나 임계를 낮춰 조건을 못 맞추면 높이 검사를 켜지 않고 근거리 띠 검사만 쓴다(§10).
      실측 var보다 큰 실제 변동(험지에서 차체가 더 들림)이 오면 평지 프레임이 거부될 뿐(보수적) 도랑을 받아들이지는 않는다 — 단 그 변동이 도랑 깊이와 반대 방향으로 상쇄되면 보장이 깨지므로 var는 넉넉히 잰다.
    장착·차체 높이는 실측 전이라 기본은 끄고, §7.3-1에서 재서 켠다. 끈 동안의 한계는 §10에 적는다.
  같은 판정기를 쓰는 PR #15 오버레이에도 같은 한계가 있다 — 후속으로 같은 검사를 오버레이에 넣는다(§10).
- 오버레이 노드(PR #15)와 지도 노드는 각자 판정기를 돌린다. §7.3 부하 기준을 못 맞추면 한 프로세스로 합친다(§11).

### 4.3 프레임 → 국소 칸 `build_local_scan`

장착 변환 표기: `R`은 **base 좌표 벡터를 광학 좌표로 옮기는 회전**(`v_opt = R·v_base`, pitch-down·roll·yaw로 만든다),
`t = (camera_x_m, camera_y_m, camera_z_m)`는 광학 원점의 base 좌표다.

1. 유효 점만 남기고 `h = n·p + d`(평면 기준 높이).
2. **base 수평 축**: base 전방 `x̂_b`를 광학 좌표로 옮긴 `R·x̂` 를 평면에 투영한다 — `f = normalize(R·x̂ − ((R·x̂)·n) n)`, 왼쪽 `l = n × f`.
   pitch-down θ, roll·yaw 0이면 `R·x̂ = (0, −sin θ, cos θ)`(광학 x 오른쪽, y 아래, z 앞). 광학축을 투영하면 차체 롤에서 어긋난다(Codex 1회차, 24.75 cm).
3. **base 원점의 발 지점**: base 원점의 광학 좌표 `o = R·(0 − t) = −R·t`(기본값 `(0, 0.146, −0.317)`)를 평면 축에 투영한 `(o·f, o·l)`.
   점의 base 수평 좌표는 `xb = p·f − o·f`, `yb = p·l − o·l`. 차체가 기울면 카메라 발 지점이 옆으로 움직이는 것(롤 20°·높이 0.5 m에서 0.171 m)을 포함한다.
   (r3는 `−Rᵀt`로 적어 기본 장착에서 24 cm 오차였다 — Codex 3회차 P1-1.)
4. `range_min_m ≤ p·f ≤ range_max_m ∧ |h| ≤ max_abs_height_m`(카메라 기준 전방 거리)만 남긴다.
5. 칸 키 `(⌊xb/cell⌋, ⌊yb/cell⌋)`로 묶고, 점이 `min_points_per_cell` 이상인 칸만 값을 낸다.
   칸 높이: `n`개 높이를 정렬해 `k = max(1, ⌊cell_trim_fraction·n⌋)`, 위쪽 `hu = sorted[n−1−k]`, 아래쪽 `hd = sorted[k]`.
   **심각도** `s(h) = max(h / obstacle_height_m, −h / ditch_depth_m)`로 비교해 `s(hu) ≥ s(hd)`면 `hu`, 아니면 `hd`(임계값이 비대칭이어도 위험한 쪽을 고른다 — Codex 4회차 P1-3).
   기본 5점 `[0,0,0,0,1]` → 0(외톨이 점 무시). 정렬은 `np.lexsort((h, cell_id))` 한 번.
6. **프레임 안 분류**: `h ≥ obstacle_height_m` → 1(돌출), `h ≤ −ditch_depth_m` → 2(도랑·낙차), 그 밖 → 0(주행 가능).
7. `LocalScan`: `ix, iy (int32)`, `h_m (float32)`, `code (uint8)`, `count (int32)`, `stamp_s`(depth 헤더 stamp).
- base 좌표로 내는 이유: B 보정기가 세계 좌표 변환 전의 국소 지도를 후보 자세로 옮겨 맞춰 봐야 하기 때문이다(§4.7).

### 4.4 자세 — `PlanarOdometry`와 `PoseTrack`

r1–r6은 별도 `odometry` 노드의 `/odom`을, r7은 그 코어(`StateEstimator`)를 직접 썼다. 코어의 IMU 신선도 창(0.25 s)·재연결 때 공백 구간 무시·freeze·정지 확인 전 bias 학습·
조향 방식별 기하가 지도 시간축과 맞지 않아 반례가 이어졌다(Codex 6·7회차: IMU 끊김 0.24 s에 6.88°, 바퀴 0.5 s 공백 뒤 24.9 cm·7.45°, 같은 stamp IMU 선처리로 실제 회전을 bias로 학습).
r8은 지도에 필요한 것만 하는 **평면 추측 항법**을 순수 모듈로 둔다. 기존 코어·노드는 바꾸지 않는다.

- **입력**(노드가 메시지에서 변환):
  - 자이로 표본 `(stamp, ω_z)`: `/l515/gyro/sample`의 광학 축 각속도를 `terrain_map.geometry.base_to_optical`의 `Rᵀ`로 차체 축으로 옮긴 z 성분(§4.8 회전 정의).
  - 바퀴 표본 `(stamp, [(name, s_i, δ_i, cmd_i, valid_i)])`: `/wheel_states`의 바퀴별 접지 속도 `s_i = drive_turns_per_s × 2π·wheel_radius_m`(바퀴 단위, + = 전진),
    조향각 `δ_i = steer_deg`, 명령 `cmd_i = command_turns_per_s`, 유효 `valid_i = ¬drive_stale ∧ ¬steer_stale`. 바퀴 위치 `(x_i, y_i)`는 노드가 `chassis.kinematics.default_geometry()`에서 넘긴다.
- **차체 속도(최소제곱)**(Codex 8회차 P1-1): 바퀴는 자기 방향으로 굴러간다고 보고, 자이로 `ω`(바이어스 보정, 그 시각 보간)를 알 때
  유효 바퀴마다 `s_i = (v_x − ω·y_i)·cos δ_i + (v_y + ω·x_i)·sin δ_i`. 미지수 `(v_x, v_y)` 두 개를 유효 바퀴 전체로 최소제곱 풀이한다.
  - 애커만(중간축 회전 중심이라 `v_y ≠ 0`)과 스키드(`δ = 0`) 모두 같은 식이고, 유효 바퀴가 일부여도 편향이 없다.
    검산(`chassis.kinematics` 실제 기하): 애커만 0.5 m/s·조향 1.0 → 풀이 `(0.5, 0.03915)` = 참값(단순 평균 0.5546으로 10.9 % 과대),
    스키드 0.5 m/s·0.5 rad/s, 유효 4개(오른쪽 셋·뒤 왼쪽) → 0.5(단순 평균 0.579).
  - **관측성**(Codex 9회차 P1-1): 두 열 `[cos δ_i, sin δ_i]`의 조건수를 본다. 직진·스키드처럼 모든 δ ≈ 0이면 `v_y`는 관측되지 않고(rank 1),
    조향이 아주 작으면 조건수가 커져 바퀴 속도의 작은 잡음이 큰 `v_y`가 된다(조향 0.001에서 조건수 약 2,149, ±0.001 m/s 교란에 `v_y` 1.75 m/s).
    그래서 조건수 > `ls_max_condition`(기본 50)이거나 rank가 1이면 **`v_y = 0`으로 두고 `v_x`만** `s_i + ω·y_i·cos δ_i − ω·x_i·sin δ_i = v_x·cos δ_i`로 1변수 최소제곱한다.
    이때 `v_y`를 0으로 둔 오차는 애커만에서 `|ω·x_mid|`(중간축 x = −0.06 m, ω 0.65 rad/s에서 0.04 m/s) 이하로, 조건수가 큰 경우는 조향이 작아 ω도 작다.
    풀이한 `|v_y|`가 `max_lateral_speed_m_s`를 넘으면 그 표본 무효. 유효 바퀴 < `min_valid_wheels`이거나 모든 `cos δ` ≈ 0이면 무효.
  - 스키드 미끄러짐(실효 트랙이 기하와 다름)은 좌우 대칭 바퀴에서는 상쇄되고 비대칭 부분집합에서만 남는다(§10).
- **바퀴 합의**(r13, 2026-10-19): 최소제곱은 한 바퀴의 헛돎을 그대로 평균에 넣는다(Isaac run-4138: 파형 위에서 바퀴 하나가 6.5 m/s로 헛돌 때 1초에 0.29 m 과대, 같은 봉우리가 지도에 0.29 m 어긋나 두 번 찍힘).
  유효 바퀴 `i`의 잔차 `e_i = s_i − ŝ_i`(`ŝ_i` = 풀이한 `(v_x, v_y)`와 `ω`로 위 식에서 계산한 바퀴 속도)로 다음 순서를 따른다. **`max_slip_wheels = 0`(기본)이면 1만 하고 r12와 같다.**
  합의를 켜면 유효 바퀴를 **1의 첫 판정부터** 이름순으로 정렬해 계산한다(입력 순서의 부동소수 합산 차이가 허용치 경계에서 판정을 바꾸지 않게 — Codex r13 3회차). 끄면 r12의 입력 순서 그대로다.
  1. 지금처럼 유효 바퀴 전체로 푼다(조건수·`v_y = 0` 대체·`|v_y|` 상한 그대로). **구조적으로 못 풀면**(유효 바퀴 < `min_valid_wheels`, 모든 `cos δ` ≈ 0) 그대로 무효(기존 이유).
     풀이가 유효하고 모든 `|e_i| ≤ slip_residual_m_s`면 **그 풀이를 쓴다 — 헛돎이 없으면 r12와 같은 결과**.
  2. 잔차가 허용치를 넘거나 `|v_y|` 상한으로 무효면(한 바퀴 헛돎이 `v_y`를 키울 수 있다 — Codex r13 1회차: 애커만 조향 1.0에 앞왼쪽 3 m/s면 전체 풀이 `v_y` 1.61 m/s),
     `k = 1 … max_slip_wheels`(남는 바퀴 ≥ `min_valid_wheels`인 동안) 순서로 바퀴 `k`개를 뺀 모든 조합을 같은 규칙으로 다시 푼다.
     남은 바퀴의 잔차가 모두 허용치 이하이고 풀이가 유효한 조합이 있으면 **가장 작은 `k`** 중 (잔차 제곱합, 뺀 바퀴 이름의 사전순)이 가장 작은 것을 쓴다 — **합의**, 자세 유효.
  3. 어느 조합도 합의하지 못하면(여러 바퀴가 제각각 미끄러짐 — run-4138 8.13–8.40 s) 유효한 풀이가 나온 모든 후보(1의 전체 풀이 포함) 중 (남은 잔차 최댓값, 전체 풀이 속도와의 차, 잔차 제곱합, 뺀 바퀴 이름 사전순)이 가장 작은 것으로
     **최선 추정 속도를 적분**하고(최대 잔차가 같은 후보가 여럿이면 가운데 쪽 — 바퀴가 고르게 흩어진 시험에서 이름순만 쓰면 0.875 m/s, 이 규칙은 0.625 m/s, 참값 0.3), 그 바퀴 표본의 자세는 **무효 `WHEEL_SLIP`**으로 낸다(그 시각을 감싸는 depth는 지도에 넣지 않는다). 유효한 후보가 하나도 없으면 1의 무효와 같다.
     - 최선 추정 표본은 속도가 있으므로 **바퀴 공백으로 세지 않는다**(`wheel_gap_invalid_s`·`wheel_gap_epoch_s`의 기준 시각을 갱신). 실제 표본 간격 초과·자이로 공백·스트림 정체는 그대로 센다.
     - 합의 실패가 **`slip_epoch_s`(기본 1 s) 넘게 이어지면** 새 epoch(`slip`, 자이로 상태 유지)로 지도를 비운다 — 최선 추정만으로 오래 이어 붙이면 어긋난 좌표로 기존 지도에 합류한다(agy r13-3, §4.4 공백 규칙과 같은 원칙).
       시계는 **유효한 풀이**(합의 포함)가 나와야 멈춘다. 유효 바퀴 부족 같은 구조적 무효 표본은 시계를 멈추지 않는다(한 표본씩 끼어 장기 실패 보호를 우회하지 않게 — Codex r13 3회차).
       시간 초과는 그 표본의 풀이 종류와 무관하게 **매 바퀴 표본에서, 시계를 멈추기 전에** 검사한다(실패 1 s 뒤 바퀴 부족 0.2 s를 거쳐 바로 복구해도 epoch — Codex r13 4회차).
       검사는 **바퀴 공백 판정보다 먼저** 한다 — 재생은 기록된 epoch를 표본 처리 전에 적용하므로, 실시간도 epoch 너머의 공백을 판정하지 않아야 같은 유효성이 나온다(Codex r13 5회차).
       epoch 뒤에도 실패가 이어지면 `slip_epoch_s`마다 다시 연다. 바퀴 공백 epoch처럼 실시간에서만 판정하고 재생은 기록된 결정을 따른다(풀이를 yaw 적분 앞에 해 epoch를 그 표본 처리 전에 연다).
       **바이어스 준비 전에는 시계를 시작하지도, epoch를 열지도 않는다**: 그 동안 `ω`는 미학습 바이어스를 품어 정지 중에도 합의가 실패할 수 있고, epoch가 학습 창을 지우면 학습이 교착된다(Codex r13 3회차: `slip_epoch_s` 0.5 < `bias_stationary_s` 1.0이면 7회 epoch·학습 실패). 그 동안 자세는 어차피 `BIAS_PENDING`이다.
       짧은 사건은 넘긴다: 무효+공백으로 처리한 시제품은 run-4138에서 0.36 s 실패로 새 epoch 2회(지도 두 번 비움)였다.
     - 바이어스 학습 구간 자격은 **r12 그대로**(전체 풀이 유효 ∧ 유효 바퀴 전체 정지)이고 합의 결과와 무관하다. 바이어스를 배우기 전의 `ω`는 바이어스를 품어
       정지 중에도 잔차를 키우므로(Codex r13 2회차: 바이어스 0.4 rad/s·허용치 0.10이면 최소 최대잔차 0.109로 합의 실패) 합의를 자격에 넣으면 학습이 교착된다.
       헛도는 바퀴는 측정 속도로 정지 판정에서 이미 걸러진다.
     - 누적 거리 `distance_m`은 유효 표본과 `WHEEL_SLIP` 표본의 자세를 이어 더한다(무효 구간을 직선으로 잇지 않는다, Codex r13 P2-6).
  - 계산량: 6륜·`max_slip_wheels` 2에서 표본당 2×2 풀이 최대 1 + 6 + 15 = 22회(50 Hz). 1의 전체 풀이와 2의 후보 풀이를 3에서 다시 쓴다.
  - **기본값은 끔**: 실기 바퀴 속도는 CAN 조회 슬롯(20 ms × 3)마다 따로 갱신돼 가감속 중 바퀴 사이에 시각 차가 있다(Codex r13: 1 m/s² 가속에서 150 ms 묵은 다섯 바퀴 0.35 대 최신 0.5 → 합의가 최신 값을 뺌).
    시뮬 정상 주행의 잔차 최대 0.043 m/s(truth ω 사용)는 실기 근거가 아니다. 실기 잔차를 속도·가감속·조향 방식별로 잰 뒤 켠다(§7.3). 시뮬 평가는 `max_slip_wheels 2`로 한다.
  - **못 잡는 것**: 모든 바퀴가 같은 비율로 헛돌면(갇혀서 공회전 등) 잔차가 작아 합의로 통과한다 — 위치가 계속 늘어난다. `max_slip_wheels`보다 많은 바퀴가 같은 값으로 헛돌면 그쪽이 다수가 되어
    **r12 최소제곱보다 나빠질 수 있다**(Codex r13 4회차: 참값 0.3에 네 바퀴 0.8·두 바퀴 0.3 → 합의 0.8, 최소제곱 0.633)(agy r13-1).
    스키드 선회처럼 기하 불일치가 허용치보다 크면 정상 바퀴가 빠질 수 있고, 비대칭으로 빠지면 편향이 생긴다. 가장 작은 잔차 제곱합이 가장 작은 속도 오차를 보장하지는 않는다(조건수 50 경계에서 2변수·1변수 후보가 섞임).
  - 노드 로그 요약(`terrain map: … slip excluded N, guessed M`)에 바퀴를 뺀 표본 수와 최선 추정 표본 수를 누계로 남긴다(상태 패킷 필드는 늘리지 않는다).
- **워터마크 정렬**: 두 스트림의 큐에서 `W = min(자이로 최신 stamp, 바퀴 최신 stamp) − lateness_margin_s`까지만 **확정** 처리한다.
  확정된 시각보다 이른 표본이 늦게 오면 버리고 센다(`late_dropped`). 확정 구간 안에서는 두 스트림이 stamp 순으로 합쳐지고, 같은 stamp는 바퀴 먼저다.
  **큐 상한**(Codex 8회차 P2-5): 두 큐의 최신 stamp 차가 `max_stream_skew_s`를 넘으면(한 스트림이 시간상 뒤처져 정체) 연속성 상실로 처리하고 큐를 비운다 — 큐 길이는 `max_stream_skew_s × 표본률`로 묶인다.
  한 스트림이 `wheel_gap_epoch_s` 넘게 수신되지 않아도(수신 단조시각) 같다.
- **적분**: 확정 구간을 자이로·바퀴 표본 시각으로 나눈 각 소구간에서 `ω`(사다리꼴), `(v_x, v_y)`(바퀴 표본 사이 선형 보간)로
  `yaw += ω·dt`, `[x, y] += R(yaw_mid)·[v_x, v_y]·dt`. 바퀴 표본 시각마다 불변 자세 표본 `(stamp, x, y, yaw, valid)`을 방출한다.
- **공백 규칙**(Codex 8회차 P1-2):
  - **자이로 간격 > `gyro_gap_epoch_s`(60 ms)** → 그 사이 회전은 복구할 수 없으므로(양 끝 관측이 같아도 사이에서 돌았을 수 있음 — 삼각형 반례 2.86°) **새 epoch**.
    Gateway IMU 워커는 발행 뒤 10 ms 대기라(약 100 Hz) 정상 간격은 10–20 ms다. 실기에서 간격 분포와 epoch 빈도를 잰다(§7.3).
  - **바퀴 간격 또는 유효 바퀴 부족 구간 > `wheel_gap_invalid_s`(60 ms)** → 그 구간은 마지막 속도로 외삽하고 끝 표본까지 무효. 회전은 자이로가 계속 덮으므로 위치 오차만 생긴다:
    외삽 오차는 `½·|a|·gap²`로 공백 중 가감속 `a`에 달려 있다 — **보장값이 아니라 조건부**다(예: |a| ≤ 1 m/s²·0.3 s면 4.5 cm, 2 m/s² 감속이면 9 cm까지 — Codex 9회차 8.25 cm 재현).
    그 구간 자세는 무효라 지도에 직접 들어가지 않지만, 이후 위치에 이 오차가 남는다. **> `wheel_gap_epoch_s`(0.3 s)** → 새 epoch.
- **바이어스**(Codex 8회차 P1-3): 정지 = 모든 유효 바퀴 |명령| < 0.005 turn/s ∧ |측정| < 0.02 turn/s(조종기 무입력 — 저속 회전 명령 0.0166 turn/s도 정지가 아님).
  정지가 `bias_stationary_s` 이어지고 그 구간 자이로 표준편차 < `bias_max_std_rad_s`일 때만 갱신한다.
  **학습 구간 자격**(Codex 9회차 P2-2): 구간 안 모든 바퀴 표본의 유효 바퀴 ≥ `min_valid_wheels`이고 바퀴·자이로 간격이 무효 임계 이하여야 한다.
  유효 바퀴 부족·공백·정지 아님이 한 번이라도 나오면 학습 구간을 끊고 처음부터 다시 잰다(유효 바퀴가 없을 때 "모두 정지"가 공허하게 참이 되지 않게). **첫 갱신은 그 구간 평균을 그대로**(EMA 아님) 쓰고, 그때부터 바이어스 준비.
  이후 정지 구간은 `b ← b + bias_alpha·(평균 − b)`. 준비 전 자세 표본은 모두 무효(`BIAS_PENDING`) — 주행 중 기동이면 첫 정지까지 지도가 쌓이지 않는다(운용 문서: 기동 뒤 1 s 정지).
- **유효성**: 자세 표본 `valid = 바이어스 준비 ∧ 최소제곱 조건 충족 ∧ 바퀴 합의(r13) ∧ 무효 구간 아님`.
- **연속성 상실 → 새 epoch**: 위 자이로 공백·긴 바퀴 공백·스트림 정체, 또는 stamp가 `stamp_regression_epoch_s` 넘게 역행하면 자세를 원점으로, 지도를 비우고 `map_epoch += 1`.
  콘솔은 epoch 전환으로 지도를 비운다(§4.9). 바이어스는 epoch를 넘어 유지한다(센서 특성이라).
- **`PoseTrack`**(`terrain_map/pose.py`): 자세 표본을 최근 2 s 보관한다. `pose_at(t)`는 `t`를 감싸는 두 표본이 있고, 간격 ≤ `pose_max_gap_s`이고, 둘 다 `valid`일 때만 선형 보간 자세를 돌려준다(yaw는 각도 차 정규화 후 보간).
  실패 이유: `ODOM_MISSING`(표본 없음)·`POSE_GAP`·`WHEELS_STALE`·`WHEEL_SLIP`(r13 바퀴 합의 실패, 콘솔 문구 "바퀴 합의 실패")·`BIAS_PENDING`·`WAITING`(아직 `t`가 워터마크 뒤).
- **지연 예산**(Codex 8회차 P2-4): depth가 고정 대상이 된 뒤 자세가 생기기까지 = 입력 도착 지연 + `lateness_margin_s` + 소구간 대기(최대 표본 간격 20 ms)이고, 처리 틱(`2·process_hz` = 10 Hz)까지 최대 100 ms가 더해진다.
  설정 검증 `pose_wait_s ≥ lateness_margin_s + 1/(2·process_hz) + input_lag_budget_s`(기본 0.4 ≥ 0.05 + 0.1 + 0.25)는 **입력 지연이 0.25 s 이하면 처리를 보장**한다는 뜻이다.
  그보다 늦어도 틱 위상에 따라 처리될 수 있고(Codex 9회차: 300 ms 지연은 자세 준비 380 ms·대상 나이 399 ms라 처리), 만료는 대상 나이가 `pose_wait_s`를 넘을 때만이다.
  만료 판정은 자세 확인 **뒤**에 한다(같은 틱에 자세가 생기면 처리).
- **실시간·재생 동등성 범위**: 결과는 **채택된 표본 집합과 epoch 결정**의 결정적 함수다. 실시간에서 늦게 와서 버려진 표본·수신 정지에 의한 epoch는 도착 시각에 달려 있어,
  bag의 stamp 순 재생과 다를 수 있다. 보장은 "같은 bag의 오프라인 재생끼리 결정적"이고, 실시간은 `late_dropped`·epoch 전환 수를 상태 패킷에 실어 차이 가능성을 드러낸다.
  실시간과 같은 지도가 필요하면 `processed_log_path`에 채택·버림·epoch 결정을 기록하고 재생이 그 결정을 따른다(`--decisions-from`).
- **남는 오차**: 바퀴 미끄러짐(특히 스키드 비대칭 부분집합), 기울어진 선회(§10), 바퀴 실효 반지름(하중 변형), 짧은 바퀴 공백 외삽(`½·|a|·gap²`, 가감속에 따름), 조건수가 커서 `v_y = 0`으로 둔 구간(`|ω·x_mid|` 이하). 지도 작성은 0.5 m/s 이하 권장.

### 4.5 누적 `ClassMap`

- 세계 좌표 = 추정 코어의 세계 프레임(노드 시작·epoch 시작 자세가 원점). 칸 `(gx, gy) = (⌊X/cell⌋, ⌊Y/cell⌋)`, 타일 `(tx, ty) = (gx // tile_cells, gy // tile_cells)`(바닥 나눗셈). 타일은 처음 관측될 때 만든다.
- 칸 배열: `history`(최근 `history_length`개의 `(코드, 관측 stamp)`, 미관측 = 빈 이력), `code`(다수결 결과), `display_h_m float32`(마지막 관측 `h`, 표시 전용, NaN = 미관측),
  `observations uint16`, `last_seen_s float64`. 타일: `version`, `last_seen_s`, `tile_seq`(§4.9).
- **세계 칸 변환**: 국소 칸 중심 `((ix+½)·cell, (iy+½)·cell)`을 자세로 옮긴 세계 좌표에 바닥 나눗셈. 항등 자세에서 칸이 1:1.
  같은 세계 칸에 둘 이상이 오면 심각도 `s(h)`(§4.3)가 큰 쪽(코드·높이를 함께).
- **융합**: 이력에서 새 관측 stamp보다 `history_max_age_s` 넘게 오래된 표를 먼저 버리고(Codex 4회차 P1-2 — 오래된 표가 새 관측을 이기지 못하게),
  새 `(코드, stamp)`를 끝에 넣는다(길이 초과 시 가장 오래된 것을 버림). `code` = 남은 이력 안 같은 코드 2개 이상이면 그 코드, 아니면 가장 최근 코드.
  긴 공백 뒤 첫 관측은 이력에 그것 하나만 남아 그대로 표시된다(오래된 "주행 가능" 두 표가 새 낙차 한 표를 덮지 않는다).
  `display_h_m` = 새 `h`, `observations` 포화 증가, `last_seen_s` 갱신. 높이끼리는 섞지 않는다.
  - 움직인 물체·치운 물체: 새 관측 두 번이면 사라진다(3중 2 다수결). 한 번 튄 잡음 코드는 다수결로 무시된다.
  - 단차·계단·넓은 윗면·넓은 도랑은 프레임 안 분류가 그대로 맞다(§12 ①, Codex 3회차 반례 재현).
- **관측 부족**: `observations < min_observations`면 전송 코드 4.
- **전역 version**: epoch마다 0에서 시작하는 지도 전체 공용 단조 카운터. 타일의 **전송 바이트 중 나이 비트를 뺀 부분**(표시 높이 바이트 + 관측 부족 처리를 적용한 전송 코드)이
  바뀌면 그 타일 version = 카운터 증가값(Codex 4회차 P2-6 — 관측 부족 4 → 0 해제도 version을 올린다). 다시 만들어진 타일도 더 큰 version.
  나이 비트만 바뀌는 것은 version을 올리지 않는다(순환 재송신이 나이를 실어 나른다, §4.9).
- 회전 때문에 생기는 칸 사이 빈틈은 메우지 않는다(다음 프레임이 채우고, 표시는 보간이 메운다).
- 타일 수가 `max_map_tiles`를 넘으면 `last_seen_s`가 가장 오래된 타일부터 버린다. 삭제 통지는 보내지 않는다 — 콘솔 TTL이 수렴시킨다(§4.9).
- 칸 사이 분류가 서로 기대지 않으므로(프레임 안 판정) 이웃 타일 재분류가 필요 없다.

### 4.6 (r4에서 삭제) 두 규모 분류

r3의 두 규모 분류(단차 창 + 2.2 m 기준 중앙값)는 계단에서 주행 가능 오판(Codex 3회차 P1-4)이 나와 프레임 안 분류(§4.3-6)로 대체했다. 절 번호는 이력 대조용으로 남긴다.

### 4.7 위치 보정 `PoseCorrector` — A는 그대로, B는 지도 맞춤

```python
class PoseCorrector(Protocol):
    def correct(self, scan: LocalScan, seed: Pose2D, class_map: ClassMap) -> Correction: ...
# Correction(pose: Pose2D, applied: bool, score: float | None, reason: str)
```

- **Δ 계약**: 런타임은 SE(2) 누적 보정 `Δ`를 갖는다. 추정 자세 `O`(`PoseTrack.pose_at`)에 대해 `seed = Δ ∘ O`를 넘기고,
  보정 결과 `P`를 받으면 `Δ ← P ∘ O⁻¹`로 갱신한다(이전 보정을 덮어쓰지 않고 합성). epoch 때 `Δ = 항등`.
- **지도 조회 API**: `ClassMap.sample(gx, gy) -> (code, display_h, observations, last_seen_s)` 벡터화(미관측은 4·NaN·0·NaN). 부작용 없음. B는 이것과 `scan`만 쓴다(추가 상태 없음 — Codex 3회차 P2-10).
- **A (`IdentityCorrector`)**: `P = seed`, `applied=False`, `reason="identity"`.
- **B (`HeightMatchCorrector`) — 미리 정하는 알고리즘**(구현은 시간이 남으면, `correction.py`에 클래스 하나 + 시험):
  1. 특징 조건: 국소 칸 중 코드 1·2가 `match_min_features(30)` 미만이면 보정하지 않는다(`"flat"`).
  2. 후보: `seed` 주변 `dx, dy ∈ [−match_window_m(0.15), +]`(칸 간격), `dyaw ∈ [−match_window_deg(3), +]`(`match_yaw_step_deg` 0.5) — 기본 7×7×13 = 637.
  3. 점수: 후보로 옮긴 국소 칸과 `observations ≥ 3`·`last_seen_s`가 최근 `match_recent_s(30)` 안인 지도 칸의 겹침에서 차 `e = display_h_map − h`를 구하고,
     **겹침 전체의 중앙값 `m = median(e)`를 빼** `min(|e − m|, 0.10)` 평균 + `match_code_weight(0.05)` × (코드 불일치 비율).
     표시 높이는 칸마다 다른 프레임 평면 기준이라 일정한 높이 차가 생길 수 있는데, 중앙값을 빼면 그 차에 영향을 받지 않는다
     (Codex 4회차 P2-8 — 높이 기준 1.2 cm 차이만으로 정답 자세에서 5 cm 틀린 보정을 채택한 반례. 이 반례를 B 시험 장치에 넣는다).
     겹침 < `match_min_overlap(100)`인 후보는 제외. `seed` 자체가 겹침 미달이면 보정하지 않는다(`"no_overlap"`).
  4. 두 번째 후보: 최저 후보와 `|Δgx| ≥ 2` 또는 `|Δgy| ≥ 2` 또는 `|Δyaw 단계| ≥ 2`인 후보 중 최저.
  5. 적용 자세: 최저 후보를 향해 `match_max_step_m(0.05)`·`match_max_step_deg(1)`로 자른 `P'`. **`P'`의 점수를 다시 계산**해
     `score(P') ≤ score(seed) − match_min_gain(0.01)`이고 `score(2등) − score(최저) ≥ match_min_peak(0.005)`일 때만 채택. 아니면 `seed`(`"rejected"`).
  6. 평가 지표: 합성 지도(알려진 참 자세)에서 자세 오차, 녹화 재생에서 왕복 경로 같은 장애물 칸 IoU·채택률·처리시간. IoU만으로 정확성을 인정하지 않는다.
- **A 단계에서 만들어 두는 B 시험 장치**: 합성 `ClassMap` + 알려진 오프셋 국소 칸 생성기, 연속 이동·회전 중 Δ 누적 검사,
  epoch 초기화, 모호한 무늬(일자 벽·반복 줄무늬)에서 거부, 자른 자세 재평가. B 구현 시 이 장치에 클래스만 꽂는다.
- B를 켜는 방법: 노드 파라미터 `corrector:=height_match`(또는 재생 CLI `--corrector height_match`).

### 4.8 런타임과 노드

- **입력 보관**: depth 콜백(자기 콜백 그룹, PR #15처럼 2-스레드 실행기)은 depth를 `FrameScheduler`의 pending에 넣는다.
  바퀴·자이로 콜백은 표본으로 변환해 `PlanarOdometry`에 넣고(수신 단조시각과 함께), 방출된 자세 표본을 `PoseTrack`에 넣는다(하나의 콜백 그룹에서 직렬). CameraInfo는 PR #15 노드 규칙(`CAMERA_INFO_MISSING`, 해상도 불일치)을 따른다.
- **처리 프레임 선택 `FrameScheduler`**(실시간·재생 공용):
  - **대상** = 마지막으로 처리를 시도한 depth stamp보다 `0.9 / process_hz` 이상 뒤인 pending 중 가장 오래된 것. 대상보다 오래된 pending은 버린다(의도적 생략, 손실로 세지 않음).
  - 대상은 **고정된다**: 새 depth가 들어와 pending이 차면 대상이 아닌 것 중 가장 오래된 것부터 축출한다. 대상은 축출하지 않는다(Codex 3회차 P1-5 — 대기 중 대상이 밀려 10초 동안 처리 0건).
    용량 `depth_pending_max ≥ ⌈pose_wait_s × 10 Hz⌉ + 1`(기본 5)을 설정 검증으로 강제해 대기 중에도 새 depth가 들어갈 자리가 있다.
  - 대상의 자세가 `WAITING`이면 기다리고, 대상 수신 후 `pose_wait_s`가 지나면 "자세 만료"로 버린다(손실). 다른 자세 실패도 손실. 대상이 어떤 이유로든 처리되지 못하면 손실로 센다.
  - 실시간 틱은 `2 × process_hz`로 돌며 대상이 처리 가능하면 처리한다. 결정적 재생은 같은 스케줄러에 stamp 순으로 depth와 자세를 넣는다 — 도착 지연이 없으면 두 경로가 같은 프레임을 고른다.
  - 처리한 depth stamp·epoch·결과와 추측 항법의 채택·버림·epoch 결정을 선택적으로 기록한다(`processed_log_path`, 기본 끔). 실시간 지도와 같은 지도를 재생으로 만들 때는 이 기록을 `--decisions-from`으로 준다(§4.4 동등성 범위).
  - 그 depth → 솎기 → 판정기 → FRESH면 `LocalScan` → 보정 → 융합. 처리시간을 재고 한 틱을 넘으면 PR #15처럼 별도 위치에서 경고, 종료 때 요약 한 줄.
- **상태 판단**(우선순위): `CAMERA_INFO_MISSING` → `DEPTH_STALE`(최신 depth 수신 후 0.5 s) → `NO_POSE`(최근 1 s 동안 자세 있는 대상 없음, 이유 포함) →
  `NO_GROUND`(최근 1 s 동안 FRESH이고 지지면 검사를 통과한 프레임 없음 — 이유 `GROUND_NOT_FOUND`·`NOT_SUPPORT`) → `OK`.
- **송신 틱**(`status_hz`): 상태 패킷 1개 + 바뀐 타일 최대 `max_tiles_per_tick`개(version 오름차순) + 순환 재송신 `refresh_tiles_per_tick`개.
  송신은 `powertrain_runtime.telemetry.send_datagram`과 세션 파일 규칙(PR #15 §4.5)을 그대로 쓴다.
- **노드 하나**: `ros2 run powertrain_ros terrain_map`. 다른 노드를 띄우지 않고, ROS 토픽·TF·서비스를 발행·제공하지 않는다(파라미터 서비스 기본 제외).
- **장착 회전 정의는 하나다**(Codex 4회차 P2-7): `terrain_map.geometry.base_to_optical(pitch_down, roll, yaw)`가 지도 기하(§4.3)와 IMU 축 변환(§4.4)이 함께 쓰는 유일한 정의다.
  장착 회전 `M = Rz(yaw)·Ry(pitch_down)·Rx(roll)`(ROS rpy, 고정축 X→Y→Z 순, pitch 양수 = 앞이 아래)로 base → 장착 프레임(x 앞, y 왼쪽, z 위)을 만들고,
  장착 → 광학 고정 치환 `P`(광학 x = −장착 y, 광학 y = −장착 z, 광학 z = 장착 x)를 곱해 `R = P·Mᵀ`(`v_opt = R·v_base`). IMU는 `v_base = Rᵀ·v_opt`.
  모든 각이 0이면 `Rᵀ`는 기존 `imu_tilt` `opt_to_body`의 `(z, −x, −y)`와 같다. 기존 `imu_tilt`는 20° 하향 장착을 반영하지 않아 순수 90° 선회가 84.57°로 적분된다(Codex 2회차) —
  지도는 그 노드를 쓰지 않으므로 이 결함을 물려받지 않는다. `imu_tilt` 자체의 수정은 범위 밖이며 §10에 기록한다.

### 4.9 콘솔 수신 `operator_console/terrain_map.py`

- **세션 선택과 순서 판정 분리**: PR #15 `TerrainSessionGate`의 세션 전환 규칙(같은 부팅 `(start, instance)` 순서, 다른 부팅 1 s 침묵)만
  `operator_console/session_gate.py` `SessionSelector`로 옮긴다. `TerrainSessionGate`는 그것과 자기 sequence 하한을 조합해 동작을 그대로 유지한다.
- **epoch 규칙**: 같은 세션 안에서
  - 패킷 `map_epoch` < 현재 epoch → 거부(지연된 옛 epoch 타일이 새 지도를 되돌리지 못함).
  - 패킷 `map_epoch` > 현재 epoch → 전환: `TileStore`·궤적을 비우고 상태 sequence 하한과 타일 version 하한을 초기화한 뒤 그 패킷을 받는다(상태든 타일이든).
  - 세션이 바뀌면 epoch·하한 모두 초기화.
  - 해당 epoch의 상태 패킷을 받기 전에 온 타일은 저장하되, `cell_m`·`tile_cells`·`age_step_s`를 모르는 동안 그리지 않는다.
- **순서**: 상태 패킷은 `sequence` 하한, 타일은 타일별 `version`.
- **TTL과 같은 version**: `TileStore`: `(tx, ty) → (version, tile_seq, 칸 바이트 배열, received_s)`.
  - version이 더 크면 교체. version이 **같고 `tile_seq`가 더 크면** 칸 바이트 전체를 교체하고 `received_s`를 갱신한다 — 같은 version은 나이 비트만 다르므로
    순환 재송신이 생존과 나이를 함께 갱신한다(Codex 3회차 P2-8).
  - 같은 version에서 `tile_seq`가 작거나 같으면(지연된 역순 패킷) 무시 — 흐려진 칸이 옛 나이 0 패킷으로 다시 선명해지지 않는다(Codex 4회차 P2-5).
  - version이 더 작으면 무시(TTL도 연장하지 않음).
  - 상태 패킷 `refresh_cycle_s`의 3배 + 2 s 동안 받지 못한 타일은 화면 데이터에서 지운다. 송신부가 버린 타일은 순환되지 않으므로 콘솔에서도 사라진다.
  - **순서 하한은 화면 데이터와 따로 둔다**(Codex 5회차 P2-3): `(tx, ty) → (version, tile_seq)` 하한 표는 TTL로 지우지 않고 epoch·세션이 바뀔 때만 비운다.
    TTL로 지운 타일에 같은 version·작은 `tile_seq`의 지연 패킷이 와도 하한에 막혀 나이 0으로 부활하지 않는다.
    하한 표 크기는 epoch 안에서 본 타일 수만큼이고, `4 × max_map_tiles`를 넘으면 가장 오래 갱신되지 않은 하한부터 버린다.
    버릴 때 **버린 하한들의 `tile_seq` 최댓값 `evicted_seq_floor`** 를 남기고, 하한 표에 없는 타일의 패킷은 `tile_seq > evicted_seq_floor`일 때만 받는다(Codex 6회차 P2-3).
    `tile_seq`는 epoch 안 전역 단조라, 버린 하한보다 옛 패킷은 모두 막히고 새로 보내는 패킷은 그보다 크다. 가장 오래된 하한부터 버리므로 `evicted_seq_floor`는 현재 송신 순번보다 한참 작다.
- 완전성 표시: 받은 타일 수 / 상태 패킷 `tile_count`. 콘솔을 늦게 켜면 기본 약 26 s 안에 전체가 찬다.
- `snapshot()`: 한 락에서 (상태, 타일 참조, 마지막 수신 시각).
- 콘솔은 지도에 아무 명령도 보내지 않는다. "화면에서 지우기"는 콘솔 `TileStore`만 비운다(다음 순환으로 다시 찬다).

### 4.10 콘솔 표시 `MapPanel` ("지형 지도" 페이지)

- 위치: 콘솔 상단 페이지(`실시간 화면`·`협조구동`·`시스템 상태`·로봇팔 탭) 옆에 `지형 지도` 페이지를 더한다. 영상 화면은 건드리지 않는다.
- 순수 함수 `map_rgba(store, view, config) -> (rgba uint8[H,W,4], meta)`(numpy만, 시험 가능):
  1. 보이는 범위의 타일을 표시 높이·코드·나이 배열로 모은다(미관측 NaN). 출력 크기는 `max_render_pixels` 이하.
  2. 표시용 보간: `display_upsample` 배율로 **쌍선형** 보간(미관측은 보간 전 이웃 평균으로 메우고, 결과에서 다시 가린다). 3차 보간은 경계 오버슈트로 쓰지 않는다.
  3. 음영: 기울기에서 hillshade(방위·고도·과장 계수), 높이 색상표에 곱한다. 표시 높이는 칸별 관측 프레임 기준이라 이음매가 있을 수 있다(§1 전제).
  4. 분류 색 덧칠: 돌출 빨강, 도랑·낙차 파랑 — **원본 칸 경계로 최근접 확대**하고 경계에 윤곽선을 그린다(agy 11).
     칸당 4픽셀 미만으로 그릴 때(축소·픽셀 예산)는 윤곽선을 위험 칸 **바깥쪽** 픽셀에만 그린다 — 한 칸짜리 위험이 윤곽선에 통째로 덮여 색이 사라지지 않게(r12, Codex P2).
     관측 부족은 흐리게, 미관측은 배경색(평지·도랑과 다른 색, agy 7).
  5. **칸 나이**: 칸 나이 = 칸별 5비트 나이 × `age_step_s` + (지금 − 타일 `received_s`). `stale_display_s`를 넘은 칸은 채도를 낮춘다(agy 4·6).
     나이는 시간이 흐르면 바뀌므로, 렌더 캐시는 시간과 무관한 층(보간·음영·분류색)만 담고 나이 흐림은 그릴 때마다 계산한다.
  6. **표시 정리(r12, 2026-10-18)** — Isaac 합성 기록(파형 + 상자 5개, 57 s)에서 화면을 어지럽히던 것은 판정 오류가 아니라
     사거리 끝의 흩어진 외톨이 칸과, 「주행 가능」과 거의 같은 회색으로 칠해지는 「관측 부족」 칸이었다. 원본 칸 단위로 두 규칙을 둔다(보간 전).
     - **낮은 칸**: 관측됨 ∧ |표시 높이| < `low` ∧ 코드가 돌출·도랑이 아님. `low = min(tidy_low_m, ½·min(상태 패킷 obstacle_height_m, ditch_depth_m))`
       — 송신 경계를 낮게 바꿔도 경계 근처 칸은 낮은 칸이 되지 못한다.
     - **외톨이 숨김**: 낮은 칸 중 관측된 8-이웃이 `tidy_min_neighbours` 미만이면 미관측처럼 그린다(보간·음영 입력에서도 빠진다).
       이웃은 화면 창 밖 1칸까지 센다(창 경계에서 잘려 숨겨지지 않게).
     - **약한 칸 반투명**: 숨기지 않은 낮은 칸 중 코드가 관측 부족(`min_observations` 미만)이면 알파를 `weak_alpha`로 낮춘다. 색(관측 부족 회색 덧칠)은 그대로다.
     - **불변식**: 높은 칸(|h| ≥ `low`)과 돌출·도랑 칸은 숨기지도 흐리게 하지도 않는다. 숨김은 「주행 가능/관측 부족」을 「미관측」으로만 바꾸므로
       화면이 실제보다 더 안전해 보이는 방향의 변화는 없다. 패킷·분류·재생 대조는 바뀌지 않는다(콘솔 표시 전용).
     - 정리 결과는 칸 나이와 무관하므로 정적 층 캐시에 함께 담고, 캐시 키에 상태 패킷 경계값을 넣는다.
     - 쿼터뷰 시험 렌더에서 쓴 세 번째 규칙(평지 가장자리 인공 벽 제거)은 입체 그리기에만 있는 결함이라 위에서 보는 이 페이지에는 해당이 없다.
- 그리기(cairo): RGBA를 `ImageSurface`로 올리고, 로버 위치·방향·지나온 궤적(상태 패킷 자세 이력, 콘솔 보관, epoch마다 비움)·1 m 격자·축척 막대를 덧그린다.
  글자는 PangoCairo.
- 고정 문구: "지도는 지나온 관측 기록 · 분류는 그 칸을 본 위치 기준 · 오래된 칸은 흐리게 · 낮은 외톨이 칸은 숨김 · 실시간 장애물 감지가 아님". 범례 5색(관측 부족 = "관측 부족(낮은 칸은 반투명)"). 숨김·반투명 문구는 그 규칙이 켜져 있을 때만 적는다(r12, Codex P3).
- 조작(콘솔 내부만): 로버 따라가기(기본 on, 진행 방향 위), 확대·축소(휠), 끌어서 이동, 화면에서 지우기.
- 상태 표시줄: 상태·이유, epoch, 누적 거리, 자세 끊김 횟수, 받은 타일/전체, 마지막 수신 나이, 보정기 결과, 분류 경계값.
  링크가 0.5 s 넘게 끊기면 지도는 남기고 "수신 끊김"을 크게 표시한다.
- 렌더 주기 0.2 s. 페이지가 안 보이면 그리지 않는다.

### 4.11 기준면 기울기 표시 (r11, 2026-10-18)

출발: 팀 회의록(공학페스티벌 준비)의 "단차·경사 검출 샘플". 프레임 안 분류는 완만한 경사를 평면에 흡수하므로 경사를 따로 보인다.
**표시 전용**이다 — 지도 분류·융합·재생 대조·제어 어디에도 쓰지 않는다.

- **두 값을 섞지 않는다**(`terrain_map/slope.py`).
  - `plane_tilt`: 판정기가 고른 기준면(대개 가까운 바닥 — 먼 경사로는 반영되지 않을 수 있음)의 **차체 축 대비** 앞(pitch, 앞이 오르막 +)·옆(roll, 왼쪽이 높음 +) 각. 처리한 프레임(FRESH + 지지면)마다.
  - `gravity`: 같은 기준면의 **중력 대비** 앞·옆·전체 경사와 차체 앞 들림(asin(u·x̂))·왼쪽 들림(asin(u·ŷ)) — Euler 쌍이 아니다.
    **정지 구간에서만** 낸다: depth stamp t에 대해 [t − `still_window_s`, t]의 표본만(미래 금지) — 가속도·자이로·바퀴가 각각 창을 덮고(간격 ≤ `wheel_gap_invalid_s`, 개수 ≥ 0.8 × 창 × 표본률),
    자이로 3축 크기 < `still_max_gyro_rad_s`, 가속도 축별 표준편차 < `still_max_accel_std`, 평균 크기가 g·(1 ± `gravity_tolerance`),
    **모든 유효 바퀴의 지령·측정이 정지**(odometry 정지 판정과 같은 식, 바이어스 학습 여부는 쓰지 않음)일 때만 u = 평균 가속도 방향(정지 시 비력은 위쪽 +g).
    등속·등가속 주행은 IMU가 고요해도 바퀴 조건에서 걸러진다(Codex 반례: 1 m/s² 가속이 평지를 +5.8°로 보임).
- 수식: f = normalize(x̂ − (x̂·u)u)(퇴화 시 무효), l = u×f, pitch = atan2(−n·f, n·u), roll = atan2(−n·l, n·u), total = acos(n·u), n·u ≤ 0 이면 무효. n = Rᵀ·(광학 평면 법선), 가속도도 Rᵀ로 차체 축.
  L515 SDK motion 축은 depth 광학 축과 정렬된다(Intel 답변 — 실물 확인은 §7.3).
- **기준 시각·만료**: 프레임마다 그 stamp로 판정한 불변 값을 저장, 상태 패킷은 프레임 수신 후 `slope_max_age_s`(1 s) 안일 때만 싣고 `age_ms`를 보낸다. 콘솔은 age + 수신 후 경과 > 1 s면 숨긴다.
  epoch 전환 때 정지 판정 버퍼를 모두 비운다(연속성이 끊긴 곳의 표본을 넘겨 쓰지 않음, 시계 역행은 항상 epoch).
- **재생**: 녹화에 `/l515/accel/sample`을 더한다(6토픽). 재생의 구간 분할·식별·정렬·처리 시계·`process_tick`은 지도 4토픽으로만 정하고,
  accel은 **기록 순서상 그 지도 구간이 녹화되던 동안**(구간 경계 = 정렬 전 기록 순번의 구간 시작) 들어온 것만 그 구간에 배정해 stamp 순으로 `on_accel`만 부른다.
  다른 시계 구간의 accel을 stamp로 고르지 않는다. 경사 표시는 재생 재현 보장 범위 밖이다(결정 기록 `result`에 프레임별 값을 남긴다).
- 파라미터(`MapConfig`): `still_window_s` 0.5(0.2–2), `still_max_gyro_rad_s` 0.05(0.005–0.5), `still_max_accel_std` 0.1 m/s²(0.01–1), `gravity_tolerance` 0.1(0.02–0.3), `slope_max_age_s` 1.0(0.2–5).
- 범위 밖: 주행 중 중력 기준 경사(자이로 전파 + 가속도 보정 추정기 필요), 근거리 재적합, 칸별 경사.

## 5. 패킷 계약 v1 (UDP :5010)

### 5.1 채널

| 항목 | 값 |
|---|---|
| 방향 | Jetson → 운용 PC, UDP, best effort |
| 포트 | **5010** (`L515_TERRAIN_MAP_UDP_PORT`, 저장소 사용처 없음 2026-10-17 확인) |
| 최대 크기 | 1200 B — **상태·타일 모두** 최대 표현 길이를 시험으로 검증한다 |
| 인코딩 | UTF-8 compact JSON. 타일 칸 데이터는 base64 |

### 5.2 공통 필드

`schema_version`(1), `type`(`"terrain_map_status"`·`"terrain_map_tile"`), 세션 세 값(PR #15 §5.2와 같은 형식), `map_epoch`(0 이상 정수), `sent_unix_ns`.

### 5.3 상태 패킷 `terrain_map_status`

| 필드 | 규칙 |
|---|---|
| `sequence` | 상태 패킷마다 단조 증가(epoch 전환 뒤에도 계속 증가 — 수신부가 하한을 초기화) |
| `status`·`reason` | `OK`(null) / `NO_POSE`(`ODOM_MISSING`·`POSE_GAP`·`WHEELS_STALE`·`WHEEL_SLIP`(r13)·`BIAS_PENDING`) / `NO_GROUND`(`GROUND_NOT_FOUND`·`NOT_SUPPORT`) / `SENSOR_ERROR`(`DEPTH_STALE`·`DEPTH_ENCODING`·`DEPTH_SHAPE_MISMATCH`·`CAMERA_INFO_MISSING`·`PROCESSING_ERROR`) |
| `cell_m`·`tile_cells`·`age_step_s` | 유한 > 0·정수·유한 > 0 |
| `pose` | `{x_m, y_m, yaw_rad}`(보정 후) 또는 null |
| `distance_m` | epoch 누적 주행 거리 ≥ 0 |
| `pose_gaps`·`not_support`·`late_dropped` | epoch 누적 자세 끊김 수·지지면 검사 실패 수·늦게 와 버린 바퀴·자이로 표본 수 |
| `tile_count`·`refresh_cycle_s` | 현재 타일 수, 순환 한 바퀴 시간(s) |
| `thresholds` | `{obstacle_height_m, ditch_depth_m, min_observations}`(표시용) |
| `corrector` | `{name, applied, score, reason}` |
| `proc_ms` | 마지막 처리시간 또는 null |
| `plane_tilt` (r11, 선택) | `{pitch_deg, roll_deg, age_ms}` 또는 null/없음 — 각 ±90, age 0–1e6 ms(§4.11) |
| `gravity` (r11, 선택) | `{slope: {pitch_deg, roll_deg, total_deg}, body_tilt: {pitch_deg, roll_deg}}` 또는 null/없음 — 정지 구간에서만(§4.11). 옛 파서는 모르는 키를 무시해 호환 |

최대 길이: 모든 문자열을 허용 최대 길이로, 수를 최대 자릿수로 채운 표본이 1200 B 이하(시험).

### 5.4 타일 패킷 `terrain_map_tile`

| 필드 | 규칙 |
|---|---|
| `tx`·`ty` | 정수, |값| ≤ 2^20 |
| `version` | 1 이상 정수(epoch 안 전역 카운터) |
| `tile_seq` | 1 이상 정수. epoch 안 **타일 패킷마다** 단조 증가하는 송신 순번(갱신·순환 공용). 같은 version의 선후 판정용 |
| `cells` | base64. `tile_cells²`칸 × 2 바이트(행 = `gy` 증가, 열 = `gx` 증가): 바이트 0 = 표시 높이 `round(h·100)`을 −127…127로 자른 int8(**−128 = 미관측**), 바이트 1 = 상위 3비트 분류 코드(0 주행 가능, 1 돌출, 2 도랑·낙차, 4 관측 부족, 3·5–7 예약) + 하위 5비트 칸 나이(`⌊age/age_step_s⌋`, 31 포화, 송신 시각 기준) |

- 16×16 타일: 512 B → base64 684자, 전체 약 1.0 KB ≤ 1200 B. 표시 높이는 `max_abs_height_m`(≤ 1.2 m) 안이라 int8 cm에 들어간다(분류는 원본으로 계산).
- 파서는 PR #15처럼 타입·범위·길이를 모두 검사하고 위반이면 패킷 전체를 거부한다. 미관측 칸의 분류 코드는 4, 예약 코드는 거부.

## 6. 녹화·재생

- **녹화**(Jetson): `scripts/record_terrain_map_bag.sh <이름>` →
  `ros2 bag record -o <data>/<이름> /l515/depth/image_rect_raw /l515/depth/camera_info /l515/color/camera_info /l515/gyro/sample /wheel_states`.
  지도 노드가 쓰는 원 입력만 녹화한다(자세는 재생 때 같은 평면 추측 항법으로 다시 계산). depth 640×480×2 B × 10 Hz ≈ 6.1 MB/s ≈ 369 MB/분. 압축 없음(CPU 절약). 시작 전 디스크 여유를 확인한다.
- **결정적 재생(기본)**: `python3 -m powertrain_ros.terrain_map_replay <bag> [--corrector …] [--decisions-from processed.jsonl] [--out map.npz] [--send 127.0.0.1:5010 --rate 1.0]`.
  - `rosbag2_py`로 읽은 **녹화 순서**에서 먼저 epoch 경계를 정한다(Codex 8회차 P2-6): 토픽 안에서 헤더 stamp가 `stamp_regression_epoch_s` 넘게 역행하는 지점이 경계다
    (예: `[100, 100.02, 10, 10.02]`는 두 구간). 그 이하 역행은 버린다. 그다음 **구간마다** 헤더 stamp 순으로 실시간과 같은 `MapRuntime`에 넣는다(ROS spin·벽시계 없음) —
    전체를 한꺼번에 정렬하지 않는다. 수신 단조시각이 필요한 판단(수신 정지 → epoch, 자세 대기 만료)은 재생에서 stamp를 수신 시각으로 써서 결정한다.
    실시간 노드도 헤더 stamp로만 순서를 정하므로 sim time이 필요 없다(r6까지의 "원 입력 재생 = `--clock` + sim time" 경로는 없어졌다).
  - 처리 프레임은 `FrameScheduler`(§4.8)가 고르고, `--decisions-from`이면 실시간 기록의 처리 프레임·채택·epoch 결정을 따른다. A·B 비교는 같은 bag·같은 규칙으로 같은 프레임 집합을 처리한다.
  - `--send`면 콘솔로 패킷을 보낸다(속도 배율 옵션) — 노트북 데모.
- **ROS 재생**(노드 자체 시험): `ros2 bag play --rate 1.0` + `ros2 run powertrain_ros terrain_map`. 노드의 공백·대기 판단 일부가 수신 단조시각이라 **배율 1.0만 지원**한다
  (0.05배면 정상 바퀴 간격 20 ms가 400 ms로 늘어 `wheel_gap_epoch_s`를 넘는다 — Codex 8회차). 다른 배율이 필요하면 결정적 재생을 쓴다.
- **재생 격리**: 재생 스크립트는 `ROS_DOMAIN_ID`를 생산 도메인과 다른 값(기본 78)으로 강제하고, 같으면 거부한다.
  bag의 `/wheel_states`가 실제 텔레메트리 송신기 등 다른 구독자에 들어가지 않게 하기 위해서다. 결정적 재생은 ROS 통신을 쓰지 않는다.
- 노트북: 로컬 ROS 시험 이미지에서 재생하고, 콘솔은 같은 노트북에서 :5010으로 받는다(`scripts/replay_terrain_map.sh`).

## 7. 테스트·검증

### 7.1 호스트 단위 시험(TDD, conda + 시스템 python)

- `contract`: 두 패킷 왕복, **상태·타일 최대 길이 ≤ 1200 B**, 높이 양자화·sentinel·코드 3비트·나이 5비트, 거부 사례(타입·범위·base64 길이·미관측인데 코드 ≠ 4·예약 코드).
- `geometry`·`scan` — **정답은 판정기와 독립인 세계 좌표 기준으로 만든다**: 세계 점 → 차체 자세(롤·피치) → 장착(회전·병진) → 광학 좌표로 depth를 렌더한다.
  - Codex 반례 세 개에서 `(2, 0)` 점의 base 수평 좌표 오차 < 1 cm: 카메라 원점 고정 롤 반례(광학축 투영 24.75 cm), 장착 병진 0.5 m·롤 20°(원점 이동 0.171 m),
    **기본 장착 (0.348, 0, 0.029)·20° — `−Rᵀt`로 쓰면 23.99 cm인 식 오류 반례**.
  - 프레임 안 분류: 상자 → 돌출, 패임 → 도랑, **계단(디딤폭·높이 10 cm 8단)·`0 → 0.1 → 0.2 m` 두 단 → 돌출**, 폭 60 cm·80 cm 윗면 → 돌출,
    2.4 m 도랑 → 도랑, 윗면 위에서 본 아래 바닥 → 낙차(도랑 코드).
  - 외톨이 점 무시(`[0,0,0,0,1]` → 0), 범위 밖 제외, depth 솎기 전후 같은 평면, 성긴 바닥(600후보)이 stride 2·`ground_min_points` 100에서 FRESH.
  - **지지면 검사**: 독립 렌더한 앞쪽 2.4×2.4 m·깊이 20 cm 도랑(Codex 4회차 반례, 판정기가 도랑 바닥 평면 offset 0.70 m를 FRESH로 냄) —
    시작 1.0 m는 근거리 띠 검사로 `NOT_SUPPORT`, 시작 0.45 m는 근거리 띠는 통과하지만 base 높이 검사로 `NOT_SUPPORT`, 높이 검사를 끄면 통과(한계 문서화와 일치).
    **Codex 5회차 반례**: 차체 10° 숙임 평지 → 통과(정상 바닥을 거부하지 않음), 15° 숙임 + 10 cm 도랑(0.30 m 시작) → `NOT_SUPPORT`. r5식 카메라 높이 비교로 바꾸면 둘 다 반대로 나오는 음성 대조.
    음성 대조: 두 검사를 모두 끄면 도랑 칸이 코드 0으로 들어감. 평지와 근거리 띠 밖(카메라 전방 0.8 m 이상)의 작은 상자 장면은 통과, 근거리 띠 안 상자는 제외.
  - **비대칭 임계값 심각도**: 돌출 0.08·낙차 0.05에서 `+7 cm`(0)와 `−6 cm`(2)가 한 칸 → 2. 칸 안 `hu`/`hd` 선택도 같은 규칙.
- `pose`(`PoseTrack`): 보간(yaw 감김), 간격 초과·무효 표본·`WAITING`, epoch 신호 시 비움.
- `odometry`(`PlanarOdometry`, 호스트 conda — 순수 모듈):
  - **Codex 6회차 반례**: 0.5 m/s·0.5 rad/s 선회 2 s, 바퀴 표본만 200 ms 늦게 도착 — 워터마크 정렬로 같은 depth 시각의 위치·방향 오차 < 2 cm·< 0.5°(도착 순 처리면 수 cm 이상·5.73°로 실패하는 음성 대조. 시제작: 워터마크 1.00 cm·0.00°, 도착 순 5.73°).
  - **Codex 7회차 반례**: 선회 중 자이로 끊김 → 새 epoch(6.88° 틀린 유효 자세가 나오지 않음), 바퀴 0.5 s 공백 → 새 epoch(24.9 cm 어긋난 좌표로 기존 지도에 합류하지 않음),
    0.03 rad/s 실제 회전 중 기동 → 정지 전까지 `BIAS_PENDING`, 같은 stamp 자이로·바퀴는 바퀴 먼저.
  - **Codex 8회차 반례**(실제 `WheelStates` 형태 입력부터): 애커만 0.5 m/s·조향 1.0 90° 선회 → 최소제곱 속도로 위치 오차 < 2 cm(단순 평균이면 14.6 cm로 실패하는 음성 대조),
    스키드 유효 4개 비대칭 부분집합 → < 2 cm(평균이면 22.3 cm), 자이로 200 ms 공백 안 삼각형 회전 → 새 epoch(유지하면 2.86° 오차), 제자리 0.03 rad/s 저속 회전(명령 0.0166 turn/s) → 정지 아님·bias 갱신 없음,
    첫 정지 구간 → 그 평균이 바로 bias(EMA 20 % 아님), 자이로 100 Hz·바퀴 stamp 20 ms·수신 25 ms 정체 주입 → `max_stream_skew_s`에서 새 epoch·큐 상한 유지.
  - **Codex 9회차 반례**: 직진(모든 δ = 0) → rank 1이라 `v_y = 0`·`v_x`만 풀이, 애커만 조향 0.001에 바퀴 속도 ±0.001 m/s 교란 → 조건수 상한으로 `v_y` 0(1.75 m/s가 나오지 않음),
    0.8 s 정지 뒤 0.2 s 바퀴 전부 stale·자이로 0.02 rad/s → 학습 구간이 끊겨 bias 갱신 없음, 2 m/s² 감속 중 0.29 s 바퀴 공백 → 오차가 `½·|a|·gap²` 식과 맞음(조건부 예산 확인).
  - 늦은 표본(워터마크 이전) 버림·카운트, 바퀴 짧은 공백 → 해당 구간 무효·외삽 오차 상한, stamp 역행 > 1 s → 새 epoch.
  - **바퀴 합의(r13)**(`max_slip_wheels 2`): 직진 한 바퀴 헛돎(나머지 0.3 m/s, 한 바퀴 3 m/s) → 그 바퀴를 빼고 0.3 m/s·유효(최소제곱이면 0.75 m/s로 실패하는 음성 대조),
    두 바퀴 헛돎 → 둘 다 빼고 합의, 애커만·스키드 선회 기하(헛돎 없음, 0.5 m/s·0.5 rad/s, 회전반경 0.7 m) → 합의 단계에 들어가지 않고 r12 풀이와 같음,
    **애커만 조향 1.0 중 앞왼쪽 3 m/s**(전체 풀이가 `|v_y|` 상한으로 무효) → 그 바퀴를 빼고 참값 `(0.5, 0.0391)` 복원,
    바퀴가 제각각(합의 없음) → 최선 추정 적분·`WHEEL_SLIP` 무효·공백·epoch 아님, `slip_epoch_s` 넘게 이어지면 새 epoch(`slip`), 그 전 복구면 epoch 유지,
    합의 실패 사이의 실제 바퀴 공백·자이로 공백은 그대로 무효·epoch, 빼면 `min_valid_wheels` 미만 → 빼지 않음,
    합의 후보 동률(같은 SSE)·실패 후보 동률(Codex r13 `[0, .25, .5, .75, 1, 1.25]` → 최대 잔차 0.375 동률 셋)이 입력 순서를 뒤집어도 같은 선택,
    조건수 50 경계(애커만 조향 0.045, 전체 47.75·중간 바퀴 제외 43.64·코너 제외 50.85–53.55)에서 잡음 표본의 결정성,
    `max_slip_wheels = 0` → **r12 모듈(커밋 전 HEAD)에서 뽑은 자세 고정값**과 같음(새 구현끼리 비교하면 r12 회귀를 못 잡는다 — Codex r13 3회차), `WHEEL_SLIP` 표본 양옆 `PoseTrack` 보간 차단,
    허용치 경계에서 입력 순서만 바꾼 표본이 같은 판정(Codex 반례 `[0.33333333333333337, 0.2, 0.2, 0.2]`), 바이어스 준비 전 slip epoch 없음(`bias_stationary_s`·`slip_epoch_s` 1.0·0.5와 1.5·1.0),
    합의 실패 중 0.5 s마다 유효 바퀴 부족 표본이 끼어도 slip epoch가 열림, 실패 1 s 뒤 바퀴 부족 0.2 s를 거쳐 복구해도 epoch(0.9 s면 없음), 네 바퀴 같은 헛돎은 잘못된 다수(한계 고정),
    합의 실패 뒤 바퀴 메시지 0.2 s 누락·복구에서 실시간과 재생의 자세열이 같음,
    바이어스 학습 자격이 합의와 무관(정지 중 큰 미학습 바이어스로 합의가 실패해도 학습 — Codex r13 2회차 반례 0.4 rad/s·0.10, 0.12 rad/s·0.03), 헛도는 바퀴가 있으면 정지 아님, 누적 거리가 `WHEEL_SLIP` 구간 경로를 따라 더해짐, 노드 로그 누계,
    합의 실패·복구를 포함한 재생 결정성과 `--decisions-from` 동등성.
  - `wheel_radius_m`·바퀴 위치가 `chassis.kinematics.default_geometry()`와 같음(노드 전달 경로 시험).
  - 장착 회전: 모든 각 0에서 `Rᵀ` = `(z, −x, −y)`, pitch 20°·roll 20° 각각과 복합각(roll 20°·pitch 20°·yaw 30°)에서 평평한 차체 순수 요 회전이 그대로 복원
    (다른 곱 순서로 바꾸면 실패하는 음성 대조).
- `schedule`: 실시간·재생 같은 입력에서 같은 선택, 도착 지연 30 ms 반례, **자세 지연 250 ms·10 Hz에서 대상이 축출되지 않고 처리됨**(Codex 3회차 P1-5 반례),
  **입력 지연 → 워터마크 → 스케줄러 전체 경로**(Codex 8·9회차): 바퀴 도착 지연 0·200·250 ms는 모든 틱 위상에서 처리(보장 구간), 300 ms는 처리(위상에 따름 — 실패로 보지 않음),
  450 ms(> `pose_wait_s` − 여유)는 만료(손실), 같은 틱에 자세가 생기면 만료보다 처리 우선, 설정 검증(`pose_wait_s` 하한),
  의도적 생략과 손실 구분, 설정 검증(용량 부족 거부), `--decisions-from`.
- `classmap`: 항등 칸 1:1, 두 자세(1 m 전진·90° 회전)에서 같은 상자가 같은 세계 칸, 3중 2 다수결(잡음 1회 무시·치운 물체 2회 뒤 사라짐·윗면 위로 올라선 뒤 2회면 주행 가능),
  **이력 만료**(Codex 4회차 반례: `t=0,0.2` 코드 0 두 표 뒤 `t=120` 낙차 한 표 → 낙차), 같은 세계 칸 충돌의 심각도 규칙(45° 회전·`(0.01, 0.02)` m 이동 반례),
  표시 높이는 마지막 관측값(섞지 않음), 전역 version — 나이 비트만 바뀌면 그대로, **관측 부족 해제(4 → 0)는 올림**, 재생성 타일, 상한 초과 제거.
- `correction`: Identity 불변, Δ 합성(`P ∘ O⁻¹`), `ClassMap.sample` 부작용 없음, B 시험 장치(알려진 오프셋, 모호 무늬 거부, 자른 자세 재평가,
  **높이 기준 1.2 cm 차이에서 정답 자세 유지** — Codex 4회차 반례)의 형태 시험.
- `runtime`: 상태 우선순위, FRESH만 융합, epoch 시 전부 비움, 처리시간 기록, 송신 일정(갱신 우선·순환·상한).
- 콘솔(시스템 python): `SessionSelector` 분리 후 PR #15 시험 그대로 통과, 역순·유실 패킷에서도 상태·타일 수렴(Codex 1회차 1024/4096 반례),
  epoch 2 → 지연 epoch 1 → epoch 2 반례, 상태보다 먼저 온 타일, 같은 version 순환이 TTL과 **칸 나이**를 갱신(79 s 반례·오래된 칸이 30 s 뒤 흐려짐),
  **같은 version 역순**(`v=7, seq 20, age 32 s` 뒤 `v=7, seq 12, age 0` → 무시), **TTL 삭제 뒤 같은 역순 패킷도 무시**, **관측 부족 해제 두 패킷 역순**,
  작은 version은 연장 안 함, `map_rgba`(분류 경계 유지, 쌍선형 오버슈트 없음, 미관측 투명, 칸 나이 흐림, 픽셀 상한).

### 7.2 실행 스모크(프로젝트 §5 완료 규칙)

- 콘솔 `runtime_smoke`에 map 채널 주입 → 지도 페이지 프로브(타일 수·상태)·traceback 없음. 음성 대조: 채널 생략 → FAIL.
- ROS(로컬 컨테이너, 도메인 77): **compose 서비스와 같은 진입점**(`bash -lc` → ROS·설치본 source → `exec ros2 run powertrain_ros terrain_map …`)으로 지도 노드를 기동하고,
  fixture가 합성 depth·CameraInfo·IMU(gyro·accel)·`/wheel_states`(직진)를 발행 → UDP로 상자 타일·OK 상태·전진하는 자세.
  - 격리 확인: 지도 노드가 만든 발행자가 없음(`ros2 node info`), `/tf`에 변환 메시지 없음.
  - 음성 대조: `/wheel_states`를 모두 stale로 → `NO_POSE`/`WHEELS_STALE`.
- 재생 스모크: 위 fixture를 녹화 → 결정적 재생이 같은 타일·분류를 내는지(`--decisions-from`으로 실시간 결정 재현), **녹화 순서 `[100, 100.02, 10, 10.02]` epoch 분할**, `ros2 bag play` + 노드가 `NO_POSE` 없이 지도를 만드는지, 도메인 78 강제.

### 7.3 실기 (Jetson, 사용자 승인 후)

1. 정지 상태에서 상자·단차·도랑(또는 패임) 앞 → 지도 위치·분류. 줄자 거리와 비교. L515 장착 위치·기울기를 줄자·수평계로 재서 파라미터 교체.
   평지 정지 상태의 `n·o + d` 중앙값(30 s)과 줄자로 잰 base_link 높이를 비교해 `support_base_height_m`을 정하고 켠다. 험지 위 정지 자세 몇 개에서 값의 퍼짐을 재서 허용폭을 정한다.
2. 5 m 직진·90° 회전 주행(0.5 m/s 이하) → 같은 상자가 한 덩어리로 쌓이는지, 누적 거리·회전각 대 실측.
   자세 대기(`WAITING`) 시간 분포·만료 수, IMU 대기 초과 수를 보고한다.
   큰 도랑·바로 앞 단차 장면에서 `NOT_SUPPORT` 빈도를 기록한다.
   **바퀴 합의 허용치(r13)**: 직진·애커만·스키드 제자리 선회·급가속/급정지를 담은 bag을 `--param max_slip_wheels=2`로 결정적 재생해(재생 요약 JSON의
   `slip_excluded`·`slip_guessed`) 헛돎 없는 주행에서 누계가 0에 가깝고 지도가 합의 끔과 같은지 본다(가감속 중 바퀴별 측정 시각 차 포함).
   아니면 `slip_residual_m_s`를 올려 다시 재생하고, 통과하면 실기 기본값을 2로 켠다(체크리스트 4-7).
3. 자원(10분): 두 SRT + 오버레이 노드 + 지도 서비스 동시 — 합격 기준(기본, 실측으로 조정):
   - `proc_ms` p95 ≤ 150 ms(처리 목표 5 Hz 간격 200 ms 안).
   - **처리 손실률** = (스케줄러가 고른 대상 중 처리하지 못한 수) / (고른 대상 수) ≤ 10 %. 의도적 생략(10 Hz 중 5 Hz)은 분모·분자에 넣지 않는다.
   - FRESH 비율(처리한 프레임 중) — stride·`ground_min_points` 결정 근거로 기록.
   - 차체 50 Hz tick 간격 p99가 서비스 끄기 전 대비 +2 ms 이내, 워치독 이벤트 0, Jetson RAM 여유 ≥ 1 GB.
4. 녹화 1회(데모·B 평가용) → 노트북 결정적 재생으로 같은 지도(`--decisions-from`).

### 7.4 B 평가(착수 시)

같은 녹화를 결정적 재생으로 A·B 비교 — 왕복 경로 같은 장애물 칸 IoU, 보정 채택률, 처리시간, 합성 참 자세 오차. 기준을 넘으면 기본값을 바꾸는 별도 커밋.

## 8. 배포

### 8.1 compose

- `docker/docker-compose.integrated.yml`에 `powertrain_terrain_map`(이미지 `powertrain-sw:ros`, profile **`terrain-map`**, host network, ipc host,
  `/workspace:ro`, `/run/powertrain` bind(`create_host_path: false`), 세션 파일 환경변수, `cpus: 1.0`, `mem_limit: 768m`, OMP/OPENBLAS 1, `restart: unless-stopped`, healthcheck 비활성).
- **진입점**(Codex 3회차 P1-6): 생산 이미지 ENTRYPOINT(`docker/powertrain_ros_entrypoint.sh`)는 인자를 무시하고 Gateway를 띄운다. PR #15 오버레이 서비스처럼
  `entrypoint: [/bin/bash, -lc]`로 덮어쓰고, `command`에서 `set -e` → `test -f /workspace/ros2/install/setup.bash` → `source /opt/ros/humble/setup.bash` →
  `source /workspace/ros2/install/setup.bash` → `exec ros2 run powertrain_ros terrain_map --ros-args -p operator_host:=127.0.0.1`.
- 지원 배포는 저장소 소스가 `/workspace`에 마운트되고 `PYTHONPATH`에 들어가는 현재 Compose 경로뿐이다(지도 노드가 루트 패키지 `terrain_map`·`terrain_overlay`를 import한다. `setup.py`는 `powertrain_ros`만 설치한다).
- 노드가 끝나면 컨테이너가 종료되고, `restart: unless-stopped`가 종료 코드와 관계없이 다시 띄운다(새 세션).
- `robot-start` 기본 목록에는 §7.3-3 통과 전까지 넣지 않는다(PR #15와 같은 원칙).

### 8.2 안전 경계

- 지도 노드는 `/l515/*`·`/wheel_states`를 **구독만** 한다. ROS 토픽·TF·서비스를 새로 내지 않으므로 기존 노드의 입력이 바뀌지 않는다.
  (r1–r6은 `odometry`·`imu_tilt`를 띄워 `/odom`·`/imu/filtered`를 재매핑했다. `chassis_node`가 `/odom`·`/imu/filtered`를, `chassis_telemetry_sender`가 `/odom`·`/imu/filtered`·`/odom_diagnostics`를 구독하므로
  재매핑이 빠지면 진단·텔레메트리가 바뀌는 위험이 있었다 — r7에서 그 노드들을 띄우지 않아 위험 자체가 없어졌다.)
- 배포 전 확인: 지도 서비스 기동 전후 `ros2 topic list -v`의 발행자 목록이 같고, `/tf`에 새 변환이 없음. 다르면 배포를 멈추고 사용자에게 보고한다.
- 기존 `odometry`·`imu_tilt`·`state_estimation.py` 코드는 바꾸지도 import하지도 않는다(지도 전용 평면 추측 항법, §4.4).

## 9. 문서

- `docs/network-ip-setup.md`·`docs/integrated-operation.md` 포트 표에 :5010, 켜는 법, 지도 작성 권장 속도 0.5 m/s.
- `operator_console/README.md`에 지도 페이지·인자.
- `terrain_map/README.md`: 파라미터 표(§4.1)·분류 규칙·녹화/재생 사용법 — 선이 파트가 기준값·표시 방식을 바꿀 때 보는 곳.
- 결과: `docs/reports/2026-10-xx-l515-terrain-map.md`.

## 10. 남은 위험

- **위치 오차**: 미끄러짐 많은 험지에서 지도가 휜다(A의 본질적 한계). 누적 거리·자세 끊김·칸 나이를 함께 보여 오해를 줄이고, B로 완화.
- **기울어진 선회**: 기존 추정 코어가 차체 z축 자이로를 적분하므로 차체가 롤 φ로 기운 채 선회하면 방향이 `cos φ` 비율로 작아진다(20°에서 약 6 %), 가짜 피치도 생긴다(Codex 3회차, 약 5°).
  평면 추측 항법이 차체 z축 자이로를 쓰기 때문이며 이번 범위에서 보정하지 않는다(중력 방향 투영이 필요). 경사면 선회가 많은 코스에서는 B 또는 보정이 필요하다.
- **바퀴 미끄러짐**: 최소제곱 속도는 바퀴가 미끄러지지 않는다고 본다. 한두 바퀴의 헛돎은 바퀴 합의로 거르지만(r13, 실기 잔차 측정 전에는 꺼 둔다), 모든 바퀴의 공통 미끄러짐·갇힌 공회전은 못 잡는다.
  스키드 선회의 미끄러짐은 좌우 대칭이면 상쇄되지만 비대칭 유효 부분집합에서는 남고, 허용치가 실기 기하 불일치보다 작으면 정상 바퀴를 빼 편향을 만들 수 있다. 험지 슬립은 위치 오차로 쌓인다(B로 완화).
- **계약 추가(r13)**: 상태 이유 `WHEEL_SLIP`이 늘었다. 수신부가 모르는 이유를 거부하므로 **콘솔과 Jetson은 같은 커밋으로 배포**한다(옛 콘솔은 그 상태 패킷을 버린다).
- **자이로 공백 epoch 빈도**: 60 ms 넘는 자이로 공백마다 지도가 비워진다. Gateway 부하에서 간격 분포를 실측해 잦으면 임계·Gateway IMU 주기를 조정한다(§7.3).
- **위치 시각**: 표본을 stamp 순서로 코어에 넣으므로 입력 지연은 틀린 위치가 아니라 대기·손실로 나타난다. 위치 해상도는 바퀴 표본 간격(20 ms) 보간이다.
  자이로가 `gyro_gap_epoch_s`, 바퀴가 `wheel_gap_epoch_s` 넘게 끊기면 새 epoch로 지도를 비운다(어긋난 좌표로 합류하지 않는 대가). 기동·재시작 직후 첫 정지(1 s)까지 지도가 쌓이지 않는다. 옆 미끄러짐은 무시한다. 지도 작성은 0.5 m/s 이하 권장.
- **관점 의존 분류**: 같은 칸이 아래에서는 돌출, 위에서는 주행 가능으로 바뀐다. 문구로 "본 위치 기준"을 고정 표시한다.
- **경사로**: 평면 앞쪽이 높아지는 구간이 돌출로 보인다. 경사 진입 전 운용자가 영상으로 확인해야 한다.
- **원거리 높이 오차**: 평면 1° 오차는 3 m에서 5 cm. 가까이 다가가며 다수결로 덮이지만, 멀리서 한 번 본 칸은 틀릴 수 있다.
- **CPU**: 판정기 두 개(오버레이·지도) + 지도 처리. stride 2·5 Hz로 시작해 §7.3-3 기준으로 판정, 못 맞추면 노드 통합(§11).
- **먼지·부유물**: 높이 순서통계·다수결·`min_observations`로 일부 걸러지지만 짙으면 허위 장애물이 생긴다(agy 8). 실기에서 확인.
- **조도**: 직사광선 아래에서는 지도가 비거나 `NO_GROUND`가 계속된다. 시연 장소를 미리 확인한다.
- **장착값 미측정**: 운용 URDF와 CAD 값이 다르다. 실측 전에는 지도 위치에 수 cm 오차가 있을 수 있다.
- **서비스 재시작**: 콘솔 지도가 비워진다. 녹화가 있으면 재생으로 다시 볼 수 있다.
- **지지면 검사의 한계**: 로봇 바로 앞(카메라 전방 0.3–0.8 m ≈ 앞바퀴 앞 0.1–0.6 m)이 바닥이 아니면 그 프레임은 버려진다 — 도랑 가장자리·바위 앞에서는 지도가 늦게 찬다.
  base 높이 검사를 켜기 전(높이 실측 전)에는 **근거리 띠 안에서 시작하는 넓은 도랑**의 바닥을 지지면으로 받아들일 수 있다 — 실측 즉시 켠다.
  근거리 띠 안(카메라 전방 0.3–0.8 m)의 작은 물체도 근거리 중앙값을 올려 그 프레임을 버리게 한다(Codex 5회차: 0.35 m의 30 cm 상자 → 중앙값 0.133). 보수적 제외로 받아들인다.
  같은 판정기를 쓰는 **PR #15 오버레이는 이 검사가 없어 앞쪽 큰 도랑 바닥을 바닥으로 잡을 수 있다** — 후속 수정 대상(같은 검사 이식).
- **base 높이 검사의 보장 범위**: `var ≤ tol < ditch_depth − var`일 때만 켤 수 있고, 그때 분류 임계 깊이 이상의 근거리 도랑은 지지면으로 받아들이지 않는다.
  실측 var가 임계의 절반 이상이면 켜지 못하고 근거리 띠 검사만 남는다(근거리 띠 안에서 시작하는 넓은 도랑 오인 가능).
- **기존 `imu_tilt` 장착 결함**: 운용 중인 `imu_tilt`는 20° 하향 장착을 반영하지 않아 선회각이 cos20° 비율로 작게 적분된다(Codex 2회차). 지도는 이 노드를 쓰지 않지만,
  시각화·자율주행 launch는 영향을 받는다 — 별건으로 보고한다.
- **10-23 일정**: Jetson 접속이 지금 안 된다. 호스트·로컬 컨테이너·결정적 재생 경로를 먼저 완성하고 실기는 접속 즉시.

## 11. 확장 여지

오버레이·지도 노드 통합(판정기 1회), 프레임 사이 일관된 고도(IMU 피치 적분 + 정합), 경사 속성 칸, 기울어진 선회 보정, 입체 메시 보기(GtkGLArea), 지도 저장·불러오기, 루프 클로저.

## 12. 자체 검토

### ① 직접 재계산 (Claude, 2026-10-17·18)

- 타일 패킷: 16² × 2 B = 512 B → base64 ⌈512/3⌉×4 = 684자. Codex 실측 compact JSON 992 B(r1 형식). r4는 `z0_cm`·타일 나이를 빼서 그보다 작거나 같다 — 시험으로 확정.
- 전송량: 틱당 상태 1 + 타일 ≤ 20(12 + 8) × 5 Hz ≈ 105 pkt/s × ≤ 1.2 KB ≈ 126 KB/s ≈ 1.0 Mbit/s.
- 순환 한 바퀴: 1024 / (8 × 5) = 25.6 s. TTL = 3 × 25.6 + 2 ≈ 79 s — 같은 version 순환이 생존·나이를 갱신한다.
- 판정기 입력: stride 2 → 320×240 = 76,800점.
- 장착(보관 CAD): base_link → `l515_bottom_screw_frame` (0.33340, 0, 0.01189) pitch 0.349 rad, → `l515_link` (0.0075, 0, 0.02115).
  base 기준 x = 0.3334 + 0.0075·cos20° + 0.02115·sin20° = **0.3477**, z = 0.01189 − 0.0075·sin20° + 0.02115·cos20° = **0.0292** m.
  `R·x̂ = (0, −0.342, 0.940)`, `o = −R·t = (0, 0.146, −0.317)`(|o| = 0.349 m ≈ |t|). `−Rᵀt = (−0.027, 0.348, 0.010)`은 다른 벡터(Codex 3회차 P1-1 확인).
- Codex 반례 재현: 광학축 투영 오차 24.75 cm(차체 전방 투영 0.0 cm), 카메라 원점 0.5 m·롤 20° 이동 0.5·sin20° = 0.17101 m, IMU 90°·cos20° = 84.5723°.
- 프레임 안 분류 확인(numpy, 5 cm, 경계 ±0.08): 계단 10 cm×8단 → 첫 단부터 돌출, `0→0.1→0.2` → 돌출, 80 cm 윗면 → 돌출, 2.4 m 도랑 → 도랑,
  윗면 위에서 본 아래 바닥(−0.2) → 도랑(낙차). 다수결: `[1,1,0]→1`, `[1,0,0]→0`, `[1,2,0]→0`(최근).
- 순서통계: n = 5, f = 0.1 → k = 1 → `[0,0,0,0,1]`에서 0.
- 칸 나이 5비트 × 2 s = 62 s ≥ `stale_display_s` 30 s.
- pending 용량: `⌈0.3 s × 10 Hz⌉ + 1 = 4`.
- 지연 예산: 0.06 + 0.02 = 0.08 s → 1.5 m/s 12 cm, 0.5 m/s 4 cm.
- 기울어진 선회: 롤 20°에서 `1 − cos20° = 6.0 %`.
- 회전 정의 검산(numpy): `R = P·Mᵀ`, `M = Rz·Ry·Rx` — 모든 각 0에서 `Rᵀ·(x,y,z) = (z, −x, −y)`(기존 `opt_to_body`와 같음), pitch 20°에서 `R·x̂ = (0, −0.342, 0.940)`,
  복합각(20°, 20°, 30°)에서 차체 요 회전 `(0,0,1)`이 광학으로 갔다가 `Rᵀ`로 정확히 되돌아옴.
- 최소제곱 속도 검산(`chassis.kinematics` 실제 기하): 애커만 0.5 m/s·조향 1.0 → ω 0.6489, 풀이 `(0.5, 0.03915)` = 참값(평균 0.55458),
  스키드 0.5 m/s·0.5 rad/s 유효 {front_right, mid_right, rear_left, rear_right} → 풀이 0.5(평균 0.579).
- 평면 추측 항법 순서 시제작(순수 python): 선회 중 바퀴만 200 ms 늦게 도착 → 워터마크 처리 1.00 cm·0.00°(시작 속도 보유 탓), 도착 순 처리 5.73°.
- base 높이 보장 조건: 측정 = 실제 높이 + D(평면이 깊이 D 도랑 바닥일 때). 평지 수락 ⇔ |var| ≤ tol, 도랑 거부 보장 ⇔ D − var > tol.
  `ditch_depth_m` 0.08에서 var 0.03이면 tol ∈ [0.03, 0.05). Codex 6회차 반례(var −0.03, D 0.085, tol 0.06)는 0.085 − 0.03 = 0.055 < 0.06이라 통과했던 것 — tol 0.03이면 거부.
- 지지면 검사(독립 렌더, 카메라 0.5 m·20°, 기존 판정기): 도랑 시작 1.0 m → 평면 offset 0.70, 근거리 띠 90,880점 중앙값 +0.200 → 실패.
  도랑 시작 0.45 m → offset 0.70, 근거리 띠 39,040점 중앙값 0.000 → 통과(그래서 높이 검사를 더함).
- base 원점 높이 검사(독립 렌더, base 원점 0.471 m 고정, 카메라 장착 (0.348, 0, 0.029)·20°): 평지 수평 offset 0.500·base 0.471, 평지 10° 숙임 offset 0.439·base 0.471(통과),
  15° 숙임 + 10 cm 도랑(0.30 m 시작) offset 0.509·base 0.571, 수평 + 20 cm 도랑(0.45 m) offset 0.700·base 0.671(둘 다 실패). Codex 5회차 offset 값과 일치.
- 녹화량: 614 KB × 10 Hz ≈ 6.1 MB/s ≈ 369 MB/분. 메모리: 칸당 이력 3 B + 코드 1 + 높이 4 + 관측 2 + 시각 8 = 18 B × 256 ≈ 4.6 KB/타일 → 1024 타일 ≈ 4.7 MB.
- 안전 경계: 초안은 `/odom`을 그대로 띄우려 했으나 `chassis_node`·`chassis_telemetry_sender` 구독을 찾아 재매핑으로 격리했다(§8.2).

### ② Codex 적대 검토 1회차 (2026-10-17) — 판정: 미수렴(P1 5, P2 8)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | 원 입력 재생에서 추정기 출력 stamp가 현재 시각 → depth와 불일치 | 맞음 | 결정적 재생기 기본, 원 입력 재생은 `--clock` + sim time 필수(§6) |
| P1-2 | 바퀴 6축 stale에도 `/odom`이 정상처럼 나옴 | 맞음 | 지도 노드가 `/wheel_states` 유효성 직접 확인(§4.4) |
| P1-3 | 프레임 기준 높이 평균이 단차 윗면을 지움 | 맞음 | r2 δz → r4 프레임 안 분류·다수결로 대체(§4.5) |
| P1-4 | 광학축 투영 축은 차체 수평 축이 아님(롤 20°에서 24.8 cm) | 맞음 | 차체 전방 투영(§4.3) |
| P1-8 | eviction 통지 유실 시 영구 잔존 | 맞음 | 콘솔 TTL, 전역 version(§4.5·§4.9) |
| P2-5 | 최신 한 장 보관으로는 자세 대기 불가, 원점 근처 reset 미검출 | 맞음 | 스케줄러·pending, epoch 규칙(§4.4·§4.8) |
| P2-6 | 칸 반올림·q=0.9 외톨이 점 | 맞음 | 바닥 나눗셈, trim 순서통계(§4.3·§4.5) |
| P2-7 | 양자화가 경계 분류를 바꿈 | 맞음 | Jetson 분류 코드 전송(§5.4) |
| P2-9 | 전역 sequence 게이트가 역순 타일 거부 | 맞음 | 세션 선택·순서 판정 분리(§4.9) |
| P2-10 | eviction 32개 상태 패킷 1256 B | 맞음 | 목록 제거, 최대 길이 시험(§5.3) |
| P2-11 | B의 Δ 합성·재평가·재현성 | 맞음 | Δ ← P∘O⁻¹, 재평가, 결정적 재생(§4.7·§6) |
| P2-12 | `~/reset` 이름 충돌, 재생 격리 없음 | 맞음 | 노드 이름 분리, 도메인 78(§4.8·§6) |
| P2-13 | stride가 판정기 뒤 | 맞음 | 판정기 앞 솎기, 렌더 상한, 수치 합격 기준 |

### ② Codex 적대 검토 2회차 (2026-10-17, r2 대상) — 판정: 미수렴(P1 5, P2 7)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | 국소 중앙값이 넓은 단차·폭 60 cm 돌출·도랑을 모두 0칸으로 | 맞음 | r3 두 규모 분류 → r4 프레임 안 분류로 대체 |
| P1-2 | δz가 치운 물체를 기준으로 삼음, 인라이어 0개, 겹침 부족 시 0 | 맞음 | r4에서 δz 폐기 |
| P1-3 | 장착 병진 무시(0.171 m), `imu_tilt` 20° 미반영(84.57°) | 맞음(재현) | base 원점 발 지점 투영(§4.3), `imu_tilt` 장착 회전(§4.8) |
| P1-4 | 구동 fresh·조향 stale에서 지도는 통과, odometry는 `used=0` | 맞음 | odometry와 같은 유효성 판정(§4.4) |
| P1-5 | 지연된 옛 epoch 타일이 새 지도를 되돌림 | 맞음 | 같은 세션 낮은 epoch 거부(§4.9) |
| P2-6 | 같은 version 순환을 버리면 79 s에 정상 타일 TTL 삭제 | 맞음 | 같은 version 생존 갱신(§4.9) |
| P2-7 | 타일 나이 하나로 칸 나이 표시 불가 | 맞음 | 칸별 5비트 나이(§5.4) |
| P2-8 | twist 예측은 reset 식별자가 아님 | 맞음 | 재시작 경로를 서비스 재시작으로 한정 + 정상 간격 안전망(§4.4·§4.8) |
| P2-9 | 실시간·재생 프레임 선택 불일치, 손실률 분모, bag 역행 | 맞음 | 공용 스케줄러, `--frames-from`, 손실률 정의, 역행 제거(§4.8·§6·§7.3) |
| P2-10 | 재분류 ±1타일이 허용 파라미터에서 부족 | 맞음 | r4에서 이웃 재분류 자체가 없어짐(§4.5) |
| P2-11 | 솎기로 200점 자격이 깨짐, pitch 50°와 판정기 40° | 맞음 | `ground_min_points` 100, pitch 상한 35°(§4.1·§4.2) |
| P2-12 | TF broadcaster는 플래그와 무관하게 `/tf` 발행자 생성 | 맞음 | 변환 메시지 무송신으로 판정(§7.2·§8.2) |

### ② Codex 적대 검토 3회차 (2026-10-18, r3 대상) — 판정: 미수렴(P1 6, P2 4)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | `R` 정의(base→광학)와 `o = −Rᵀt`가 모순, 기본 장착에서 23.99 cm | 맞음(재현) | `o = −R·t`로 표기·정의 통일, 반례 시험(§4.3·§7.1) |
| P1-2 | δz "주행 가능" 후보 제한으로도 80 cm 윗면 소실·치운 넓은 물체 자리 바닥 상승 | 맞음 | δz 폐기, 프레임 안 분류 + 다수결(§1 전제·§4.5) |
| P1-3 | 초기 겹침이 적으면 영구 융합 정지 | 맞음 | δz 폐기로 거부 경로 자체가 없어짐 |
| P1-4 | 계단(10 cm×8단)·두 단차 가운데가 주행 가능 | 맞음 | 프레임 안 분류는 디딤판을 모두 돌출로 판정(재현, §12 ①) |
| P1-5 | 새 프레임 우선 축출로 대기 대상이 밀려 10초 처리 0건 | 맞음 | 대상 고정·축출 보호, 용량 하한 설정 검증, 대상 손실 집계(§4.8) |
| P1-6 | 생산 ENTRYPOINT가 Gateway를 띄워 launch 미도달 | 맞음 | `bash -lc` 진입점 덮어쓰기·source·exec 계약(§8.1), 스모크도 같은 진입점(§7.2) |
| P2-7 | 장착 roll·yaw 미전달, 차체 기울어진 선회 오차(84.57°·가짜 피치 5°) | 부분 | 장착 세 값 모두 gyro·accel 회전(§4.8). 기울어진 선회는 공용 코어 한계로 범위 밖·위험 명시(§10) |
| P2-8 | 같은 version에서 칸 나이 갱신 불명확 | 맞음 | 같은 version은 칸 바이트 전체 교체(코드·높이 동일, 나이만 새 값, §4.9) |
| P2-9 | odom 지연 "최대 40 ms"는 보장 아님(80 ms 공백에 12 cm) | 맞음 | 오차 상한식·`pose_max_gap_s` 0.06·권장 속도·공백 주입 시험(§4.4·§7.1) |
| P2-10 | B의 δz 평가에 필요한 상태가 조회 API에 없음 | 맞음 | δz 폐기, B는 `sample()`의 표시 높이·코드만 사용(§4.7) |

### ② Codex 적대 검토 4회차 (2026-10-18, r4 대상) — 판정: 미수렴(P1 3, P2 5)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | 앞쪽 큰 도랑 바닥이 판정기 평면이 되어 도랑 전체가 주행 가능(offset 0.70 m, 43,173점 코드 0) | 맞음 | 근거리 띠 지지면 검사, `NOT_SUPPORT`(§1·§4.2), PR #15에도 같은 한계 기록(§10) |
| P1-2 | 오래된 다수표가 새 관측을 이기고 나이 0으로 표시 | 맞음 | 이력에 stamp, `history_max_age_s` 만료 후 다수결(§4.5) |
| P1-3 | `|h|` 충돌 규칙이 비대칭 임계값에서 낙차를 지움 | 맞음 | 임계값 정규화 심각도로 칸 값·충돌 선택(§4.3·§4.5) |
| P2-4 | 지연 상한이 odometry가 실제 쓴 바퀴 시각을 보장 못 함(10 cm) | 맞음 | 조건부 예산으로 명시, 재생·실기에서 odom 대 바퀴 적분 차 p99 실측(§4.4·§7.3·§10) |
| P2-5 | 같은 version 역순 패킷이 칸 나이를 되돌림 | 맞음 | 타일 송신 순번 `tile_seq`(§4.9·§5.4) |
| P2-6 | 관측 부족 해제(4 → 0)에 version이 안 오름 | 맞음 | version 기준 = 나이 비트를 뺀 전송 바이트(§4.5) |
| P2-7 | IMU 복합 회전 순서 미정(86.70°·15.58°) | 맞음 | 공용 `base_to_optical`, `R = P·Mᵀ`, 복합각 시험·음성 대조(§4.8·§7.1) |
| P2-8 | B 점수가 높이 기준 차이에 민감(1.2 cm로 5 cm 오보정) | 맞음 | 겹침 중앙값을 뺀 오프셋 불변 점수, 반례를 B 시험 장치에(§4.7) |

### ② Codex 적대 검토 5회차 (2026-10-18, r5 대상) — 판정: 미수렴(P1 1, P2 2)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | 고정 카메라 높이 검사가 차체 숙임에서 정상 바닥 거부(0.439)·숙임+도랑 통과(0.509) | 맞음(재현) | 평면에서 잰 base 원점 높이 `n·o + d` 검사로 교체 — 차체 자세 무관(§4.2) |
| P2-2 | odom 변화량 대 바퀴 적분 비교는 일정 지연을 지움(오차 10 cm, p99 0) | 맞음 | `odometry` `stamp_source=wheel`(지도 launch만) — stamp = 적분한 바퀴 표본 시각, 지연은 대기로 드러남, 주입 시험(§4.4) |
| P2-3 | TTL 삭제 뒤 version·tile_seq 하한이 사라져 지연 패킷이 부활 | 맞음 | 하한 표를 화면 데이터와 분리, epoch·세션 때만 비움(§4.9) |
| 참고 | 근거리 띠 안 작은 상자는 과잉 제외(0.133) | 인정 | 보수적 제외로 §10에 명시, 시험 문구에 위치 범위 |
| 참고 | `setup.py`는 `powertrain_ros`만 설치 — 소스 마운트 없는 배포는 `terrain_map` import 불가 | 인정 | 지원 배포는 `/workspace` PYTHONPATH(현 Compose)로 한정 — §8.1에 명시 |

### ② Codex 적대 검토 6회차 (2026-10-18, r6 대상) — 판정: 미수렴(P1 2, P2 1)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | `stamp_source=wheel`로도 코어가 미래 IMU 방향으로 바퀴 변위를 회전 — 바퀴만 200 ms 늦은 선회에서 10.02 cm·5.73°, 반복 stamp에서도 yaw 변화 | 맞음 | 별도 `odometry`·`imu_tilt` 노드 폐기, 코어를 지도 노드 안에서 stamp 순서로 공급(`OrderedOdometry`), 방출 표본 불변(§4.4) |
| P1-2 | base 높이 변동이 도랑 깊이와 상쇄(3 cm 낮아짐 + 8.5 cm 도랑 → 통과) | 맞음 | 보장 조건 `var ≤ tol < ditch_depth − var`를 설정 검증으로 강제, 기본 tol 0.03, 못 맞추면 높이 검사 끔(§4.2·§10) |
| P2-3 | 하한 표 상한 초과 시 하한을 버리면 지연 패킷 부활 | 맞음 | 버린 하한의 `tile_seq` 최댓값 `evicted_seq_floor` 유지, 하한 없는 타일은 그보다 큰 `tile_seq`만 수락(§4.9) |

### ② Codex 적대 검토 7회차 (2026-10-18, r7 대상) — 판정: 미수렴(P1 3, P2 4)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | `imu_wait_s` 만료가 바퀴 yaw 전환을 일으키지 않음(코어 0.25 s 창, 0.24 s에 6.88°) | 맞음 | 코어 폐기 → 평면 추측 항법, 자이로 끊김은 새 epoch(§4.4) |
| P1-2 | 바퀴 공백 뒤 이동 누락 좌표로 지도에 재합류(24.9 cm·7.45°) | 맞음 | `long_gap_s` 넘으면 연속성 상실 → 새 epoch, 짧은 공백은 해당 표본 무효(§4.4) |
| P1-3 | 지지면 보장 조건에 돌출 임계 빠짐(돌출 2 cm 설정에서 5 cm 윗면 통과) | 맞음 | `tol < min(obstacle, ditch) − var`(§4.1·§4.2) |
| P2-4 | 온라인 순서 확정 조건·큐 상한 없음(도착 순 처리 8.76 cm) | 맞음 | 워터마크 `min(최신 stamp) − lateness_margin_s`까지 확정, 늦은 표본 버림·카운트, 수신 정지 시 epoch로 큐 상한(§4.4) |
| P2-5 | `--frames-from`만으로 실시간 지도 재현 불가, 재생 역행 처리 불일치 | 맞음 | 보장을 오프라인 재생끼리 결정성으로 좁힘, `--decisions-from`(채택·버림·epoch 기록), 역행 규칙 통일(§4.4·§6) |
| P2-6 | `/chassis/safety_state`는 헤더 없는 monotonic 시각 — 조향 방식을 센서 시간축에 못 놓음 | 맞음 | 조향 방식 불필요한 전진량(유효 바퀴 평균 속도)으로 바꿔 구독 삭제(§4.4) |
| P2-7 | raw IMU bias 설정·기동 순서(실제 회전을 bias로 학습) | 맞음 | 바퀴 정지 확인 구간에서만 bias, 준비 전 `BIAS_PENDING`, 같은 stamp는 바퀴 먼저(§4.4) |

### ② Codex 적대 검토 8회차 (2026-10-18, r8 대상) — 판정: 미수렴(P1 3, P2 3)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | 유효 바퀴 단순 평균이 애커만 10.92 % 과대(90° 뒤 14.64 cm), 스키드 비대칭 부분집합 0.579 m/s(22.34 cm) | 맞음(재현) | 자이로 ω를 아는 최소제곱 `(v_x, v_y)` — 실제 기하로 두 반례 정확(§4.4) |
| P1-2 | 짧은 자이로 공백의 끝 표본만 무효화하면 누락 회전(2.86°)이 이후 유효 자세에 남음 | 맞음 | 자이로 간격 > 60 ms는 새 epoch, 바퀴 공백만 외삽·무효(위치 오차 상한)(§4.4) |
| P1-3 | 정지 임계 0.05 turn/s가 제자리 0.03 rad/s 회전(0.0166 turn/s)을 정지로 보고 bias 학습, 첫 EMA 20 % | 맞음 | 명령 < 0.005·측정 < 0.02·자이로 표준편차 상한, 첫 구간 평균 그대로(§4.4) |
| P2-4 | 워터마크·틱·`pose_wait_s` 결합 예산 없음(250 ms 지연에서 초과) | 맞음 | `pose_wait_s ≥ margin + 틱 + input_lag_budget_s` 설정 검증(기본 0.4), 자세 확인 후 만료 판정, 전체 경로 시험(§4.4·§7.1) |
| P2-5 | 수신 정지 감시로는 큐 상한 불성립(정체 60 s에 1,200개) | 맞음 | 스트림 최신 stamp 차 `max_stream_skew_s` 초과 → 새 epoch·큐 비움(§4.4) |
| P2-6 | 재생에서 전체 정렬이 epoch 구간을 옮김, ROS 재생 배율 무관 주장 오류 | 맞음 | 녹화 순서로 epoch 경계 먼저·구간별 정렬, ROS 재생은 배율 1.0만(§6) |

### ② Codex 적대 검토 9회차 (2026-10-18, r9 대상) — 판정: 미수렴(P1 1, P2 3)

| # | 지적 | 판정 | 반영 |
|---|---|---|---|
| P1-1 | 최소제곱 `v_y` 관측 불가(직진 rank 1)·불안정(조향 0.001, 조건수 2,149, `v_y` 1.75 m/s) | 맞음 | 조건수 > 50 또는 rank 1이면 `v_y = 0`·`v_x`만, `|v_y|` 상한 0.2 m/s(§4.4) |
| P2-2 | bias 학습 구간에 유효 바퀴 수·연속성 조건 없음(stale 구간 포함 학습) | 맞음 | 구간 전체 유효 바퀴 ≥ 4·공백 없음, 깨지면 구간 재시작(§4.4) |
| P2-3 | 300 ms 지연 만료 시험 기대값이 규칙과 충돌(실제로는 처리) | 맞음 | 예산 = 처리 보장 지연으로 정의, 시험 300 ms 처리·450 ms 만료(§4.4·§7.1) |
| P2-4 | 짧은 바퀴 공백 4.5 cm는 가속도 1 m/s² 전제의 조건부 값 | 맞음 | `½·|a|·gap²` 조건부로 명시, 예시 2개(§4.4) |

### ② Codex 적대 검토 10회차 (2026-10-18, r10 대상) — 판정: **수렴**(새 P1/P2 없음)

9회차 4건 해소 확인(실제 기하·규칙 재계산: 조향·유효 바퀴 198개 조합 비유한 해 없음, 지연값별 18,000개 위상 조합에서 0/200/250 ms 처리·450 ms 만료, 외삽 8.25 cm ≤ 조건부 상한 8.41 cm).
P3 문구 잔재 셋(pending 기본값 4 → 5, 기존 코어 import 문구, `long_gap_s` 이름)을 고쳤다.

### ② Codex 상의 — r11 기준면 기울기(2026-10-18, 4회차 수렴)

| 회차 | 지적 | 처리 |
|---|---|---|
| 1 | P2 가속도 크기·EMA로 주행 중 중력 판별 불가(가속 시 +2.29°, 들림 지연) | 정지 구간에서만 중력 값 |
| 1 | P2 평면·중력 기준 시각 불일치, 만료 기준 없음 | depth stamp까지의 창, 프레임별 불변 값, age_ms·콘솔 만료 |
| 1 | P2 지지면 통과 ≠ 근거리 바닥 경사(2 m 앞 10° 경사로 → 0°) | "판정기가 고른 기준면" 표기, 차체 기준·중력 기준 분리 |
| 2 | P2 IMU만으로는 등가속·등속을 정지로 오판(+5.82°) | 바퀴 지령·측정 정지 조건, "정지 구간 측정" 표기 |
| 2 | P2 accel 역행이 재생 구간 분할을 바꿔 지도 순서 변경 | 지도 4토픽으로만 구간·식별·처리, accel은 표시 버퍼만 |
| 3 | P2 stamp 범위만으로 다른 시계 구간 accel을 고름 | 기록 순서 구간 배정 + epoch 시 버퍼 비움 |
| 4 | P2 경계를 정렬 뒤 첫 항목으로 잡으면 늦은 CameraInfo가 경계를 옮김 | 정렬 전 기록 순번(구간 min) — 수렴 |

### r12 표시 정리 검토(2026-10-18)

① 직접 재계산: Isaac 합성 기록 run-4138을 실제 `MapPanel.draw_into`로 그려 17.2·30·52 s의 숨김/반투명 칸 19/222·48/380·117/779를 냈다.
몽키패치 시제품과 구현 결과는 지도 영역에서 RGB 채널 차이 최대 1이다(반투명 알파를 시제품은 `int(255×0.45)=114`, 구현은 `round`로 115) — Codex 2회차 P3로 정정.

② Codex 적대 검토:

| 회차 | 지적 | 처리 |
|---|---|---|
| 1 | P2 칸당 1~2픽셀에서 한 칸짜리 위험이 윤곽선에 덮여 색이 사라짐(r12 이전부터 있던 결함) | 4픽셀 미만이면 윤곽선을 위험 칸 바깥쪽에만 그린다. 시험으로 고정 |
| 1 | P3 규칙을 꺼도 문구가 숨김·반투명을 말함 | 켜진 규칙만 문구·범례에 적는다 |
| 1 | (확인) 숫자 재현, 높은 칸·위험 코드 숨김 반례 없음, 창·타일 경계와 캐시 무효화 통과 | — |
| 2 | P2 배율 회귀시험이 실제로는 모두 배율 1만 검사(2 px 결함 재주입도 통과) | 실제 배율 1·2·3과 픽셀 예산 축소(4→2)를 만들고 배율을 단언. 재주입 음성 대조로 확인 |
| 2 | P3 "픽셀 단위로 같다" 기록이 틀림(최대 RGB 차 1) | ① 문장 정정 |
| 3 | **수렴**(새 P1/P2 없음) — 상수 재주입 음성 대조 재현 | — |

③ `agy` 독립 검토(①② 미공유, "적히지 않은 전제" 질문):

| # | 지적 | 판정 | 처리 |
|---|---|---|---|
| 1 | 폭 1~2칸 통로·디딤돌이 숨겨짐 | 반박 | 폭 1칸 길도 앞뒤 이웃 2개라 숨지 않는다. 디딤돌은 대개 3 cm 이상(높은 칸) |
| 2 | 저반사 바닥의 듬성한 점이 구멍처럼 보임 | 부분 | 미관측으로만 바뀐다(위험 표시 아님). 체스판 모양은 대각 이웃으로 남는다. 필요하면 `tidy_min_neighbours=0` |
| 3 | 낮은 칸만 숨기면 높은 잡음 칸이 떠 보임 | 맞음(의도) | 높은 칸은 숨기지 않는 쪽이 안전하다. 남은 위험으로 기록 |
| 4 | 장애물 틈 너머 1칸 바닥 단서가 숨겨짐 | 부분 | 보수적 방향(미관측). 한 칸·한 번 관측은 약한 근거다. 끌 수 있다 |
| 5 | 앞쪽 바닥이 늦게 나타남(팝인) | 경미 | 지도가 쌓이는 동안 원래 생기는 현상이다 |
| 6 | 경사로 중간에 3 cm 띠 | 반박 | 표시 높이는 프레임 기준면 대비라 완만한 경사는 0 근처다(§4.11) |

### r13 바퀴 합의 검토(2026-10-19)

① 직접 재계산(원자료):
- run-4138(시뮬 Hall 각도 랩 결함 있음) 바퀴 관절각을 실기 Hall 해상도로 양자화한 속도로, 7.8–9.6 s 6륜 평균 적분 0.722 m 대 truth 0.434 m(중앙값 0.485 m).
  전체 경로 평균 7.806 m·중앙값 6.787 m 대 truth 7.025 m. 헛돎(바퀴 > 1 m/s)은 모두 관절각 랩 직후였다.
- 정상 주행 잔차(랩 ±1.5 s 제외, truth `ω`): 직진·애커만·정지 최대 0.043 m/s, 바퀴별 평균 ±0.004 → 허용치 0.10.
- 실제 구현 재생(pipeline_terrain_map.py, 칸 5 cm): 합의 끔 최종 위치 오차 0.904 m(r12와 같음), `max_slip_wheels 2` 0.447 m·epoch 0·바퀴 뺀 표본 224·최선 추정 58.
  파형 튐 7.9 → 8.9 s 오차 증가 +0.290 → +0.023 m. 칸 2.5 cm 쿼터뷰에서 겹쳐 보이던 가운데 봉우리가 하나로, 누적 거리 표시 7.7 → 6.9 m(truth 6.99 m).
- 수식 검산: 1단계 반례 평균 `(5×0.3 + 3)/6 = 0.75`, 풀이 횟수 `1 + C(6,1) + C(6,2) = 22`.
- **시뮬 결함 우회 재기록**(ws1 Slurm 4250·4251, 같은 하네스·같은 지령, 기록 하네스에서만 HallPLL 입력 각도를 펴는 우회 — `simulation/jetin_rover_description`은 바꾸지 않음):
  우회 끔 4250은 run-4138과 경로·끝 위치가 같다(결정적). 우회 켬 4251은 바퀴 헛돎 112 → 0표본, 제어기가 본 속도 최저 −14.12 → −0.06 m/s, 바퀴 토크 최대 14.30(포화) → 3.15 N·m.
  지도 위치 오차(칸 5 cm 재생): 4251 합의 끔 0.132 m, 켬 0.075 m(epoch 0). 켬에서 바퀴를 뺀 표본 114·최선 추정 25는 모두 파형을 넘는 구간이고,
  뺀 바퀴의 실제 미끄러짐(엔코더 − truth 운동 기대값) 중앙값 0.122 m/s 대 남긴 바퀴 0.044 m/s — 파형 경사에서 바퀴가 수평보다 더 구르는 차이를 대체로 골라냈다.
  결함 있는 4250: 합의 끔 0.904 m(튐 +0.290) → 켬 0.447 m(튐 +0.023).

② Codex 적대 검토:

| 회차 | 지적 | 처리 |
|---|---|---|
| 1 | P1 전체 풀이가 `|v_y|` 상한으로 무효면 탐색을 안 해 애커만 한 바퀴 헛돎(전체 `v_y` 1.61)을 못 고침 | 구조적 무효만 즉시 무효, 상한 무효는 조합 탐색 |
| 1 | P1 "무효·epoch면 1.43 m"는 epoch 원점 재정렬을 안 한 비교 | 근거를 "지도 두 번 비움"으로 교체 |
| 1 | P2 시제품이 무효 처리 경로를 검증 안 함 | 실제 구현으로 재측정(①) |
| 1 | P2 실패 후보 동률 규칙 없음(입력 순서로 0.875↔0.375) | 이름순 정렬·결정적 키. 구현 중 가운데 쪽 우선(전체 풀이와의 차)으로 보강 |
| 1 | P2 실기 기본값 근거 부족(CAN 조회 슬롯별 시각 차) | 기본 끔, 실기 잔차 측정 뒤 켬(§7.3) |
| 1 | P2 무효 구간에서 누적 거리가 현으로 줄어듦 | `WHEEL_SLIP` 자세도 거리에 더함 |
| 2 | P2 미학습 바이어스로 정지 중 합의 실패 → 학습 교착 | 학습 자격은 r12 그대로(`fit_ok`), 합의와 무관 |
| 3 | P2 slip epoch가 바이어스 학습 창을 지워 교착(`slip_epoch_s` < `bias_stationary_s`) | 바이어스 준비 전에는 slip epoch 없음 |
| 3 | P2 입력 순서 합산 차이가 허용치 경계 판정을 바꿈 | 합의를 켜면 첫 판정부터 이름순 |
| 3 | P2 유효 바퀴 부족 표본이 slip 시계를 멈춰 장기 실패 보호 우회 | 시계는 유효 풀이에서만 멈춤 |
| 3 | 시험이 새 구현끼리 비교해 r12 회귀를 못 잡음 | HEAD r12 모듈의 고정값으로 비교 |
| 4 | P2 slip 시간 초과 검사가 최선 추정 표본에만 있어, 실패 뒤 바퀴 부족을 거쳐 복구하면 epoch를 건너뜀 | 매 바퀴 표본에서 시계 해제 전에 검사 |
| 4 | P2 "r12보다 나빠지지 않는다"가 틀림(네 바퀴 같은 헛돎 → 0.8 대 0.633) | 한계 문구 정정, 시험으로 고정 |
| 4→5 | (4회차 수정이 낳은 회귀) 바이어스 준비 전 실패가 시계를 시작해 두었다가 준비 순간 epoch | 바이어스 준비 전에는 시계도 시작하지 않음 |
| 5 | P2 slip epoch와 짧은 바퀴 공백이 겹치면 실시간 `POSE_GAP`·재생 유효로 갈림 | 시간 초과 검사를 바퀴 공백 판정 앞으로. 재생 시험 추가 |

음성 대조(결함을 다시 넣으면 해당 시험 실패): 바이어스 자격에 합의 결과 사용, 최선 추정을 공백으로 셈, 이름순만으로 동률, slip epoch를 yaw 적분 뒤에 엶(선회 시험에서만 드러남 — 직진 시험은 통과해 버려 선회로 바꿈), 거리에서 `WHEEL_SLIP` 제외,
`|v_y|` 상한 무효를 탐색 안 함, 바이어스 전 slip epoch, 이름순 정렬 없음, 바퀴 부족이 시계를 멈춤 — 9건 모두 실패 확인.

③ `agy` 독립 검토(①② 미공유, "적히지 않은 전제" 질문):

| # | 지적 | 판정 | 처리 |
|---|---|---|---|
| 1 | 3–4바퀴가 같이 헛돌면 헛도는 쪽이 합의가 됨 | 맞음(한계) | "못 잡는 것"에 명시. 이때는 r12 최소제곱보다 나빠질 수 있다(Codex 4회차 수치, 시험으로 고정) |
| 2 | 비조향 중간 바퀴는 선회 때 늘 긁혀 상시 탈락 | 부분 | 애커만은 중간축이 회전 중심이라 긁힘이 없다(시뮬 선회 잔차 0.043). 스키드는 실기 측정 대상(§7.3) |
| 3 | 합의 실패 중 최선 추정을 무기한 적분하면 지도가 찢어짐 | 맞음 | `slip_epoch_s` 상한 |
| 4 | 로커-보기 관절 운동이 잔차를 키워 정상 바퀴를 뺌 | 부분 | 턱을 오르는 바퀴는 수평 속도를 재지 않으므로 빼는 게 맞다. 바퀴 위치 수 cm 변화는 무시할 크기 |
| 5 | 자이로 오차 레버암이 앞뒤 바퀴를 빼게 함 | 반박 | 0.01 rad/s × 0.5 m = 0.005 m/s ≪ 0.10 |

### ③ 독립 검토 — `agy` Gemini 3.8 Flash (High), ①② 결론 미공유, "적히지 않은 전제" 질문

| # | 지적 | 판정 | 처리 |
|---|---|---|---|
| 1 | 직사광선에서 깊이 소실 → 시연 중단 | 맞음(알려진 위험) | 전제·위험에 조도 사전 확인 |
| 2 | 험지에서 평면 실패 시 FRESH만 쓰면 지도가 빔 → HELD 융합 | 부분 | HELD는 이전 평면이라 높이 오차 위험. FRESH만 유지하고 `NO_GROUND` 상태로 표시 |
| 3 | 차체 피치업 시 평면 연장선 대비 평지가 도랑처럼 측정 | 반박 | 평면은 그 프레임에 **보인 바닥**에 맞추므로 차체 자세와 무관 |
| 4 | 후진 시 사각지대·오래된 지도를 맹신 | 맞음 | 칸 나이 흐림, 고정 문구(§4.10) |
| 5 | 슬립 스파이크가 epoch 오발 | 반박 | 정상 간격 0.2 m 예측 차는 미끄러짐으로 생기지 않음, 긴 간격은 예측 안 함(§4.4) |
| 6 | 지나간 사람이 다시 안 보면 영구 장애물 | 부분 | 삭제 대신 칸 나이 흐림. 다시 보면 다수결로 두 번에 사라짐 |
| 7 | 물웅덩이 → 미관측 → 도랑으로 오인 | 맞음 | 미관측은 별도 색·범례 |
| 8 | 먼지 구름이 높은 분위에 허위 장애물 | 부분 | trim 순서통계·다수결·`max_abs_height_m`·`min_observations`, 실기 확인 |
| 9 | 급선회 시 빗살 빈틈 | 반박 | 화각 약 70°라 5 Hz 연속 프레임이 크게 겹침 |
| 10 | 판정기 중복 CPU | 맞음(알려진 위험) | §7.3-3 기준, 실패 시 통합 |
| 11 | 보간이 수직 턱을 경사처럼 보이게 | 맞음 | 분류 경계 윤곽선(§4.10) |
| 12 | B가 피치 변화에 흔들림 → 10-23에서 B 배제 | 부분 | 표시 높이는 본 바닥 기준이라 피치와 무관. B는 기본 꺼짐 |
