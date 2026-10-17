# L515 2.5D 지형 지도 — 실기 확인 체크리스트 (Task 12)

> 실기 연결이 될 때 **한 번에** 끝내기 위한 목록이다. 근거: 스펙
> `docs/specs/2026-10-17-l515-terrain-map-design.md` §7.3·§8.2, 계획 `docs/plans/2026-10-18-l515-terrain-map.md` Task 12.
> 호스트·ROS 컨테이너·설치 스모크·로컬 배포 격리 게이트는 2026-10-18에 통과했다(브랜치 `feat/l515-terrain-map`).
> 여기 남은 것은 **실물이 있어야만 확인되는 것**뿐이다.

## 0. 현장 가기 전에 (노트북에서, 실기 없이)

| # | 할 일 | 왜 | 상태 |
|---|---|---|---|
| 0-1 | 브랜치 push 여부 결정 → Jetson으로 옮기는 방법 정하기(push 후 `git fetch` 또는 scp) | 아직 로컬 커밋만 있다 | ☐ |
| 0-2 | **현장 분석 스크립트 준비**: bag 하나를 받아 ① 자이로·바퀴 메시지 간격 분포 ② 평면 기준 base 원점 높이 `n·o+d` 프레임별 값(중앙값·퍼짐) ③ FRESH·`NOT_SUPPORT` 비율 ④ 처리시간 분포를 뽑는 오프라인 도구 | 스펙 §7.3-1·2·3 수치 중 지금 바로 뽑을 도구가 없는 것 | ☐ (미작성) |
| 0-3 | 콘솔 쪽 상태 패킷 기록 도구(:5010 상태 패킷의 `proc_ms`·`pose_gaps`·`not_support`를 파일로) — 또는 0-2를 결정 로그로 대체 | `proc_ms` p95 계산 | ☐ (미작성) |
| 0-4 | 줄자·수평계(경사계 앱 가능)·상자(높이 10–20 cm)·단차/패임 장소, 주행 거리 5 m 표시(테이프) | §7.3-1·2 | ☐ |
| 0-5 | Jetson 디스크 여유 ≥ 4 GB 확인 계획(녹화 ≈ 369 MB/분) | §6 | ☐ |

## 1. 배포 (Jetson SSH 직후, 홈 `~`)

```bash
# Jetson 호스트
cd ~/power-train-sw-integrated
git fetch --all && git status            # 팀원 미커밋 파일이 있으면 보존(건드리지 않음)
git checkout feat/l515-terrain-map       # 0-1에서 정한 방법으로 받은 브랜치
docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml run --rm \
  --entrypoint /bin/bash powertrain_terrain_map -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace/ros2 && colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros'
```
⚠️ `/workspace`가 읽기 전용 마운트면 빌드는 기존 ROS 빌드 절차(`scripts/robot_prepare.sh`가 하는 colcon 단계)를 따른다 — 현장에서 확인.
✅ `Summary: 3 packages finished`

```bash
./scripts/robot-start                    # 기존 통합 서비스
```
✅ `robot-start PASS: ...`

## 2. 배포 격리 게이트 (스펙 §8.2) — 지도 서비스 켜기 **전후**

```bash
# Jetson, ROS 컨테이너 안(예: powertrain_ros_* 컨테이너에 docker exec)
python3 /workspace/ros2/src/powertrain_ros/test/terrain_map_isolation_gate.py snapshot /tmp/before.json
# Jetson 호스트
INTEGRATED_PROFILES=terrain-map INTEGRATED_SERVICES=powertrain_terrain_map ./scripts/robot-start
sleep 10
# 다시 ROS 컨테이너 안
python3 /workspace/ros2/src/powertrain_ros/test/terrain_map_isolation_gate.py snapshot /tmp/after.json
python3 /workspace/ros2/src/powertrain_ros/test/terrain_map_isolation_gate.py compare /tmp/before.json /tmp/after.json
```
✅ `"verdict": "PASS"` (`/rosout`·`/parameter_events` 외 발행자 증감 없음, `/terrain_map` 실행)
- ☐ **음성 대조**: 같은 이름 발행자 하나 더(예: 다른 터미널에서 `ros2 topic pub /wheel_states ...`를 잠깐 띄움) → `FAIL` 확인 후 끄기. (로컬에선 비교 함수 단위시험만 했다 — 실제 스냅숏이 엔드포인트 GID를 담는지 여기서 확인)
- ❌ FAIL이면 **배포 중단**, 결과 JSON 보관.

## 3. 장착값 실측과 정지 장면 (§7.3-1)

| # | 할 일 | 기록할 값 | 합격·판단 |
|---|---|---|---|
| 3-1 | L515 depth 원점 위치(base_link 기준 x·y·z)와 하향각·롤·요를 줄자·수평계로 측정 | `camera_x_m`·`camera_y_m`·`camera_z_m`·`camera_pitch_down_deg`·`camera_roll_deg`·`camera_yaw_deg` | CAD 기본(0.348, 0, 0.029, 20°)과 차이 기록 → 노드 파라미터로 교체 |
| 3-2 | 평지 정지 30 s 녹화(§6 녹화 스크립트) → 0-2 도구로 `n·o+d` 중앙값 | base_link 높이(측정) vs `n·o+d` 중앙값 | 둘 차이 기록 → `support_base_height_m` 값 |
| 3-3 | 험지 위 정지 자세 3–5곳에서 같은 측정 | `n·o+d` 퍼짐 | `support_base_variation_m` ≤ `support_height_tol_m` < 8 cm − var 이면 켬, 아니면 끈 채 둠(§4.2) |
| 3-4 | 정지 상태로 상자·단차·도랑(패임) 앞 → 콘솔 지도 | 지도상 위치 vs 줄자 거리, 분류 색 | 위치 오차 ≲ 1칸(5 cm), 돌출=빨강·도랑=파랑 |

- 시작 직후 콘솔에 "로버를 1초 정지하세요"가 뜨면 정지해 둔다(자이로 바이어스 학습).

## 4. 주행 (§7.3-2) — 0.5 m/s 이하

```bash
# Jetson: 지도 노드에 결정 기록 켜기(재생 검증용) — compose 명령에 -p processed_log_path:=/run/powertrain/terrain_map_decisions.jsonl 추가해 재기동
# 녹화 시작(별도 터미널, ROS 셸)
scripts/record_terrain_map_bag.sh field1
```

| # | 주행 | 기록할 값 | 합격·판단 |
|---|---|---|---|
| 4-1 | 1–2 s 정지 → 5 m 직진 | 콘솔 누적 거리 vs 테이프 5 m | 오차 기록(목표: 수 % 이내, 스키드 미끄러짐 포함) |
| 4-2 | 90° 제자리/애커만 회전 | 콘솔 자세 yaw vs 실제 90° | 오차 기록(목표 ≲ 2–3°) |
| 4-3 | 상자를 지나 왕복 | 같은 상자가 한 덩어리로 쌓이는지 | 두 덩어리로 갈라지면 위치 오차 → B 필요성 근거 |
| 4-4 | 큰 도랑·바로 앞 단차 장면 | 상태 `NO_GROUND / NOT_SUPPORT` 빈도 | 기록 |
| 4-5 | 전체 주행 중 | `pose_gaps`, epoch 전환 횟수·원인(결정 기록 `"k":"epoch"`의 `cause`), 자이로 간격 분포(0-2) | 60 ms 공백 epoch가 잦으면 `gyro_gap_epoch_s`·Gateway IMU 주기 조정(§10) |
| 4-6 | 자세 대기 | 결정 기록 `"k":"frame","r":"lost"` 중 `POSE_EXPIRED` 수 | 기록 |

## 5. 자원 10분 (§7.3-3) — 두 SRT + 오버레이 노드 + 지도 서비스 동시

| 항목 | 합격 기준 | 어디서 |
|---|---|---|
| `proc_ms` p95 | ≤ 150 ms | 상태 패킷(0-3) 또는 0-2 |
| 처리 손실률 = lost / (processed + lost) | ≤ 10 % | 노드 종료 로그 `terrain map: processed …, lost …` (`docker logs powertrain_terrain_map`) |
| FRESH 비율 | 기록(stride·`ground_min_points` 결정 근거) | 0-2 |
| 차체 50 Hz tick 간격 p99 | 서비스 끄기 전 대비 +2 ms 이내 | bag의 `/wheel_states` header stamp 간격(0-2) — 지도 끈 상태 1분 녹화도 같이 |
| 워치독 이벤트 | 0 | 기존 운용 로그 |
| Jetson RAM 여유 | ≥ 1 GB | `free -h` |

- ❌ 못 맞추면: stride 3·`process_hz` 낮추기 → 그래도 안 되면 오버레이·지도 판정기 통합(§11). `robot-start` 기본 목록에는 통과 전까지 넣지 않는다.

## 6. 녹화 → 노트북 재생 (§7.3-4)

```bash
# 노트북: bag과 결정 기록을 받아 온 뒤
scripts/replay_terrain_map.sh ~/terrain_bags/field1 --decisions-from terrain_map_decisions.jsonl
```
✅ 거부 메시지 없이 끝나고 콘솔에 실시간과 같은 지도. `refusing: …`이면 메시지 그대로 기록(녹화기·노드 수신 차이, CameraInfo 유실 등).
- ☐ 같은 bag으로 실제 메시지 필드·자이로 부호 확인(반시계 회전에서 yaw 증가).

## 7. 끝나고

- 결과 보고서 `docs/reports/2026-10-xx-l515-terrain-map.md`: 위 표의 값, 바꾼 파라미터와 근거, 실패·조정 내역.
- 실측 장착값·`support_base_*`를 노드 기본값/compose 파라미터에 반영할지 결정(별도 커밋).
- 위치 보정 B 착수 여부 판단(4-3 결과 기준) — 시험 장치는 `terrain_map/tests/b_fixtures.py`.
- 선이와 :5010·"지형 지도" 화면 합의 결과 반영.
