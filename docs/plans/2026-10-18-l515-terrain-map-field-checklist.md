# L515 2.5D 지형 지도 — 실기 확인 체크리스트 (Task 12)

> 실기 연결이 될 때 **한 번에** 끝내기 위한 목록이다. 근거: 스펙
> `docs/specs/2026-10-17-l515-terrain-map-design.md` §7.3·§8.2, 계획 `docs/plans/2026-10-18-l515-terrain-map.md` Task 12.
> 호스트·ROS 컨테이너·설치 스모크·로컬 배포 격리 게이트는 2026-10-18에 통과했다(브랜치 `feat/l515-terrain-map`).
> 여기 남은 것은 **실물이 있어야만 확인되는 것**뿐이다.

## 0. 현장 가기 전에 (노트북에서, 실기 없이)

| # | 할 일 | 왜 | 상태 |
|---|---|---|---|
| 0-1 | Jetson은 같은 내부망에 붙여 **Jetson에서 pull**한다(2026-10-18 결정). GitHub에서 받으려면 그 전에 `feat/l515-terrain-map`을 push, push 없이 하려면 노트북에서 직접 받는다(§1 명령) | 아직 로컬 커밋만 있다 | ☐ |
| 0-2 | 현장 분석 도구 `python3 -m powertrain_ros.terrain_map_field_report` — bag(흘려 읽음): 자이로·바퀴·depth 간격 분포(60/300 ms 초과 수, 도착 순서 뒤바뀜), 차체 50 Hz tick p99, 프레임별 `n·o+d`와 `support_base_*` 제안값·켤 수 있는지, FRESH·`NOT_SUPPORT` 비율 / 결정 기록: 손실률·로봇에서 잰 처리시간 p95·epoch 원인 | 스펙 §7.3-1·2·3 수치 | ✅ 2026-10-18 작성 |
| 0-3 | 처리시간은 노드 결정 기록(`processed_log_path`)의 프레임별 `ms`로 잰다(UDP 기록기 불필요) | `proc_ms` p95가 노트북이 아닌 Jetson 값이어야 함 | ✅ 2026-10-18 작성 |
| 0-6 | 노트북 결정적 재생은 bag 전체를 메모리에 올린다(≈ 370 MB/분, `--send`면 검증·송신 두 번 재생). 재생용 녹화는 **3분 이하**로 끊는다 | 노트북 RAM 15 GB | ☐ |
| 0-4 | 줄자·수평계(경사계 앱 가능)·상자(높이 10–20 cm)·단차/패임 장소, 주행 거리 5 m 표시(테이프) | §7.3-1·2 | ☐ |
| 0-5 | Jetson 디스크 여유 ≥ 4 GB 확인 계획(녹화 ≈ 369 MB/분) | §6 | ☐ |

## 1. 배포 (Jetson SSH 직후, 홈 `~`)

```bash
# Jetson 호스트
cd ~/power-train-sw-integrated
git status                               # 팀원 미커밋 파일이 있으면 보존(건드리지 않음) — 있으면 멈추고 확인
# (가) GitHub에 push해 둔 경우
git fetch origin && git checkout -B feat/l515-terrain-map origin/feat/l515-terrain-map
# (나) push 없이 노트북에서 직접 (노트북 IP·사용자는 현장 값)
git fetch light@<노트북IP>:ZETIN/robotics/power-train-sw feat/l515-terrain-map:feat/l515-terrain-map
git checkout feat/l515-terrain-map
git log --oneline -1                     # 노트북의 브랜치 끝 커밋과 같은지 확인
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
| 3-2 | 평지 정지 30 s 녹화 → 아래 ①로 기준 높이 H(측정 전용 출력) | base_link 높이(줄자) vs H(`n·o+d` 중앙값) | 둘 차이 기록, H가 `support_base_height_m` 값 |
| 3-3 | 험지 위 정지 자세 3–5곳을 각각 녹화(또는 한 녹화에서 `--window`로 구간 지정) → ②로 **모든 자세를 H 기준으로 합쳐** 판정 | 합친 `support_base_variation_m`(모든 표본의 H 대비 최대 편차, 잘라내지 않음) | ②의 `can enable: True`일 때만 켬. `--reference-height` 없는 실행은 "measurement only"라 승인으로 쓰지 않는다 |

```bash
# ROS 셸(Jetson 컨테이너 또는 노트북 ROS 이미지). --param에는 노드가 실제로 쓴 장착값을 그대로 준다.
M="--param camera_x_m=<실측> --param camera_z_m=<실측> --param camera_pitch_down_deg=<실측>"
# ① 평지 기준 높이 H (측정 전용)
python3 -m powertrain_ros.terrain_map_field_report --bag ~/terrain_bags/flat1 --window 0:30 $M --out flat1.json
# ② 험지 자세 전부를 H 기준으로 합쳐 판정 (자세마다 --bag/--window 한 쌍)
python3 -m powertrain_ros.terrain_map_field_report --reference-height <H> $M \
  --bag ~/terrain_bags/rough1 --window 0:15 --bag ~/terrain_bags/rough2 --window 0:15 --bag ~/terrain_bags/rough3 --window 0:15 \
  --out rough.json
```
✅ ①의 끝줄: `support_base_height_m <H> (median of these samples) · … · measurement only (no --reference-height)`
✅ ②의 끝줄: `support_base_height_m <H> (given) · support_base_variation_m … · can enable: True/False — pooled over 3 recordings`
- 높이 후보는 근거리 띠 검사를 통과한 평면만 쓴다(도랑 바닥을 바닥으로 잡은 프레임은 `rejected_base_heights`로 따로 보인다).
| 3-4 | 정지 상태로 상자·단차·도랑(패임) 앞 → 콘솔 지도 | 지도상 위치 vs 줄자 거리, 분류 색 | 위치 오차 ≲ 1칸(5 cm), 돌출=빨강·도랑=파랑 |
| 3-5 | **경사 확인**(r11): 평지 정지 → 콘솔 "정지 구간 측정"의 기준면 경사·차체 들림이 ±1° 안. 경사로(수평계로 각 측정) 위에 1–2 s 정지 → 기준면 경사(중력 기준)와 차체 앞 들림 vs 수평계 | 콘솔 값 vs 수평계 각 | 오차 기록(목표 ≲ 1–2°). 주행 중에는 "정지 구간 측정" 줄이 사라지는지 확인 |
| 3-6 | 가속도 축 확인: 정지 시 콘솔 차체 들림이 수평계와 부호까지 맞는지(앞 들림 +, 왼쪽 들림 +) | 부호 | 틀리면 장착 회전 파라미터·IMU 축 가정 재확인(스펙 §4.11) |

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
| 4-5 | 전체 주행 중 | `pose_gaps`, epoch 전환 횟수·원인, 자이로 간격 분포 | 60 ms 공백 epoch가 잦으면 `gyro_gap_epoch_s`·Gateway IMU 주기 조정(§10) |
| 4-6 | 자세 대기 | 손실 이유별 수(`POSE_EXPIRED` 등) | 기록 |

```bash
python3 -m powertrain_ros.terrain_map_field_report --bag ~/terrain_bags/field1 \
  --decisions /run/powertrain/terrain_map_decisions.jsonl --out field1.json
```
✅ `gyro interval … gaps >60 ms: N`, `loss rate … (≤ 0.1: PASS)`, `proc_ms p95 … on the robot (≤ 150.0: PASS)`, `epochs N {원인: 수}` — 세부(손실 이유·처리 결과별 수)는 `field1.json`.
⚠️ `proc_ms: old log …`이 나오면 그 로그는 이 버전 전의 노드(검출 시간만 기록)라 처리시간 판정에 쓰지 않는다 — 노드를 이 브랜치로 갱신한 뒤 다시 잰다.

## 5. 자원 10분 (§7.3-3) — 두 SRT + 오버레이 노드 + 지도 서비스 동시

| 항목 | 합격 기준 | 어디서 |
|---|---|---|
| `proc_ms` p95 | ≤ 150 ms | `terrain_map_field_report --decisions` (로봇에서 잰 값) |
| 처리 손실률 = lost / (processed + lost) | ≤ 10 % | 같은 도구, 또는 노드 종료 로그 `terrain map: processed …, lost …` |
| FRESH 비율 | 기록(stride·`ground_min_points` 결정 근거) | `--bag` (자원 시험 중 1–2분만 녹화해 분석) |
| 차체 50 Hz tick 간격 p99 | 서비스 끄기 전 대비 +2 ms 이내 | `--bag`의 `wheel (chassis tick) interval p99` — 지도 끈 상태 1분 녹화와 비교 |
| 워치독 이벤트 | 0 | 기존 운용 로그 |
| Jetson RAM 여유 | ≥ 1 GB | `free -h` |

- ❌ 못 맞추면: stride 3·`process_hz` 낮추기 → 그래도 안 되면 오버레이·지도 판정기 통합(§11). `robot-start` 기본 목록에는 통과 전까지 넣지 않는다.

## 5-A. 해상도 비교 (선택 — §5를 기본값으로 통과한 뒤)

Isaac 합성 기록(2026-10-18, 참값 자세)에서 칸 2.5 cm는 **깊이 전 픽셀(`pixel_stride` 1)일 때만** 지도가 유지됐다.
- 칸만 줄이면(stride 2) 칸당 점이 모자라 면적이 30.7 → 15.7 m²로 줄었다.
- stride 1에서는 면적이 30.9 m²로 유지되고, 상자가 또렷해졌다.
  - 관측된 상자 윗면 내부 칸의 높이 오차 하위 10 %: −12.5 → −1.8 cm. 경계 위치 오차가 아니라 윗면 높이가 깎이던 정도다.
  - 관측 부족을 뺀 상자 내부 칸의 돌출 판정률: 94 → 97 %
- 평지 오판은 0 %로 같고, 외톨이·구멍 비율도 비슷하다.
- 대가는 처리량이다. 점이 4배라 처리 시간이 늘어난다.
  - 노트북 단발 실행의 전체 시간을 처리 프레임 수로 나누면 약 +46 ms다(9 → 21 s, 260프레임).
  - 이 값에는 로딩·평가가 섞여 있다. Codex가 재실행해 보니 처리 함수 평균만으로도 약 +60 ms였다.
  - 노드 처리 시간 p95와 Jetson 값은 아직 재지 않았다.
- 기록 오도메트리 실제 파이프라인에서는 타일 81 → 280개, 순환 주기 2.2 → 7.0 s, 패킷 최대 951 B(≤ 1,200 B)였다.
- 콘솔 한 번 그리기는 약 232–269 ms가 걸린 경우가 있었다(Codex 측정). 그리기 주기가 0.2 s라 화면 반응성도 확인해야 한다.
- 산출물: `power-train-sim/out/2026-10-18-terrain-map-isaac/resolution_compare_*.png`.

| # | 할 일 | 기록할 값 | 합격·판단 |
|---|---|---|---|
| 5A-1 | §5와 같은 동시 부하(두 SRT + 오버레이)에서 지도 노드만 다시 띄운다. 기존 `--ros-args` 뒤에 `-p cell_m:=0.025 -p pixel_stride:=1 -p processed_log_path:=/run/powertrain/terrain_map_decisions_c25.jsonl`을 붙인다(**결정 기록 파일은 설정마다 따로** — 노드는 이어 쓰기라 한 파일에 섞이면 분석 도구가 거부한다). 녹화하며 2–3분 주행(§4 상자 왕복 포함) | `proc_ms` p95, 손실률, 차체 tick p99, 상태 패킷 `tile_count`·`refresh_cycle_s` | §5 표와 같은 기준(p95 ≤ 150 ms, 손실 ≤ 10 %, tick +2 ms 이내) |
| 5A-2 | 같은 경로·같은 거리에서 5 cm 기본값과 번갈아 콘솔로 본다(화면 캡처) | 상자 경계·모서리 모양, 관측 면적, 미관측·관측 부족 구역, 먼 가장자리 잡음, 콘솔 화면 갱신이 끊기지 않는지 | 경계가 나아지면서 지도가 성겨지거나 화면이 버벅이지 않아야 한다 |
| 5A-3 | 판단 | — | 5A-1 통과 **그리고** 5A-2에서 경계가 실제로 나아지면 기본값 변경을 제안(스펙 §4.1 표 개정 → Codex 검토). 하나라도 아니면 5 cm·stride 2 유지 |

- ⚠️ `max_map_tiles`는 기본 1024로 둔다. 2.5 cm 타일은 0.4 m라 지도 상한이 약 655 → 164 m²로 줄지만, 비교 주행에는 충분하다.
  올리려면 콘솔 하한 표 크기(`4 × max_map_tiles`, 콘솔 기본 4,096)도 함께 바꿔야 한다(스펙 §4.9).
- 중간 크기(3–4 cm)는 이번 범위에서 뺀다(시험 범위 선택).
  - stride 2로는 칸당 점이 기본값의 36 %(3 cm)·64 %(4 cm)로 줄어, 2.5 cm·stride 2처럼 지도가 성겨질 위험이 있다.
  - stride 1로는 입력 픽셀 수가 2.5 cm와 같다. 다만 칸·타일 수가 적어 융합·전송 비용은 더 작다.
  - 5A-1이 처리량에서 아깝게 실패하면 그때 `-p cell_m:=0.035 -p pixel_stride:=1`을 한 번 더 잰다.
- 처리 시간이 넘치면 `process_hz`를 낮추는 대신 5 cm로 돌아간다. 지도 갱신이 느려지는 것보다 칸 크기가 덜 중요하다.
- 분석: `python3 -m powertrain_ros.terrain_map_field_report --bag ~/terrain_bags/<녹화> --param cell_m=0.025 --param pixel_stride=1 --decisions /run/powertrain/terrain_map_decisions_c25.jsonl --out c25.json`
  — bag 분석은 노드 설정을 결정 기록에서 되살리지 않으므로 `--param`을 꼭 같이 준다. 한 파일에 여러 실행이 섞였으면 `--session <instance>`로 고른다(§6 재생도 같다).
- 끝나면 지도 노드를 기본 파라미터로 재기동한다.

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
