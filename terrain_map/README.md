# terrain_map — L515 주행 경로 2.5D 지형 지도

운용자 **시각 보조 전용**이다. 지나온 길의 바닥 높이와 돌출·도랑을 칸으로 모아 운용 콘솔
"지형 지도" 페이지에 보인다. 주행 제어·정지·속도 제한·안전 판정에는 아무것도 주지 않는다.
설계 정본: `docs/specs/2026-10-17-l515-terrain-map-design.md`, 구현 계획:
`docs/plans/2026-10-18-l515-terrain-map.md`.

## 흐름

```
Gateway depth·CameraInfo·gyro + chassis /wheel_states
  → terrain_map_node (powertrain_ros, 구독만 · ROS 발행 없음)
      └ MapRuntime: 평면 추측 항법(PlanarOdometry) → 처리 프레임 선택(FrameScheduler)
        → depth 솎기 + 바닥 평면(TerrainDetector) → 지지면 검사 → 프레임 안 분류(build_local_scan)
        → 칸별 분류 이력 다수결(ClassMap) → 상태·타일 패킷
  → UDP :5010 → operator_console LatestMapReceiver → map_rgba → "지형 지도" 페이지
```

| 모듈 | 하는 일 |
|---|---|
| `config.py` | `MapConfig` — 아래 표의 모든 수치와 범위·상호 검증(범위 밖이면 `ValueError`, 조용한 보정 없음) |
| `contract.py` | :5010 패킷 v1(상태·타일), 칸 바이트 형식, 분류 코드. 표준 라이브러리만(콘솔 시스템 python) |
| `geometry.py` | 장착 회전 `base_to_optical`, 바닥 평면 위 차체 수평 좌표, SE(2) |
| `scan.py` | depth 솎기, 지지면 검사, 프레임 → 국소 칸(높이·분류) |
| `odometry.py`·`pose.py` | 지도 전용 평면 추측 항법(자이로 요 + 유효 바퀴 최소제곱 속도), 자세 보간 |
| `schedule.py` | 처리할 depth 선택(대상 고정·자세 대기·만료) |
| `classmap.py` | 칸별 최근 3개 분류 다수결, 표시 높이, 전역 version, 축출 |
| `correction.py` | 위치 보정 훅 — A는 그대로(`identity`), B(지도 맞춤)는 아직 없음 |
| `runtime.py` | 위를 묶는 틱 논리(실시간 노드와 결정적 재생이 같이 씀) |

## 분류 규칙 (한 프레임 안에서 판정)

1. 그 프레임의 바닥 평면 기준 높이 `h`를 칸마다 구한다(위아래로 잘라낸 순서통계, 위험한 쪽 선택).
2. `h ≥ obstacle_height_m` → **돌출(1)**, `h ≤ −ditch_depth_m` → **도랑·낙차(2)**, 그 밖 → **주행 가능(0)**.
3. 지지면 검사를 통과한 프레임만 쓴다 — 카메라 앞 근거리 띠의 중앙 높이가 `support_tol_m` 안이어야 한다
   (앞쪽 넓은 도랑 바닥을 바닥으로 잡은 프레임 거부). `support_base_height_m`을 실측해 켜면 차체 원점 높이도 본다.
4. 지도는 칸마다 최근 `history_length`개 판정의 다수결(2개 이상 같으면 그것, 아니면 최근 것)이다.
   높이는 프레임끼리 섞지 않고 마지막 관측만 표시한다. 관측 `min_observations` 미만 칸은 "관측 부족(4)".

분류는 **그 칸을 본 위치 기준**이다. 다른 위치에서 다시 보면 판정이 바뀔 수 있고, 오래된 칸은 콘솔에서 흐리게 보인다.

## 기준면 기울기 (표시 전용, r11)

콘솔 상태 표시줄 아래에 나온다. 지도 분류·제어에는 쓰지 않는다.

- **기준면 기울기(차체 기준)**: 판정기가 고른 기준면(대개 가까운 바닥 — 먼 경사로는 반영되지 않을 수 있음)이 차체 축에 대해 앞(오르막 +)·옆(왼쪽 높음 +)으로 기운 각. 처리한 프레임마다.
- **정지 구간 측정**: 로버가 정지해 있던 0.5 s(가속도·자이로 고요 + 모든 바퀴 지령·측정 정지) 동안 잰 중력으로 계산한 **기준면 경사(중력 기준)**와 **차체 앞·왼쪽 들림**. 주행 중에는 나오지 않는다(가속을 중력과 구분할 수 없음).
- 둘 다 관측 후 1 s가 지나면 사라진다. 경사로에서 확인하려면 경사로 위·앞에서 1–2 s 정지한다.
- 수치는 `MapConfig`의 `still_window_s`·`still_max_gyro_rad_s`·`still_max_accel_std`·`gravity_tolerance`·`slope_max_age_s`(아래 표 외 — 스펙 §4.11).

## 파라미터 (`MapConfig`, 전부 변경 가능)

노드 파라미터 이름이 그대로 `MapConfig` 필드다(`ros2 run powertrain_ros terrain_map --ros-args -p cell_m:=0.04` 식).
`support_base_height_m`·`support_base_variation_m`은 기본 꺼짐이다 — 실측한 값을 줄 때만 켜진다(상호 조건은 아래 표).

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

- 노드 파라미터와 콘솔 CLI로 바꾼다. 범위 밖이거나 서로 맞지 않으면(패킷 > 1200 B, pending 용량 부족 등. 나이 표현 범위는 두 쪽 범위가 계약으로 묶여 있어(`31·2.0 = 62 ≥ 60`) 각자 범위 검증만 한다) 기동 실패(조용한 클램프 없음).
- 분류 경계는 Jetson에서만 쓴다. 콘솔은 받은 분류 코드를 그대로 그린다.

콘솔 표시 수치(`DisplayConfig`, 위 표의 "콘솔" 행)는 콘솔 쪽에서 바꾼다. 이름이 경로마다 다르니 주의한다.

| 표 이름 | 콘솔 CLI | 통합 설정 `terrain_map_display` 키 | 기본·범위 |
|---|---|---|---|
| `display_upsample` | `--map-display-upsample` | `display_upsample`(정수) | 4 · 1–8 |
| `hillshade_azimuth_deg` | `--map-hillshade-azimuth` | `hillshade_azimuth_deg` | 315 · 0–360 |
| `hillshade_altitude_deg` | `--map-hillshade-altitude` | `hillshade_altitude_deg` | 40 · 5–85 |
| `hillshade_exaggeration` | `--map-hillshade-exaggeration` | **`z_exaggeration`** | 6 · 1–20 |
| `stale_display_s` | `--map-stale-s` | `stale_display_s` | 30 · 5–60 |
| `max_render_pixels` | `--map-max-render-pixels` | `max_render_pixels`(정수) | 2,000,000 · 10,000–4,000,000 |
| (높이 색 범위) | — | `height_range_m` | 0.3 · 0.05–1.0 |

`~/.config/powertrain/operator.json` 예시(모르는 키·범위 밖·비정수는 콘솔 기동을 거부한다):

```json
{"terrain_map_port": 5010, "terrain_map_display": {"z_exaggeration": 6, "stale_display_s": 30}}
```

## 켜는 법 (Jetson)

송신 서비스는 기본 기동에 들어 있지 않다(현장 부하 확인 전까지).

```bash
INTEGRATED_PROFILES=terrain-map INTEGRATED_SERVICES=powertrain_terrain_map scripts/robot-start
```

- **기동 뒤 로버를 1초 정지**한다 — 자이로 바이어스를 정지 구간에서만 배우고, 그 전에는 지도가 쌓이지 않는다
  (콘솔에 "로버를 1초 정지하세요").
- 지도 작성 권장 속도 0.5 m/s 이하. 미끄러짐이 많은 험지에서는 지도가 휠 수 있다(위치 보정 B 전).
- 연속성이 끊기면(자이로·바퀴 공백, 수신 정지, 시계 역행) 새 epoch로 지도를 비우고 원점에서 다시 쌓는다.

## 녹화·재생

```bash
# Jetson (ROS 셸): 지도 입력 5개 토픽 녹화, Ctrl+C로 끝
scripts/record_terrain_map_bag.sh run1
# 노트북: 결정적 재생 → 콘솔 :5010 (재생 도메인 78 강제)
scripts/replay_terrain_map.sh ~/terrain_bags/run1 --rate 1.0
# ROS 재생(노드 자체 시험, 배율 1.0만)
scripts/replay_terrain_map.sh --ros ~/terrain_bags/run1
# 지도만 파일로
python3 -m powertrain_ros.terrain_map_replay ~/terrain_bags/run1 --out map.npz
```

실시간과 **같은 지도**를 재생으로 만들려면 노드에 `-p processed_log_path:=/data/processed.jsonl`을 주고 녹화한 뒤
`--decisions-from processed.jsonl`(로그에 실행이 여럿이면 `--session <instance>`)로 재생한다. 노드가 받은 표본이
bag에 없으면 재생은 거부한다(같은 지도를 만들 수 없으므로).
결정 재생은 기록에 남은 실시간 설정(`MapConfig`)·바퀴 기하를 그대로 쓴다 — `--param`은 기록값과 같을 때만 허용하고,
설정을 바꿔 비교하려면 `--decisions-from` 없이 재생한다. 끝에 처리 프레임·결과·카메라 모델을 기록과 대조해 다르면
지도를 저장하지 않고 거부하며, `--send`는 이 검증을 통과한 뒤에만 보낸다.

## 확인

- 호스트: `PYTHONPATH=motor_control python -m pytest terrain_map terrain_overlay`
- ROS(도메인 77): `ros2/src/powertrain_ros/test/test_terrain_map_node.py`·`test_terrain_map_replay.py`,
  설치 스모크 `installed_terrain_map.py`(`--inject-double-radius` 음성 대조, `--bag-play`)
- 배포 격리 게이트: `ros2/src/powertrain_ros/test/terrain_map_isolation_gate.py snapshot/compare` —
  지도 서비스 기동 전후 발행자 목록이 같고 `/tf`에 새 발행자가 없어야 한다(다르면 배포 중단).
