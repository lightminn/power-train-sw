# L515 지형 장애물 오버레이 설계

- 작성일: 2026-10-17 (같은 날 검토 ②③ 반영 r2, Codex 2회차 r3, 3회차 r4, 4회차 r5, 5회차 r6, 구현 반영 r7)
- 브랜치: `feat/l515-terrain-overlay` (기준 `origin/main` `0ea5757`)
- 출발점: PR #9(`7716592`)·PR #13(`11d8daa`)의 `lidar_dev/` 프로토타입. 원작자 seoyeon0777, 2026-10-17 사용자가 인수.
- 상태: 설계 승인(`78d14b1`) → **구현 반영 개정 r7**(계획 `docs/plans/2026-10-17-l515-terrain-overlay.md`, 브랜치 커밋 `9be520c`–`d20b23c`).
  구현·계획 검토·브랜치 리뷰에서 바꾼 점을 본문에 반영했고 근거는 §11 ④에 모았다. 실기 E2E(§7.3)는 아직 하지 않았다.

## 1. 목적과 범위

L515 깊이 영상에서 바닥 평면을 추정하고, 바닥보다 높은 장애물의 윤곽을 운용 콘솔의
**L515 전방 화면 위에** 거리와 함께 그린다.

- **운용자 시각 보조 전용이다**(2026-10-17 사용자 결정). 주행 제어·정지·속도 제한·
  `SafetyInterlock`·US-100 경로에 어떤 입력도 주지 않는다. 안전 판정 근거로 쓰지 않는다.
- 원격주행 인수 범위(§2 통합 운용 경로)에 맞는 부가 표시다. 자율주행 WP6 지형 코어
  (`powertrain_autonomy`, `autonomy_controller_node`)와 벤치용 `obstacle_zones_node`는
  건드리지 않으며 이 기능과 코드를 공유하지 않는다. 같은 depth 토픽을 구독할 뿐이다.
- 거리 기준은 기본이 **L515 depth 광학 원점**이다. 실차 장착을 잰 뒤 `front_offset_m`을 설정하면
  "차체 앞 기준"으로 바뀐다(§4.1). 보관 URDF 기준으로는 depth 원점이 base_link 앞 0.348 m,
  앞바퀴 앞끝이 0.541 m라 차이는 약 0.19 m다 — 실차 값이 아니므로 기본값으로 쓰지 않는다.

### 전제 (적어 두는 가정)

- **조명**: L515는 적외선 LiDAR라 직사광선·강한 실외광에서 깊이가 대부분 사라질 수 있다. 이 기능은
  깊이가 나오는 환경에서만 의미가 있고, 깊이가 비면 `SENSOR_ERROR`(`DEPTH_EMPTY`)로 표시만 한다.
  대회장 조도에서 유효 화소 비율을 실측한다(§7.3).
- **최소 거리**: L515 최소 측정거리(약 0.25 m) 안쪽과 검은·반사·얇은 물체는 깊이 구멍이 난다 →
  윤곽이 없다고 장애물이 없는 것이 아니다. 화면 배지에 "참고용" 성격을 고정 표기한다(§4.6).
- **지면**: 전방 3 m 안을 **한 평면**으로 근사한다. 경사 전환부·요철 험지에서는 평면 밖 지면이
  장애물로 그려질 수 있다(표시 전용이라 안전 영향 없음, 운용자 혼동 위험은 §10).

### 비목표

- 차체 좌표 변환·오도메트리 결합·경로 계획.
- **바닥보다 낮은 장애물(구덩이·단차 내리막) 검출** — 평면 아래는 표시하지 않는다. 배지에 명시.
- 장애물 추적·히스테리시스(표시는 매 프레임 결과 그대로).
- L515 컬러 왜곡·외부파라미터 회전 보정(§4.3, 합격 기준 미달 시 후속 과제).
- open3d·GPU 의존, 센서 교체.

## 2. 출발점 결함 (2026-10-17 검토, Codex 교차검증 동의)

| 등급 | 결함 | 이 설계의 처리 |
|---|---|---|
| P1 | `TerrainDetector()` 무인자 생성 → `TypeError`, 노드 기동 불가 | 노드가 CameraInfo 수신 후 생성(§4.1) |
| P1 | 미터 변환한 깊이를 넘기는데 `process()`가 다시 ×0.00025 → 정상 장면도 `NO_GROUND` | 코어 입력을 raw uint16 + `depth_scale_m` 하나로 고정, uint16 아니면 `TypeError`(§4.2) |
| P1 | 기본 토픽 `/l515/depth/image_raw` 없음 | 정본 `/l515/depth/image_rect_raw` |
| P2 | UDP :5003은 D435i YOLO metadata 포트 | 신규 :5009(§5.1) |
| P2 | Jetson 이미지에 open3d 없음 | numpy voxel+RANSAC(§4.2) |
| P2 | intrinsics를 녹화 meta.json에서 읽음(`record.py`는 컬러 정렬 깊이·640×480 컬러를 npz로 저장) | `/l515/{depth,color}/camera_info`(§4.1) |
| P2 | 앞축·오른쪽축을 따로 평면 투영해 비직교 → 기울면 정면 장애물이 "왼쪽·경로 밖" | 오른쪽축 = 앞축×법선(§4.2) |
| P2 | 이전 바닥 평면 재사용 중에도 패킷은 그냥 `OK`, 센서 단절 중 hold 카운터 정지 | 단조시계 평면 수명 + 패킷 `ground.state`(§4.2, §5.2) |
| P2 | `sendto()`가 try 밖 → 네트워크 오류로 노드 종료 | 송신 경계에서 예외 흡수(§4.5) |
| P3 | `mask_to_color` 이중 boolean 인덱싱(0-depth 포함 마스크에서 `IndexError`) | 재작성(§4.3) |
| P3 | 패킷 stamp가 촬영시각이 아닌 생성시각, heartbeat 재송신이 새 프레임처럼 보임 | `depth_stamp_ns`·`depth_age_ms` 분리(§5.2) |

## 3. 아키텍처

```
[Jetson · powertrain-sw:ros 이미지 · compose 서비스 powertrain_terrain_overlay]
L515 Gateway ── /l515/depth/image_rect_raw (16UC1, 640×480, raw 비정렬, 약 10 Hz)
             ── /l515/depth/camera_info
             ── /l515/color/camera_info (1280×720)
                        │
                        ▼
  powertrain_ros/terrain_overlay_node.py   (얇은 ROS 어댑터: 구독·최신값 보관·타이머·송신)
     ├ terrain_overlay.detector    raw depth → 바닥 평면·높이·장애물 마스크·zone
     ├ terrain_overlay.projection  depth 화소 → 컬러 화소
     ├ terrain_overlay.contours    컬러 마스크 → 윤곽 다각형·장애물별 거리
     ├ terrain_overlay.contract    패킷 v1 인코딩(≤1200 B)
     └ powertrain_runtime.telemetry.send_datagram → 운용 세션의 콘솔 주소 :5009
                        │ UDP JSON, best effort, latest-only
                        ▼
[운용 PC · operator_console (시스템 python, cv2 없음)]
  operator_console/terrain.py  LatestTerrainReceiver(:5009) → terrain_overlay.contract.parse_packet
  operator_console/app.py      TerrainCanvas — "전방 카메라" VideoPanel 위 투명 레이어
```

### 3.1 모듈 경계

| 모듈 | 책임 | 의존 | 실행 위치 |
|---|---|---|---|
| `terrain_overlay/contract.py` | 상수(포트·스키마·한계), `encode_packet(dict)→bytes`, `parse_packet(bytes)→TerrainFrame` 검증 | **표준 라이브러리만** | Jetson·콘솔 공용 |
| `terrain_overlay/camera.py` | `CameraModel`(핀홀, CameraInfo `k`에서 생성) | 표준 라이브러리 | Jetson |
| `terrain_overlay/detector.py` | `TerrainDetector` | numpy, cv2 | Jetson |
| `terrain_overlay/projection.py` | `DepthToColorProjector` | numpy | Jetson |
| `terrain_overlay/contours.py` | 마스크→다각형·거리, 크기 제한 축소 | numpy, cv2 | Jetson |
| `terrain_overlay/runtime.py` | 송신 틱 판단(§4.1 1–5)·자기 검증·처리시간 기록. rclpy 없이 시험한다 | numpy + 위 모듈 | Jetson |
| `ros2/src/powertrain_ros/powertrain_ros/terrain_overlay_node.py` | ROS 어댑터(메시지 변환·실행기·UDP 송신) | rclpy + 위 모듈 | Jetson |
| `operator_console/terrain.py` | 수신 스레드·세션 게이트·표시 지연 버퍼·표시 판단 순수 함수 `terrain_view` | `terrain_overlay.contract` | 콘솔 |
| `operator_console/app.py` `TerrainCanvas` | `terrain_view` 결과 그리기 | GTK/cairo/PangoCairo | 콘솔 |

- `contract.py`는 numpy·cv2를 import하지 않는다 — 콘솔은 `/usr/bin/python3`(cv2 없음, 2026-10-17 확인)로 돈다.
- 의존 방향: `powertrain_ros` → `terrain_overlay`, `operator_console` → `terrain_overlay.contract`.
  `terrain_overlay`는 ROS·GTK·`powertrain_ros`·`operator_console`을 import하지 않는다.
- `remote_video/contract.py`에 `L515_TERRAIN_UDP_PORT = 5009`를 추가하고, `terrain_overlay.contract`는
  같은 값을 자체 상수로 갖는다(콘솔이 `remote_video`를 import하지 않는 현 구조 유지). 두 값의 일치는 테스트로 묶는다.

### 3.2 `lidar_dev/` 정리

- 삭제: `terrain.py`, `packet.py`, `terrain_node.py`(새 코어로 대체), `before.txt`·`after.txt`(빈 파일).
- 유지·수정: `ground.py`·`overlay.py`·`scan_terrain.py`·`view_depth.py`는 오프라인 분석 도구로 남기고
  `terrain_overlay.detector`를 쓰도록 바꾼다. open3d는 선택 의존이다 — `ground.py`는 open3d가 없으면 3D 보기 없이
  2D 결과만 내고, 3D 보기 전용인 `view_depth.py`는 설치 안내를 출력하고 종료 코드 1로 끝난다.
- `record.py`: 출력 형식을 오프라인 도구 입력(`<폴더>/meta.json` + `depth/*.png`, **raw 비정렬 깊이**와
  depth/color intrinsics 모두 기록)으로 맞춘다. 컬러에 정렬한 깊이는 저장하지 않는다.
- `record.py`는 깊이·컬러 저장이 둘 다 성공한 쌍만 세고, 하나라도 실패하면 경로와 "불완전한 녹화"를 알리고 종료 코드 1로 끝난다.
- `ground.py`의 "장애물 주 평면 각도" 분석(open3d RANSAC)은 옮기지 않는다(판정기 범위 밖, open3d 없이 못 돈다).
- `requirements.txt`(macOS pip freeze)는 삭제하고 `lidar_dev/README.md`에 오프라인 도구 실행법·의존(numpy, opencv, 선택 open3d)을 적는다.

## 4. 구성 요소 설계

### 4.1 ROS 어댑터 `terrain_overlay_node`

- 엔트리포인트: `ros2 run powertrain_ros terrain_overlay` (`setup.py` console_scripts 추가).
- 구독: `/l515/depth/image_rect_raw`, `/l515/depth/camera_info`, `/l515/color/camera_info`.
  - depth QoS는 best effort·volatile·**KEEP_LAST 1**이다(Gateway 발행 QoS와 호환). `qos_profile_sensor_data`(깊이 5)로 두면
    처리가 느릴 때 쌓인 프레임이 늦게 "새로 수신"되어, depth가 끊긴 뒤 `DEPTH_STALE`이 1.2 s에야 나갔다(2026-10-17 시제작 실측).
  - CameraInfo 두 토픽은 `qos_profile_sensor_data`.
- **실행 구조**: `MultiThreadedExecutor(num_threads=2)`. depth 구독은 자기 콜백 그룹에, CameraInfo 두 구독과 타이머는
  다른 한 그룹에 둔다(각각 `MutuallyExclusiveCallbackGroup`). 처리 중인 틱이 depth 수신 시각(`rx_mono`)을 늦추지 않게 하기 위해서다.
  depth 보관과 틱의 입력 읽기는 한 락으로 보호한다.
- 파라미터(기본값):

| 이름 | 기본 | 근거 |
|---|---|---|
| `operator_host` | `""` | §4.5 |
| `operator_port` | 5009 | §5.1 |
| `publish_hz` | 10.0 | Gateway raw depth 주기 0.1 s(`gateway_workers.py`). **허용 5–10** (stale 판정 시간 보장, 아래) |
| `depth_scale_m` | 0.00025 | L515 1/4000 (`l515_cloud_node.DEPTH_SCALE_M`과 동일) |
| `depth_stale_s` | 0.5 | 새 depth가 이보다 오래 없으면 `SENSOR_ERROR` |
| `ground_hold_s` | 0.5 | 평면 재사용 상한(§4.2-6). lidar_dev는 15프레임≈1.5 s |
| `color_offset_m` | `[0.0, -0.024, 0.0]` | depth 광학 프레임에서 본 컬러 원점(보관 URDF, Codex 재계산 확인) |
| `path_half_width_m` | 0.4445 | 로버 바깥 반폭 0.3945 m + 여유 0.05 m (WP6 계획 `docs/plans/2026-07-28-wp6-real-course-autonomy-development-plan.md` 27행). lidar_dev는 0.52 |
| `front_offset_m` | 0.0 | 모든 거리에서 뺄 값(0 이상, 결과는 0으로 하한). 0이면 거리 기준 = 카메라, >0이면 차체 앞. 실차 측정 후 설정 |
| `alignment_verified` | false | §7.3-3 합격 후 true. false면 콘솔이 "정합 미검증"을 표시 |

- CameraInfo: 두 토픽에서 `k`(fx, fy, cx, cy)와 `width/height`, `distortion_model`, `d`를 받는다.
  둘 다 받기 전에는 `SENSOR_ERROR`/`CAMERA_INFO_MISSING`. depth CameraInfo 해상도가 depth 이미지와
  다르면 `SENSOR_ERROR`/`DEPTH_SHAPE_MISMATCH`. 컬러 CameraInfo가 1280×720이 아니면
  `SENSOR_ERROR`/`COLOR_SHAPE_MISMATCH`. K 값이 바뀌면 detector·projector를 재생성한다(같으면 유지).
  처음 받을 때 두 CameraInfo의 `distortion_model`·`d`를 1회 로그로 남긴다.
- 깊이 수신 콜백: `encoding == "16UC1"`, `is_bigendian == 0`, `step == width*2`를 확인한 뒤
  `np.frombuffer(...).reshape(h, w).copy()`로 최신 1장만 보관한다. 위반 시 그 프레임은 버리고
  `SENSOR_ERROR`/`DEPTH_ENCODING`을 표시한다. 보관 항목: 배열, 헤더 stamp(ns), 수신 단조시각.
- 타이머 틱(`publish_hz`), `now`는 단조시각. 틱 논리는 `terrain_overlay/runtime.py`에 있고 시계를 주입받는다.
  아래 판단을 **두 번** 한다 — 첫 번째는 새 depth를 처리해 캐시를 채우고, 처리 뒤 시계를 다시 읽은 두 번째가
  송신할 상태·stale·만료·두 age를 **송신 시각 기준**으로 정한다(두 번째는 캐시를 쓰므로 다시 처리하지 않는다).
  0단계로 `CAMERA_INFO_MISSING` → `COLOR_SHAPE_MISMATCH` → `DEPTH_ENCODING`(마지막 불량 프레임이 마지막 정상 프레임보다
  새롭고 stale 시간 안일 때) 순으로 먼저 본다. 처리시간 `proc_ms`도 같은 시계로 잰다.
  1. 최신 depth가 없거나 `now − rx_mono > depth_stale_s` → `SENSOR_ERROR`/`DEPTH_STALE`.
  2. 새 depth(헤더 stamp가 직전 처리와 다름) → 코어 처리(`process(depth, frame_rx_s=그 depth의 수신 단조시각)`), 결과를 캐시.
  3. 새 depth 없음(아직 stale 아님) → 캐시 결과를 재사용한다. 재계산하지 않고 `depth_stamp_ns`도 그대로 둔다.
  4. 2·3 모두 **송신 직전에 같은 만료 검사**를 거친다. 시간 기준은 detector와 노드 모두
     **depth 수신 단조시각**이다(처리 시각이 아니다): `depth_age_s = now − rx_mono`,
     `ground_age_s = now − fresh_rx_s`(평면을 마지막으로 FRESH로 만든 depth의 수신 시각, 결과에 포함).
     결과가 `OK`인데 `ground_age_s > ground_hold_s`(초끼리 비교)면 `NO_GROUND`/`GROUND_EXPIRED`로 바꿔 보낸다
     (새로 처리한 결과도 처리 지연 때문에 여기서 만료될 수 있다).
     `ground.state`는 결과 값 그대로(FRESH는 "이 평면이 그 depth에서 새로 맞춰졌다"는 뜻) 두고,
     전송 직전에만 `depth_age_ms = 1000×depth_age_s`, `ground.age_ms = 1000×ground_age_s`로 바꾼다.
  5. 코어 예외 → `SENSOR_ERROR`/`PROCESSING_ERROR`, 로그는 5 s당 1회.
- **stale 판정 시간 계약**: depth가 끊긴 뒤 `SENSOR_ERROR` 패킷이 나가기까지
  ≤ `depth_stale_s + 1/publish_hz + 최대 처리시간 + 0.05 s`. 처리시간이 0이면 0.65 s(5 Hz에서 0.75 s)다.
  - 최대 처리시간은 한 틱(`1/publish_hz`) 이하여야 한다. 그러면 상한은 10 Hz에서 0.75 s, 5 Hz에서 0.95 s다.
  - 성공·예외와 관계없이 모든 처리 시도를 반올림 전 값으로 한 틱과 비교해, 넘으면 `processing_overruns`를 세고
    일반 오류와 다른 호출 위치에서 경고를 남긴다(rclpy 로그 제한은 호출 위치별이다).
  - 노드 종료 때 제한 없는 요약 한 줄 `terrain processing summary: max … ms, overruns N`을 남긴다. §7.3-2는 이 줄로 판정한다.
  - 콘솔은 수신이 끊긴 동안 0.5 s 연결 생존 한계로 독립적으로 윤곽을 지운다. 늦은 오류 패킷이 도착해 연결이 되살아난 뒤에는
    표시 지연 버퍼와 depth 나이 규칙(§4.6)이 적용된다.
- 매 틱 패킷 1개를 보낸다(패킷 자체가 생존 신호). 유일한 예외는 §4.4의 오류 패킷 자기 검증마저 실패한 틱(미송신·로그)과
  세션 파일 만료(§4.5)다. `sequence`는 송신한 패킷마다 단조 증가,
  세션 식별은 노드 기동 시 1회 정하는 세 값이다(§5.2): `sender_boot_id` = `/proc/sys/kernel/random/boot_id`
  (하이픈 뺀 32자리 hex), `sender_start_mono_ns` = 기동 시 `time.monotonic_ns()`, `sender_instance` = 기동 시 `secrets.token_hex(8)`
  (16자리 hex, 시작값이 같을 때의 유일성·동률 판정용). 세션 키는 세 값의 조합이다.
  **지원 배포 전제(clock domain)**: 모든 송신 노드는 Jetson 호스트의 `CLOCK_MONOTONIC`을 그대로 본다 — 즉 time namespace를 쓰지 않는
  컨테이너(현 docker compose 기본)나 호스트 프로세스다. 이 전제에서 같은 부팅 안의 나중 기동 노드는 시작값이 같거나 크다(역행 없음, 같을 수는 있음).
  2026-10-17 로컬 docker로 boot_id 일치·monotonic 연속을 확인했고, Jetson에서는 배포 때 같은 확인(§7.3-6)을 한다.
  전제가 깨진 배포(time namespace 사용)는 지원하지 않으며, 그 확인이 실패하면 배포를 중단한다.

### 4.2 판정기 `TerrainDetector` (lidar_dev `terrain.py` 이식)

입력: raw depth `uint16[h,w]`(다른 dtype이면 `TypeError` — 미터 배열 오입력을 조용히 처리하지 않는다),
`frame_rx_s`(그 depth의 수신 단조시각 — 평면 수명의 유일한 시간 기준). 이하 `t = frame_rx_s`. 출력: `TerrainResult`(상태·평면·장애물 마스크·fwd/lat 배열·zone).

1. `z = depth × depth_scale_m`, `valid = z > 0`. 유효 화소 < 5 % → `SENSOR_ERROR`/`DEPTH_EMPTY`.
2. 3D 점 `P = (xn·z, yn·z, z)` (정규화 좌표는 생성 시 1회 계산).
3. **평면 만료를 먼저 적용**: 직전 평면이 있고 `t − fresh_rx_s > ground_hold_s`(초)면 이번 피팅 전에 폐기하고
   `expired = True`로 기록한다(만료된 평면이 새 평면에 섞이지 않게 — 아래 6에서 평활도 쓰지 않는다).
   `expired`는 다음 FRESH 수용 때, 또는 detector 재생성 때 False로 돌아간다.
4. **바닥 후보**: 화면 아래 절반의 유효점 → 2 cm voxel 축소(정수 격자 키 `np.unique`, 격자당 첫 점).
   200점 미만이면 이번 프레임 평면 실패. 200점 이상이면 **고정 시드로 최대 3000점을 무작위 추출**해 RANSAC 채점에 쓴다.
5. **RANSAC(numpy)**: 고정 시드로 1000개 3점 조합을 만들고, **100개씩 묶어** 채점한다
   (거리 행렬 ≤ 3000×100 float32 ≈ 1.2 MB, 전체 화소 행렬을 만들지 않는다). 인라이어(거리 < 2 cm) 최대 평면을 고른 뒤
   4의 전체 후보점 중 인라이어로 최소제곱 재피팅한다 — **3×3 공분산 행렬의 고유분해(`np.linalg.eigh`, 최소 고유값의 벡터 = 법선)**로
   하며 N×N 행렬을 만드는 `np.linalg.svd(full_matrices=True)`는 쓰지 않는다. 법선은 카메라 원점이 평면 위(+)가 되게 부호를 맞춘다.
   채점·재피팅 전 과정에서 추가 메모리는 O(N)이다(N ≤ 아래 절반 화소 153,600, float64 3열 ≈ 3.5 MiB).
6. **평면 수용 조건**(lidar_dev와 동일): 기울기 `arccos|n_y| ≤ 40°`, 카메라 높이 `0.25 < d < 0.80 m`.
   - 수용 → **이번 프레임 평면을 그대로 쓴다**(lidar_dev의 0.7/0.3 지수 평활 제거: 로커-보기 차체가 흔들리는
     동안 평활은 현재 프레임과 맞지 않는 평면을 만들고, 5의 공분산 재피팅이 이미 잡음을 줄인다). `state = FRESH`, `fresh_rx_s = t`.
   - 실패, 3 이후 남은 직전 평면 있음 → 직전 평면 사용, `state = HELD`.
   - 그 외 → `NO_GROUND`. reason은 `expired`면 `GROUND_EXPIRED`, 아니면(기동 후 평면을 한 번도 못 찾았거나 재생성 직후) `GROUND_NOT_FOUND`.
7. **좌표축**(직교화): `f = normalize(ẑ − (ẑ·n)n)` (광축의 바닥 투영 = 앞), `r = normalize(f × n)`을
   `r·x̂ > 0`이 되도록 부호 조정(광학 x = 오른쪽). `fwd = P·f`, `lat = P·r`, `H = P·n + d`.
8. **분류**: 바닥 `|H| < 0.03`, 장애물 `0.05 < H < 2.0 ∧ 0 < fwd < 3.0`, 5×5 열림 연산.
9. **zone**: `left: lat < −hw`, `center: |lat| ≤ hw`, `right: lat > hw` (hw = `path_half_width_m`).
   zone 거리 = 그 zone 장애물 점 `fwd`의 2 백분위수 − `front_offset_m`(0 하한).

수치 상수(0.03/0.05/2.0/3.0 m, 2 cm, 1000회, 40°, 0.25–0.80 m)는 lidar_dev 값을 그대로 옮긴다.
조정은 실기 검증 결과로만 한다. 바뀐 것은 평활 제거, 평면 수명 0.5 s(시간 기준), 채점 표본 상한뿐이다.

### 4.3 투영 `DepthToColorProjector`

- depth 화소 (u, v, z) → depth 광학 3D `p_d` → 컬러 광학 3D `p_c = p_d − color_offset_m` (회전 없음)
  → 컬러 화소 `(fx_c·x/z + cx_c, fy_c·y/z + cy_c)`. `z ≤ 0` 또는 프레임 밖은 버린다.
- 입력 마스크의 0-depth 화소는 먼저 제외한다(이중 인덱싱 제거).
- **왜곡 계수는 쓰지 않는다(핀홀).**
  - depth: RealSense SDK의 L515 depth는 `RS2_DISTORTION_NONE`이라 핀홀이 맞다(Codex가 SDK 2.50.0 소스로 확인).
  - color: 비영 계수가 있을 수 있으나, Gateway의 `camera_info_from_intrinsics`가 RealSense 모델을
    `plumb_bob`/`equidistant`로만 접고(`l515_adapter.py` `_distortion_model`) SDK에도 Brown/inverse 표기
    혼동 TODO가 있어, CameraInfo만으로는 `cv2.projectPoints` 적용 방향을 확정할 수 없다.
  - 그래서 핀홀로 시작하고 **§7.3-3의 합격 기준으로 판정**한다. 미달이면 `alignment_verified=false`를 유지해
    콘솔에 "정합 미검증"이 계속 뜨고, SDK에서 extrinsics·왜곡 모델을 직접 받아 쓰는 후속 과제로 넘긴다.

### 4.4 윤곽·패킷 생성 `contours`

- 장애물 마스크를 컬러 해상도 마스크로 투영 → 9×9 닫힘 + 5×5 팽창(640×480 → 1280×720 성긴 점 메우기,
  lidar_dev 값) → 외곽 윤곽 → 면적 < 150 px² 제거 → `approxPolyDP`(ε = 둘레 1 %, 꼭짓점 > 16이면 ε×1.5 반복).
- **근사 결과가 3점 미만이면** 그 윤곽의 `cv2.minAreaRect` 네 꼭짓점(프레임 안으로 clip)으로 대체한다
  (가늘고 긴 윤곽이 2점으로 줄어 패킷 전체가 거부되는 반례 — 1100×8 px, 면적 7693 px² → 2점, 2026-10-17 재현).
- 장애물별 거리: 다각형 내부로 투영된 장애물 점의 `fwd` 5 백분위수 − `front_offset_m`(0 하한).
  `in_path`: 그 점들 중 `|lat| ≤ hw`가 하나라도 있음.
- 거리순 정렬, 최대 4개.
- 크기 제한 1200 B: 꼭짓점 > 6인 다각형을 `[::2]`로 줄이기(7점 이상이므로 결과 ≥ 4점) → 그래도 넘으면
  가장 먼 장애물부터 제거. 인코딩 직후 `parse_packet`으로 자기 검증하고, 실패하면 **정상 패킷을 고치지 않고**
  오류 패킷을 처음부터 새로 만든다: `status=SENSOR_ERROR`, `reason=PROCESSING_ERROR`, `ground=null`,
  `zones` 셋 다 `null`, `obstacles=[]`, `proc_ms=null`, 공통 필드는 노드 상수(세션·sequence·frame_size·설정값)와
  `depth_stamp_ns`/`depth_age_ms`만 다시 채운다. 이 오류 패킷도 자기 검증하며, 그것마저 실패하면 그 틱은 송신하지 않고 로그만 남긴다
  (계약 위반 패킷을 내보내지 않는다).

### 4.5 송신

`powertrain_runtime.telemetry.send_datagram(sock, payload, (operator_host, operator_port))`를 쓴다.
동작(`destination.py`): 환경변수 `POWERTRAIN_OPERATOR_SESSION_FILE`이 **있으면** 그 파일의 유효한 세션 주소로
보내고, 파일이 없거나 만료면 보내지 않는다. 환경변수가 **없으면** `operator_host`로 보낸다.
`OSError`는 노드 안에서 잡아 카운트·5 s당 1회 로그로 남기고 다음 틱을 계속한다.
`operator_host`가 비었고 환경변수도 없으면 기동 실패(chassis_telemetry와 같은 조건).

### 4.6 콘솔

- `operator_console/terrain.py` `LatestTerrainReceiver(port, display_delay_s=0.0)`:
  - 데몬 스레드, 최신값 보관, 무효 패킷 카운트(`LatestMetadataReceiver`와 같은 골격).
  - **세션 게이트**(`SourceSequenceGate`는 같은 주소 재시작을 sequence ≤ 3일 때만 받아 10 Hz 재시작이 거부되는
    반례가 있어 재사용하지 않는다). **송신자는 주소가 아니라 세션 `(sender_boot_id, sender_start_mono_ns, sender_instance)`로
    식별하고, 같은 부팅 안의 세션은 `(start, instance)` 사전식 순서로 순서를 매긴다.** 상태: 현재 세션, 그 세션의 마지막 sequence, 마지막 수용 시각.
    - 현재 세션(세 값 모두 같음, 주소 무관): `sequence`가 마지막보다 클 때만 수용. 주소가 바뀌어도 sequence 하한은 유지한다.
    - 같은 부팅의 다른 세션: `(start, instance)`가 현재보다 **크면** 즉시 전환·수용(노드 재시작), **작으면 항상 거부**
      (옛 세션의 지연·중복 패킷은 수신 공백과 무관하게 영원히 거부된다 — 침묵 시간에 기대지 않는다).
      시작값이 같은 두 노드는 instance로 갈리므로 한 sequence 흐름으로 섞이지 않는다(instance 큰 쪽만 표시).
    - 다른 부팅의 세션: 부팅 사이에는 순서를 알 수 없으므로 현재 세션이 **`BOOT_SWITCH_SILENCE_S = 1.0 s`** 넘게
      수용 패킷이 없을 때만 전환. Jetson 재부팅은 수십 초가 걸려 그동안 현재 세션이 침묵한다. 재부팅 전 패킷이 재부팅 후에야
      도착해 옛 부팅으로 되돌아가는 경우는 실질적으로 없으며, 일어나도 옛 세션은 더 송신하지 않으므로 **마지막으로 수용한 옛 패킷 뒤 1 s**에
      새 부팅 세션으로 복귀한다(지연 패킷이 계속 도착하면 그만큼 늦어진다 — §10).
    - 현재 세션이 없으면(콘솔 기동 직후) 바로 수용.
    - 두 노드가 같은 부팅에서 동시에 송신하는 오설정이면 **`(start, instance)`가 큰 쪽만** 표시된다.
    - 세션이 바뀌면 표시 버퍼(아래)에서 이전 세션 패킷을 모두 버린다.
  - **수신 생존과 표시 패킷을 분리한다.** 수신기는 두 가지를 제공한다.
    - `last_rx_monotonic`: 마지막으로 **수용한** 패킷의 수신 시각(연결 생존 판정용, 지연과 무관).
    - `display_frame(now)`: 표시 지연 `d = display_delay_s`에 대해 수신 시각 ≤ `now − d`인 가장 새 패킷. `d = 0`이면 최신 패킷.
      버퍼는 `d + 0.5 s`만큼만 보관한다.
  - **표시 지연 `d`**: 허용 범위 0–0.4 s(그 이상이면 설정 오류로 기동 실패). §7.3-4에서 잰
    `Δ = (영상 속 출현 시각) − (윤곽 출현 시각)`으로 정한다 — `Δ > 0`(윤곽이 먼저)이면 `d = Δ`, `Δ ≤ 0`(윤곽이 늦거나 같음)이면
    `d = 0`(오버레이를 앞당길 수는 없다). `Δ > 0.4 s`면 기본값을 0.4로 두고 보고한다.
- `LatestTerrainReceiver.snapshot(now)`은 `(최신 패킷, 표시 패킷, last_rx)`를 한 락에서 읽는다(서로 다른 세션이 섞이지 않게).
  수신 스레드는 예외가 나도 살아 있고, `parse_packet`은 `RecursionError`·`OverflowError`·잘못된 타입도 `ValueError`로 거부한다.
- `TerrainCanvas(Gtk.DrawingArea)` — "전방 카메라" `VideoPanel`에 겹친다. 판정은 순수 함수 `terrain_view`가 하고 캔버스는 그리기만 한다.
  - **레이어**: 영상 대기 화면보다 위, 헤더보다 아래에 pass-through(입력 통과)로 붙는다. 그래서 배지는 영상 연결 전·끊김 중에도 보인다.
    D435 metadata 캔버스의 위치(영상 레이어 안)와 입력 처리는 바꾸지 않는다.
  - **글자는 PangoCairo로 그린다**(말줄임 포함). cairo 기본 텍스트 API는 한글 대체 글꼴이 없어 빈 네모로 나온다(2026-10-17 재현).
  - 100 ms 재그리기 타이머는 캔버스 `destroy`에서 해제한다.
  판정 값:
  `link_age = now − last_rx_monotonic`(연결), 표시 패킷 `F = display_frame(now)`,
  `depth_age = F.depth_age_ms/1000 + (now − d − F.received_monotonic)` — 표시 지연으로 의도적으로 늦춘 `d`는 빼고 잰다
  (영상도 그만큼 늦게 오므로). 즉 "영상 시간 기준 depth 나이"다.
  - **배지(항상)**: `link_age > 0.5 s`(또는 수신 없음)면 "지형: 수신 없음". 아니면 `F`(없으면 최신 패킷)의 상태별
    `지형 OK · 경로 1.23 m` / `지형 OK · 경로 돌출 장애물 없음`(중앙 zone이 null) / `지형: 바닥 유지 0.3 s` (HELD) /
    `지형: 바닥 미검출` / `지형: 센서 오류(DEPTH_STALE)`.
    - "장애물 없음"이 아니라 "돌출 장애물 없음"이라 쓴다. 구덩이가 없다는 뜻으로 읽히지 않게 하기 위해서다.
    - **배지 대상 패킷 `S`(= `F`, 없으면 최신 패킷)가 OK인데 그 depth 나이**
      `S.depth_age_ms/1000 + max(0, now − d − S.received_monotonic)`가 **0.6 s를 넘거나 `depth_age_ms`가 null이면**
      거리·"없음" 요약을 내지 않고 `지형: 깊이 지연 0.7 s`(경고색, null이면 숫자 없이 `지형: 깊이 지연`)로 바꾼다(HELD보다 우선).
      `F`가 아직 없어(`d > 0`, 첫 패킷 직후) 최신 패킷을 쓸 때도 같다. 마지막 OK 패킷 뒤 링크가 끊기면 0.5 s 동안
      옛 거리가 현재 판단처럼 보이던 문제(브랜치 리뷰 P2, §11 ④)를 막는다. 윤곽은 계속 `F`로만 판단한다(아래).
    거리 옆에 기준(`카메라 기준`/`차체 앞 기준`)을 붙이고, `alignment_verified=false`면 "정합 미검증",
    항상 "참고용 · 낮은 장애물/구덩이 미표시"를 작게 붙인다.
  - **윤곽(조건부)**: `F` 존재 ∧ `F.status == OK` ∧ `link_age ≤ 0.5 s` ∧ `depth_age ≤ 0.6 s` ∧ `frame_size`가 협상된 영상 크기와 같음일 때만.
    영상 크기 미협상(`None`)이면 그리지 않는다(기존 `overlay_size_matches`는 None을 허용하므로 terrain 쪽은 엄격하게).
    영상 상태가 LIVE가 아니면(대기·ERROR·EOS·파이프라인 재시작 직후 포함) 그리지 않는다 — `VideoPanel`이 영상 상태를
    `set_video_live`로 즉시 전달한다.
    배지는 OK인데 윤곽을 숨겼으면 안내문 앞에 이유를 붙인다: `윤곽 숨김: 영상 끊김` / `영상 크기 미확인` / `영상 W×H ≠ 1280×720`.
  - 다각형: `in_path`면 빨강, 아니면 주황 외곽선 + 반투명 채움. 각 다각형 위에 `dist_m`("1.23 m").
  - `fit_overlay_transform`으로 레터박스를 반영한다.
- `VideoPanel`: 오버레이 캔버스를 여러 개 가질 수 있게 일반화한다(현재는 `_metadata_canvas` 하나에만
  영상 크기를 전달). 기존 D435 metadata 경로 동작은 바꾸지 않는다. PiP 전환은 `VideoPanel` 전체가 이동하므로
  캔버스도 함께 따라간다. L515 스트림의 RGB/DEPTH/OVERLAY 모드는 모두 컬러 정렬 1280×720이라 좌표계가 같다.
- 표시 옵션: 기존 "물체 표시"·"대상 거리" 옆에 "지형 장애물" 체크박스(기본 ON, 무영속 — 기존 옵션과 동일).
- 포트·설정: `--terrain-port`(기본 5009), `--terrain-display-delay-ms`(기본 0), `__main__.py`의 통합 설정 기본값 dict,
  `demo_source.LIVE_PORTS`(데모가 실포트를 막는 목록)에 5009 추가. 데모 소스에 terrain 패킷 생성기를 추가한다.

## 5. 패킷 계약 v1

### 5.1 채널

| 항목 | 값 |
|---|---|
| 방향 | Jetson → 운용 PC, UDP, best effort, latest-only |
| 포트 | **5009** (`L515_TERRAIN_UDP_PORT`). 5000·5002–5008 사용·예약, 5001은 구 YOLO 좌표 포트라 회피 |
| 주기 | `publish_hz`(기본 10 Hz, 허용 5–10). 새 depth가 없어도 매 틱 송신 |
| 최대 크기 | 1200 B (단편화 회피) |
| 인코딩 | UTF-8 JSON, `separators=(",", ":")` |

### 5.2 필드

```json
{"schema_version":1,"type":"l515_terrain","sender_boot_id":"0082f17fdc0a49969ff0fe1dc4979560",
 "sender_start_mono_ns":707316067583855,"sender_instance":"3fa95c0e81d2b7a4","sequence":123,
 "sent_unix_ns":1760000000000000000,
 "depth_stamp_ns":1760000000000000000,"depth_age_ms":35.2,
 "status":"OK","reason":null,
 "frame_size":[1280,720],"distance_origin":"l515_depth_optical","alignment_verified":false,
 "ground":{"state":"FRESH","age_ms":35.2,"camera_height_m":0.512,"camera_tilt_deg":21.4},
 "path_half_width_m":0.4445,
 "zones":{"left":null,"center":1.23,"right":2.10},
 "obstacles":[{"dist_m":1.23,"in_path":true,"poly":[[612,410],[700,410],[700,520],[612,520]]}],
 "proc_ms":18.4}
```

| 필드 | 형식·규칙 |
|---|---|
| `schema_version` | 정수 1. 다르면 거부 |
| `type` | `"l515_terrain"` 고정. 다르면 거부(:5003 YOLO 패킷 등 오수신 방지) |
| `sender_boot_id` | 32자리 소문자 hex(커널 boot_id에서 하이픈 제거) |
| `sender_start_mono_ns` | 0 이상 정수(bool 아님). 노드 기동 시 CLOCK_MONOTONIC |
| `sender_instance` | 16자리 소문자 hex. 노드 기동 시 무작위 |
| `sequence` | 0 이상 정수(bool 아님), 틱마다 +1 |
| `sent_unix_ns` | 0 이상 정수. 송신 벽시계(진단용, 콘솔 판정에 안 씀) |
| `depth_stamp_ns` | 0 이상 정수 또는 `null`. 처리한 depth의 ROS 헤더 stamp, 재송신이면 이전과 같음 |
| `depth_age_ms` | 유한 수 ≥ 0 또는 `null`. depth 수신 후 송신까지 경과(단조시계) |
| `status` | `OK` / `NO_GROUND` / `SENSOR_ERROR` |
| `reason` | `OK`면 `null`. `NO_GROUND`면 `GROUND_NOT_FOUND`·`GROUND_EXPIRED`. `SENSOR_ERROR`면 `DEPTH_STALE`·`DEPTH_EMPTY`·`DEPTH_ENCODING`·`DEPTH_SHAPE_MISMATCH`·`COLOR_SHAPE_MISMATCH`·`CAMERA_INFO_MISSING`·`PROCESSING_ERROR`. 목록 밖이면 거부 |
| `frame_size` | `[1280, 720]` 고정 — 다각형 좌표계(L515 컬러) |
| `distance_origin` | `"l515_depth_optical"`(front_offset 0) 또는 `"rover_front"` |
| `alignment_verified` | bool |
| `ground` | `status == OK`일 때만 객체(아니면 `null`): `state` `FRESH`/`HELD`, `age_ms` 유한 ≥ 0, `camera_height_m` 유한 (0.25, 0.80), `camera_tilt_deg` 유한 [0, 40]. `age_ms`는 평면을 마지막으로 FRESH로 만든 depth의 수신 후 송신까지 경과(ms). FRESH 캐시 재송신이면 0보다 클 수 있다 |
| `path_half_width_m` | 유한 수 > 0 |
| `zones` | 키 `left`·`center`·`right` 모두 존재. 값은 유한 수 ≥ 0 또는 `null`. `status != OK`면 셋 다 `null` |
| `obstacles` | ≤ 4개. `dist_m` 유한 ≥ 0, `in_path` bool, `poly` 3–16개 `[x,y]` 정수(bool 아님), `0 ≤ x < 1280`, `0 ≤ y < 720`. `status != OK`면 빈 배열 |
| `proc_ms` | 유한 수 ≥ 0 또는 `null`(코어 처리 전 오류) |

`parse_packet`은 위 규칙을 모두 검사하고, 하나라도 어기면 패킷 전체를 거부한다(부분 표시 없음).
최상위의 알 수 없는 추가 필드는 무시한다(전방 호환). 1200 B 초과도 거부한다.

## 6. 오류 처리 요약

| 상황 | 송신부 | 콘솔 |
|---|---|---|
| CameraInfo를 받은 뒤 depth 끊김(Gateway 정지 등) | ≤ 0.65 s + 최대 처리시간(10 Hz 상한 0.75 s) 안에 `SENSOR_ERROR`/`DEPTH_STALE`, 이후 계속 송신 | 윤곽 지움, 배지 "센서 오류" |
| 처리시간이 한 틱 초과 | 그대로 송신, `processing_overruns` 증가·경고(별도 호출 위치), 종료 시 요약 줄 | — |
| CameraInfo 미수신(처음부터 Gateway 미기동 포함 — 0단계에서 `DEPTH_STALE`보다 먼저 판단) | `SENSOR_ERROR`/`CAMERA_INFO_MISSING` | 같음 |
| 바닥 일시 실패 | 0.5 s까지 `OK`+`HELD` | 윤곽 표시 + "바닥 유지 x.x s" |
| 바닥 계속 실패·평면 만료 | `NO_GROUND`(`GROUND_NOT_FOUND`/`GROUND_EXPIRED`) | 윤곽 지움, "바닥 미검출" |
| 오래된 `OK` 직후 통신 단절 | — | `depth_age > 0.6 s`에서 윤곽 지움·배지 "깊이 지연", `link_age > 0.5 s`에서 "수신 없음" — 먼저 오는 쪽 |
| 영상 끊김·크기 미확인/불일치 | — | 윤곽만 지움, 배지 유지, 안내문 "윤곽 숨김: …" |
| 처리 예외·자기 검증 실패 | `SENSOR_ERROR`/`PROCESSING_ERROR`, 로그 제한 | 윤곽 지움, 배지 |
| 송신 실패(OSError) | 흡수·카운트, 다음 틱 계속 | 0.5 s 후 "수신 없음" |
| 세션 만료 | 송신 안 함(기존 규칙) | "수신 없음" |
| 잘못된 패킷 | — | 거부, 무효 카운트 증가 |
| 노드 재시작 | 새 세션(같은 boot_id, 더 큰 start), sequence 0부터 | 첫 패킷부터 즉시 수용, 옛 세션 패킷은 이후 계속 거부 |
| Jetson 재부팅 | 새 boot_id | 옛 세션 1 s 침묵 뒤 수용 |

## 7. 테스트·검증

### 7.1 호스트 단위 테스트 (TDD)

- `terrain_overlay/tests/` (conda: numpy·cv2):
  - 합성 장면(카메라 높이 0.5 m·20° 하향, 1.2 m 앞 높이 0.2 m·폭 0.3 m 상자) → `OK`, 중앙 zone 1.20±0.03 m, 장애물 1개 `in_path`.
  - 입력 계약: float 배열 입력은 `TypeError`.
  - **음성 대조**: 스케일 변환을 이중 적용하도록 버그를 재주입(monkeypatch)하면 위 합성 장면 시험이 FAIL함을 확인하는 시험을 별도로 둔다.
  - 롤 기울기(Codex 반례 법선 `(0.55, −0.779, 0.30)`)에서 횡거리 0 장애물이 `center`.
  - 평면 수명(모든 시각은 `frame_rx_s`): FRESH → 실패 0.3 s HELD → 0.6 s 실패 `NO_GROUND`/`GROUND_EXPIRED` → 계속 실패해도 `GROUND_EXPIRED` 유지 →
    FRESH 수용 후 다시 실패·만료 전 HELD. 기동 직후 실패는 `GROUND_NOT_FOUND`. 처리 미호출 간격도 시간으로 반영.
  - 재피팅 메모리: 아래 절반 전체가 바닥인 장면(N 최대)에서 `tracemalloc` 최대 < 50 MB.
  - **만료 후 첫 성공 프레임**: 기울기 0° 평면 FRESH → 3 s 공백 → 30° 장면 FRESH → 바닥점 H ≈ 0(만료 평면 미혼합).
  - RANSAC 결정성(같은 입력 → 같은 평면), 채점 표본 상한(전 화면 유효 깊이에서도 처리 메모리 < 50 MB, `tracemalloc`).
  - 평면 수용 조건 경계(높이 0.25/0.80, 기울기 40°).
  - 투영: 중심점 1 m가 컬러 cy보다 `fy_c×0.024` px 아래. 0-depth 포함 마스크에서 예외 없음.
  - 윤곽: 1100×8 px 장애물 → 4점 다각형. 장애물 많은 장면에서 패킷 ≤ 1200 B, 꼭짓점 3–16, 좌표 범위, 자기 검증 통과.
  - 자기 검증 실패 경로: 정상 패킷 생성을 일부러 망가뜨리면(monkeypatch) 상태별 규칙을 지키는 `PROCESSING_ERROR` 패킷이 나가고
    `ground`·`zones`·`obstacles`·`proc_ms`가 비어 있음.
- `contract` 테스트(conda와 시스템 python 둘 다): 인코딩↔파싱 왕복, §5.2 각 규칙의 거부 사례(NaN·bool·범위·상태별 필수값·reason 목록),
  `type` 불일치 거부, `remote_video.contract.L515_TERRAIN_UDP_PORT == terrain_overlay.contract.UDP_PORT`.
- `operator_console/tests/` (시스템 python):
  - 세션 게이트(A, B는 같은 boot_id, `(start, instance)` A < B): 같은 세션 sequence 감소·중복 거부 /
    **같은 세션의 주소 이동은 sequence 하한 유지하며 수용**(`(주소1,A,100)` → `(주소2,A,99)` 거부 → `(주소2,A,101)` 수용 →
    `(주소1,A,102)` 수용) / B 첫 패킷 즉시 수용 / **수신 공백 반례**: 0.00 A/100 → 1.10 B/0 → 1.30 B/1 →
    2.41 A/99 거부 → 2.42 B/12 수용(Codex 4회차 표) / 동시 송신 A·B → B만 수용 /
    **시작값 동률**: A·B가 같은 boot·같은 start, instance만 다름 → `A/100` 수용 → `B/0` 수용(전환, instance B > A) →
    `A/101` 거부 → `B/1` 수용(한 흐름으로 섞이지 않음) /
    다른 boot_id 세션은 현재 세션 활동 중 거부, 1.0 s 침묵 뒤 수용 / 세션 전환 시 버퍼 비움 /
    boot_id·start·instance 형식 위반 거부.
  - 표시 지연: `d = 0.3 s`, 10 Hz 정상 수신에서 `link_age < 0.15 s`이고 윤곽이 계속 표시됨(“수신 없음” 아님),
    `display_frame`이 약 0.3 s 전 패킷을 고름. `d` 범위 밖(−0.1, 0.5)은 기동 실패.
  - 표시 판단 함수(그리기와 분리한 순수 함수): 상태·`link_age`·`depth_age`(d = 0에서 490 ms 패킷 수신 후 단절 → 0.11 s 뒤 숨김,
    d = 0.3이면 그 패킷이 표시되기 시작한 뒤 0.11 s, 즉 마지막 수신 T 기준 T+0.41 s에 숨김)·크기 불일치·크기 미협상.
  - `LIVE_PORTS`에 5009 포함, 인자 기본값.
- `ros2/src/powertrain_ros/test/` (Jetson 컨테이너): 세션 필드(boot_id 형식, `monotonic_ns`를 같은 값으로 고정한 두 생성에서도 instance가 달라 세션 키가 다름), 노드 틱 로직 — **수신→처리 지연 반례** — (가) t=0 수신 프레임 FRESH, t=0.59 수신 프레임 피팅 실패 →
  detector가 `frame_rx_s` 기준 0.59 > 0.5로 `GROUND_EXPIRED`(HELD·age 590 ms 패킷이 나가지 않음),
  (나) t=0.45 수신 프레임 피팅 실패를 t=0.56에 송신 → detector는 HELD지만 송신 직전 검사로 `GROUND_EXPIRED`, stale 시간 계약, 재송신 시 stamp 유지·`ground.age_ms`(ms) 갱신,
  **FRESH 캐시 재송신이 자기 검증을 통과**(age_ms > 0 허용), **캐시 HELD가 0.5 s를 넘으면 `GROUND_EXPIRED`**,
  encoding 거부, 송신 예외 흡수.

### 7.2 실행 스모크 (§5 완료 선언 규칙)

- 콘솔: `/usr/bin/python3 -m operator_console.runtime_smoke`에 terrain 채널 주입을 추가하고, probe 파일로 배지 상태가
  주입 중 OK, 중단 후 "수신 없음"으로 바뀌는지 확인한다. 이 스모크는 SRT 영상을 공급하지 않으므로(영상 크기 미협상)
  **윤곽이 실제 영상 위에 그려지는지는 증명하지 못한다** — 그건 §7.3-1에서 실기로 본다.
- Jetson: 설치된 엔트리포인트 `ros2 run powertrain_ros terrain_overlay`를 도메인 77 격리로 기동,
  fixture 노드가 합성 depth·CameraInfo를 발행 → 로컬 UDP 수신기가 `OK` 패킷·장애물 1개를 받음 → killpg.
  fixture depth 발행을 멈추면 0.65 s(+ 측정 여유 0.05 s) 안에 `SENSOR_ERROR`/`DEPTH_STALE`, SIGINT 뒤 종료 코드 0.
  (로컬 x86 실측에서 fixture 처리는 수십 ms였으며, 일반 계약(§4.1, 처리시간 항 포함)보다 엄격한 이 스모크 전용 기준이다.)
  음성 대조: 깊이 이중 스케일을 재주입(`--inject-double-scale`)하면 OK 패킷이 없어 FAIL.
- compose 배포 후 컨테이너 상태 + 로그에 traceback·재시작 루프 없음.

### 7.3 실기 E2E (Jetson + 운용 PC) — 정지 상태에서만

1. **표시 경로**: 실제 Gateway·SRT 영상에서 윤곽이 영상 위에 겹치는지, MAIN↔PiP 전환과 L515 RGB/DEPTH/OVERLAY 모드
   전환에서 따라오는지 확인(스크린샷 기록).
2. **자원**: L515·D435i 두 SRT 인코딩이 도는 상태에서 서비스를 켜기 전/후를 비교한다 — `proc_ms` p95 < 80 ms,
   노드 종료 요약 줄의 `overruns 0`(최대 처리시간 ≤ 한 틱, §4.1 stale 계약의 전제),
   노드 RSS 최대값, L515 SRT 화면 FPS, Gateway raw depth 주기, **차체 노드 50 Hz tick 간격과 워치독 이벤트**(`chassis_node` 실행시간 관측),
   `tegrastats` CPU·RAM. 하나라도 악화되면 `publish_hz` 5 또는 RANSAC 횟수 축소 후 재측정한다.
3. **정합 합격 기준**: 상자를 1.0 m·2.0 m에 두고 화면 중앙과 가장자리(좌우 끝 20 % 띠)에서 윤곽 경계와 영상 속 상자 경계의 차이를 잰다.
   **중앙 ≤ 15 px, 가장자리 ≤ 30 px**(1280×720 기준)면 합격 → `alignment_verified=true`. 미달이면 false 유지("정합 미검증" 표시)하고 후속 과제.
   `dist_m` 대 줄자 거리도 기록(±5 cm 목표, 참고용).
4. **영상 대 오버레이 지연**: 카메라 앞에서 물체를 빠르게 넣고 빼며 콘솔 화면을 녹화해, 영상 속 물체 출현과 윤곽 출현의 시간차를 잰다.
   `Δ = 영상 속 출현 − 윤곽 출현`(10회 중앙값, 10 ms 반올림)으로 `--terrain-display-delay-ms` 기본값을 §4.6 규칙대로 정한다.
5. **환경**: 대회장(또는 같은 조도)에서 유효 깊이 화소 비율과 `NO_GROUND` 빈도를 기록. 실차 L515 장착 높이(`camera_height_m`)가
   0.25–0.80 m 안인지, `front_offset_m`에 쓸 depth 원점–차체 앞 거리를 줄자로 측정.
6. **세션 전제**: `powertrain_terrain_overlay` 컨테이너에서 ① boot_id가 호스트와 같음, ② `readlink /proc/self/ns/time`이
   호스트 `readlink /proc/1/ns/time`(호스트에서 sudo)과 같음, ③ `/proc/self/timens_offsets`의 monotonic·boottime offset이 모두 0
   — 셋을 직접 확인한다(시각 비교만으로는 time namespace 미사용을 증명하지 못한다). 하나라도 다르면 배포 중단.
7. **단절**: Gateway 정지 → 콘솔 "센서 오류", 재기동 → 자동 복구. 노드 재시작(`docker restart`) → 콘솔이 새 세션을 받아 복구
   (재시작 소요와 콘솔 표시 복구까지 시간 기록). 운용 PC 네트워크 단절·복구 → 노드 생존.

## 8. 배포

- `docker/docker-compose.integrated.yml`에 `powertrain_terrain_overlay` 서비스(이미지 `powertrain-sw:ros`,
  host network, ipc host, `/workspace:ro`, `/run/powertrain` bind, healthcheck 비활성 — chassis_telemetry와 같은 형태).
  명령: `ros2 run powertrain_ros terrain_overlay --ros-args -p operator_host:=127.0.0.1 -p operator_port:=5009`.
  자원 격리: `cpus: 1.0`, `mem_limit: 512m`, 환경변수 `OMP_NUM_THREADS=1`·`OPENBLAS_NUM_THREADS=1`.
  `restart: unless-stopped`. 세션 파일 `POWERTRAIN_OPERATOR_SESSION_FILE=/run/powertrain/operator-session.json`을 쓴다(§4.5).
  서비스는 **opt-in compose 프로필 `terrain-overlay`**에 둔다. 기본 기동 목록에서 빠지는 것을 구조로 보장하기 위해서다.
  켜는 법: `INTEGRATED_PROFILES=terrain-overlay INTEGRATED_SERVICES=powertrain_terrain_overlay scripts/robot-start`, 또는
  `docker compose … --profile terrain-overlay up -d powertrain_terrain_overlay`.
- 생산 이미지 엔트리포인트는 Gateway를 띄우므로, Jetson 시험 명령은 이 서비스(`bash -lc` 엔트리포인트)로
  `docker compose … --profile terrain-overlay run --rm --no-deps -e ROS_DOMAIN_ID=77 powertrain_terrain_overlay '<cmd>'`처럼 실행한다.
- **롤백**: 모든 변경은 `feat/l515-terrain-overlay`에만 있고 main 병합 전이다. 기능을 버리면 병합하지 않거나 병합 커밋을
  되돌린다. `lidar_dev` 구 파일(`terrain.py`·`packet.py`·`terrain_node.py` 등)은 git 이력에 남는다.
- **`scripts/robot-start` 기본 기동 목록에는 처음에 넣지 않는다.** 수동으로 `docker compose ... up -d powertrain_terrain_overlay`
  로 띄워 §7.3-2(차체 tick·워치독 영향 없음 포함)를 통과한 뒤 별도 커밋으로 추가한다.
- 이미지 변경 없음(numpy·opencv-python-headless는 `Dockerfile.ros`에 있음 — 실제 Jetson 이미지 설치 여부는 배포 때 확인).
- `ros2` 재빌드 필요(새 엔트리포인트).

## 9. 문서

- `docs/network-ip-setup.md`·`docs/integrated-operation.md`의 포트 표에 :5009 추가.
- `lidar_dev/README.md` 신설(오프라인 도구).
- `.claude/CLAUDE.md`·`AGENTS.md` §3 구조에 `terrain_overlay/`·`lidar_dev/` 한 줄씩, §2에 기능 한 줄(검증 후).
- 결과는 `docs/reports/2026-10-xx-l515-terrain-overlay.md`.

## 10. 남은 위험

- **CPU 경쟁**: Orin Nano에서 두 카메라의 x264 소프트웨어 인코딩, 50 Hz 차체 루프와 경쟁한다. §7.3-2로 판정, 기본 기동은 그 뒤.
- **정합**: 왜곡 미보정·공칭 외부파라미터(회전 없음). §7.3-3 기준으로 판정, 미달 시 "정합 미검증" 표시 유지.
- **영상 지연**: 표시 지연 버퍼는 고정 지연 근사다. 네트워크 지터로 SRT 지연이 변하면 회전 중 어긋남이 남는다.
- **조명·재질**: 직사광선·검은/반사 물체에서 깊이 손실 → 윤곽 누락. "참고용" 표기로만 완화한다.
- **지면 모델**: 단일 평면이라 경사 전환부·요철에서 지면이 장애물로, 구덩이는 표시되지 않는다. 5 cm 임계로 험지에서
  윤곽이 과다하면 운용자가 옵션을 끌 수 있다 — 실기 결과로 임계 조정 여부를 판단한다.
- **표시 흔들림**: 추적·평활이 없어 윤곽 모양이 프레임마다 바뀐다. 실기에서 거슬리면 후속으로 다룬다.
- **부팅 간 세션 순서**: 다른 부팅의 세션은 순서를 모르므로 1 s 침묵 규칙에 기댄다. 재부팅 전 패킷이 재부팅 뒤 도착하는 극단적
  지연이면 잠깐 옛 세션으로 돌아갈 수 있다. 복구는 마지막으로 수용한 옛 패킷 뒤 1 s이며, 지연 패킷이 이어지면 더 늦어진다.
- **clock domain 전제**: time namespace를 쓰는 배포에서는 시작값 비교가 무의미하다. 지원하지 않으며 §7.3-6 확인으로 막는다.

## 11. 자체 검토

### ① 직접 재계산 (Claude, 2026-10-17)

- 카메라 위치: 보관 URDF `l515_bottom_screw_frame`(x 0.3334, pitch 0.349 rad) + `l515_link`(0.0075, 0, 0.02115)
  → base_link 기준 x ≈ 0.3334 + 0.0075·cos20° + 0.02115·sin20° = 0.348 m(Codex 0.347679 m). 앞바퀴 앞끝 0.4377 + 0.10356 = 0.541 m. 차 ≈ 0.19 m.
- 경로 반폭: 가운데 바퀴 y 0.3595(`kinematics.py` 269행) + 바퀴 반폭 0.035 = 0.3945, + 여유 0.05 = 0.4445 m
  (WP6 계획 27행 `2×(0.3945+0.05)=0.889`와 일치, 로버 바깥폭 0.789 m).
- 투영 오프셋: `p_c = p_d − (0, −0.024, 0)` → 1 m 중심점은 컬러에서 `fy_c × 0.024` px 아래. Codex의 URDF chain 재계산과 같음.
- depth step: Gateway `image_from_array`는 `step = strides[0]` = 연속 uint16이면 `width×2`(`l515_adapter.py`).
- 직교성: `f ⟂ n`(투영), `r = f × n` ⟂ f, n → 셋이 직교.
- stale 시간 계약: 0.5 + 1/10 + 0.05 = 0.65 s, 5 Hz에서 0.5 + 0.2 + 0.05 = 0.75 s(r6 당시, 처리시간 0 가정 — 처리시간 항을 넣은 r7 값은 ④).
- RANSAC 채점 메모리: 3000 × 100 × 4 B = 1.2 MB. 표본 상한이 없으면 아래 절반 최대 307,200/2 = 153,600점 × 1000 × 4 B ≈ 586 MiB(Codex 계산과 같음).
- 콘솔 depth 나이 반례: 490 ms 패킷 → 0.6 s 한계까지 0.11 s 뒤 숨김(이전 설계는 수신 후 0.5 s, 즉 실제 depth 0.99 s까지 표시).

### ② Codex 적대 검토 (2026-10-17, 1회차) — 판정: 구현 보류(P1 0, P2 7, P3 3)

| # | 지적 | 확인 | 처리 |
|---|---|---|---|
| P2-1 | 만료 평면이 새 평면과 평활돼 섞임(0°→30°에서 바닥점 H +0.3456 m) | 설계 논리상 맞음 | 만료를 피팅 전에 적용 + 평활 제거(§4.2-3, 6), 시험 추가 |
| P2-2 | 캐시 재송신이 HELD 수명을 넘김 | 맞음 | 매 틱 `ground.age_ms` 재평가, `GROUND_EXPIRED`(§4.1-3) |
| P2-3 | 콘솔이 depth 나이를 무시 | 맞음 | `depth_age ≤ 0.6 s` 조건(§4.6) |
| P2-4 | `SourceSequenceGate` 같은 주소 재시작은 seq ≤ 3만 수용 | `udp_source.py` 44–55행으로 확인 | `sender_session` + 전용 게이트(§4.6, §5.2) |
| P2-5 | `approxPolyDP`가 3점 미만을 만들어 패킷 전체 거부 | 1100×8 px → 2점 재현 | `minAreaRect` 대체 + 송신 전 자기 검증(§4.4) |
| P2-6 | 왜곡 생략에 합격 기준이 없음 | 맞음(depth NONE은 근거 있음) | 중앙 15 px·가장자리 30 px 기준 + `alignment_verified`(§4.3, §7.3-3) |
| P2-7 | 전 화소 벡터화 RANSAC은 수백 MiB, 기본 기동 시 자원 간섭 미검증 | 맞음 | 표본 3000·100개 묶음, compose 자원 제한, robot-start는 §7.3-2 통과 뒤(§4.2-5, §8) |
| P3-1 | 상태별 패킷 규칙·수치 범위 미완결, 배지/윤곽 구분 | 맞음 | §5.2 표 확장, §4.6 배지·윤곽 분리 |
| P3-2 | `publish_hz` 1 Hz면 0.5 s stale 보장 불가 | 맞음 | 허용 5–10 Hz, 계약 0.65/0.75 s |
| P3-3 | 스모크가 실제 영상 위 표시를 증명 못함, 음성 대조가 입력 계약과 섞임 | 맞음 | §7.2에 한계 명시·§7.3-1 실기 확인, 버그 재주입 음성 대조와 dtype 시험 분리 |

반박 없음으로 확인된 주장: Gateway QoS·stamp·주기·step, 헤더 stamp 동일성 판정, `send_datagram` 동작(환경변수 유무 구분은 §4.5에 반영),
시스템 python cv2 부재, Dockerfile.ros 의존, URDF 재계산, PiP·스트림 모드 좌표계, `VideoPanel`·`LIVE_PORTS`·`robot-start` 현황.

### ③ 독립 검토 — `agy` Gemini 3.8 Flash (High), ①② 결론 미공유, "적히지 않은 전제" 질문

| 지적 | 처리 |
|---|---|
| L515 직사광선 취약·최소거리·검은/반사 재질 | §1 전제·§7.3-5 실측·§10, 배지 "참고용" |
| SRT 영상과 UDP 오버레이 지연 비대칭 → 회전 중 어긋남 | 콘솔 표시 지연 버퍼 + §7.3-4 측정(§4.6) |
| 단일 평면 가정, 구덩이 미검출, 5 cm 임계 과다 경보 | 비목표·전제 명시, §10, 배지 "구덩이 미표시" |
| HELD 1.5 s·0.7/0.3 평활이 로커-보기 흔들림과 안 맞음 | HELD 0.5 s, 평활 제거(§4.2) |
| depth↔color 회전·왜곡 무시 | §4.3 합격 기준(②-6과 같은 처리) |
| 카메라 기준 거리를 범퍼 여유로 오인 | `front_offset_m`·`distance_origin`·배지 기준 표기(§4.1, §4.6) |
| 순수 numpy CPU 부하가 영상 인코딩을 방해 | §7.3-2 두 인코딩 부하에서 측정, compose 자원 제한 |
| 센서를 스테레오 카메라로 교체 권고 | **채택 안 함** — 센서 분리 결정(L515=파워트레인)·범위 밖 |
| 윤곽 흔들림(추적 없음) | §10 위험으로 기록, 실기 관찰 후 판단 |

### ② Codex 적대 검토 2회차 (2026-10-17, r2 대상) — 판정: 미수렴(P1 0, P2 3, P3 3)

1회차 판정: P2-1·P2-5·P3-2 해소, P2-6·P2-7·P3-3 설계상 해소, P2-2·P2-3·P3-1은 아래 새 지적과 결합, P2-4 미해소.

| # | 지적 | 확인 | 처리(r3) |
|---|---|---|---|
| P2-a | FRESH 재송신이 `age_ms=0` 규칙을 위반해 자기 검증에 걸리고, 초/ms 단위 혼재 | 문서 모순 맞음 | 내부 초 단위·전송 직전 ms 변환, FRESH도 age_ms > 0 허용(§4.1-3, §5.2), 노드 시험 추가 |
| P2-b | 같은 주소 새 세션 수용 후 이전 세션으로 롤백 가능(`A/100→B/0→A/99→B/0` 전부 수용) | 규칙상 맞음 | 은퇴 세션 집합·세션 전환 시 버퍼 비움(§4.6), 반례 시험 추가 |
| P2-c | 표시 지연 `d`가 수신 나이에 더해져 정상 수신을 "수신 없음"으로 만듦, `d` 범위·부호 미정 | 맞음 | 연결 생존(`last_rx`)과 표시 패킷 분리, depth 나이에서 `d` 제외, `d` 0–0.4 s, `Δ` 부호 규칙(§4.6, §7.3-4) |
| P3-a | 자기 검증 실패 시 오류 패킷 구성 규칙 불명확 | 맞음 | 처음부터 새로 구성·재검증, 실패 시 미송신(§4.4) |
| P3-b | 만료 후 피팅 실패 reason이 `GROUND_NOT_FOUND`로 나와 시험 기대(`GROUND_EXPIRED`)와 다름 | 맞음 | `expired` 플래그로 reason 결정(§4.2-3, 6), 시험 기대 통일 |
| P3-c | 전체 후보 SVD(`full_matrices=True`)는 N×N 행렬(N=10,000에서 수백 MiB) | 맞음 | 3×3 공분산 `eigh`(§4.2-5), 메모리 시험 |

평활 제거 자체에서는 새 P1/P2가 없다는 판정.

### ② Codex 적대 검토 3회차 (2026-10-17, r3 대상) — 판정: 미수렴(P1 0, P2 3)

2회차 판정: P2-a·P2-c·P3-a·P3-b·P3-c 해소, P2-b는 기존 반례만 해소.

| # | 지적 | 확인 | 처리(r4) |
|---|---|---|---|
| P2-d | 같은 세션이 다른 주소로 옮기면 자기 세션을 은퇴시키고 sequence가 감소해도 수용 | 규칙상 맞음 | 송신자를 세션으로 식별, 주소 이동 시 sequence 하한 유지(§4.6) |
| P2-e | 은퇴 이력 16개 밖으로 밀린 세션이 현재 송신자를 차단 | 규칙상 맞음 | 이력 집합 폐기 → "현재 세션 1.0 s 침묵 시에만 전환" 규칙. 옛 세션 복귀는 새 세션 침묵이 전제라 불가(§4.6), 축출 반례 시험 |
| P2-f | detector는 처리 시각, 노드는 수신 시각으로 평면 나이를 재 0.5 s 상한을 넘김 | 맞음 | 시간 기준을 depth 수신 시각 하나로 통일(`frame_rx_s`), 새 결과·캐시 모두 송신 직전 같은 만료 검사(§4.1-4, §4.2) |
| P3 | 문구 잔여(SVD, `rx_age`, "반드시 송신" 예외) | 맞음 | 수정 |

표시 경계(F=None, d=0/0.3/0.4, expired 수명)는 추가 P1/P2 없음 판정. d=0.4에서는 숨김이 link 조건(0.5 s)으로 먼저 일어남.

### ② Codex 적대 검토 4회차 (2026-10-17, r4 대상) — 판정: 미수렴(P1 0, P2 1, P3 1)

3회차 판정: P2-d·P2-f 해소. 1 s 전환과 link 0.5 s의 상호작용, 송신 직전 만료와 FRESH 의미는 일관(새 P2 아님).

| # | 지적 | 확인 | 처리(r5) |
|---|---|---|---|
| P2-g | 침묵 1 s 규칙은 수신 공백(유실·단절·수신 스레드 정지)이 생기면 옛 세션 복귀를 허용(0.00 A/100 → 1.10 B/0 → 1.30 B/1 → 2.41 A/99 수용 → B/12 거부). UDP 지연 상한 근거 없음(RFC 8085 §3.3) | 규칙상 맞음 | 세션을 `(boot_id, start_mono_ns)`로 바꿔 같은 부팅 안에서 순서 판정 — 옛 세션은 공백과 무관하게 거부. 부팅 간만 침묵 규칙(§4.1, §4.6, §5.2, §10) |
| P3 | "재시작은 1 s보다 길다"는 미실측 가정, CameraInfo 대기 중에도 오류 heartbeat를 보냄 | 맞음 | 같은 부팅 재시작은 침묵과 무관해져 이 가정이 불필요. 재시작 복구 시간은 §7.3-7(r6 번호)에서 실측·기록 |

### ② Codex 적대 검토 5회차 (2026-10-17, r5 대상) — 판정: 미수렴(P1 0, P2 1, P3 3)

4회차 P2-g 반례는 해소(수신 공백과 무관하게 A/99 거부·B/12 수용). 컨테이너 재생성·패킷 형식(예시 629 B)은 문제 없음.

| # | 지적 | 확인 | 처리(r6) |
|---|---|---|---|
| P2-h | `monotonic_ns()`는 연속 호출에서 같은 값을 줄 수 있어(유일성 보장 없음) 두 노드가 한 세션으로 섞임 | clock_gettime 계약상 맞음 | `sender_instance`(무작위 16 hex) 추가, `(start, instance)` 사전식 순서·동률 시험(§4.1, §4.6, §5.2) |
| P3 | time namespace면 boot_id가 같아도 monotonic 비교 불가 | 맞음(현 compose엔 해당 없음) | 지원 배포 전제 명시, §7.3-6 확인·실패 시 배포 중단 |
| P3 | 예시 FRESH `ground.age_ms=0`이 정의(=depth 나이 35.2)와 다름 | 맞음 | 예시 수정 |
| P3 | 부팅 간 "1 s 뒤 복구"는 마지막 옛 패킷 기준이라 조건부 | 맞음 | 문구 수정(§4.6, §10) |

### ② Codex 적대 검토 6회차 (2026-10-17, r6 대상) — 판정: **수렴**(새 P1/P2 없음)

P2-h 해소(동률 시험 수용·전환·거부·수용 확인), 수신 공백 반례 해소 유지, 패킷 예시 667 B·FRESH 35.2 ms 일치.
남은 P3 두 건(§4.1 "두 값"→"세 값", §7.3-6 time namespace를 시각 비교가 아니라 ns 링크·offset으로 직접 확인)은 r6 최종본에 반영했다.
이 판정은 설계 수렴이며 구현·Jetson·실기 인수는 §7로 따로 판정한다.

### ④ 구현 반영 개정 r7 (2026-10-17) — 계획 검토와 브랜치 리뷰에서 바꾼 점

구현 계획(`docs/plans/2026-10-17-l515-terrain-overlay.md`)은 계획 단계에서 독립 검토 3회(직접 재계산, Codex 5회 수렴, `agy`)를 거쳤고,
구현 브랜치는 Codex 적대 리뷰(`--base 78d14b1`) 2회에서 수렴했다. 그 과정에서 스펙과 다르게 정한 점을 본문에 반영했다.

| 출처 | 바꾼 점 | 본문 |
|---|---|---|
| 시제작 실측 | depth QoS 깊이 5 → KEEP_LAST 1(처리 지연 때 stale 1.2 s) | §4.1 |
| Codex 계획 1회차 P2 | 처리 중 depth 수신 시각이 밀림 → 2-스레드 실행기·콜백 그룹 분리 | §4.1 |
| Codex 계획 1회차 P2 | stale·만료를 처리 전 시각으로 판단 → 처리 뒤 시계 재독, 송신 시각 기준 재판단 | §4.1 |
| Codex 계획 2–4회차 P2 | 처리시간이 stale 계약을 늦춤 → 계약식에 최대 처리시간 항, 초과 기록·종료 요약 줄 | §4.1, §6, §7.3-2 |
| Codex 계획 1회차 P2 | 수신기 값 세 개를 따로 읽어 세션이 섞일 수 있음 → `snapshot()` | §4.6 |
| Codex 계획 1회차 P2 | cairo 기본 텍스트 API에서 한글이 빈 네모 → PangoCairo | §4.6 |
| Codex 계획 1회차 P2 | 대기 화면이 배지를 가림 → 지형 레이어를 대기 화면 위로, 입력 통과 | §4.6 |
| Codex 계획 2회차 P2 | ERROR·EOS·재시작 뒤 옛 윤곽이 남음 → 영상 LIVE가 아니면 즉시 숨김 | §4.6 |
| Codex 계획 1회차 P2 | 타입 구멍·예외 → 파서 엄격화, 수신 스레드 보호 | §4.6 |
| Codex 계획 1회차 P1 | Jetson 시험 명령이 Gateway 엔트리포인트를 띄움 → terrain 서비스로 실행 | §8 |
| 계획 결정 | compose opt-in 프로필 `terrain-overlay`, 롤백 방법 | §8 |
| 계획 결정 | `camera.py`·`runtime.py` 추가(노드 틱 시험을 rclpy 없이) | §3.1 |
| `agy` | 빈 경로 문구 "경로 돌출 장애물 없음", 숨긴 이유 안내문 | §4.6 |
| Codex 브랜치 1회차 P2 | 오래된 depth의 OK 배지가 옛 거리를 현재처럼 보임 → "깊이 지연" 배지 | §4.6, §6 |
| Codex 브랜치 1회차 P2 | `record.py`가 저장 실패를 무시 → 실패 시 종료 코드 1 | §3.2 |
| Codex 스펙 r7 검토 P2 | 배지 대상(`F` 없으면 최신)·null 표기 명시, CameraInfo 미수신과 depth 단절 분리, `view_depth.py`는 2D 대체 없음, 롤백 | §3.2, §4.6, §6, §8 |

재계산(①과 같은 방식):
- stale 상한: 10 Hz에서 0.5 + 0.1 + 0.1 + 0.05 = 0.75 s, 5 Hz에서 0.5 + 0.2 + 0.2 + 0.05 = 0.95 s(처리 ≤ 100 ms면 0.85 s).
- 배지 경계: 490 ms 패킷, d = 0 → 0.11 s 뒤 depth 나이 0.6 s 초과로 "깊이 지연"(윤곽 숨김과 같은 시각, 같은 식). d = 0.3이면 T+0.41 s.
- 실측(로컬 x86 ROS Humble 컨테이너): 설치 엔트리포인트 스모크 `proc_ms` 21.5–24.5, `stale_after_s` 0.435–0.445, 종료 코드 0.
  Jetson 처리시간과 정합·지연은 §7.3에서 잰다 — 미실측.

