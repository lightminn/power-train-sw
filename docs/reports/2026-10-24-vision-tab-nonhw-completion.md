# 비전 탭 비HW 후속 검토 — 2026-10-24

## 결론

바닥 관측 격자와 확인 전 비활성인 자세 보정 인터페이스를 구현했고, 새 변경 뒤
콘솔 전체 회귀·비전 GTK/SRT/UDP lifecycle·순수 지도/오버레이 회귀를 다시 확인했다.
실제 Gateway/L515, Jetson 배포, 물리 정합, 팀 보정 합의는 완료로 처리하지 않는다.
후속 코드의 배터리 600초는 완료했다. AC 600초 재실행은 현재 노트북 전원 미연결로 남아 있다.

## 구현 범위

- `:5009` v1 선택 `floor_mask`: 128×72, 10×10 color pixel bin, zlib+base64 MSB bitset.
  유효 투영 표본이 둘 이상이고 모두 detector floor인 칸만 표시한다. 미관측/혼합 칸은
  투명하며 통과 가능·안전 영역이 아니다. 1200 byte 제한에서는 이 선택 필드를 먼저
  생략해 기존 장애물/거리 정보를 보존한다.
- 기존 TerrainCanvas와 receiver의 같은 delay-aligned snapshot에서 바닥을 그린다.
  영상/깊이/link stale, 영상 크기 불일치, field 누락 시 그리지 않는다. 별도 소켓,
  카메라, decoder는 추가하지 않았다.
- ROS overlay adapter에 `publish_floor_mask` bool 파라미터를 추가했다(기본 true).
  실제 Jetson 배포·노드 실행은 하지 않았다.
- 자세 보정은 `sensor_to_body_xyzw`와 raw roll/pitch 부호가 확인된 JSON만 활성화한다.
  기본/pending은 `confirmed=false`이고 19.5°를 코드에서 차감하지 않는다.
  좌표 확인이 명시되고 현재 정지 gravity와 telemetry 수신 age가 각각 0.2초 이내일 때만
  센서 시각 미확인인 참고 차이를 표시한다.
- 성능 하니스의 timeline/환경 보관을 제한하고 전체 원시 표본은 JSONL로 스트리밍한다.
  실제 console observer는 창 mapped/visible/backend 변화와 0.5초 초과 heartbeat/draw
  구간의 sink age/frame counter를 최근 64건만 보관한다.

## 실행 결과

| 검사 | 결과 | 범위 |
|---|---:|---|
| 전체 `operator_console` | 887 passed / 기존 10 failed / 3 skipped | 직전 871 pass 대비 신규 16 pass, 새 실패 0 |
| `runtime_smoke --vision-only` | PASS | missing/live/encoder age/stale/recovery/floor hide+restore/roundtrip/shutdown |
| `terrain_overlay terrain_map lidar_dev/tests` | 543 passed / 4 skipped | sender core, packet bound/error/legacy, 지도 회귀 |
| ROS terrain overlay node | 미실행 | 노트북에 `rclpy` 없음; Jetson/ROS 환경 필요 |
| ROS floor parameter 정적 계약 | 1 passed | 기본 true 선언과 RuntimeConfig 전달을 AST로 확인 |
| 고장 주입 9종 | 기대 FAIL | 각 누락·조작·stale·복구·숨김 렌더·중복 decoder·종료·GTK stall 검출 |

전체 콘솔 실패 10개는 이전과 같은 로봇팔 경계/계약 9개와 arm safe-disabled
runtime smoke 1개다. 비전 요구를 위해 타 팀 계약을 완화하지 않았다.

### 95초 메모리 추적

AC가 연결돼 있던 X11 합성 실행에서 비전 95.03초 동안 L515 sink 14.995 FPS,
지도 pair 1.989 Hz, 최대 draw 192.1 ms, GTK heartbeat 177.7 ms, worker/callback 오류 0을
기록했다. RSS는 비전 첫 표본 351,336 KiB에서 마지막 355,560 KiB로 4,224 KiB 증가했다.
tracemalloc current는 warm-up 후 약 8.0 MiB에서 30/60/90초에 8.02/7.63/7.66 MiB로
유지됐다. 이는 95초 범위에서 Python heap의 지속 선형 증가 증거는 아니지만,
제품 장시간 무누수를 입증하는 600초 결과도 아니다.

원시 자료는 `2026-10-24-vision-tab-nonhw-evidence/`에 있다. 600초 전/후 비교를 시작했으나
전원 센서가 `ADP1=0`, USB power=0을 보고해 `--require-ac` 조건을 만족하지 못했고 즉시
중단했다. 이 실행은 `rejected-no-ac-*`로 보존하며 성능 결과로 사용하지 않는다.

### 배터리 600초 가능한 범위 시험

전원을 연결할 수 없는 현재 환경에서 `--require-ac` 없이 같은 후속 코드를 600.006초
연속 실행했다. 모든 626개 환경 표본에서 AC는 off였고 CPU 클럭 범위는 0.399–2.800 GHz,
최고 온도는 45.0 ℃였다. 따라서 아래 값은 AC 성능 인수가 아니라 배터리 참고 결과다.

- L515 sink 15.000 FPS, 최대 sink 간격 140.1 ms, 최대 video draw 간격 118.6 ms.
- GTK heartbeat 최대 68.1 ms, 0.5초 초과 stall event 0.
- Top/Oblique 개별 1.996 Hz, 각각 1203 raster, 최대 반영 간격 603.4 ms.
- 지도 worker 1198회, 평균 render 18.76 ms, callback/worker 오류 0.
- 비전 첫 RSS 343,176 KiB → 마지막 347,944 KiB(+4,768 KiB), 첫/마지막 60초
  평균 차이 +3,980 KiB. 종료 후 일반 `gc.collect()`에는 변화가 없었고 개발용
  `malloc_trim(0)` 뒤 348,284 → 315,148 KiB로 감소해 native allocator 보유분이 컸다.
- 비전 뒤 실시간 화면에서 worker render 0, L515 15.000 FPS를 유지했다.

RSS는 참조가 계속 쌓이는 Python 자료구조 없이도 완만히 증가했으며 600초 동안 중단이나
처리 악화는 없었다. 한 번의 allocator trim 결과만으로 누수 없음이나 장기 plateau를
단정하지 않는다. 제품에는 강제 GC/trim을 추가하지 않았다.

## 남은 항목

1. 노트북 AC 연결 후 후속 코드로 합성 600초를 다시 실행해 배터리 결과와 분리된
   RSS 추세·FPS·0.5초 정지·지도 약 2 Hz·클럭/온도를 확인한다.
2. 광민·도현의 좌표계·부호·영점·장착 회전 확인 파일을 받아 보정과 지도 정지 참고 차이를
   검증한다. 확인 전에는 raw IMU와 차체 자세를 계속 분리한다.
3. 실제 AS5048B 생산자는 없으므로 실제 각도·출처·측정 age는 미수신 상태를 유지한다.
4. Jetson의 갱신 overlay sender와 실제 Gateway/L515로 바닥 경계, 영상/overlay 지연,
   기존 탭 왕복, 중복 연결 없음과 600초 실기 인수를 수행한다.

로컬 자료 재대조 결과, 현재 보관 CAD/시뮬/콘솔 URDF의
`base_link→l515_bottom_screw_frame` pitch는 0.349065850399 rad(20°)이고,
`imu_tilt_node`가 발행하는 `base_link→l515_link` 회전은 identity다. 반면 노트북
`live_overlay.py`/`tmap_live.py`의 가속도 미수신 fallback은 15°이며, 19.5°는 이번 요청과
합성 fixture 값이다. 저장소에 확인자·시각·실측 방법이 있는 생산 calibration JSON은 없다.
따라서 셋 중 하나를 자동 선택하지 않은 것이 현재 확인 가능한 유일한 안전한 결론이다.

