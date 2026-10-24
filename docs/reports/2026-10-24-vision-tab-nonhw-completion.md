# 비전 탭 비HW 후속 검토 — 2026-10-24

## 결론

바닥 관측 격자와 확인 전 비활성인 자세 보정 인터페이스를 구현했고, 새 변경 뒤
콘솔 전체 회귀·비전 GTK/SRT/UDP lifecycle·순수 지도/오버레이 회귀를 다시 확인했다.
실제 Gateway/L515, Jetson 배포, 물리 정합, 팀 보정 합의는 완료로 처리하지 않는다.
후속 코드의 AC 600초 재실행은 현재 노트북 전원 미연결로 남아 있다.

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

## 남은 항목

1. 노트북 AC 연결 후 후속 코드로 합성 600초를 다시 실행해 RSS 추세·FPS·0.5초 정지·지도
   약 2 Hz·클럭/온도를 확인하고 이 보고서와 PR을 갱신한다.
2. 광민·도현의 좌표계·부호·영점·장착 회전 확인 파일을 받아 보정과 지도 정지 참고 차이를
   검증한다. 확인 전에는 raw IMU와 차체 자세를 계속 분리한다.
3. 실제 AS5048B 생산자는 없으므로 실제 각도·출처·측정 age는 미수신 상태를 유지한다.
4. Jetson의 갱신 overlay sender와 실제 Gateway/L515로 바닥 경계, 영상/overlay 지연,
   기존 탭 왕복, 중복 연결 없음과 600초 실기 인수를 수행한다.

