# L515 경량 파이프라인 단독 HIL 보고서

> 실행일: 2026-07-11 KST  
> 상태: **COMPLETE — 연결 상태·분리 fail-closed·자동 재연결 HIL 완료**
> 실행 commit: `eac10dd76083df3052de0d981b642e0f2a6e7159`

## 1. 소유권과 격리

- Jetson `~/power-train-sw`는 `main...origin/main [ahead 41]`과 미추적
  `motor_control/vision/tests/`를 유지했다. checkout에 pull/reset/write하지 않았다.
- 정확한 commit을 `git archive`로
  `/tmp/powertrain-l515-eac10dd76083df3052de0d981b642e0f2a6e7159`에 전송했다.
- 고유 이미지 `powertrain-sw:ros-l515-eac10dd76083`
  (`sha256:9e0ae8e4...`, arm64)와 임시 컨테이너만 사용했다. 생산 이미지·컨테이너는
  재태깅하거나 교체하지 않았다.
- 시작 전 카메라 소유 프로세스와 `/dev/video*` 점유자는 없었다. 알 수 없는 프로세스를
  종료하지 않았다. 종료 시 임시 HIL 컨테이너만 삭제했고 이미지·snapshot·로그는 보존했다.

## 2. SDK·장치 게이트

| 항목 | 결과 |
|---|---|
| pyrealsense2 | `2.50.0`, ARM64 Python 3.10 source build PASS |
| L515 | `Intel RealSense L515`, SDK serial `f0271544` |
| 정본 serial 매칭 | `00000000F0271544`와 대소문자·선행 0 정규화 후 유일 매치 |
| firmware | `01.05.08.01` — 최소 `1.5.8.1` PASS |
| USB | SDK `3.2`, host 5000 Mbps |
| profile | color BGR8 640×480×30, depth Z16 640×480×30 |
| IMU profile | accel/gyro 각 100·200·400 Hz 지원 |

SDK가 serial을 축약·소문자로 반환해 기존 완전일치 preflight가 장치를 거부했다. strict
canonical match를 추가했으며 D435i `250222071245` fallback과 중복 매치는 계속 거부한다.

## 3. HIL 중 발견·수정

1. `frame.get_data()`는 SDK 2.50.0에서 NumPy 배열이 아니라 `BufData`였다. adapter 입구에서
   `np.asanyarray`로 변환한다. 실물에서 color `(480,640,3) uint8`, depth `(480,640)
   uint16`, 둘 다 C-contiguous를 확인했다.
2. `Image.data = bytes`는 640×480 color 한 장당 평균 0.192 s가 걸려 약 3 Hz·CPU 99%로
   제한했다. `array.array('B', ...)`는 setter 평균 약 1.1 µs였고 30 Hz를 회복했다.
3. 연결 세션 offset은 공유하되 직전 device timestamp를 stream별로 보관하도록 수정했다.
   color/depth 교차 timestamp를 재연결로 오판하던 4.8–6.4 ms 역행이 사라졌다.

관련 자동시험 51개가 통과했다.

## 4. 60초 연결 상태 계측

5초 subscriber warm-up 뒤 정확히 60초를 측정했다. 최종 동시 자원 계측 run 결과는 다음과
같다.

| 토픽 | count | mean Hz | 완전 5초 창 | max interval | stamp |
|---|---:|---:|---:|---:|---|
| color image | 1778 | 29.633 | 27.6–30.0 Hz | 134.9 ms | 단조 증가, 비증가 0 |
| depth image | 1795 | 29.916 | 29.8–30.2 Hz | 70.3 ms | 단조 증가, 비증가 0 |
| accel | 1802 | 30.033 | 30.0–30.2 Hz | 38.7 ms | 단조 증가, 비증가 0 |
| gyro | 1802 | 30.033 | 29.8–30.2 Hz | 38.9 ms | 단조 증가, 비증가 0 |

color 첫 경계 창은 27.6 Hz였으나 이후 11개 창은 29.4–30.0 Hz였다. 별도 안정화 run은
color/depth 평균 29.97 Hz, 최소 완전 창 29.2 Hz였다. 평균 기준은 통과했고 지속 성능은
안정적이지만, 위 동시 자원 계측 run의 첫 경계 창은 문언상 28 Hz 기준보다 0.4 Hz 낮아
그 사실을 숨기지 않는다.

- 자원 33 samples: CPU 평균 113.17%, 최대 162.14%; RAM 평균 153.83 MiB, 최대 156 MiB.
- kernel USB error/reset/disconnect/timeout delta: 0.
- 발행 목록은 color/depth Image+CameraInfo, accel, gyro뿐이다.
  `PointCloud2`와 D435i 토픽은 없었다.
- 최종 node log에 예외가 없었다.

## 5. 물리 분리·재연결 — PASS

사용자 승인과 현장 조작으로 L515만 분리·재연결했다. firmware update와 물리 reset은 하지
않았다.

- 분리 검증 `18:49:23–18:49:41 KST`: host에는 D435i `8086:0b3a`만 남고 L515
  `8086:0b64`는 없었다. SDK serial 목록도 `['250222071245']`뿐이었다.
- 기존 launch/node PID `32482/32508`은 살아서 지정 L515를 재시도했다. color topic을 8초
  관찰해 메시지 0건을 확인했고 D435i data fallback은 없었다.
- 같은 포트 재연결 뒤 L515가 Bus 002 Device 006으로 돌아왔다. 컨테이너나 노드를 재시작하지
  않았고 PID `32482/32508`이 그대로 유지됐다.
- 12.029초 continuity: color/color_info 360건(29.927 Hz), depth/depth_info 359/360건
  (29.844/29.927 Hz), accel/gyro 각 359건(29.844 Hz). 여섯 토픽의 첫 메시지는 subscriber
  시작 후 45.7–71.6 ms 안에 관찰됐고 stamp는 모두 단조 증가했다.
- reconnect 구간 kernel USB error/reset/disconnect/timeout delta 0, node exception 0.
- 스트리밍 중 별도 SDK context probe는 장치가 기존 node에 점유돼 `failed to set power state`를
  반환했다. 이는 concurrent-open 제한이며, host exact PID·동일 node PID·여섯 토픽 연속성으로
  자동복구와 D435 미선택을 판정했다.

## 6. 증거 위치

Jetson snapshot의 `task-6-image-build.log`, `task-6-enumeration.log`,
`task-6-metrics.log`, `task-6-resource.log`, `task-6-node-preserved.log`,
`task-6-kernel.log`, `task-6-usb-error-count.log`, `task-6-topics-before.log`,
`task-6-topics-after.log`, `task-6-disconnect-*`, `task-6-reconnect-*`를 보존했다.
