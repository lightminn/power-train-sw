# 비전 탭 5단계 — 렌더링 부하 제한과 성능 계측

브랜치 `agent/vision-tab-20261023`, PR 대상 `feat/l515-terrain-map`.
이 단계는 콘솔 실행 설정·대기열·한글 캐시와 로컬 합성 계측을 다룬다.
**실기 10분 성능 인수와 카메라부터 화면까지 지연은 미검증**이다.

## 변경 범위

- 패키지 진입 시 NumPy보다 먼저 `OPENBLAS_NUM_THREADS=1`을 설정한다.
  기존 셸 값이 8처럼 커도 덮어쓴다. 직접 app 실행·통합 `__main__`·모듈 실행에 공통 적용한다.
  이미 NumPy/BLAS를 초기화한 다른 프로세스에서 콘솔을 임베드하는 경우는 이 설정의 보장 범위 밖이다.
- 서비스 파일에 같은 환경값을 명시하고 `jetson_gui_up.sh`의 노트북 실행 안내에도 넣었다.
  서비스 설치·재시작·로봇 배포는 하지 않았다. 실제 BLAS 스레드는 `threadpoolctl` 검사로 확인한다.
  `threadpoolctl 3.7.0`은 개발 검사용으로 시스템 Python의 사용자 영역에 설치했다.
- 기존 두 영상 파이프라인 각각에 디코딩 후 raw 프레임 한 개만 보관하는
  `video_latest` queue를 둔다. `leaky=downstream`, byte/time 한도는 끄고 frame 한도로 제한한다.
  느린 변환/출력에서 오래된 **디코딩 프레임**을 버린다. H.264 참조 프레임을 무작위로 버리지 않는다.
  `sync=false`, decoder `max-threads=1`은 유지한다. 수신 연결·디코더를 새로 만들지 않는다.
- 스트리밍 콜백의 FPS·해상도 `idle_add`를 없애고 기존 GTK health 타이머(200 ms)가
  최신 스칼라 값만 표시한다. UI가 바빠도 알림 콜백을 쌓지 않는다.
  최초 협상 해상도의 오버레이 전달도 이 타이머가 처리하므로 최대 한 주기 늦을 수 있다.
- 카메라 지형 배지·거리, D435 한글 탐지 글자에 widget별 Pango layout LRU 128개 캐시를 추가했다.
  font/text/width/변환/표면 배율을 구분하고 style·scale 변경·destroy 때 비운다.
  새 지도 화면의 기존 64개 캐시도 유지한다. GTK label은 기존 label 재사용을 유지한다.
- 지도·자세는 기존 공유 worker/receiver를 계속 사용한다. 지도 worker의 상태 10 Hz,
  이미지 쌍 약 2 Hz, 최신 결과 한 개, 비표시 중 작업 중지 동작은 기존 3·4단계 구현을 검증했다.
  `terrain_map/`, `terrain.py`, `terrain_map.py`, `terrain_map_panel.py`, `mobility_view.py`는 수정하지 않았다.

## 계측 방법과 한계

개발 도구 `operator_console/tests/vision_benchmark.py`는 **실제 GTK 콘솔과 SRT 디코더**를 실행한다.
입력은 별도 자식 프로세스에서만 만드는 1280×720@15 영상 두 개, 지도 3 tile/약 12.5 Hz,
동일 주기의 자세·바퀴·지형 오버레이다. 로컬 합성 입력이며 RealSense나 Gateway를 열지 않는다.
모터/ops 토큰도 없다. CPU/RSS는 측정 부모 프로세스만이며 송신·H264 인코더 자식은 제외한다.
기준선은 `930da9f`의 분리된 임시 worktree에 같은 도구만 복사해서 실행한다.

실시간 화면 → 비전 → 실시간 화면 순으로 각각 3초 안정화 후 20초씩 계측한다.
BLAS 제어를 확인하려고 두 실행 모두 셸에서 `OPENBLAS_NUM_THREADS=8`을 주고,
수정 버전만 콘솔 내부에서 1로 제한한다. 이는 미설정 셸의 기본값 비교가 아니라 **8→1 제한 시험**이다.

- 프로세스 CPU 시간/벽시계 시간, 현재 RSS와 프로세스 최대 RSS를 기록한다. CPU 100%는 논리 코어 하나다.
- sink pad 도착 FPS·최대 간격, GTK video widget draw 빈도·최대 간격을 각각 기록한다.
  오버레이 때문에 같은 영상을 재그릴 수 있어 draw 빈도는 고유 영상 FPS와 다르다.
- 20 ms GTK heartbeat의 실제 최대 간격으로 메인 루프 지연을 본다.
- 지도 이미지 쌍의 화면 반영률, worker 계산 횟수, 렌더 시간, 숨김 중 계산 횟수를 본다.
- sink의 running-time과 buffer PTS 차이는 내부 지연 지표다.
  송신부 capture timestamp와 화면 광학 측정이 없으므로 **camera-to-screen 지연이 아니다**.
- 전원 online, CPU별 scaling_cur_freq, thermal_zone 온도를 1초 주기로 함께 기록한다.
  초기 계측은 `ADP1=0` 배터리 상태였으며, 이후 전원 전환이 섞인 기준선도 있었다.
  이 결과는 별도 보관하고, 최종 비교는 `--require-ac`로 전원 연결을 확인하며 다시 실행했다.
  클럭 고정은 하지 않았으므로 CPU/RSS의 작은 차이를 개선율로 확정하지 않는다. AC 구간에서는 프로그램 시험 외 부하를 의도적으로 겹치지 않았다.
  별도 thermal_throttle 누계도 수집하되, 0이라는 사실만으로 모든 전력/클럭 제한을 배제하지 않는다.
- 짧고 작은 지도 시험이므로 대형 지도·장시간 메모리 안정성을 보장하지 않는다.

```bash
# 노트북 / 저장소 루트 — 개발용 합성 시험, 카메라 불필요
cd /home/kimseon/power-train-sw-vision
OPENBLAS_NUM_THREADS=8 xvfb-run -a /usr/bin/python3 \
  -m operator_console.tests.vision_benchmark \
  --require-ac --seconds 20 --output /tmp/vision-performance.json
```

이 도구는 pytest fixture 생성 함수를 재사용하므로 개발 의존성이 필요하다.
부하 시험은 다른 테스트와 동시에 돌리지 않는다. JSON과 로그는 아래 evidence에 보관한다.

## 검사 중 발견 사항

첫 수정본은 `_published_video_size` 초기화가 잘못된 클래스에 들어가 GTK 콜백 오류가 발생했다.
VideoPanel에 초기화하도록 수정했고 해당 측정은 폐기했다(`rejected-initial-after.log`).
최종 도구는 PyGObject가 삼키는 콜백 예외도 수집해 실패로 처리한다.

이후 한 실행에서 sink는 15 FPS이고 GTK heartbeat는 약 0.11초였지만 video draw 간격이
**8.20초**, 지도 반영률이 0.94 Hz였다(`draw-stall-after.json/log`).
이것은 정상 측정으로 채택하지 않았으며 수신 FPS만으로 표시 성능을 통과시키지 않는다.
다음 짧은 재검사와 20초 구간 재검사에서는 재현되지 않았다.
**그리기 정지 1회의 원인은 아직 확정하지 못했으며, 해결됐다고 주장하지 않는다.**
실기/장시간 시험에서 draw·map 반영 지표를 포함해 재검증해야 한다.
정지 당시 heartbeat는 계속 동작했으므로 지도 계산에 의한 전체 GTK 정지라고 단정하지 않는다.
현재 짧은 정상 구간에서 계산 worker를 별도 프로세스로 옮길 근거는 부족해 스레드 구조를 유지했다.

## 결과·검증 기록

AC 연결 상태에서 세 구간 각각 20초를 실행했다. 기준선·수정본의 계측 도구 SHA256은 같다.
전원·클럭/온도 표본은 각각 58개, 58개다. CPU는 한 논리 코어를 100%로 계산한다.

| 구간 | CPU 전→후 | L515 sink FPS 전→후 | 지도 쌍 Hz 전→후 | RSS 최대 전→후 |
|---|---:|---:|---:|---:|
| 실시간 화면 | 114.17→113.59% | 14.98→14.94 | 0.00→0.00 | 249.4→251.7 MiB |
| 비전 | 93.87→93.25% | 14.99→15.00 | 2.00→2.00 | 303.4→303.7 MiB |
| 실시간 화면 복귀 | 113.14→113.31% | 14.99→15.01 | 0.00→0.00 | 305.2→305.4 MiB |

비전 구간의 최대 GTK heartbeat 간격은 55→52 ms, video draw 간격은 115→115 ms,
지도 이미지 쌍 생성 시간 관측 평균은 10.30→11.49 ms였다.
sink 내부 최대 지연 지표는 39.8→40.0 ms였다. 이는 실제 영상 end-to-end 지연이 아니다.
두 정상 실행에서 숫자가 비슷해 **CPU가 유의하게 개선됐다고 주장하지 않는다**.
이번 최적화의 검증된 효과는 BLAS/글자 캐시/decoded queue/알림 보관량의 상한이다.
숨긴 동안 비전 지도 계산·화면 반영은 0회이며 수신은 계속된다.

AC 구간의 전체 코어 보고 클럭은 전 0.399~4.800 GHz, 후 0.399~4.800 GHz였다.
x86_pkg_temp는 전 49~72°C, 후 44~75°C였다. idle 코어의 낮은 클럭을 처리 병목으로 해석하지 않는다.
별도 조회한 core/package thermal_throttle 누계는 두 시점 모두 전 코어 0이었다.
이 증거에서는 온도 스로틀 이벤트가 잡히지 않았지만 전력 제한과 DVFS를 배제하지는 않는다.

- `test_text_layout`·영상 대기열/BLAS·한글/terrain GTK 집중 검사: 15 passed.
- 650 ms GTK 정지 주입 → `AssertionError: GTK stall`, exit 1.
- queue leaky 제거 → 느린 sink 시험 FAIL, exit 1.
- NumPy import 직전 BLAS 제한 해제(8개) → 제한 검사 FAIL, exit 1.
- 글자 캐시 제한을 1,000,000개로 변경 → 상한 검사 FAIL, exit 1.
- 실행 안내/통합 기동 스크립트 검사: 33 passed / 3 failed / 1 skipped.
  실패 3개는 수정 전 `930da9f`에서도 같은 ID·원인으로 재현했다.
  두 개는 기존 colcon 목록에 `dynamixel_control`이 들어간 계약 차이,
  한 개는 시스템 Python의 `pygame` 부재가 legacy 서비스 거부 검사보다 먼저 발생한 환경 차이다.
  변경한 `jetson_gui_up` 검사에는 실패가 없다.
- 필수 `operator_console.runtime_smoke`: 기존 arm safe-disabled 표시 FAIL, terrain_map OK.
  이번 단계에서 게이트를 약화하거나 실패를 숨기지 않았다. 비전 검사의 runtime_smoke 통합은 6단계다.
- 전체 콘솔: **845 passed / 10 failed / 3 skipped**, 183.91초.
  4단계의 실패 ID 10개와 동일하고 새 실패는 없다(`test-comparison.json`).
  명령: `PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" xvfb-run -a /usr/bin/python3 -m pytest operator_console -q`.
- 비교 계측 후 heartbeat/sink/draw의 구간 끝 정지도 검출하도록 보강했다.
  콜백이 한 번도 없거나 시작/끝에서 멈춘 경우를 0초 지연으로 오인하지 않는다.
  추가 경계 검사 1 passed, 이를 0으로 보고하는 고장 주입은 FAIL/exit 1이었다.
  보강한 도구도 합성 5초×3구간(AC)에서 별도 실행해 통과했다(`boundary-verified`).
  이 마지막 경계 검사는 위 전체 회귀 수집 후 추가돼 별도 결과로 기록한다.
  AC 전후 비교에는 동일한 이전 도구를 사용했으며, 계측본과 최종 도구 hash를 각각 남겼다.

[원시 계측·로그·고장 주입 자료](2026-10-24-vision-tab-step5-evidence/)에 보관했다.
`ac-baseline.json`/`ac-after.json`이 최종 AC 비교이고, `before`·`baseline`·`after`·`repeat`는
배터리 조건의 별도 시험이다. `power-transition-baseline`은 AC 전환이 섞여 비교에서 제외했다.
`draw-stall-after`는 실제 관측한 미해결 실패이며 `stall-negative`는 의도한 고장 주입이다.

## 다음 작업

6단계는 `runtime_smoke`의 비전·자세·지도 상태 전이, 탭 왕복, 종료, 음성 대조를 확장한다.
7단계의 실제 Gateway/L515 600초 시험에서는 AC 연결과 클럭/온도 기록,
기존 화면과 비전의 표시 간격·FPS·지도 반영률·CPU/RSS 비교와 동작 영상을 남겨야 한다.
현재 Jetson 별칭은 이름 해석에 실패했으며 장착 보정·AS5048B 송신도 여전히 미확인이다.
