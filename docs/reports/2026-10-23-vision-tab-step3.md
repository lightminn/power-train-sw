# 비전 탭 3단계 — 두 지도·기준값·작업 스레드 연결

작업 브랜치 `agent/vision-tab-20261023`, 기준/PR 대상 `feat/l515-terrain-map`.
노트북 Graphite GUI 위에 Top·Oblique 지도까지 연결했다. 차체 자세 영역은 다음 단계다.
원래 노트북 작업 폴더, Gateway, 로봇 서비스·센서는 변경/기동하지 않았다.
Jetson은 이번에도 `jetson` 이름 해석 실패로 상태 확인이 불가능했다.

## 구현과 재사용

- 기존 `self._map_receiver` :5010 하나의 snapshot을 사용한다.
  추가 지도 소켓·카메라 연결·디코더·지도 생성 런타임을 만들지 않는다.
- Top은 광민 `MapRenderer`·기존 클래스/높이 음영을 사용한다.
  worker가 전용 캐시를 소유하고, 완성된 RGBA를 회전·배치해 로버 방향이 위가 되게 한다.
  작은 화면에서도 앞쪽 약 2.5 m와 로버가 보이도록 기본 시야를 배치한다.
  물리 눈금, 로버 화살표, 송신 기준값을 표시한다.
- Oblique는 `lidar_dev.live_map`의 tmap_local_grid, classify_heights, filter_hazards,
  tidy_low, cell_colors, oblique_image를 호출한다.
  확인되지 않은 카메라 장착 위치를 추정하지 않고 로버 base 원점을 사용한다.
  화면에도 '로버 기준'을 표시한다. 자세가 없으면 `(0,0,0)` 대체 없이 투영을 보류한다.
- Top의 송신 클래스와 Oblique의 JSON 표시 정책은 별도 범례다.
  JSON 정책이 바뀌어도 Gateway 클래스는 바뀌지 않는다.
  sender FREE 코드로 정책상 위험 셀을 취소하지 않는다.
  기존 필터가 외톨이 위험 셀을 통과로 낮추는 경우, 비전에서는 그 셀을 **미확인**으로 표시한다.
- `lidar_dev/live_map.py`, `obstacles_map.py`의 import 문만 패키지·스크립트 양쪽을 지원하게 바꿨다.
  기존 함수 알고리즘은 그대로다. `python -m lidar_dev.obstacles_map --help`와
  `python lidar_dev/obstacles_map.py --help` 모두 실행했다.
- **수정 없음**: `terrain_map/`, `operator_console/terrain.py`, `terrain_map.py`,
  `terrain_map_panel.py`, `mobility_view.py`. 노트북 GUI 기준도 유지한다.

## 화면 조작·유효성

두 지도 위의 +/−와 휠은 같은 줌을 사용한다. 휠 한 단계는 기존과 같이 ×1.2다.
Top을 끌면 따라가기가 꺼지고 선택 위치를 탐색한다.
따라가기 중 Top은 로버 방향이 위이며, 해제하면 북쪽이 위다.
Oblique는 따라가기 중 로버 전방, 해제하면 선택 위치와 마지막 시점 방향을 기준으로 본다.
로버가 화면 범위 안에 있을 때 실제 위치/방향 화살표를 표시한다.

상태 패킷 수신 후 0.5 s를 넘으면 수신 끊김을 표시한다.
Top의 관측 기록과 마지막 화살표는 회색/과거 상태로 남긴다.
Oblique의 현재 자세 투영은 즉시 숨기고 새 유효 상태를 기다린다.
새 패킷의 상태가 NO_POSE 등일 때도 수신 LIVE와 투영 가능을 구분한다.
처음에는 미수신, 재진입 직후에는 수신 확인 중, 처리 예외는 오류로 표시한다.
원시 처리 오류를 삼켜 정상 상태로 바꾸지 않는다.

각 지도는 관측 기록이며 실시간 장애물 감지나 주행 허가가 아니라는 문구를 화면에 표시한다.
Top 범례는 기존 note_and_legend 용어를 사용하고, 전체 고정 주석도 툴팁에서 볼 수 있다.
영상의 지형 오버레이는 기존 :5009와 같은 인스턴스다. 실제 바닥 마스크가 없어
바닥 영역을 칠할 수 없다는 한계는 계속 남아 있다.

## 계산·표시 분리

`vision_render.py`의 한 작업 스레드가 두 지도를 한 쌍으로 0.5 s 주기로 생성한다.
두 화면 각각 약 2 Hz이며 GTK에는 계산이나 이미지 변환을 맡기지 않는다.
worker가 premultiplied BGRA까지 준비하고 GTK는 cairo surface로 감싸 그리기만 한다.
소켓 수신과 영상 표시 경로는 독립적이다.

설정/결과는 각각 최신 한 개만 보관한다. 탭 숨김 시 worker와 UI poll 타이머를 멈추고,
수신기는 유지한다. 숨김·크기·시점 변경 전 작업이 뒤늦게 끝나도 세대가 다르면 게시하지 않는다.
다시 열 때 과거 화면을 LIVE로 재사용하지 않는다. 종료 시 worker를 깨워 종료한다.

Top 내부 렌더 예산은 최대 500,000 픽셀, 완성 이미지 최대 900×480(비율 유지)다.
Oblique는 먼저 필요한 타일만 선택한 후 원본 로컬 격자 함수를 호출한다.
전방·횡방향 각각 최대 160 셀로 제한하고 실제 표시 범위를 명시한다.
고립된 먼 타일 때문에 거대한 조밀 배열을 만들지 않는 검사도 포함했다.
한글 draw layout은 캔버스마다 최대 64개 캐시로 제한한다.

이는 3단계 구현 조건이다. BLAS 실행 경로 통일, 장시간 GIL/CPU/메모리 비교,
프로세스 분리 필요성 판정은 5·7단계에서 계속 확인한다.

## 기준값 교체·실행

기본은 저장소 `lidar_dev/config/obstacle_criteria_v0.2.json`, A_no_stop 프로필이다.
B_stop20 또는 파일의 다른 유효 프로필을 선택할 수 있다.
v0.2에 trench_depth_m이 없으면 원본 함수처럼 step_up_m을 보수 대체한다.
높이는 원본과 같이 1 cm 측정 여유를 더해 분류한다.
표시 버전·프로필·SHA256 앞 8자는 상단, 파일 경로·전체 해시는 툴팁에 나온다.

```bash
# 운용 PC
cd /home/kimseon/power-train-sw-vision
/usr/bin/python3 -m operator_console.app --host <Gateway-IP> --page vision \
  --l515-criteria /path/to/obstacle_criteria.json --l515-criteria-profile A_no_stop

# 같은 옵션을 통합 진입점에서도 지원
/usr/bin/python3 -m operator_console --config ~/.config/powertrain/operator.json --page vision \
  --l515-criteria /path/to/obstacle_criteria.json
```

기존 페어링 JSON에 다음 선택 키를 추가할 수도 있다(기존 필수 항목은 보존).

```json
{
  "l515_criteria": "/path/to/obstacle_criteria.json",
  "l515_criteria_profile": "A_no_stop"
}
```

경로/프로필 각각 CLI > JSON > 환경 변수 L515_CRITERIA/L515_CRITERIA_PROFILE > 기본값 순이다.
JSON 상대 경로는 설정 파일 폴더 기준, CLI/환경 상대 경로는 실행 폴더 기준이다.
프로필을 지정하지 않으면 해당 파일의 기본 프로필을 사용한다.
10/26 파일이 오면 파일 교체 또는 경로/프로필 설정 변경 후 **콘솔을 재시작**한다.
자동 hot reload는 하지 않는다. 파일 교체로 버전/해시가 달라지는 검사도 통과했다.
잘못된 파일·프로필·비유한/잘못된 경계값은 수신기를 만들기 전에 명시적으로 거부한다.
잘못된 CLI 파일 경로로 실제 진입점을 실행해 usage error도 확인했다.

## 검증

새 핵심 검사 10건은 기준값 우선순위·파일 교체·v0.2 호환·실제 래스터·누락 자세·stale,
원거리 타일 예산·위험 필터·worker 스레드/주기/숨김/세대/종료를 확인한다.
실제 GTK/SRT + 통합 설정 전달 경로의 집중 검사 34건도 통과했다(중복 실행 포함).

새 E2E는 로컬 합성 H.264/SRT 영상과 실제 UDP :지도·오버레이·차대 수신 경로를 사용한다.
서로 다른 3개 타일에 돌출·꺼짐을 넣고, 다음을 확인했다.

- 두 지도 생성·로버 화살표·기존 영상/윤곽 동시 표시, 각각 약 2 Hz 갱신.
- 수신 중단 → 끊김/Oblique 숨김 → 재수신 복구.
- 줌·끌기·따라가기, 탭 숨김 동안 렌더 횟수 불변/수신 시퀀스 증가, 복귀 후 새 결과.
- 짧은 검사 중 카메라 프레임 수신과 GTK heartbeat 지속.

worker 활성화를 강제로 막은 음성 대조는 LIVE 지도 검사에서 예상 FAIL했다.
첫 별도 음성 대조 시도는 오류 주입 전 기존 GStreamer _on_realize의 PLAYING 진입에서
native segfault가 나 유효한 음성 대조로 세지 않았다. 시작 순서를 일반 앱과 맞춘 별도
검사에서 기대한 assertion FAIL을 확인했다. 최초 오류 로그도 보존했다.
이 사실과 2단계의 native startup abort를 포함해 실기 장시간 안정성을 인수하지 않는다.

전체 `operator_console + lidar_dev/tests` 결과는 **921 passed / 10 failed / 7 skipped**다.
실패 10개는 2단계에서도 실패했던 동일 ID이며 새 실패 ID는 없다.
마지막 상태 초기화/안내 문구 보강 후 격리 비전 UI 4건도 다시 모두 통과했다.
로그 끝 공백과 마지막 빈 줄만 정리해 보존했다.

- [전체 검사](2026-10-23-vision-tab-step3-evidence/full-tests.log)
- [기준선 대비 실패 ID](2026-10-23-vision-tab-step3-evidence/test-comparison.json)
- [집중 검사 34건](2026-10-23-vision-tab-step3-evidence/focused-tests.log)
- [최종 비전 UI 4건](2026-10-23-vision-tab-step3-evidence/vision-ui-tests.log)
- [로컬 SRT/UDP E2E](2026-10-23-vision-tab-step3-evidence/local-e2e.log)
- [worker 비활성 고장 음성 대조](2026-10-23-vision-tab-step3-evidence/worker-negative.log)
- [첫 음성 대조 기동 오류](2026-10-23-vision-tab-step3-evidence/initial-negative-startup-fault.log)

```bash
PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
  GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest operator_console lidar_dev/tests -q
/usr/bin/python3 -m operator_console.runtime_smoke
```

필수 runtime_smoke는 이번에도 지도 LIVE/STALE을 통과한 뒤 기존 로봇팔
`arm tabs did not render live tool in safe disabled state` 판정에서 실패했다.

## 짧은 기능 검사 수치 — 성능 인수 자료가 아님

별도 약 3.05 s 샘플에서 지도 쌍 1.97 Hz, 합성 카메라 약 15.05 fps,
최대 GTK heartbeat 간격 약 46 ms, 마지막 지도 쌍 생성 약 19.5 ms였다.
테스트 프로세스 CPU는 약 111%(100%=한 코어), peak RSS 약 397 MiB였다.
**같은 프로세스의 합성 H.264 인코더도 포함한 값**이라 콘솔 전용 CPU 수치가 아니다.

전원 ADP1 online=0이었다. CPU 현재 클럭 범위는 시작 약 1.83–2.86 GHz,
끝 약 0.76–1.70 GHz로 기록됐다. 전원·클럭 조건이 통제되지 않아 최적화 전후 비교,
발열 제한 원인 판정이나 실기 수용 기준으로 사용할 수 없다.
[원본 조건·수치](2026-10-23-vision-tab-step3-evidence/local-metrics.json)를 보존한다.

## 결과와 다음 단계

[동작 화면](2026-10-23-vision-tab-step3-evidence/vision-maps.png)은 TEST 표기가 있는
합성 데이터의 1100×680 실제 GTK 캡처다. 로봇 실물 화면이 아니다.

3단계의 지도 통합·설정·조작·스레드 분리 및 로컬 검증을 완료했다.
다음은 4단계의 원시 IMU/보정 자세 구분·바퀴·AS5048B 인터페이스다.
장착각·좌표계·부호·영점 합의는 아직 받지 못했으며 임의 19.5° 차감은 하지 않았다.

Gateway/L515 실기, 영상/오버레이 실제 정합·지연, 같은 조건의 최적화 전후 비교,
전원 연결 상태의 CPU/RSS/클럭, 카메라 FPS 회귀, 0.5 s 정지 여부, 10분 연속 운용은 미검증이다.
runtime_smoke에 비전/자세 전체 전이를 추가하는 일도 6단계에 남아 있다.
PR·push·배포는 수행하지 않았다.
