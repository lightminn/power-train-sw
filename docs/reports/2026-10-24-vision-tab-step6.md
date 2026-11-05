# 비전 탭 6단계 — 실행 검사와 고장 검출

브랜치 `agent/vision-tab-20261023`, PR 대상 `feat/l515-terrain-map`.
비전의 수신·표시·전환·종료 검사를 `operator_console.runtime_smoke`에 통합했다.
**로컬 합성 SRT/UDP 검사이며 Gateway/L515 실기 인수가 아니다.**

## 변경 내용

- 기존 7개 수신 채널의 runtime 검사를 보존하고, 정상 실행 시 비전 전용 실행 검사를 추가한다.
  기존 arm 실패가 있어도 비전 단계의 통과/실패를 별도로 출력한다.
  전체 게이트가 실패한 상태를 통과로 바꾸지 않는다.
- `run_vision_smoke()`는 별도 Xvfb/시스템 Python 프로세스로 `vision_smoke`를 실행한다.
  두 1280×720@15 테스트 영상과 UDP fixture만 사용한다. 카메라 SDK·실물 센서·모터 명령은 사용하지 않는다.
  영상/데이터 포트는 임시 localhost 포트이며 기존 Gateway/콘솔과 충돌하지 않는다.
- `VisionPanel.probe()`로 실제 위젯의 표시 상태·라벨과 지도/자세 모델의 유효값·나이·갱신 횟수를 관찰한다.
  `--smoke-probe-file`이 있을 때만 앱 진단 경로에 포함한다. 계산이나 수신기를 추가하지 않는다.
- `OperatorConsole._on_destroy()`에서 비전 worker·타이머를 먼저 닫고 공유 receiver를 닫는다.
  panel destroy에서도 같은 close를 호출하며 중복 호출을 허용한다.
- GTK callback exception, 비정상 종료, 90초 실행/종료 timeout, 누락된 단계/결과 파일을 실패로 처리한다.
  Xlib이 stdout에 진단을 출력하는 경우가 있어 JSON 결과는 별도 임시 파일로 읽는다.
  child 종료 성공과 결과 파일을 모두 확인하므로 예전 PASS 파일/출력 한 줄만으로 통과하지 않는다.
- 기존 성능 도구와 실행 도구는 `smoke_timing.callback_gap`을 공유한다.
  콜백이 전혀 없거나 구간 끝에서 멈춰도 지연을 0으로 판정하지 않는다.
- `terrain_map/`, `terrain.py`, `terrain_map.py`, `terrain_map_panel.py`, `mobility_view.py`,
  `rover_3d.py` 및 lidar_dev 알고리즘은 수정하지 않았다. 기존 두 로컬 작업 폴더도 보존했다.

## 실제 실행으로 확인하는 항목

| 단계 | 검사 내용 |
|---|---|
| 미수신 | 네 영역 생성/표시, 지도·자세 WAITING, roll/pitch/엔코더 결측을 숫자로 채우지 않음 |
| 정상 수신 | 두 지도 raster, 실제 SRT decode/draw, 공유 카메라의 장애물 오버레이, raw pitch +19.5° 유지 |
| 출처·바퀴 | 장착 보정 미확인 표시, 실제 피드백 −12.5°/+0.12 rev/s 표시; 지령 2.0으로 대체하지 않음 |
| 엔코더 fixture | 실제 수신 0은 0으로 표시, 없는 두 각도는 미수신, 출처 fixture-only 표시 |
| 오래된 측정 | 최신 패킷이어도 엔코더 측정 나이 1800 ms이면 값 숨김/오래됨, null IMU는 이전 값 재사용 안 함 |
| 끊김 | 정상 raw 값을 다시 받은 뒤 UDP 송신 중지(영상 SRT는 유지); 자세·바퀴 STALE/숫자 숨김, 지도 STALE/Oblique 철회 |
| 복구 | 송신 재개 후 지도/자세 LIVE, 누락된 엔코더를 과거 값으로 복원하지 않음 |
| 표시 간격 | 정상 표시 구간에서 sink 수신만이 아니라 실제 GTK draw와 heartbeat의 간격 검사 |
| 숨김 | systems/mission으로 이탈 시 비전 지도 계산·UI 타이머·자세 갱신 정지, 지도/자세 수신 sequence 증가 |
| 카메라 재사용 | 최초 receiver pipeline은 두 개(D435/L515 각각 하나), 이후 신규 parse_launch 금지, sink/widget/receiver identity 유지 |
| 탭 왕복 | systems·mission 왕복과 D435 MAIN 선택 상태에서 기존 L515 parent/역할/전환 버튼 복원 |
| 종료 | 활성 비전에서 destroy, worker 종료·UI 타이머 제거·receiver thread 종료·파이프라인 NULL·UDP 포트 재바인드 |

`vision_smoke`는 정상 종료 검사를 마친 뒤 기존 콘솔 진입점과 같은 `os._exit` 정책으로
libsrt의 알려진 전역 소멸자 경로를 피한다. worker/receiver/pipeline 검사를 생략하고
exit code만 0으로 만드는 방식은 사용하지 않는다. worker 종료 누락 고장도 검출한다.
종료 후에는 실제 앱처럼 GTK 이벤트 루프를 다시 돌리지 않고 worker 종료만 기다린다.

## 음성 대조

각 고장은 별도 자식 프로세스에서만 주입한다. 제품 파일을 바꿔 두지 않는다.

| 고장 | 기대 실패 |
|---|---|
| 지도 UDP 누락 | map/telemetry LIVE timeout |
| 텔레메트리 UDP 누락 | map/telemetry LIVE timeout |
| 미수신 값을 0/유효로 조작 | missing values fabricated |
| stale 판정 시간을 100초로 조작 | disconnect/stale timeout |
| 끊김 이후 복구 송신만 누락 | recovery timeout |
| 숨김 후 worker 강제 활성화 | hidden render continues |
| 탭 동작 중 디코더 추가 시도 | duplicate camera/decoder creation |
| worker.close 누락 | worker shutdown timeout |
| GTK를 650 ms 정지 | camera draw stalled |

JSON 수집 경로도 네이티브 stdout 혼입, 파일 부재, 잘못된 JSON, 단계 누락,
비정상 종료, callback traceback 여섯 경우를 별도 검사한다.

## 실행 방법

```bash
# 노트북 / 저장소 루트; Xvfb는 하니스가 실행하므로 직접 감싸지 않는다.
cd /home/kimseon/power-train-sw-vision
/usr/bin/python3 -m operator_console.runtime_smoke --vision-only
# 기대: VISION-RUNTIME-SMOKE: PASS 와 단계별 JSON 증거

/usr/bin/python3 -m operator_console.runtime_smoke
# 현재: 비전/지도 통과를 출력하지만 기존 arm safe-disabled 실패로 전체 FAIL
```

GTK/GStreamer SRT·x264/decoder/gtksink, NumPy/OpenCV가 필요하다.
영상 재접속·실제 camera-to-screen 지연은 이 UDP 상태 전이 검사로 인수하지 않는다.
기본 실행에는 pytest를 사용하지 않는다. 아래 회귀 검사에는 pytest가 필요하다.

```bash
VISION_SMOKE_EVIDENCE_DIR="$PWD/docs/reports/2026-10-24-vision-tab-step6-evidence" \
  PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
  xvfb-run -a /usr/bin/python3 -m pytest operator_console -q
```

## 결과

최초 전체 회귀는 861 passed / 11 failed / 3 skipped(281.19초)였다.
이 중 10개는 5단계와 같은 실패이고, 추가 1개는 `test_tool_binding::test_udp_to_existing_console`의
복구 뒤 도구 kind가 dual_gripper가 아니라 unknown이 되는 간헐 실패였다.
현재/수정 전 `c22d654`에서 단독 실행은 모두 통과했다.
수정 전 테스트의 복구 패킷 처리에 한 번 1.1초 지연을 넣자 같은 assertion에서 재현됐다.
실제 바인딩은 패킷 나이 1초 초과 시 도구를 unknown으로 내리는 것이 맞다.

이 테스트의 복구 구간만 실제 브리지처럼 50 ms 주기로 새 sequence를 보내도록 보강했다.
보강 뒤 같은 1.1초 지연 주입에도 통과했다(`tool-fixed-delay.log`).
초기 송신 중단 후 1.05초 기다리는 stale 검사는 그대로 유지했다.
제품의 로봇팔 매핑·신선도 임계값·권한 게이트는 바꾸지 않았다.
표시 바인딩을 항상 unknown으로 고장 내면 보강된 검사도 FAIL/exit 1이었다.
최종 전체 회귀는 **862 passed / 10 failed / 3 skipped**, 281.37초다.
실패 ID는 5단계의 10개와 동일하며 새 실패는 없다(`test-comparison.json`).
비전 정상 실행 1개·고장 주입 9개·결과 수집 단위 검사 6개도 이 전체 검사에 포함돼 통과했다.
원래 전체 게이트의 arm safe-disabled 실패는 그대로 FAIL이며, 비전 단계만 별도로 통과한다.
보강한 tool binding 파일 전체는 7 passed였고, 공유 timing/map 집중 검사는 15 passed였다.
[증거 폴더](2026-10-24-vision-tab-step6-evidence/)에 최종/초기 전체 로그와
단계별 실제 표시 값, 고장별 실패 원인, 기준선 비교, 보호 파일 hash를 보관했다.

기본 runtime_smoke에서는 기존 `arm tabs did not render live tool in safe disabled state`가 남으며,
그 뒤 `terrain_map ok`, `vision runtime ok`가 출력된다. 이 실패를 약화하거나 우회하지 않았다.

하니스 개발 중에는 준비 직후의 overlay health 반영을 기다리지 않은 검사,
송신 타이머가 receiver close 뒤까지 살아 있던 검사, 종료 후 GTK를 다시 돌리던 검사,
Xlib stdout을 JSON으로 읽던 검사를 수정했다. 초기 실패 로그도 구분해서 보관한다.
이는 5단계의 **8.20초 draw 정지 원인을 해결했다는 의미가 아니다**.

## 남은 범위

다음은 7단계: 실제 Gateway/L515 연결, 전원 연결·CPU 클럭 기록 조건의 600초 시험,
기존 화면 대비 FPS·최대 표시 정지·지도 2 Hz·CPU/RSS·지연 비교, 동작 영상과 PR 정리다.
현재 Jetson 별칭은 여전히 이름 해석에 실패해 실제 연결 검증을 수행하지 못했다.

5단계에서 관측한 표시 정지 1회의 원인은 미확정이다. 이번 짧은 합성 실행에서 재현되지
않았더라도 장시간 안정성을 통과로 처리하지 않는다. 장착 보정 합의, 실제 AS5048B 송신,
물리 영상/오버레이 좌표·시간 정합, 현재 패킷에 없는 바닥 영역 마스크도 미완료 상태다.
