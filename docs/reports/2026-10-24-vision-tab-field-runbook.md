# 비전 탭 실기 600초 검증 절차 — 아직 미수행

이 절차는 실제 Jetson/Gateway/L515가 연결된 뒤 수행한다.
현재 노트북은 `172.20.10.3/28`, 설정의 `jetson-orin.local`과 관례 alias `jetson`은 해석되지 않았다.
실제 접속 주소·사용자와 센서/송신 상태를 확인하기 전에는 실기 완료로 체크하지 않는다.

## 1. Jetson 현황 확인 — 읽기 전용

노트북에서 접속한다. 아래 IP는 실제 전달받은 주소로 넣는다.

```bash
ssh -o ConnectTimeout=5 zetin@<JETSON_IP>
```

SSH 접속 직후 Jetson 홈에서 확인한다.

```bash
hostname
cd ~/power-train-sw
git status --short --branch
git log -3 --oneline
git log --oneline '@{u}..'
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
```

기대: 실제 checkout/팀원의 미커밋 상태, 기존 Gateway 소유 컨테이너,
`powertrain_terrain_map`·`powertrain_terrain_overlay`·차대 텔레메트리·세션 서비스의 실행 여부를 알 수 있다.
checkout을 reset/clean/stash하거나 기존 Gateway를 중복 기동하지 않는다.
지도 코드를 수정하거나 `lidar_dev/live_overlay.py`를 로봇에서 실행하지 않는다.

기존 `terrain-map`/`terrain-overlay`는 opt-in 서비스다. 준비된 통합 스택을 사용할 때
Jetson 저장소 루트의 명시적 기동 명령은 다음과 같다. **현재 미실행**이며,
위 체크아웃/소유권 확인과 설치 매니페스트·CAN 준비를 먼저 충족해야 한다.

```bash
INTEGRATED_PROFILES="terrain-overlay terrain-map" \
INTEGRATED_SERVICES="powertrain_terrain_overlay powertrain_terrain_map" \
  scripts/robot-start
```

profiles/services는 공백 구분이다.  세션 목적지와 기동 여부가 확인되지 않으면
콘솔의 미수신을 정상으로 기록한다. 단순히 다른 카메라/수신기를 추가해서 채우지 않는다.
구체적인 기동 경로는 [통합 운용 문서](../integrated-operation.md)의 현재 설치 경로와 대조한다.
이번 변경은 로봇 배포나 Gateway/서비스 재시작을 수행하지 않았다.

## 2. 노트북 실행 — 콘솔 한 개만 사용

전원을 연결하고 실제 Jetson과 같은 운용망으로 연결한다. 아래는 노트북 저장소 루트에서 실행한다.

```bash
cd /home/kimseon/power-train-sw-vision
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m operator_console.vision_monitor \
  --metrics-output /tmp/vision-physical-events.json --integrated \
  --config ~/.config/powertrain/operator.json --page vision
```

기대: 페어링된 기존 세션 경로로 연결하고 비전 탭을 연다. 선택 센서/프로필이 없으면 미수신이다.
기존 수동 RX 경로를 이미 운용 중인 환경에서는 위 통합 콘솔과 동시에 실행하지 않고 다음 중 하나만 사용한다.

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m operator_console.vision_monitor \
  --metrics-output /tmp/vision-physical-events.json \
  --host <JETSON_IP> --page vision \
  --l515-criteria lidar_dev/config/obstacle_criteria_v0.2.json \
  --l515-criteria-profile A_no_stop
```

기대: 기존 L515 `:5000`, terrain `:5009`, map `:5010`, chassis `:5005` 수신 경로를 사용한다.
수동 RX 경로는 기존 송신 목적지가 이 노트북으로 설정돼 있어야 한다.
기존 SRT listener, 계정, pairing token을 콘솔에서 새로 만들지 않는다.
토큰/세션 ticket을 실행 로그·영상·PR에 포함하지 않는다.

## 3. CPU/RSS·환경 수집 — 실제 콘솔 PID만 관찰

다른 노트북 터미널에서 실행 중인 콘솔 PID를 확인한다.

```bash
pgrep -af '^/usr/bin/python3 -m operator_console'
```

실제 콘솔의 PID를 `CONSOLE_PID`에 넣는다. 수집기는 수신 소켓이나 영상 연결을 만들지 않으며
지정한 프로세스를 종료하지 않는다. CPU는 `/proc/<PID>/stat`의 CPU 시간 **차분**으로 계산한다.
프로세스 시작 시각을 확인해 PID 재사용을 거부한다. 현재 RSS, 전원·CPU별 클럭·온도·열 제한 누계를 함께 기록한다.

```bash
cd /home/kimseon/power-train-sw-vision
CONSOLE_PID=<확인한_PID>
/usr/bin/python3 -m operator_console.tests.sample_process \
  --pid "$CONSOLE_PID" --seconds 600 --require-ac --output /tmp/vision-physical-cpu.json
```

기대: 600초 실제 관찰 후 `status=complete`와 JSON 결과.
중간 종료·PID 재사용·전원 분리는 `incomplete`/실패로 기록한다.
관측하지 못한 CPU/RSS를 0으로 채우지 않는다. 출력 파일이 있으면 새 경로를 선택한다.
`complete`는 **프로세스 수집 완료**만 의미한다. 실제 카메라/FPS/표시 정지/지도/실기 인수 통과가 아니다.

같은 콘솔에서 먼저 실시간 화면을 측정한 뒤 비전 탭으로 전환해 별도 파일에 다시 측정한다.
최적화 전 비교는 `930da9f`를 별도 worktree에서 같은 Gateway·입력·전원·수집 도구로 측정해야 한다.
비교용 콘솔 두 개를 동시에 실행하지 않는다. 기존 합성 20초 비교를 실기 기준선으로 대체하지 않는다.

## 4. 600초 인수 기록

아래는 실제로 데이터를 확인한 뒤 결과/원시 파일/동작 영상의 경로를 채운다.
미수신·실행 불가 항목은 수치 0이나 PASS로 채우지 않는다.

| 항목 | 현재 결과 | 실제 시험에서 필요한 증거 |
|---|---|---|
| Gateway/L515 소유권·checkout | 미검증 | 단일 기존 Gateway, 원격 git 상태·실행 서비스 |
| 네 영역 동시 실제 수신 | 미검증 | 영상+오버레이+지도2개+자세가 보이는 실기 영상 |
| L515 FPS 동일 | 미검증 | 같은 입력에서 실시간/비전 비교; 실제 decoder FPS 계측 |
| 최대 표시 정지 ≤0.5 s | 미검증 | 실제 video draw/sink 및 GTK heartbeat 전체 간격; 구간 끝 정지도 포함 |
| Top/Oblique 각각 약 2 Hz | 미검증 | 각 지도 반영 count/수집 시간 및 실제 화면 |
| 콘솔 CPU/RSS | 미검증 | 위 PID 수집 JSON, 전원/클럭/온도/열 제한 누계 |
| 실제 지연·좌표 정합 | 미검증 | 물리 타깃/영상과 overlay 기준, capture→display 지연 자료 |
| 수신 끊김·복구 | 미검증 | 서비스 소유권을 유지한 상태에서 연결 변화에 대한 GUI 기록 |
| 기존 탭 회귀·영상 연결 중복 없음 | 미검증 | 같은 프로세스 탭 왕복, 연결/decoder 수와 기존 탭 확인 |
| body calibration | 미확인 | 광민·도현의 좌표/부호/영점/장착 회전 확인 |
| AS5048B 실제 송신 | 미연결 | 실제 각도·출처·측정 age 필드; 없으면 계속 미수신 |

CPU 수집만으로 FPS나 draw 지연을 추정하지 않는다. CPU 수집기는 해당 지표를 측정하지 않는다.
위 `vision_monitor`가 **같은 콘솔**의 기존 sink/draw/heartbeat/각 지도 반영률을
`/tmp/vision-physical-events.json`에 매초 기록한다. `console_pid`를 CPU 수집기의 PID와 대조한다.
종료 시 `status=closed`이며 `hardware_verified=false`는 자동 인수하지 않는다는 의미다.
`segments`에서 실시간/비전 구간을 나눠 sink FPS와 최대 draw/heartbeat 간격,
`top_images`·`oblique_images`의 count/Hz/최대 간격을 각각 비교한다.
repaint Hz는 프레임 FPS와 다르며 미관측 지표 null을 0이나 PASS로 바꾸지 않는다.
최근 탭 구간 64개만 유지하므로 장시간 시험 중 불필요한 탭 반복을 피하고 `dropped_segments`도 확인한다.
이 계측을 합성 시험에서 실행하고 650 ms 고장을 넣어 검출하는 검사도 포함했다.
광학 camera-to-screen 지연과 물리 overlay 좌표는 별도 타깃/시각 측정이 필요하다.
클럭 하락을 바로 처리 병목으로 판단하지 않고 전원·온도·throttle 누계와 CPU/표시 간격을 함께 본다.
5단계의 8.20초 draw 정지는 미해결 항목으로 남으며, 한 번의 정상 단기 실행으로 삭제하지 않는다.

## 현재 제공되는 합성 확인

```bash
# 실제 센서 없이 로컬 영상/UDP를 사용하는 기능 시연
/usr/bin/python3 -m operator_console.runtime_smoke --vision-only
/usr/bin/python3 -m operator_console.runtime_smoke --vision-only \
  --record-webm /tmp/vision-synthetic-demo-new.webm
```

기대: 별도 Xvfb/X11의 콘솔 창만 녹화하며 화면에 합성 입력 표시.
이 동작 영상과 source fixture의 posture/encoder 값은 실측 데이터가 아니다.
