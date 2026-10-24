## 변경 결과

기존 관제탑에 **비전** 탭을 추가해 L515 영상/지형 오버레이, Top view, Oblique view, 자세·바퀴 상태를 동시에 표시합니다. 사용자가 확인한 노트북 Graphite GUI를 기준으로 기존 탭과 같은 화면 틀을 사용합니다.

**실기 인수 전 Draft입니다.** 현재 Jetson의 이름 해석/접속이 되지 않아 Gateway+L515 10분 검증을 수행하지 못했습니다. 합성 입력 결과를 실기 통과로 처리하지 않습니다.

- Base: `feat/l515-terrain-map` (`a40ea17`, #19 병합 기준). `main` 대상이 아닙니다.
- Head: `agent/vision-tab-20261023`.

## 현재 GUI와 재사용 범위

| 화면/데이터 | 구현 및 연결 |
|---|---|
| 실시간 화면 | 기존 L515 `:5000`·D435 `:5002` VideoPanel 유지. 비전 진입 시 L515 위젯 전체를 이동하고 원래 MAIN/PiP 역할로 복원 |
| 지형 오버레이 | 기존 `:5009` receiver/TerrainCanvas 공유. 두 탭의 토글도 같은 상태 사용 |
| 지형 지도 탭 | 기존 기능 유지. 동일 `:5010` receiver를 새 Top/Oblique worker에 전달 |
| 협조구동·시스템·3D·로봇팔 | 노트북 Graphite 화면/자산 이식, 기존 기능 유지. 로봇팔 제어 계약 변경 없음 |
| 비전 자세 | `:5005` raw roll/pitch·바퀴별 실제 steer/speed·stale/fault 사용. 지령을 피드백으로 대체하지 않음 |

광민의 `terrain_map/`, `operator_console/terrain.py`, `terrain_map.py`, `terrain_map_panel.py`는 수정하지 않았습니다. 지도 계산/렌더링은 기존 MapRenderer와 `live_map`의 격자·분류·필터·색상·Oblique 함수를 재사용합니다. `lidar_dev/live_map.py`·`obstacles_map.py`는 package/script import 호환만 수정했고 알고리즘은 유지했습니다.

도현/노트북 GUI의 팔레트·폰트·하단 탭·3D 자산을 재사용했습니다. `mobility_view.py`·`rover_3d.py`는 확인된 노트북 버전을 그대로 이식했고 이후 수정하지 않았습니다. 새 3D 모델·지도 생성기·카메라 연결·디코더를 추가하지 않았습니다.

## 추가한 동작

- 한 화면 네 영역, 공유 오버레이 on/off, 지도 zoom/follow/drag, 로버 방향 화살표, 상태/색상 범례.
- 지도 계산은 worker에서 쌍당 약 2 Hz. GTK에는 준비된 raster만 전달하고 숨기면 비전 계산/표시 타이머를 멈춤. receiver는 계속 수신.
- 값/출처/유효성/나이를 구분. 미수신·오래됨·끊김·복구를 표시하고 결측은 0으로 채우지 않음.
- raw IMU, 장착 보정 미확인 차체 자세, 정지 창 지도 기록을 분리. 19.5° 일괄 차감 없음.
- AS5048B 선택 연결 인터페이스: `suspension_joints_rad`, `suspension_age_ms`, `suspension_source`, `suspension_frame`. 실제 송신이 없으면 미수신.
- 기준값 CLI/JSON/env 설정과 버전·프로필·SHA256 표시. v0.2 사용, 10/26 파일은 코드 변경 없이 경로/파일 교체 후 재시작.
- NumPy import 전 `OPENBLAS_NUM_THREADS=1`, decoded raw 프레임 1개 leaky queue, `sync=false` 유지, 최신 값만 GTK에 전달, 한글 Pango cache 상한 설정.
- 종료 시 비전 worker/timer를 공유 receiver보다 먼저 정리.

## 실행

```bash
# 노트북 / 이 브랜치 checkout; Gateway는 기존 프로세스를 사용
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m operator_console.app \
  --host <JETSON_IP> --page vision \
  --l515-criteria lidar_dev/config/obstacle_criteria_v0.2.json \
  --l515-criteria-profile A_no_stop

# 페어링된 통합 콘솔도 같은 옵션 지원
/usr/bin/python3 -m operator_console --config ~/.config/powertrain/operator.json --page vision
```

기준값 우선순위는 CLI > JSON > env > 기본값입니다. 필드별 독립 선택이며 JSON의 상대 경로는 설정 폴더 기준입니다. 이미 들어오는 데이터만 받으며 카메라 SDK를 열거나 `live_overlay.py`를 로봇에서 실행하지 않습니다.

지도/오버레이 송신은 기존 Compose opt-in `terrain-map`/`terrain-overlay`입니다. 실제 Jetson checkout·기동 상태·세션 목적지를 확인하지 못했으므로 이번에 로봇 서비스/체크아웃을 변경하지 않았습니다.

## 검증과 성능

- 전체 콘솔: **862 passed / 10 failed / 3 skipped**. 실패 ID 10개는 변경 전 단계와 동일합니다.
- `runtime_smoke --vision-only`: PASS. 정상/결측/오래된 측정/끊김/복구, 숨김 중 수신 유지, MAIN/PiP 왕복, decoder/receiver 공유, worker/timer/socket/pipeline 종료 확인.
- 고장 9종: 지도/텔레메트리 누락, 결측 0 조작, stale 무력화, 복구 누락, 숨김 렌더 강제, decoder 추가, worker 종료 누락, 650 ms GTK 정지를 모두 검출.
- 별도 CPU/RSS 수집기: 실제 대상 프로세스 관찰·종료/미관측 처리 검사 2개 통과. 카메라 연결을 추가하지 않으며 수집 완료를 실기 통과로 판정하지 않음.
- 기본 `runtime_smoke`: 비전·지도 단계 통과, 기존 **arm safe-disabled 표시 실패로 전체 FAIL**. 게이트를 완화하지 않았습니다.
- 도구 복구 검사에서 단발 패킷이 1초 임계값을 넘기는 간헐 실패를 기준선에서 지연 주입으로 재현했습니다. 테스트의 LIVE 복구 입력만 연속 송신으로 보강했고 stale 검사와 제품 임계값은 유지했습니다.

AC 연결, 합성 영상 2×1280×720@15 + 지도 3 tile + 오버레이/자세 입력, 같은 도구로 20초씩 측정한 결과입니다. 인코더/송신은 별도 프로세스이며 아래 CPU/RSS에서 제외합니다. CPU 100%는 논리 코어 하나입니다.

| 비전 구간 | 최적화 전 | 최적화 후 |
|---|---:|---:|
| L515 sink FPS | 14.99 | 15.00 |
| 각 지도 반영률 | 약 2.00 Hz | 약 2.00 Hz |
| 콘솔 CPU | 93.87% | 93.25% |
| RSS 최대 | 303.4 MiB | 303.7 MiB |
| 최대 GTK heartbeat 간격 | 55 ms | 52 ms |
| 최대 video draw 간격 | 115 ms | 115 ms |

작은 CPU 차이를 개선율로 단정하지 않습니다. 프레임/알림/캐시 보관량 상한과 숨김 중 계산 중지를 검증했습니다. sink running-time 지표는 실제 camera-to-screen 지연이 아닙니다. 전원·CPU별 클럭·온도 표본도 보관했습니다.

## 동작 영상과 자료

[합성 동작 영상(WebM)](2026-10-24-vision-tab-step7-evidence/vision-synthetic-demo.webm) — 화면에 **합성 입력 시연 · 실기 검증 아님** 표시. 저장소의 합성 시연 원본 파일이며 실제 카메라 인수 영상이 아닙니다.

![합성 비전 화면](2026-10-24-vision-tab-step7-evidence/vision-synthetic-demo.png)

- [현재 GUI/데이터 조사](2026-10-23-vision-tab-audit.md)
- [Graphite GUI·카메라 통합](2026-10-23-vision-tab-step2.md)
- [지도·기준값·worker](2026-10-23-vision-tab-step3.md)
- [자세·엔코더 계약](2026-10-23-vision-tab-step4.md)
- [성능 원시 결과·한계](2026-10-24-vision-tab-step5.md)
- [실행 검사·고장 검출](2026-10-24-vision-tab-step6.md)
- [7단계 실기 준비·미검증 목록](2026-10-24-vision-tab-step7.md)

## 미완료·미검증

- **Gateway+실제 L515 600초 연속 시험 미수행**: 현재 `jetson`/`jetson-orin.local` 해석 실패. 현재 SSH/IP 정보가 필요합니다.
- 실기 카메라 FPS 비교, 0.5초 초과 표시 정지 없음, 지도 2 Hz, CPU/RSS/네트워크 지연, Gateway 충돌 없음과 기존 탭 실기 회귀는 미검증.
- 5단계에서 video draw가 **8.20초 멈춘 실행 1회**를 관측했습니다. 이후 짧은 검사에서 재현되지 않았지만 원인 미확정이며 해결로 표시하지 않습니다.
- 녹화 준비 중 같은 프로세스의 합성 SRT 송·수신/Wayland 상속 구성에서 native SRT epoll/unwind 충돌을 재현했습니다. 시험 송신기를 별도 프로세스로 옮기고 리슨 준비와 X11 격리를 보장했습니다. 이것을 실기 네이티브 안정성 해결로 간주하지 않습니다.
- **바닥 영역 칠하기 미충족**: 현 `:5009`에는 ground 상태·장애물 polygon만 있고 바닥 mask/polygon이 없습니다. 현재는 바닥 상태 배지와 장애물 윤곽입니다.
- 광민·도현과 좌표계/부호/영점/장착 회전 합의 및 실기 자세 보정은 미확인. raw 값을 차체 기울기로 적용하지 않습니다.
- AS5048B 실제 송신·필드별 측정 시각 연결 없음. 현재 인터페이스/미수신 표시만 제공합니다.
- 물리 영상/오버레이의 좌표·시간 정합과 실제 end-to-end 지연 미검증.

위 항목은 실제 데이터/연결 확인 후 후속 검증하고, 결과와 영상을 추가하기 전에는 실기 완료로 병합하지 않습니다.
