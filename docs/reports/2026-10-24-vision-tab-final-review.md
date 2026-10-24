# 비전 탭 최종 요구사항 대조 — 2026-10-24

**판정: 실기 시험을 위한 구현·로컬 검증은 준비했으나, “실제 HW 테스트만 남고 모든 요구가 끝났다”는 판정은 불가하다.**
바닥 영역 오버레이 계약, 자세 보정 합의·비교, 이전 표시 정지의 원인 확인은 실기 600초 시험과 별개의 열린 항목이다.
PR #20은 Draft이며 인수 완료나 병합 완료로 처리하지 않는다.

원본 `/home/kimseon/문서/카카오톡 받은 파일/vision-tab-task.md`의 모든 절과 이 채팅의 요구를 다시 대조했다.
아래 `로컬 확인`은 구현/소스 검사/합성 실행 범위이며 실제 센서 인수 통과를 뜻하지 않는다.
`외부 대기`는 계약·보정·송신 등 현재 콘솔만으로 완료할 수 없는 항목,
`미검증`은 아직 수행하지 않은 실제 환경 시험, `미해결`은 과거 실패의 원인이 미확정인 항목이다.

## 이번 검토에서 보완한 누락

1. JSON의 선택 기준값 프로필이 `null`이면 환경 변수 프로필을 무시하던 선택 우선순위를 수정했다.
2. 선택 AS5048B group 전체가 `null`인 패킷도 다른 실제 roll/pitch·바퀴 데이터를 버리지 않고 각도 미수신으로 받도록 수정했다.
   숫자 문자열·bool·비유한 각도 등 잘못된 측정값 거부는 유지한다.
3. PR에 누락돼 있던 `robot-start`의 두 opt-in profile/service 실행 명령, 오버레이 지연 기본 0 ms와 실기 미보정,
   엔코더 네 키·단위·제안 부호/영점 및 출처 미표기 규칙을 추가했다. 로봇에서 이 명령을 실행하지 않았다.
4. 실제 콘솔에서도 기존 sink/draw/GTK heartbeat와 각 지도 반영률을 기록할 관찰 도구 `vision_monitor`를 추가했다.
   기존 CPU/RSS 수집기는 영상 지표를 측정하지 못했으므로 두 자료를 함께 수집하도록 절차를 보완했다.
   수신기·디코더·카메라를 추가하지 않고 정상 수동/페어링 진입점을 사용한다. 광학 end-to-end 지연은 별도 실측이다.
5. AC 연결·X11 격리에서 비전 탭 합성 입력 600초 연속 관찰을 추가했다. 실기 시험을 대체하지 않는다.

## 전체 대조표

### 1. 브랜치·GUI 조사·보존

| ID | 요구 | 판정·근거 |
|---|---|---|
| 1.01 | lightminn/power-train-sw, feat/l515-terrain-map에서 새 브랜치 | 로컬 확인: agent/vision-tab-20261023, merge-base a40ea17 |
| 1.02 | PR 대상도 feat/l515-terrain-map | 확인: Draft PR #20. main 대상 아님 |
| 1.03 | PR #19 병합·main 미반영 상태 기준 | 시작 기준 a40ea17 및 PR19 기록 보존; main을 새 기준으로 섞지 않음 |
| 1.04 | 기존 탭·기능·실제 데이터 범위 먼저 정리 | 완료: 1단계 audit와 2단계 Graphite 이식 보고서 |
| 1.05 | 광민 지도·도현 화면 기능 재사용/중복 최소화 | 로컬 확인: MapRenderer/live_map/Pango/VideoPanel/TerrainCanvas/Graphite 자산 재사용 |
| 1.06 | 네 화면 동시 표시 유지 | 로컬 GTK/SRT/UDP 실행·합성 영상에서 확인 |
| 1.07 | terrain_map/ 수정 금지 | 기준 a40ea17 대비 변경 0 |
| 1.08 | operator_console/terrain.py 수정 금지 | 기준 대비 변경 0 |
| 1.09 | operator_console/terrain_map.py 수정 금지 | 기준 대비 변경 0 |
| 1.10 | operator_console/terrain_map_panel.py 수정 금지 | 기준 대비 변경 0 |
| 1.11 | mobility_view 수정 전 협의 | 승인된 노트북 파일을 그대로 복사했고 이후 수정 0; 승인본 SHA256 일치 |
| 1.12 | 채팅: 전체 디자인은 승인한 노트북 GUI 버전 | Graphite 틀/기존 탭/3D 자산 적용; 당시 스크린샷 및 import 해시 보존 |
| 1.13 | 기존 작업·사용자 미커밋 변경 보존 | 별도 worktree 사용; 원래 두 사용자 작업트리를 reset/clean/stash하지 않음 |

### 2. 참고 코드·의존성

| ID | 요구 | 판정·근거 |
|---|---|---|
| 2.01 | live_overlay 배치/동작·10/22 보고서 참고 | 설계·1~3단계 기록에 반영 |
| 2.02 | live_overlay.py 로봇 실행 금지/Gateway 단독 소유 | 준수: 로봇 실행·직접 카메라 SDK 접근 없음 |
| 2.03 | tmap_local_grid 재사용 | Oblique worker에서 기존 함수 호출 |
| 2.04 | classify_heights/filter_hazards/cell_colors 재사용 | 기존 함수를 그대로 호출; 정책/필터 단위검사 |
| 2.05 | top_image/oblique_image 렌더링 참고 | Top은 요청대로 MapRenderer, Oblique는 oblique_image 재사용 |
| 2.06 | gm_features 오버레이·한글 캐시 참고 | 공유 TerrainCanvas 유지, bounded Pango layout cache 사용 |
| 2.07 | criteria.py/v0.2 기준값 처리 | load_criteria 재사용, 파일 유효성·프로필 검사 |
| 2.08 | 기존 terrain/map/map_panel/app.Stack 연결 | 기존 receiver 전달·새 Stack page·공유 위젯 이동 |
| 2.09 | cv2 (a)/(b) 선택·시스템 Python 실행 | (a) 운용 PC 사용자 영역 OpenCV-headless 설치, /usr/bin/python3 실기동 확인 |
| 2.10 | lidar_dev package/script import 호환 | import 경로만 호환 수정, 지도 알고리즘 변경 없음 |
| 2.11 | 한글은 KText 직접 적용 대신 PangoCairo 캐시 | 구현·캐시 상한/재사용/음성 대조 확인 |

### 3. 데이터 수신·공유·유효성

| ID | 요구 | 판정·근거 |
|---|---|---|
| 3.01 | 비전은 받기 전용/콘솔 카메라 직접 열기 금지 | 준수: 새 센서 접근·송신/제어 경로 추가 없음 |
| 3.02 | L515 :5000·terrain :5009 기존 수신 | 기존 VideoPanel/TerrainCanvas/receiver 공유 |
| 3.03 | 기존 실시간 화면과 연결·디코더 공유 | 새 비전 연결 0; 로컬 pipeline factory 총 2(L515+D435) 검사 |
| 3.04 | gtksink 포함 전체 L515 widget reparent | 진입 이동/퇴장 원복; MAIN/PiP 두 출발 상태 검사 |
| 3.05 | D435 MAIN 상태의 진입 처리 | D435 MAIN/비전/복귀/재전환 GTK 실행 검사 통과 |
| 3.06 | tee/추가 gtksink 금지 | 소스 및 생성 수 검사, 추가 디코더 고장 검출 |
| 3.07 | :5010 기존 _map_receiver 재사용/새 소켓 금지 | 동일 인스턴스 전달; 합성 연결·종료/포트 회수 검사 |
| 3.08 | :5005 기존 _chassis_receiver 사용 | 동일 telemetry snapshot 읽기 |
| 3.09 | roll_rad/pitch_rad/wheel steer_deg/drive_turns_per_s/stale | 실제 피드백 필드 연결; 명령값 대체 없음 |
| 3.10 | 출처/유효 여부/마지막 갱신 age | UI source/age 상태 모델, 결측 None 유지 |
| 3.11 | 미수신·오래됨·끊김·복구 구분 | 실제 GTK 합성 단계 missing/live/aged/disconnected/recovered 검사 |
| 3.12 | map LIVE/STALE 0.5초 기준 | LINK_LOST_S=0.5 공유 기준; 위젯/모델 검사 |
| 3.13 | 실제 Gateway 충돌/카메라 중복 연결 없음 | 로컬 구조/생성 수 확인; 실제 Gateway 연결은 미검증 |

### 4. 화면·지도·기준값

| ID | 요구 | 판정·근거 |
|---|---|---|
| 4.01 | 카메라·Top·Oblique·자세 한 화면 | 2×2 구성, mapped slot 네 개 동시 검사 |
| 4.02 | 카메라 장애물 오버레이 | 기존 polygon overlay 공유; 합성 정합 확인 |
| 4.03 | 카메라 바닥 영역 오버레이 | **외부 대기/미충족**: :5009에 바닥 영역 mask/polygon 없음. 바닥 상태 배지만 구현 |
| 4.04 | overlay on/off·공유 여부 PR 기록 | 두 탭 동일 상태 공유, GTK toggle 검사·PR 기재 |
| 4.05 | 영상/overlay 좌표 정합 | 합성 좌표 및 widget overlay 확인; **물리 타깃 정합 미검증** |
| 4.06 | 표시 지연 및 0–400 ms 옵션/선택값 기록 | 기존 옵션 유지, 현재 0 ms 명시; **실기 지연 맞춤 미검증** |
| 4.07 | 지도 로버 위치/방향 화살표 | 두 지도 모두 표시, stale/current 구분; no pose 추정 금지 검사 |
| 4.08 | 자세 별도 패널 | raw IMU·차체 보정 상태·wheel·encoder·정지 기록 패널 |
| 4.09 | 확대/축소 ×1.2 | scroll/버튼 동일 1.2 동작, 합성 GTK 검사 |
| 4.10 | 따라가기·끌면 해제 | follow 상태/drag 동작 검사 |
| 4.11 | 상태·색상 범례 용어 일치 | note_and_legend 재사용·선택 기준값 정보 병기 |
| 4.12 | 기존 렌더링으로 높낮이·장애물 가독성 | MapRenderer shading/기존 Oblique 분류·색상; 합성 이미지 확인, 실제 지형 가독성 미검증 |
| 4.13 | Top 순수 numpy MapRenderer 그대로 사용 | 작업 스레드 전용 instance/cache; 기존 MapPanel 무수정 |
| 4.14 | 기준값 CLI + 통합 JSON + L515_CRITERIA | 우선순위 CLI>JSON>env>default, 상대경로·null fallback 검사 |
| 4.15 | 현재 v0.2 사용 | 기본 v0.2/A_no_stop, 외부 경로 지정 가능 |
| 4.16 | 10/26 파일 코드 수정 없이 교체 | 외부 파일 교체 후 재시작·버전/hash 변경 검사; 최종 파일 자체는 미수령 |
| 4.17 | trench_depth_m 없으면 step_up_m 보수 대체 | 기존 live_map 규칙 유지·정책 단위검사 |
| 4.18 | 버전/프로필/SHA256 화면 표시 | 선택값·hash 표시; 설정 오류 명시적 실패 |

### 5. 자세·바퀴·AS5048B·보정

| ID | 요구 | 판정·근거 |
|---|---|---|
| 5.01 | 기존 자세 기능 참고·간단한 차체 그림·roll/pitch | 기존 그래픽 표현 참고; 비전 별도 단순 그림/원시 수치 |
| 5.02 | 실제 바퀴 조향각·속도 연결 | wheel_statuses의 수신값·stale·fault 표시 |
| 5.03 | Hall 저속 <0.3 rev/s 주의는 표시 단서만 | 문구 표시; 저속을 0으로 보정/지령 대체하지 않음 |
| 5.04 | rocker/bogie/source/age 자리 | 네 관절 슬롯·출처·측정 age 존재 |
| 5.05 | 선택 parser 필드/이름·단위·부호 PR 기록 | suspension_joints_rad 네 키, age_ms/source/frame 명시; 부호/영점은 제안·미합의로 구분 |
| 5.06 | AS5048B 실제 경로 없음 명시 | **미연결**: 송신부 없음. 인터페이스 준비만 요구 범위 완료 |
| 5.07 | 없으면 미수신, 0/추정 금지 | 누락/명시적 null/부분 누락 검사; 값 0 자체는 유효 측정으로 구분 |
| 5.08 | L515 장착 pitch를 차체 기울기로 표시 금지 | 원시 IMU(L515 기준)와 차체 보정 미확인 분리 |
| 5.09 | imu_tilt→filtered→telemetry 원시 경로 확인 | 소스 조사·4단계 보고서; 생산자 좌표 보정 변경 없음 |
| 5.10 | 19.5°/URDF15°/옛 지도20° 차이 확인 | **외부 대기**: 자료 불일치 기록, 광민·도현 합의 없음 |
| 5.11 | 좌표/부호/영점/장착각 반영 방식 합의 | **외부 대기**: 확정 없이 보정 자세 활성화하지 않음 |
| 5.12 | 19.5° 일괄 차감 금지 | 준수: 수신 rad→표시 ° 변환만 수행 |
| 5.13 | 협조구동 raw Roll/Pitch 수정 금지·PR 설명 | 파일 원본 유지, PR 명시 |
| 5.14 | plane_tilt age 1 s 초과 숨김 | 모델/위젯 검사, map link stale와 별도 판정 |
| 5.15 | gravity는 정지 측정·n초 전, 실시간 자세처럼 금지 | 기록 분리/age 증가/epoch 전환/주행 중 기록 유지 구분 검사 |
| 5.16 | 지도 정지 기울기와 telemetry 차이 확인 | **외부 대기·미검증**: 두 출처를 분리 표시하나 좌표/시각/장착 보정 확정 전 숫자 차이를 유효 비교로 계산하지 않음 |

### 6. 성능·장시간 안정성

| ID | 요구 | 판정·근거 |
|---|---|---|
| 6.01 | GTK에서 무거운 지도 계산/이미지 생성 금지 | 비전 worker에서 계산·BGRA 준비, GTK는 raster/cairo 표시 |
| 6.02 | Top/Oblique 각각 약 2 Hz | PERIOD_S=.5, 최신 한 쌍만 게시; 합성 600초 개별 paint count 추가 측정 |
| 6.03 | 기존 20–30 ms 비용 고려 | viewport/격자/타일/caches 상한, render_ms 기록 |
| 6.04 | GIL 영향/메인 정지 실측, 필요 시 process | worker thread 사용·heartbeat/draw/sink 별도 계측. 현재 제어된 합성 시험으로 무조건 fork 전환 근거 없음; 과거 8.20초 draw 원인은 미해결 |
| 6.05 | OPENBLAS_NUM_THREADS=1, numpy 이전 | package __init__에서 지정; runtime BLAS thread 1 확인 |
| 6.06 | service/python -m/gui_up 모든 실행 경로 | service Environment·package entry·안내 명령 반영 및 검사 |
| 6.07 | 한글 캐시/메모리 상한 | widget당 Pango LRU 상한/파기 시 해제; cache 음성 대조 |
| 6.08 | 비전 숨김 렌더/갱신 중지 | map worker condition 대기, UI timer 해제; hidden-render 고장 검출 |
| 6.09 | 숨김 중 수신 계속 | UDP sequence 진행·복귀 시 최신값 표시 확인 |
| 6.10 | 영상 sync=false 유지 | 기존 sink flag 유지·실제 pipeline 검사 |
| 6.11 | 누적 대신 최신 프레임 | raw queue max-size-buffers=1/leaky downstream; overload 검사 |
| 6.12 | 지도 처리와 영상 수신/표시 분리 | map worker와 기존 Gst streaming; receive/decode 생성 추가 없음 |
| 6.13 | 모든 영역 켠 FPS/지연/CPU/RSS 비교 | 5단계 AC 동일 합성 조건 전후 자료; 실제 camera→screen 지연·실기 전후는 미검증 |
| 6.14 | 같은 조건 최적화 전/후 비교 | AC 20초 pair 보존; 작은 CPU 차이를 개선율로 주장하지 않음 |
| 6.15 | AC 연결·CPU 클럭 동시 기록 | 5단계/이번 600초 power·주파수·온도 표본; 새 PID sampler는 thermal throttle 누계도 수집 |
| 6.16 | 발열 clock 제한과 프로그램 병목 구분 | 지표 분리 수집; 8.20초 기록의 원인을 발열/GIL/Wayland 중 하나로 단정하지 않음 |
| 6.17 | 오래 켜도 끊기지 않음 | 합성 600초 추가 결과는 아래; **실기 미검증·이전 정지 원인 미해결** |

### 7. 검증·PR·채팅의 작업 방식

| ID | 요구 | 판정·근거 |
|---|---|---|
| 7.01 | runtime_smoke에 비전/지도/telemetry 연결 | 기본 검사에 포함, --vision-only도 제공 |
| 7.02 | 생성/정상/오래됨/끊김/복구 | 별도 실제 GTK/SRT/UDP child 단계 단언 |
| 7.03 | 지정 시스템 Python/pytest 경로 | /usr/bin/python3·PYTHONPATH 지정, 실제 전체 실행 기록 |
| 7.04 | 변경 전 기준선·기존 실패 분리 | 최초 771 pass/11 fail/3 skip 및 단계별 실패 ID 비교 보존 |
| 7.05 | 일부러 고장 넣어 FAIL 확인 | 9종 각각 특정 원인 검출; 650 ms 정지는 신규 관찰 JSON에도 확인 |
| 7.06 | 실제 Gateway+L515 600초 | **미검증**: 접속 이름 해석 실패, 합성 600초로 완료 대체 금지 |
| 7.07 | 실제 camera FPS 기존 탭과 동일 | 합성 동일 약 15fps; 실제 비교 미검증 |
| 7.08 | 실제 화면 >0.5초 정지 없음 | 합성 통제 조건 관찰; 과거 8.20초 1회 미해결·실기 미검증 |
| 7.09 | 실제 두 지도 각각 2Hz | 합성 개별 paint 확인; 실제 미검증 |
| 7.10 | 실제 콘솔 CPU PR 기록 | 합성/실기 구분; 실제 CPU 아직 미측정, 수집 도구·절차 준비 |
| 7.11 | opt-in 두 서비스 켜는 명령 PR | 이번 검토에서 정확한 공백 구분 profiles/services 명령 추가; 로봇 미실행 |
| 7.12 | 기존 탭·PiP·카메라 왕복 회귀 | 로컬 GTK 정상/기존 fail ID 비교; 실제 탭 성능 미검증 |
| 7.13 | 카메라 중복 연결·Gateway 충돌 금지 | 로컬 생성 수/음성 대조 확인; 실제 충돌 없음은 미검증 |
| 7.14 | 미수신을 측정값처럼 금지 | 결측/오래된 encoder/잘못된 값/원시 자세 구분 및 0 조작 고장 검출 |
| 7.15 | PR 현재 GUI·재사용·범위 | PR 본문/1~4단계 보고서/보존 해시 |
| 7.16 | PR 실행·필요 데이터 | manual/integrated 명령·포트·criteria·encoder 계약 명시 |
| 7.17 | PR 동작 영상·성능 | 합성 WebM/PNG·AC 비교·미검증 표시; 실제 영상 없음 |
| 7.18 | PR 미연결·실기 미수행 명시 | floor mask/AS5048B/보정/실기/이전 정지 별도 명시·Draft 유지 |
| 7.19 | 채팅: 우선순위·한두 작업 단위·다음 요청 시 진행 | 단계 1~7 순차 기록, 이번은 사용자 요청 최종 검토 범위 |
| 7.20 | 채팅: 저장하고 재개 | 전 단계 커밋·별도 worktree·계획 파일·Draft PR로 인계; 이번 결과도 커밋/원격 저장 |
| 7.21 | 채팅: 남은 단계 수는 간단히 답변 | 실제 완료/미완료 기준은 본 문서; “7단계뿐=다른 요구 전부 완료”로 해석하지 않음 |
| 7.22 | 기본 smoke/전체 pytest green 여부 | 전체 green 아님: 기존 arm 관련 10개 실패를 유지하고 새 실패와 구분 |

## 실기 전/후 남은 작업의 구분

1. **콘솔 구현·도구:** 이번 보완과 로컬 검증까지 기록. 기존 로봇팔 10개 실패는 이 PR에서 계약을 바꿔 숨기지 않는다.
2. **데이터 계약/팀 확인:** 바닥 mask/polygon 공급, 장착 회전·좌표·부호·영점 합의, 정지 지도와 IMU 비교 기준. AS5048B는 현재 요청대로 연결 자리만 준비했고 실제 생산자는 없음.
3. **안정성 미해결:** 5단계 8.20초 draw 정지 원인. 최신 통제 조건 600초 정상 실행만으로 과거 실패를 해결 처리하지 않는다. 실제 조건에서 재현/계측해야 한다.
4. **실제 HW 인수:** 기존 Gateway 소유권/체크아웃 확인 → opt-in 송신 확인 → 실시간/비전 비교 → 실제 600초 영상/FPS/draw/maps/CPU/RSS/클럭·온도/끊김 복구/좌표·지연 → PR 증거 갱신.

## 새 실행 관찰 도구

`python -m operator_console.vision_monitor --metrics-output <새 JSON 경로>`는 정상 수동 app 진입점을 사용한다.
`--integrated`를 추가하면 정상 페어링 진입점을 사용한다. 기존 콘솔 대신 **한 개만** 실행한다.
매초 atomic JSON으로 기존 두 sink buffer, 두 video draw, GTK heartbeat, Top/Oblique의 서로 다른 그려진 raster를 기록한다.
탭별 최근 64구간만 보관하고 초과 구간 수를 기록하며, 미관측 지표는 null이다. 종료 시 closed 파일을 남긴다.
UI repaint Hz와 카메라 frame FPS는 서로 다른 지표이며 이 도구로 광학 end-to-end 지연을 주장하지 않는다.
CPU/RSS·AC/clock/temperature는 기존 sample_process로 같은 console_pid를 관찰한다.
구체적 실행과 인수 기준은 [실기 절차](2026-10-24-vision-tab-field-runbook.md)를 따른다.

## 최종 실행 결과

### 합성 600초 관찰

- 명령: `GDK_BACKEND=x11 OPENBLAS_NUM_THREADS=1 xvfb-run -a /usr/bin/python3 -m operator_console.tests.vision_benchmark --seconds 20 --vision-seconds 600 --require-ac --metrics-output /tmp/vision-finalreview-soak-events.json --output /tmp/vision-finalreview-soak.json`.
- AC 연결, X11/Xvfb, 1100×680 창, 지도 각각 518×115, 2×1280×720@15 SRT + 3 map tiles/12.5Hz + overlay/chassis 합성 입력.
  source/encoder는 별도 프로세스이고 콘솔 CPU에서 제외한다. 실시간 20초 → 비전 600초 → 실시간 20초를 같은 콘솔에서 관찰했다.
- 비전 **600.007초**, L515 sink **14.998 FPS**, Top/Oblique 개별 그려진 raster **각 1.997 Hz**.
  renderer worker 1199회/측정 구간이며, event probe는 전환 준비 시간을 포함한 603.021초를 기록해 count 1204개다.
- 최대 video draw **122.3 ms**, sink 간격 **144.3 ms**, GTK heartbeat **53.2 ms**. 이번 조건에서 카메라/GTK 0.5초 초과 정지는 관측되지 않았다.
- 지도 개별 최대 반영 간격 **607.8 ms**: 지도는 평균 약 2Hz이며 일정하게 매 500ms 이내를 보장한다고 주장하지 않는다.
  카메라/GTK 정지 조건과 별도 지표로 공개한다.
- 콘솔 CPU **87.79%**(논리 코어 하나=100%), 최대/종료 RSS **346.92 MiB**.
  실시간 첫 구간 종료 RSS 284.24 MiB → 비전 종료 346.92 MiB로 **62.68 MiB 증가**했다.
  하니스의 timeline 3160행·환경 표본 633개 등 측정 자료도 같은 프로세스에 누적되므로
  이 자료만으로 제품 메모리 누수나 안정된 plateau를 확정하지 않는다. **메모리 증가 원인/장기 안정성은 추가 확인 대상**이다.
- callback 예외 0, worker error 없음, 비전 후 실시간 구간 hidden map render 0.
  AC·CPU별 클럭·온도 원시 표본을 보관했다. 낮은 클럭만으로 thermal throttling이나 GIL 원인을 확정하지 않는다.
- 이번 실행의 `revision`은 실행 시작 당시 HEAD b918a8a이며 이번 검토의 미커밋 계측/설정 보완을 포함한다.
  지도·자세·계측 실행 소스 해시는 증거 manifest로 별도 기록한다.
- [원시 결과](2026-10-24-vision-tab-final-review-evidence/synthetic-600s.json),
  [개별 event 결과](2026-10-24-vision-tab-final-review-evidence/synthetic-600s-events.json),
  [실행 로그](2026-10-24-vision-tab-final-review-evidence/synthetic-600s.log).

이 결과는 기존 단기 AC 최적화 전/후 비교를 대체하거나 개선율을 입증하는 자료가 아니다.
5단계의 8.20초 정지·초기 네이티브 충돌을 해결 처리하지 않고 실제 Gateway+L515 인수도 계속 미검증으로 둔다.

### 회귀 검사

- 지정 경로 전체 콘솔 pytest: **871 passed / 10 failed / 3 skipped**, 276.88초.
  이전 단계의 실패 ID 10개와 정확히 같고 새 실패 0개다. 별도 skip 3개는 통과로 합산하지 않는다.
- 기본 `runtime_smoke`: 비전/지도 PASS, 기존 arm safe-disabled 표시 실패로 전체 FAIL.
- 비전 positive/고장 9종 모두 기대 결과 확인. 새 event 도구에서도 650ms 주입 정지가 검출됐다.
- `vision_monitor` 실제 수동 CLI를 X11에서 8초 실행하고 SIGTERM 종료:
  metrics closed, 영상 CONNECTING·지도/telemetry WAITING, 미수신 frame Hz null을 확인했다.
  통합 진입점은 routing 단위검사 및 없는 config의 실제 명시적 오류(exit 2)를 확인했다.
  실제 페어링/Gateway 세션은 미검증이다. 통합 진입점과 동일하게 정리 후 native exit guard를 유지한다.
- [회귀 비교](2026-10-24-vision-tab-final-review-evidence/test-comparison.json),
  [전체 로그](2026-10-24-vision-tab-final-review-evidence/full-tests.log),
  [기본 smoke](2026-10-24-vision-tab-final-review-evidence/runtime-smoke.log),
  [보존 검사](2026-10-24-vision-tab-final-review-evidence/source-boundaries.json),
  [고장 event](2026-10-24-vision-tab-final-review-evidence/negative-drawing-stall-events.json).

**실기 이전 검토는 수행했지만 요구 전체 완료는 아니다. 외부 계약/보정 합의·이전 정지 원인·메모리 증가 확인·실제 HW 인수를 열린 항목으로 유지한다.**
