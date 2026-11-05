# 비전 탭 1단계 — GUI·데이터 경로 조사 결과

2026-10-23, 기준 `a40ea170f871b86d4772763d55d3158ffcaabb29`.
범위: 첨부 작업 문서와 채팅 요구사항 확인, 브랜치 준비, 코드 경로 조사,
재사용 설계, 단계별 계획. 기능 구현·실기 검증은 이후 단계다.

## 저장소 확인

- GitHub PR #19 메타데이터: merged=true, base `feat/l515-terrain-map`, merge commit `a40ea17`.
  근거: <https://github.com/lightminn/power-train-sw/pull/19>.
- `git fetch --all` 후 origin/feat/l515-terrain-map = 위 커밋,
  origin/main = `6c7f8fb6d4ca771fc676f0d272bf16e003a5c0df`.
  `git merge-base --is-ancestor a40ea17 origin/main` exit 1: main에 포함되지 않음.
- 새 worktree `/home/kimseon/power-train-sw-vision`, 브랜치 `agent/vision-tab-20261023`.
  시작점과 향후 PR base 모두 `feat/l515-terrain-map`.
- 기존 power-train-sw와 power-train-sw-l515에는 미커밋/미추적 작업이 있어 보존했다.
  기존 로컬 GUI 수정본과 원격 기준 브랜치를 섞지 않았다.
  따라서 아래 현황은 지정 기준 커밋의 GUI이며 기존 폴더의 최신 미커밋 GUI 전체를 뜻하지 않는다.
- AGENTS.md의 Jetson 체크아웃 확인 지침에 따라 읽기 전용 SSH를 시도했으나
  `ssh -o BatchMode=yes -o ConnectTimeout=5 jetson ...`은 이름 해석 실패(exit 255).
  Jetson 미커밋 상태·설치 버전·현재 실시간 데이터는 미확인이다.

## 현재 GUI와 실제 코드 연결 범위

| 탭 | 구현된 화면·기능 | 연결 데이터 / 근거 |
|---|---|---|
| 실시간 화면 | L515 메인, D435 PiP, 화면 전환, 객체·거리·지형 표시 옵션, 상태·이벤트 | app.py VideoPanel, TerrainCanvas, swap_camera_views; L515 SRT :5000, D435 :5002, 검출 :5003, 지형 :5009 |
| 협조구동 | Roll/Pitch 숫자, 6륜 그림·조향·속도·고장·운용 상태 | mobility_view.py MobilityDashboard.update, RoverCoordinationGraphic; self._chassis_receiver :5005 |
| 시스템 상태 | 하위 시스템·수신·진단 상태 | status_view.py, app.py _refresh_health / _refresh_diagnostics; 기존 상태 수신기 공유 |
| 지형 지도 | Top 지도, 음영·클래스 색·범례, 궤적, 따라가기, 줌·끌기, 로버 화살표, 끊김, 기준면·정지 기울기 | terrain_map_panel.py MapPanel; self._map_receiver :5010 |
| 로봇팔 수동 | 수동 조작·툴 관련 UI | app.py ARM_MANUAL_TAB_NAME; 기존 arm telemetry :5007 및 게이트된 ops 콜백 |
| 로봇팔 보정 | 캘리브레이션·jog UI | app.py ARM_CALIBRATION_TAB_NAME; 기존 ArmOpsCallbackAdapter |

탭 등록은 app.py Gtk.Stack 부근(기준본 4412행 이후)이다.
위 표는 코드가 지원하는 수신 경로이며 이번 조사에서 실물 송신·수신을 확인한 표가 아니다.
환경 센싱 수신기도 존재하지만 이 기준본에 독립 환경 탭이 등록되지는 않는다.
비전 탭과 Oblique 지도는 현재 등록돼 있지 않다.

## 재사용 조사와 범위 조정

| 기존 기능 | 재사용 방법 | 추가 작업 |
|---|---|---|
| VideoPanel + TerrainCanvas | 기존 L515 인스턴스 전체 재배치 | 탭 진입·이탈 시 부모·PiP 복원, 공유 토글 |
| pipeline_description | srtsrc→demux→parser→avdec_h264(max-threads=1)→videoconvert→gtksink | sync=false 보존, 최신 프레임·정지 계측 |
| LatestMapReceiver | 동일 self._map_receiver 전달 | 새 소켓 없이 두 뷰가 snapshot 소비 |
| MapRenderer / map_rgba | 작업 스레드의 전용 캐시 인스턴스로 호출 | 준비된 이미지의 GTK 표시·로버 화살표·조작 |
| MapPanel 상수·note_and_legend·surface_from_rgba | import해서 사용 | 기존 MapPanel의 메인 스레드 계산 방식은 수정하지 않음 |
| live_map의 tmap_local_grid·classify_heights·filter_hazards·cell_colors·oblique_image | 기존 알고리즘을 사용 | cv2/import 호환, 무효 자세 방어, 격자 크기 제한·작업 스레드 |
| criteria.load_criteria | cv2 없이 읽기 가능 | CLI/설정/환경 경로, 프로필·버전·SHA256 UI |
| gm_features.KText | 캐시 접근법 참고 | GTK PangoCairo용 제한 캐시; KText 자체는 cv2 의존 |
| RoverCoordinationGraphic | 개략 차체·바퀴 그림 재사용 가능 | 비전용 읽기 전용 래퍼·원시 IMU 별도 숫자 |

기존 협조구동 숫자는 원시 roll/pitch를 Roll/Pitch로 표시한다. 차체 그림 투영은
시점 yaw를 사용하며 roll/pitch에 따른 기울기 표현은 없다. 원시값 명명 문제는
해당 팀원 파일을 수정하지 않고 이 문서 및 향후 PR에 남긴다.
별도 3D 모델·지도 생성기·카메라 수신기를 새로 만들지 않는 범위로 조정했다.
네 화면 동시 표시는 유지한다.

## 구현 전 확인할 차이

1. **바닥 영역 미전송**: terrain_overlay.contract.TerrainFrame에는 ground 상태와 장애물
   polygon만 있다. 실제 바닥 칠하기는 현재 수신 계약으로 불가능하다.
   기존 상태 배지·장애물 윤곽을 재사용하되 바닥 영역 요구는 미충족으로 남긴다.
2. **자세 보정 미확인**: imu_tilt_node → /imu/filtered → chassis_telemetry_sender_node
   quaternion→roll/pitch→:5005 경로를 확인했다. mount rotation.w=1.0,
   메시지 frame_id는 base_link다. 장착각·부호·영점이 확인된 차체 자세로 해석하지 않는다.
3. **신선도는 필드별**: TelemetrySnapshot에는 패킷 수신 시각,
   WheelStatus에는 stale가 있다. snapshot 전체의 stale 속성은 없다.
   송신기는 1 s 지난 IMU를 null로 만든다. 패킷 LIVE와 자세 유효는 다르다.
4. **정지 측정**: plane_tilt에는 age_ms가 있고 gravity는 정지 창 측정이다.
   IMU 원시값·지도 기준면·중력 기준 차체각을 같은 수치로 표시하지 않는다.
5. **지도 색 의미**: 기존 Top의 송신 클래스와 v0.2 표시용 정책 분류는 다르므로
   각각의 범례·기준 출처를 구분한다. 로컬 기준값 교체가 송신 지도 판정을 바꾸지는 않는다.
6. **의존성**: 문서에는 시스템 Python에 numpy가 있다고 적혔지만 이 PC에서는 없다.
   cv2와 pytest도 없다. 기존 함수의 sys.path 가정을 포함해 다음 단계에 해결한다.

## 기준선 실행 시도

코드를 바꾸기 전 시스템 Python 3.12.3에서 실행했다. 기능 실패와 환경 실패를 구분한다.

| 검사 | 결과 | 판정 |
|---|---|---|
| 지정 PYTHONPATH로 /usr/bin/python3 -m pytest operator_console -q | No module named pytest | 실행 불가; 통과·실패 개수 없음 |
| /usr/bin/python3 -m operator_console.runtime_smoke | CONSOLE-RUNTIME-SMOKE: FAIL; console exited early(rc=1) | 기준선 기동 실패, 실제 화면 검증 미수행 |
| /usr/bin/python3 -m operator_console.app --help | terrain_map.py import numpy에서 ModuleNotFoundError | 콘솔 기동 차단 원인 확인 |
| importlib.util.find_spec | gi/cairo 있음, numpy/pytest/cv2 없음 | 환경 상태 |
| 실행 도구 | xvfb-run, gst-inspect-1.0 있음 | 개별 GStreamer 플러그인 가용성은 별도 확인 필요 |

원본 stdout/stderr는 [증거 폴더](2026-10-23-vision-tab-evidence/)에 보관한다.
임의 패키지 설치나 실기 연결 없이 기준선 장애를 기록했다.
환경 준비 후 기준 커밋에서 다시 검사해야 회귀 비교가 가능하다.

## 이번 완료와 남은 검증

완료: 문서 확인, PR/브랜치 확인, 별도 작업 브랜치 생성, 현재 코드 경로 조사,
수정 금지 범위와 재사용 설계, 순차 작업 계획, 기준선 실행 시도 및 환경 장애 기록.
기능 코드 변경 없음. 광민 지도·협조구동·시험대 코드 변경 없음.

미검증: Jetson 상태, 실제 데이터 수신, 좌표·시간 정합, 보정값, 영상 FPS·멈춤,
2 Hz 지도, CPU/RSS/클럭·전원 조건, 10분 연속 운용, 기존 탭 실기 회귀.
비전 runtime_smoke 추가·고장 음성 대조·동작 녹화·PR 작성도 아직 수행하지 않았다.

다음 차례는 [순차 계획](../plans/2026-10-23-vision-tab.md)의 2단계다.
