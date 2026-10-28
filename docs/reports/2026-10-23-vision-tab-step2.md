# 비전 탭 2단계 — 노트북 Graphite GUI와 단일 L515 영상 공유

기반 브랜치는 `feat/l515-terrain-map`의 `a40ea17`, 작업 브랜치는
`agent/vision-tab-20261023`이다. PR 대상은 계속 `feat/l515-terrain-map`이다.
이번 범위는 의존성 준비·디자인 기반 이식·탭 뼈대·영상 재배치까지다.
지도 두 뷰와 자세의 실제 연결은 다음 단계이며, 이번 결과로 완료 처리하지 않는다.

## 사용자 지정 노트북 GUI 이식

사용자가 `/home/kimseon/power-train-sw`의 미커밋 Graphite GUI를 기준으로 지정했다.
원본 폴더는 수정하거나 checkout·stash하지 않고 필요한 변경을 별도 작업 트리에 복사했다.

- Graphite 회색 표면·초록 강조·글꼴·하단 탭·가로 상태 도크·축약 문구를 가져왔다.
- 기존 협조구동·시스템 상태·3D 차체·3D 팔을 가져왔다.
  `mobility_view.py`, `status_view.py`, `themes.py`, `telemetry.py`는 해당 노트북 파일과 동일하다.
  협조구동 파일에 새 비전 기능을 덧붙이지 않았다.
- 기반 브랜치의 지형 수신·지도·오버레이 코드는 그대로 유지했다.
  `terrain_map/`, `operator_console/terrain.py`, `terrain_map.py`, `terrain_map_panel.py`,
  `lidar_dev/`는 시작점 대비 변경이 없다.
- app.py는 3-way 병합 후 충돌을 수동 해소했다. 원본의 별도 장애물 :5009 수신기는
  생성하지 않는다. 기존 지형 :5009 수신기와 포트·패킷 의미가 충돌하기 때문이다.
  협조구동의 데이터 모델 import를 위해 obstacle_telemetry.py는 가져왔지만 별도 수신 인스턴스는 없다.
- 기존 '지형 보기' 버튼과 `--show-terrain-preview`는 비전 탭을 연다.
  HTTP 시험대/추가 카메라 연결을 열지 않는다.
- 3D 실행에 쓰는 `display-meshes`, 모델·뷰어·Three.js·글꼴을 복사했다.
  실행 중 요청하지 않는 약 102 MB의 원본 `meshes/`는 복제하지 않았다.
  가져온 3D 실행 자료는 약 6.9 MB이고 실제 WebKit 스모크로 로딩을 검증했다.

[가져온 파일·원본/결과 SHA256](2026-10-23-vision-tab-step2-evidence/laptop-gui-import-sha256.json)에
출처를 남겼다. app.py는 통합본이며, 가져온 3D 스모크는 수신 포트 인자를 바꿨고,
협조구동 검사의 오래된 문구 기대값 하나는 현재 축약 문구로 맞췄다.
원본 폴더의 해당 파일들이 이식 중 바뀌지 않았음을 SHA256으로 확인했다.

## 비전 뼈대와 영상 소유권

`operator_console/vision_panel.py`에 카메라·차체 자세·Top·Oblique 네 영역을 배치했다.
페이지 자체는 수신기·디코더·타이머를 만들지 않는다.
지도·자세 슬롯은 '표시 준비 중'으로 표시하며 숫자·지형·정상 상태를 만들어 넣지 않는다.

비전 진입 시 기존 `self._l515` VideoPanel을 TerrainCanvas까지 통째로 이동한다.
실시간 화면의 원래 부모와 화면 전환 버튼 상태를 저장하고, 비전에서 나갈 때 복원한다.
L515가 메인인 경우와 D435가 메인이고 L515가 PiP인 경우를 모두 지원한다.
다른 탭으로 나가도 원래 부모에 복원하며, 실시간 화면 복귀 후 화면 전환도 유지한다.
비전에서는 L515 고정이므로 V/화면 전환은 동작하지 않는다.

기존 GStreamer pipeline/sink/영상 widget과 지도·차대 수신기를 그대로 공유한다.
탭 전환에 `Gst.parse_launch`를 추가 호출하지 않으며 `sync=false`도 변경하지 않았다.
지형 체크 상태는 기존 overlay_view_state와 양방향 동기화한다.
노트북 GUI의 D435 표시 옵션 기본 꺼짐·기존 숨김 배치도 유지한다.

`:5009`는 실제 바닥 마스크가 없으므로 바닥 상태 배지와 장애물 윤곽만 가능하다.
녹색 바닥 영역 칠하기·좌표/시간 정합 실기 검증은 여전히 미완료다.

## 실행 환경과 실행법

시스템 Python 3.12.3에 gi/cairo와 필요한 GStreamer 플러그인이 있음을 확인했다.
관리자 권한 없이 pip의 다른 Python 지정 기능으로 사용자 영역에 의존성을 설치했다.
설치 결과는 numpy 2.5.4, OpenCV-headless 5.0.0(배포판 버전 5.0.0.93), pytest 9.1.1이다.
RealSense SDK나 카메라 직접 접근 모듈은 설치하지 않았다.
OpenCV는 다음 지도 단계에서 기존 함수 재사용을 위한 준비이며 현재 비전 페이지는 import하지 않는다.

```bash
# 운용 PC, 작업 트리
cd /home/kimseon/power-train-sw-vision
/usr/bin/python3 -m operator_console.app --host <Gateway-IP> --page vision

# 기존 페어링/인증 설정이 준비된 통합 진입점
/usr/bin/python3 -m operator_console --config ~/.config/powertrain/operator.json --page vision
```

새 페이지 옵션은 기본 실행 경로를 바꾸지 않는다. 시스템 서비스·Gateway·모터·센서는
기동/정지/설정하지 않았다. Jetson은 여전히 이름 해석 실패로 접속하지 못했다.

## 검증과 기준선

의존성 준비 후 기능 변경 전 기준 커밋의 콘솔 검사는 **771 passed / 11 failed / 3 skipped**였다.
실패는 로봇팔 계약·과거 커밋 기준 변경 경계·툴 표시·runtime_smoke이며
[원본 로그](2026-10-23-vision-tab-step2-evidence/baseline-pytest.log)에 남겼다.
기준선 runtime_smoke도 `arm tabs did not render live tool in safe disabled state`로 실패했다.

새 검사는 실제 로컬 `videotestsrc → H.264 → MPEG-TS → SRT → 기존 콘솔 디코더 → gtksink`
경로를 사용한다. 로봇 카메라를 열지 않는다.

- L515 메인 / D435 메인 두 경우 각각 3회 비전 왕복.
  각 왕복에서 실제 디코딩 프레임 수신이 전후로 이어짐을 확인했다.
- 기존 pipeline/sink/영상 widget·오버레이 부모·수신기 identity 유지,
  새로운 pipeline 생성 금지, 네 슬롯 동시 map, 원래 부모·MAIN/PiP 복원,
  다른 페이지 경유·복귀 후 화면 전환을 확인했다.
- 지형 토글 양방향 공유, '지형 보기'의 비전 이동, 별도 HTTP 창과 장애물 수신기 부재를 확인했다.
- 영상 이동 함수를 메모리에서 비활성화한 음성 대조는 실제 카메라 부모 검사에서 FAIL했다.
  [음성 대조](2026-10-23-vision-tab-step2-evidence/vision-negative.log).
- 기존 3D 스모크는 실제 GTK/WebKit/UDP 경로에서 PASS했다.
  차체/팔·바퀴 팝업·회전·줌 차단·미수신/stale·1100×680 배치도 확인했다.
  [3D 스모크](2026-10-23-vision-tab-step2-evidence/rover-3d-smoke.log).
- 필수 runtime_smoke는 지도 LIVE/STALE 검사를 통과한 뒤 기준선과 같은 arm safe-disabled
  검사에서 실패했다. 전체 콘솔이 green이라고 판정하지 않는다.
  [필수 스모크](2026-10-23-vision-tab-step2-evidence/runtime-smoke.log).

초기 전체 실행에서 기존 GTK 전역 상태와 실영상 테스트를 한 프로세스에서 돌리던 중
native abort가 발생했다. 원인을 확정하지 않고 새 SRT 테스트를 격리 Xvfb 자식 프로세스로
옮겼다. 제품 수신기를 늘리지 않고 검사 프로세스만 분리했다.
최종 격리 Xvfb 전체 콘솔 검사는 **797 passed / 10 failed / 3 skipped**였다(176.79 s).
남은 실패 ID 10개는 모두 기준선 실패 ID에 포함된다. 이번 최종 실행에 새 실패 ID는 없다.
로봇팔 과거 커밋 기준 변경 경계 3건·계약/게이트 6건·runtime_smoke 1건을
해결하거나 건너뛰지 않았다. 기준선 실패 중 tool-binding 한 건은 최종 실행에서 통과했지만,
로봇팔 실기 또는 전체 안정성 해결을 뜻하지 않는다.

```bash
PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
  GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest operator_console -q
/usr/bin/python3 -m operator_console.runtime_smoke
GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m operator_console.rover_3d_smoke
```

새 비전 검사는 3건 모두 통과했고 3D 상태 변환 단위검사 11건도 통과했다.
기존 GUI 검사는 새 탭 목록·Graphite 표면색·축약 문구 기대를 실제 요구에 맞춰 갱신했다.
통합 기본 진입점은 기본 페이지에서 불필요한 stack 접근을 하지 않도록 유지했다.
전체 검사에서 callback traceback이나 native abort가 재발하지 않았으며,
기존 로봇팔 판정 실패는 계속 기록했다.

- [최종 전체 검사](2026-10-23-vision-tab-step2-evidence/final-pytest.log)
- [기준선 대비 실패 ID 비교](2026-10-23-vision-tab-step2-evidence/test-comparison.json)
- [비전·3D 상태 변환 검사](2026-10-23-vision-tab-step2-evidence/vision-and-rover-tests.log)

증거 로그는 출력의 줄 끝 공백과 마지막 빈 줄만 정리했다. 가져온 글꼴 라이선스와
Three.js 파일은 원본 공백을 포함해 보존했다.

## 화면 증거

1100×680에서 실제 GTK 창을 캡처했다. 영상은 로컬 합성 컬러바이며 로봇 실기가 아니다.

- [비전](2026-10-23-vision-tab-step2-evidence/vision-graphite.png)
- [실시간 화면](2026-10-23-vision-tab-step2-evidence/mission-graphite.png)
- [협조구동](2026-10-23-vision-tab-step2-evidence/mobility-graphite.png)

## 다음 단계·미검증

다음은 Top/Oblique 지도 연결, 작업 스레드의 각 2 Hz 렌더링, 기준값 경로/프로필,
지도 조작·로버 화살표·범례 통합이다. 이번에는 계산이나 이미지 생성기를 붙이지 않았다.

비전 자세 연결·보정 확인·AS5048B 필드 계약·실제 송신은 미완료다.
노트북 GUI에서 가져온 기존 3D·협조구동은 원시 IMU를 사용하므로 장착 보정된 차체 자세로
인수하지 않는다. 비전 자세는 후속 단계에서 별도로 원시/보정/정지 측정을 구분한다.

Gateway+실제 L515의 FPS·지연·프레임 정지 0.5 s·CPU/RSS/발열·클럭·전원 조건,
각 지도 2 Hz와 10분 연속 검증은 수행하지 않았다. 실물 동작 영상도 아직 없다.
push·PR 생성·배포는 수행하지 않았다.
