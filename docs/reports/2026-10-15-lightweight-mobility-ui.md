# 협조구동 3D 경량화 및 가독성 개선

원본 URDF 관절 계층·좌표·회전축을 유지하고 시각 형상을 단순 차체, 색으로 구분한 로커/보기, 관절, 바퀴와 모터로 교체했다. 로봇팔·볼트·배선은 표시하지 않는다. 원본 변환 CAD 자산은 참조용으로 보존하되 런타임에 읽지 않는다. 표시 삼각형은5,871,120→1,368, 실제 CAD 메시 요청은0건이다.

픽셀비1, 저비용 Lambert 재질, 최대20fps 움직임, 정지 시 변경에만 렌더링, 동일 수신 상태 전달 병합을 적용했다. 줌·팬·두 손가락 조작은 끄고 고정 고도에서360° 회전한다. 모터 모드만 바뀌어도 상태 색을 다시 그린다.

Noto Sans CJK KR 기본14px, 제목16~23px를 적용했다. 수신·안전 상태는 유지하며 불필요한 기술 상세와 중복 지표를 숨겼다. 바퀴 팝업은 모터 상태·전류(A)·회전속도(rpm)·조향각만 표시한다.

## 검증

- 실제 GUI 격리 UDP→두 자세→바퀴 클릭→수신 중단 검사 PASS. 관절 입력만 바꿔 앞왼쪽 바퀴 위치 약120mm 변화.
- 관련 파서·수신/제어 경계·협조구동 재배치 검사12개 PASS. `git diff --check` PASS.
- 1100×680 페이지559/559px, 1600×900 페이지779/779px로 기본 화면 세로 넘침 없음.
- 실제 GTK 글꼴 Noto Sans CJK KR,14px 확인. 휠 이벤트 전후 카메라 거리 동일, 반 바퀴 회전 위치 변화 확인.
- Xvfb 소프트웨어 렌더링30프레임 측정: 중앙값2ms,95백분위10ms,46 draw calls. 이 수치는 실제 영상 동시 구동의 장시간 성능 보장은 아니다.
- 필수 전체 `operator_console.runtime_smoke`는 기존 arm safe-disabled 판정에서 계속 실패. 기존 CAD 작업에서 변경 전 동일 실패를 재현했으며, 전체 검사 통과로 보고하지 않는다.
- 새 더미 시연 GUI를 데스크톱에서 재기동하고 접근성 트리에서 협조구동/6륜 제어 상태/더미 시연 표시를 확인했다.

[성능·입력·글꼴 검사](2026-10-15-rover-3d-evidence/lightweight-performance.json), [실제 GUI 스모크](2026-10-15-rover-3d-evidence/lightweight-smoke.log), [전체 런타임 결과](2026-10-15-rover-3d-evidence/lightweight-runtime-smoke.log), [1100×680 화면](../ui_screenshots/2026-10-15-mobility-fit-1100x680.png).

## 후속 형상 보완

사용자가 지나친 단순화를 지적해, 기존 경량 골격에 둥근 로커·보기 파이프, 원통형 조향부, 차체 덮개·패널·프레임, 바퀴 트레드·스포크·허브와 원통형 모터를 추가했다. 반복 트레드/스포크/볼트는 instanced mesh로 묶었다. 원본 관절 위치와 축, 더미 동작, 표시 정책과 줌 차단은 유지한다.

삼각형6,440개/83 draw calls/CAD 메시 요청0건이다. Xvfb 소프트웨어 렌더링30프레임의 중앙값2ms·95백분위3ms이며 실제 장시간 영상 동시 구동 수치는 아니다. 1600×900 페이지779/779px, 휠 이벤트 전후 카메라 거리 불변과 반 바퀴 회전을 확인했다. 실제 격리 UDP GUI 스모크 PASS: 두 자세의 관절·바퀴 위치 변화, 전류3.50A/속도12rpm 팝업, 단절 전이를 재검증했다. 전체 필수 런타임 검사는 기존 로봇팔 safe-disabled 판정에서 동일 실패다.

현재 팝업 상태: `정상`은 최신 수신·오류 플래그 없음, `오류`는 구동 오류/조향 오류/FAULT 모드, `갱신 지연`은 해당 바퀴 표본 stale, `미연결`은 차체 수신 단절 또는 바퀴 정보 없음이다. 구동 활성/대기 모드는 별도 항목으로 노출하지 않는다.

[형상 보완 화면](../ui_screenshots/2026-10-15-mobility-refined-shape.png), [경량 성능·회전 검사](2026-10-15-rover-3d-evidence/refined-shape-performance.json), [관절·팝업·단절 검사](2026-10-15-rover-3d-evidence/refined-shape-smoke.log), [전체 런타임 검사](2026-10-15-rover-3d-evidence/refined-shape-runtime-smoke.log).

## 180° 회전 시 재질 차이 수정

수정 전 실제 WebKit 0°/180° 캡처에서 반대면이 크게 어두워지고 원통 면이 각져 보이는 현상을 재현했다. 텍스처 교체 로직은 없으며, 강한 고정 편측 DirectionalLight, Y-up인 반구광 기본 위치, 전체 flat shading이 원인이었다.

카메라를 따라가는 주/보조 DirectionalLight와 주변광을 사용하고 반구광을 Z-up으로 맞췄다. 원통의 기본 vertex normal로 부드러운 면을 복원했다. 동일 자세의0°/180° 재캡처에서 차체·바퀴·파이프가 비슷한 밝기/질감으로 보임을 확인했다. 실제 앞뒤 구조 차이는 유지한다.

삼각형6,440/83 draw calls/CAD 요청0, Xvfb 소프트웨어30프레임 중앙값3ms·95백분위약5ms다. 줌 차단과 회전도 유지한다. UDP→관절→바퀴 클릭→단절 검사는 PASS. 전체 런타임은 기존 arm safe-disabled 실패가 계속 남아 있다. 데스크톱 더미 시연을 재기동했다.

[수정 전0°](../ui_screenshots/2026-10-15-lighting-before-0.png), [수정 전180°](../ui_screenshots/2026-10-15-lighting-before-180.png), [수정 후0°](../ui_screenshots/2026-10-15-lighting-after-0.png), [수정 후180°](../ui_screenshots/2026-10-15-lighting-after-180.png), [성능·회전 결과](2026-10-15-rover-3d-evidence/lighting-after.json), [GUI 스모크](2026-10-15-rover-3d-evidence/lighting-smoke.log).

## 원본 CAD 부품 형상 복원 (최종)

추가 디테일 요청에 따라 도형으로 재구성한 모델을 원본 CAD 기반 표시 LOD로 교체했다. 원본 차체 패널·프레임·마운트, 로커/보기 관절 하우징, 조향 실린더/브래킷, 바퀴·모터 외형과 연결 부품20개를 사용한다. 로봇팔/도구는 표시하지 않는다. 기존 URDF 계층/피벗/회전축, 카메라 기준 조명, 360° 회전·줌 차단, 변경 시 렌더링과20fps 제한은 유지한다. 모터 상태는 원본 허브 위의 작은 색상 링으로 표시한다.

오프라인 `scripts/build_rover_display_lod.py`는 원본 참조 자산을 보존하면서 별도 `display-meshes`를 생성한다. 0.05mm 이음부 병합 후 연결된 부품을 나누어 각각1mm 표시용 클러스터/QEM을 적용한다. 4mm 미만 미세 부품은 생략한다. 원본 전체를 단번에 줄일 때 작은 하우징/축이 사라지는 것을 피하려고 부품별 예산과 경계 손실 확인을 적용했다. 최종 빌드 실행 PASS.

자산20개239,650삼각형/4,148,356bytes, 상태 링 포함241,090삼각형/26 draw calls다. 원본 파일 SHA-256과 부품별 개수·용량·축별 경계 차이는 `display-mesh-manifest.json`에 기록했다. 최대 경계 차이는 약3.62mm이며 모든 표면의 오차 측정은 아니다. 런타임은 경량 파일20개만 읽고 기존 무거운 참조 메시를 읽지 않는다.

검증 결과:

- 실제 GUI 격리 UDP→두 관절 자세→바퀴 클릭→수신 중단 검사 PASS. 1100×680 콘텐츠559/559px.
- 1600×900 콘텐츠779/779px, 줌 이벤트 전후 거리 불변, 반 바퀴 회전 확인.
- 실제 WebKit0°/180° 캡처로 원본 부품과 양쪽 조명 확인.
- Xvfb 소프트웨어 렌더링30프레임 중앙값2ms·95백분위4ms. 실제 장시간 영상 동시 구동 성능을 뜻하지 않는다.
- 전체 필수 runtime_smoke는 이전부터 있던 arm safe-disabled 판정에서 계속 FAIL. 전체 GUI 검사 통과로 표시하지 않는다.
- 데스크톱 더미 시연 GUI를 새 CAD LOD로 재기동했다. 실측이 아니라 더미 수신 상태 표시는 유지한다.

[원본 CAD 기반 화면](../ui_screenshots/2026-10-15-mobility-cad-lod.png), [기본 방향](../ui_screenshots/2026-10-15-cad-lod-views-0.png), [180° 방향](../ui_screenshots/2026-10-15-cad-lod-views-180.png), [변환 로그](2026-10-15-rover-3d-evidence/display-lod-build.log), [성능·창 맞춤 검사](2026-10-15-rover-3d-evidence/cad-lod-performance.json), [GUI 수신 스모크](2026-10-15-rover-3d-evidence/cad-lod-smoke.log).

## 최종 표면 정리와 줌 차단 강화

사용자의 약간만 다듬기 요청에 따라 CAD 기반 부품 형상과 관절은 유지했다. 6mm 미만의 독립 미세 부품을 생략하고, 표시 면은45° 임계 법선 보간으로 작은 삼각형 이음을 완화했다. 큰 패널/브래킷 모서리는 분리된 법선으로 유지한다. 법선은 자산 로딩 때1회 계산한다.

카메라 최소/최대 거리를 같게 고정하고 캔버스 wheel 기본 동작을 막았다. OrbitControls의 줌/팬 비활성과 두 손가락 제스처 비활성도 유지한다. 실제 WebKit에서 휠 확대/축소 방향 및 Ctrl+휠 후 카메라 거리가 모두 동일함을 확인했다. 고정 고도360° 회전은 유지한다.

표시 자산237,036삼각형/4,097,500bytes, 상태 링 포함238,476삼각형/26 draw calls. Xvfb30프레임 중앙값2ms·95백분위2ms이며 실물 영상 동시 장시간 부하 측정은 아니다. 실제 UDP→관절→바퀴 클릭→단절 검사 PASS, 1100×680 콘텐츠559/559 및1600×900 콘텐츠779/779. 전체 runtime_smoke는 기존 arm safe-disabled 판정에서 동일 실패다.

새 더미 시연 GUI를 재기동했다. [기본 방향](../ui_screenshots/2026-10-15-polished-cad-views-0.png), [반대 방향](../ui_screenshots/2026-10-15-polished-cad-views-180.png), [줌 차단·성능 검사](2026-10-15-rover-3d-evidence/polished-cad-performance.json), [관절·팝업 검사](2026-10-15-rover-3d-evidence/polished-cad-smoke.log), [변환 로그](2026-10-15-rover-3d-evidence/polished-cad-build.log).

## 푸시 스냅샷 검증

원격 `agent/operator-console-live-telemetry`가 로컬보다12커밋 앞서 있어 `f6073d2`를 기준으로 별도 작업 트리에서 이번3D 기능만 반영했다. 원격의 조향/로봇팔/ops 변경은 유지하고, 로컬의 다른 미커밋 테마·장애물 채널·로봇팔 작업은 포함하지 않았다. 확대 글꼴에서 탭 최소 너비가 늘지 않도록 기존 짧은 로봇팔/도구 탭 제목을 유지했다. 더미 진입점은 기존 원격 수신 채널만 사용하도록 독립시켰다.

푸시 대상에서 관련 파서/송신표면/app 검사120개 PASS. 실제 GUI 격리 UDP→관절→팝업→단절 검사는 PASS이며1100×680 창/페이지500/500px다. 원격 UI의 기존 헤더 배치 때문에 로컬 테마 캡처와 가용 높이는 다르지만, 전체 탭이 창 안에 들어간다. 전체 runtime_smoke는 기존 arm safe-disabled 판정에서 동일 실패다. 원래 작업 폴더의 미커밋 변경과 실행 중인 GUI는 보존했다.

[푸시 대상3D 검사](2026-10-15-rover-3d-evidence/push-snapshot-rover-smoke.log), [관련120개 검사](2026-10-15-rover-3d-evidence/push-snapshot-tests.log), [전체 런타임 결과](2026-10-15-rover-3d-evidence/push-snapshot-runtime-smoke.log).
