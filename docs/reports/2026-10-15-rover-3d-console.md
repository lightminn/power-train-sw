# 차체 3D 탭 적용 결과

2026-10-15 사용자가 여유 공간 재확인·결과 저장·최종 GUI 실행을 요청해 적용했다. 시작 시 사용자 가용 공간은 약185MiB였으며 기존 사용자 파일과 미커밋 변경을 보존했다. GitHub fetch/PR 확인을 수행했고 Jetson은 mDNS 이름 해석 실패로 실물 접근을 하지 못했다.

**현재 GUI:** `차체 3D` 탭을 추가해 운영 콘솔에서 열었다. 드래그 회전, 휠 확대, 기본 시점, 바퀴 선택 및 모터 상태 팝업을 사용할 수 있다. 카메라·다른 탭과 기존 안전 조작은 같은 콘솔의 경로를 사용한다. 실물 미연결 수치는 `미연결`로 표시한다.

3D 표시 자산은 `operator_console/assets/rover3d/`에 저장했다. 원본 표시 STL 323.93MiB를 인덱스 메시 101.59MiB로 줄였다. 관절 76개와 링크77개를 유지한다. 삼각형은6,793,332개에서5,871,120개다. 간소화로 외곽 변화가2mm를 넘은 부품에는 정리한 원본 면을 유지해 형상 손실을 피했다. 임시 변환 도구는 numpy2.5.4/fast-simplification0.2.0이며 런타임에는 필요 없다. 원본별 SHA-256과 최종 크기·개수는 `mesh-manifest.json`에 기록했다. 아직 정밀 경량화·장시간 렌더링 부하 검증은 남아 있다.

현재 실행기는 `/usr/bin/python3 -m operator_console --page rover3d`를 지원한다. 사용자 설정·인증 토큰은 기존 파일을 사용한다. 현재 실행 창은 이미지 저장을 위한 콜백만 감싼 동일한 통합 진입점이며, 시험 UDP 송신기는 연결하지 않았다.

**검증:**

- `operator_console.rover_3d_smoke`: PASS. 실제 OperatorConsole의 격리 UDP 수신기를 거쳐 네 관절각을 적용하고, 앞왼쪽 바퀴 선택 핸들러로 Iq3.50A·Hall 기반0.20rev/s 팝업을 확인했다. 송신 중단 후 수치가 미연결로 바뀌고 마지막 자세를 유지함도 확인했다. 이 수치는 명시적 시험값이며 실물 측정이 아니다.
- 신선도·잘못된 관절각 검사, 송신 표면 계약, app 및 통합 GTK 검사:141 passed /1 failed. 실패는 `test_overlay_controls_stay_visible_while_laptop_telemetry_scrolls`이며 이번 변경 이전 코드에서도 동일하게 재현됐다. 현재 기존 화면 표시 옵션이 숨겨진 상태와 테스트 기대가 다르다.
- 필수 `operator_console.runtime_smoke` 실행: 기존 arm-tab safe-disabled 판정에서 FAIL. 이번 변경을 제거한 메모리 오버레이에서도 같은 판정 실패를 재현했다. 이 작업에서 무관한 로봇팔 동작이나 기존 화면 표시 옵션을 수정하지 않았다. 전체 GUI 스모크 통과로 선언하지 않는다.
- 실제 데스크톱에서 운영 콘솔 창 SHOWING 확인. WebKit의 실모델 화면 및 앞왼쪽 상태 팝업을 저장했다. 로그에 콜백 traceback이 없으며 진단용 WebKit GI 버전 경고와 Xlib DRI2 경고는 있었다.

**저장된 근거:**

- [현재 운영 GUI의 3D 화면](../ui_screenshots/2026-10-15-rover-3d.png)
- [초기 검토 문서](2026-10-15-rover-3d-evidence/검토결과.md)
- [실제 GUI UDP/3D/팝업/단절 검사](2026-10-15-rover-3d-evidence/integrated-3d-smoke.json)
- [GUI 테스트 출력](2026-10-15-rover-3d-evidence/gui-tests.log)
- [변경 전 화면 표시 옵션 검사](2026-10-15-rover-3d-evidence/baseline-overlay-test.log)
- [변경 전 전체 런타임 검사](2026-10-15-rover-3d-evidence/baseline-runtime-smoke.log)

`2026-10-15-rover-3d-evidence/`의 최초 검토 문서는 당시의 메모리 임시 저장 상태를 설명하는 과거 기록이다. 그 자료와 캡처는 이제 디스크로 복사했고 현재 모델과 뷰어도 디스크에 있어 재부팅 후 사용할 수 있다. 기존 검토용 index/probe는 소스 보관용이며 현재 재현 진입점은 위 GUI와 `operator_console.rover_3d_smoke`다.

**미완료 실물 연동:** 현재 송신기는 로커·보기 각도를 생성하지 않는다. 보정된 관절각과 표본 나이를 전송하면 새 뷰어가 적용한다. 바퀴 Hall 값만으로 다리 각도를 추정하지 않는다. CAD 디퍼렌셜 폐루프와 실물 AS5048B 배선·영점·극성·실주행 인수는 별도다.
