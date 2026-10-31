# PR #20 메인 GUI에 PR #14 팔 제어 통합

## 변경

`integrate/pr20-arm-controls`에서 PR #20 `75ef49c`와 PR #14 `979611d`를 통합했다.
기존 사용자 체크아웃과 다른 worktree의 미커밋 파일은 수정하지 않았다.
PR #20의 대상은 `feat/l515-terrain-map`이며 이 통합 브랜치는 해당 의존성도 포함한다.
원격 PR을 머지하거나 Jetson에 배포하지 않았다.

충돌은 `operator_console/app.py`의 팔 탭 조립 한 곳이었다.
PR #20의 팔 3D 자세 카드와 기존 비전/지도/협조구동/시스템 화면을 유지하고,
PR #14의 패드 선택·안내와 창 키보드 핸들러를 함께 연결했다.
팔 UI/명령 세션/패드/브리지/벤치 설정은 PR #14와 동일하다.
관절 속도·가속도·제한·트립·저장 자세·볼락 위치·슬롯 의미를 바꾸지 않았다.
실행 시 세션의 기존 JSON 파일을 보존해야 한다.

별도 비인증 `direct_ros.py` 개발 경로는 포함하지 않았다.
기존 인증 ops, 브로커와 단일 소유 팔 브리지를 사용한다.

## 통합 검증과 검증 도구 수정

- 실제 GTK에서 팔 3D 카드와 패드 선택이 함께 표시되고, 패드가 기존 키보드 패널을 공유하며,
  비전 탭으로 전환하면 키보드 입력이 해제되는 회귀를 추가했다.
  통합본 1 passed, 원본 PR #20에 같은 검사를 적용하면 패드 위젯 부재로 1 failed였다.
- 기존 브로커 테스트 대역에 인증 시계 변환 뒤의 `deadline_s`를 제공했다.
  인증 시계 동기화가 없으면 조그를 발행하지 않는 음성 검사도 추가했다.
  제품의 기한·인증 게이트는 변경하지 않았다.
- 가상 패드에 실제 리더가 사용하는 `get_numhats`를 보충하고 실패 시 패드 상태/포커스를 기록한다.
- 가상 운용 종료의 화면 캡처를 GTK backing surface에서 X11 root로 변경했다.
  변경 전 Cairo 1.16의 MIME surface assertion으로 캡처가 중단됐다.
  코어 덤프에서 `gdk_pixbuf_get_from_window → cairo_surface_mark_dirty`를 확인했다.
  제어 경로의 예외를 무시하거나 검증 단언을 완화하지 않았다.

## 실행 결과

- 설치 ROS 빌드: `robot_arm_msgs`, `powertrain_msgs`, `dynamixel_control`, `powertrain_ros` 4 packages finished.
- `operator_console.runtime_smoke`: PASS. 비전·지형 지도 포함 7채널 LIVE/STALE, 조작 차단, traceback 없음.
- 팔 UI/세션/패드/binding: 341 passed / 4 failed. 네 실패 모두 PR #20 원본에서도 실패했다.
- 같은 범위 PR #20 원본: 284 passed / 9 failed. 원본에는 새 패드 테스트 파일이 없다.
- 자세 안전·슬롯 교체·도구 제어·볼락·ROS 브로커: 111 passed.
- 비전/지도/3D 순수 회귀: 125 passed.
- 격리 Xvfb 실제 GTK 비전 전환·수신/복구·오버레이: 6 passed.
- 설치 ROS + 실제 GTK 버튼 + 인증 TCP + UDP + 감독 가상 Xbox: 36개 검사 통과.
  좌/우 개별·동기 도구 이동, 누름/해제, 패드 단절/중립 재연결, 수신 기한 자동 정지,
  단일 그리퍼/클리너 교체, 명시적 토크 활성화/해제, 수동 제어권 반납을 포함한다.

전체 콘솔은 같은 Xvfb 환경에서 원본 858 passed / 37 failed / 7 skipped,
통합본 917 passed / 31 failed / 7 skipped였다. 실패 이름 대조에서 새 실패는 0이다.
전체 green은 아니다. 이 전체 실행은 로컬 추가 x264 플러그인 경로 없이 실행해 영상 검사 실패를
양쪽에 포함한다. 해당 경로를 제공한 위의 콘솔 스모크와 격리 비전 6개는 별도로 통과했다.
최초 호스트 디스플레이 전체 실행의 native segmentation fault는 Xvfb 비교 실행으로 대체했다.

검증은 새 `pr20-arm-validation` 컨테이너에서 소스 읽기 전용 마운트와 가상 모터로 수행했다.
장치 파일·기존 로봇 런타임을 연결하지 않았다. Xvfb/GUI/GStreamer/gdb 의존성은
임시 도구 폴더 또는 검증 컨테이너에만 준비했다. oxlint 설정은 발견되지 않았다.
검증 환경에는 WebKit2가 없어 기존 3D 뷰의 실제 브라우저 렌더링은 확인하지 못했다.
3D 코드/자산을 교체하지 않았으며 미설치 안내와 순수 상태 변환은 확인했다.

## 실물 범위

사용자가 PR #14의 Jetson 실물 체결 시험 성공을 확인했다.
이번 통합본을 Jetson에 설치해 재시험한 것은 아니다. `jetson` SSH 별칭은 해석되지 않았다.
PR #14의 실물 기록을 이번 코드의 새 실기 인수로 표기하지 않는다.
PR #20의 L515/비전 실기 인수도 이 작업으로 완료된 것은 아니다.

## 재현과 원시 증거

작업 폴더는 `/tmp/pr20-arm-controls`, 원본 비교 폴더는 `/tmp/pr20-gui-baseline`이다.
원시 로그/화면/실패 집합은 `/tmp/pr20-arm-evidence`에 보관했다.
기능 계약과 운용 스위치는 `docs/specs/2026-10-31-main-gui-arm-integration.md` 및
`docs/specs/2026-10-17-local-arm-console.md`를 따른다.

호스트에서 `PYTHONPATH=motor_control:ros2/src/powertrain_ros`로 시스템 Python을 사용한다.
스모크는 `python3 -m operator_console.runtime_smoke`, 회귀는
`python3 -m pytest -q -p no:cacheprovider operator_console`이다.
Xvfb 경로와 임시 x264의 `GST_PLUGIN_PATH`를 준비해야 한다.

컨테이너의 설치 경로는 `/tmp/runtime/install`이다. 설치 E2E는 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
source /tmp/runtime/install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 GDK_BACKEND=x11 OPENBLAS_NUM_THREADS=1
export PYTHONPATH=/usr/lib/python3/dist-packages:/ws:/ws/motor_control:/ws/ros2/src/powertrain_ros:$PYTHONPATH
export PATH=/xvfb/usr/bin:$PATH
export LD_LIBRARY_PATH=/xvfb/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
unset WAYLAND_DISPLAY
xvfb-run -a -s '-screen 0 1920x1200x24' python3 \
  /ws/scripts/verify_arm_console_tools_mock.py --output-dir /tmp/e2e-verified
```
