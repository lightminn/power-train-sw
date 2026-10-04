# 로봇팔 콘솔 신규 탭 UI 골격 (Claude 담당분)

작성일: 2026-09-10
기준 커밋: `2ffbad6673f54c2bd4724dc19acaea7d31f23f89`
브랜치: `claude/arm-console-ui-skeleton`
작업 트리: `/home/jo/ros2_ws/power-train-sw/.claude/worktrees/claude-arm-ui`

계획 문서: `docs/plans/2026-09-10-arm-console-ui.md` (Codex 소유, 이번 작업에서 수정하지 않음)

> **이 문서가 보고하는 범위는 UI 골격까지다.** 실제 키보드 명령 송신, ROS 연결,
> 모터 구동, 보정 계산·저장은 구현하지 않았다. 화면과 테스트가 통과한 것은
> **주입된 상태에 대한 UI 동작**이며, 실물 로봇팔 연결·모션은 미검증이다.

---

## 1. 무엇을 만들었나

병행 분담에서 Claude가 맡은 **신규 탭 UI 골격**이다. 데이터 송수신과 기존 앱 연결은
범위 밖이며, 그 부분은 Codex의 기존 UI·실데이터 대응 작업이 담당한다.

| 탭 | 클래스 | 내용 |
|---|---|---|
| 로봇팔 수동조작 | `ArmManualTab` | 도구 감지/변경 요청, 제어권, 팔·도구 FSM과 차단 이유, 키보드 텔레옵, 그리퍼별 전용 패널, 현재 도구 진단 |
| 로봇팔·도구 캘리브레이션 | `ArmCalibrationTab` | 내부 `로봇팔 / 현재 도구` 선택, 기어비→영점→가동범위, 단일/듀얼 끝점 보정, 측정·검증·임시 적용·저장 구분 |

### 추가 파일 (지정된 세 경로 외에는 추가·수정 없음)

```
operator_console/arm_ui/
├── __init__.py            공개 API와 사용법
├── __main__.py            python -m operator_console.arm_ui
├── contracts.py           주입 상태·콜백 계약, capability, gate() 판정
├── styling.py             .arm-ui 로만 한정된 CSS (공통 테마 미수정)
├── widgets.py             카드·타일·배지·GateGroup·HoldButton·SignalTracker
├── manual_tab.py          로봇팔 수동조작 탭
├── keyboard_panel.py      키보드 텔레옵 영역 + 창 단위 키 처리
├── tool_panels.py         단일/듀얼/정보전용 패널 + 현재 도구 진단
├── calibration_tab.py     로봇팔·도구 캘리브레이션 탭
├── fixtures.py            미리보기 전용 fixture (is_fixture=True)
└── preview.py             독립 실행 미리보기 진입점

operator_console/tests/arm_ui/
├── conftest.py                     표시장치 없으면 수집 생략
├── _arm_ui_helpers.py              위젯 순회·콜백 기록 헬퍼
├── test_arm_ui_contracts.py        상태 기본값·gate·도구 누출 판정
├── test_arm_manual_tab.py          수동조작 탭 위젯 동작
├── test_arm_calibration_tab.py     캘리브레이션 탭 위젯 동작
└── test_arm_ui_boundary.py         경계 정적 검사 + 실제 미리보기 기동

docs/reports/2026-09-10-claude-arm-ui.md   이 문서
```

**수정하지 않은 것**: `app.py`, `arm_telemetry.py`, `status_view.py`, 공통 테마,
ROS 브리지, ops 계약·브로커, 운용 런처, 파워트레인 주행·게임패드·영상·복구·CAN/USB,
`extreme-robot`, Codex의 계획 문서. `test_arm_ui_boundary.py`의
`test_existing_console_files_are_untouched`가 기준 커밋 대비 diff로 이를 강제한다.

---

## 2. 인터페이스 — 탭 생성·상태 갱신·콜백 연결

### 2.1 탭 생성

```python
from operator_console.arm_ui import (
    ARM_CALIBRATION_TAB_NAME, ARM_CALIBRATION_TAB_TITLE,
    ARM_MANUAL_TAB_NAME, ARM_MANUAL_TAB_TITLE,
    ArmCalibrationTab, ArmManualTab, ArmUiCallbacks,
)

manual = ArmManualTab(callbacks)        # callbacks 생략 시 전 조작 비활성
calibration = ArmCalibrationTab(callbacks)
```

`ArmManualTab(...)` 생성자가 CSS를 스크린당 1회만 설치한다(중복 호출 안전).

### 2.2 상태 갱신

```python
manual.update_state(state)       # state: ArmUiState
calibration.update_state(state)
```

- 전체 상태를 통째로 교체하는 방식이다. 호출 빈도 제한은 없다.
- 타이머를 내부에 두지 않는다. 갱신 주기는 호출자가 정한다.
- `update_state`는 렌더 직전에 `cleared_for_tool()`을 적용해 현재 도구와 맞지 않는
  진단 값을 버린다.

### 2.3 콜백 연결

`ArmUiCallbacks`의 모든 필드는 기본 `None`이다. `None`이면 해당 조작은 비활성이고
툴팁에 `조작 불가 — 미연결 …`이 뜬다.

```python
callbacks = ArmUiCallbacks(
    request_tool_change=lambda kind: ...,          # 명시적 도구 변경 요청
    request_control_mode=lambda mode: ...,         # "MANUAL" | "FSM"
    jog_start=lambda axis, direction: ...,
    jog_stop=lambda axis: ...,
    tool_command=lambda target, command: ...,      # target: single|left|right|both
    arm_calibration_command=lambda step, action: ...,
    ...  # 전체 목록은 contracts.ArmUiCallbacks 참조
)
```

시그니처는 **의도의 이름**이지 전송 포맷이 아니다. ops 명령으로 어떻게 바꿀지(그리고
바꿀지 여부 자체)는 연결 단계의 결정이다.

### 2.4 종료 처리

```python
manual.dispose()
calibration.dispose()
```

`dispose()`는 진행 중인 누름 조그를 종료하고, 연결한 모든 시그널 핸들러와 GLib
타이머를 해제한다. 중복 호출은 안전하다.

### 2.5 키보드(선택)

```python
manual.attach_keys(window)   # 창 단위 key-press/release 수신, 1회만 호출
```

탭이 화면에 보일 때만 동작한다(`map`/`unmap`으로 자동 arm/disarm). 입력란·스핀버튼에
포커스가 있으면 키를 가로채지 않는다.

---

## 3. Codex가 `app.py`에 연결할 최소 접점

기존 두 탭 뒤에 두 줄을 추가하면 된다. **기본 진입 화면과 기존 탭 순서는 그대로다.**

`app.py`의 스택 구성부(기준 커밋 기준 `stack.add_titled(systems_scroll, ...)` 직후):

```python
from .arm_ui import (
    ARM_CALIBRATION_TAB_NAME, ARM_CALIBRATION_TAB_TITLE,
    ARM_MANUAL_TAB_NAME, ARM_MANUAL_TAB_TITLE,
    ArmCalibrationTab, ArmManualTab, ArmUiCallbacks,
)

self._arm_manual_tab = ArmManualTab(arm_callbacks)
self._arm_calibration_tab = ArmCalibrationTab(arm_callbacks)
stack.add_titled(
    self._arm_manual_tab.widget, ARM_MANUAL_TAB_NAME, ARM_MANUAL_TAB_TITLE)
stack.add_titled(
    self._arm_calibration_tab.widget,
    ARM_CALIBRATION_TAB_NAME, ARM_CALIBRATION_TAB_TITLE)
```

필요한 접점은 이게 전부다:

1. **등록** — 위 두 `add_titled`. `switcher.set_size_request(238, 34)`는 탭이
   네 개가 되므로 폭을 늘리거나 제거해야 한다(현재 값은 두 탭 기준).
2. **갱신** — 기존 텔레메트리 틱에서
   `self._arm_manual_tab.update_state(state)` /
   `self._arm_calibration_tab.update_state(state)` 호출. `state`는 어댑터가
   `ArmUiState`로 변환한 결과이며, 어댑터가 없는 동안에는
   `arm_ui.default_state()`(= 전부 미연결)를 넘기면 화면이 정직하게 미수신을 표시한다.
3. **콜백** — 인증 ops 경로가 준비된 항목만 `ArmUiCallbacks`에 채운다. 채우지 않은
   항목은 자동으로 비활성이므로, 부분 연결 상태로도 안전하게 통합할 수 있다.
4. **키보드(선택)** — `self._arm_manual_tab.attach_keys(self)`.
5. **종료** — 기존 종료 경로에서 두 탭의 `dispose()` 호출.

`app.py` 외에 손댈 곳은 없다. 스타일도 추가 등록이 필요 없다(탭 생성자가 처리).

---

## 4. 경계 — 이 UI가 하지 않는 것

- **전송·수신 없음.** 소켓·`rclpy`·CAN·USB를 열지 않는다. 정적 검사로 강제한다
  (`test_no_transport_or_ros_imports`).
- **외부 스키마를 확정하지 않는다.** `contracts.py`는 콘솔 측 뷰 모델이다. 어떤
  토픽·필드·단위가 각 항목을 채우는지는 어댑터 작업이 정한다.
- **명령 포맷을 확정하지 않는다.** 콜백은 의도만 전달한다.
- **키맵을 정본으로 주장하지 않는다.** `keyboard_panel.KEY_BINDINGS`는 화면 안내용
  잠정 표기이며, UI 자체에 "원본 TUI 대조 전 잠정 표기"라고 명시했다. 팔 측
  텔레옵 정본과의 대조·누락표는 이 작업 범위가 아니다.
- **계산·저장 없음.** 보정 단계 상태는 전부 외부에서 주입받는다.
- **범위 밖 기능 미추가.** 청소기 방향 제어와 개발자 직접 구동은 넣지 않았고,
  정보 전용 패널에 그 사실을 명시했다.

### 안전 표시 원칙 (코드로 강제된 것)

- 생산 기본값(`default_state()`)은 **전부 미연결·미승인·미저장**이다. 기본 장착 도구도,
  기본 승인도 없다.
- `STALE`은 정상이 아니다. 지연 상태에서는 조작이 열리지 않고 마지막 정상값을
  현재값으로 다시 그리지 않는다.
- **요청 ≠ 승인.** `requested_mode`로는 어떤 모션 조작도 열리지 않는다.
  `granted_mode == MANUAL`만 연다. 단, 제어권 요청 버튼 자체는 막지 않는다(막으면
  승인을 영영 받을 수 없다).
- **선택 ≠ 명령.** 도구 드롭다운·축 토글·자세 선택은 하드웨어 명령을 내지 않는다.
- fixture 상태는 `is_fixture=True`로 표시되고 두 탭 상단에 경고 배너가 뜬다.
  생산 기본값과 코드 경로가 분리돼 있다.

---

## 5. 검증 결과

실행 환경: 호스트, `/usr/bin/python3` (시스템 `gi`), GTK 3.
**Xvfb가 이 호스트에 설치돼 있지 않아** 헤드리스 실행은 GTK의 `broadway` 백엔드로 했다
(`broadwayd :7` + `GDK_BACKEND=broadway`). 사용자 화면에 창을 띄우지 않는다.

```bash
broadwayd :7 &
GDK_BACKEND=broadway BROADWAY_DISPLAY=:7 /usr/bin/python3 -m pytest operator_console/tests/arm_ui -q
GDK_BACKEND=broadway BROADWAY_DISPLAY=:7 /usr/bin/python3 -m operator_console.arm_ui --seconds 4 --cycle
```

| 검증 항목 | 결과 | 근거 |
|---|---|---|
| 단일/듀얼/미연결 패널 전환 | 통과 | `test_panel_follows_the_detected_tool_kind`, `test_disconnected_state_does_not_render_a_detected_tool` |
| 현재 도구 외 모터 정보 잔존 없음 | 통과 | `test_previous_tool_motor_rows_do_not_survive_a_tool_change`, `test_readings_for_another_tool_are_dropped_before_rendering` |
| capability·콜백 부재 시 조작 비활성 | 통과 | `test_every_control_is_insensitive_without_callbacks`, `test_controls_stay_insensitive_when_capabilities_are_absent`, `test_no_callbacks_means_no_operable_calibration_control` |
| 선택 변경만으로 명령 미발생 | 통과 | `test_choosing_a_tool_candidate_sends_nothing`, `test_state_updates_do_not_overwrite_the_operator_candidate` |
| 작은 창 스크롤·글자 잘림 | 통과 | `test_page_fits_a_small_window_without_sideways_scrolling`(최소폭 수동 407px / 보정 272px ≤ 430px), `test_long_text_wraps_or_ellipsizes_rather_than_clipping`, `test_page_is_scrollable_so_a_small_window_does_not_truncate` |
| 실제 GTK 미리보기 기동·종료 | 통과 | `test_preview_launches_and_exits_cleanly`, `test_preview_scenarios_all_render` — 실제 프로세스 기동, 종료코드 0, traceback·CRITICAL 없음 |
| 누름 조그 생명주기 | 통과 | 권한 상실·수신 중단·도구 교체·비활성화·탭 이탈 각각에서 조그 종료 확인 |
| 기존 콘솔 파일 미수정 | 통과 | `test_existing_console_files_are_untouched` (기준 커밋 대비 diff) |

**스위트 결과**

- `operator_console/tests/arm_ui` — **86 passed**
- `operator_console` 전체 — **443 passed, 7 skipped, 0 failed**
  (skip 7건은 전부 환경 사유이며 기존과 동일: x264enc 플러그인 없음 3, pygame 없음 3,
  `xvfb-run` 없음 1. 이번 변경과 무관하다.)

### 실행 중 실제로 잡아 고친 결함 (단위 통과 상태에서 드러난 것)

1. `Gtk.Stack`이 비표시 패널을 살려두므로, 듀얼→단일 전환 후에도 듀얼 패널에
   ID 3/4 값이 남아 있었다 → 전환 시 나가는 패널을 `clear()`로 비운다.
2. 캘리브레이션의 도구별 행은 도구 변경 시 재생성되는데, gate가 그 **이전에**
   돌아 새 버튼이 한 프레임 동안 활성 상태였다 → gate를 재생성 이후로 옮기고
   생성 시 비활성으로 시작한다.
3. 재생성되는 행이 탭 공용 gate·tracker에 계속 쌓였다 → 도구 행 전용
   `target_gate`/`target_tracker`로 분리해 재생성마다 정리한다.
4. `.arm-primary`/`.arm-danger`가 `:disabled`보다 뒤에 선언돼 비활성 버튼이
   활성처럼 보였다 → 톤별 `:disabled` 규칙 추가. 화면 렌더 검토에서 발견했다.
5. 좁은 창에서 페이지가 가로 스크롤됐다 → 지표 타일과 상단 카드를 reflow 컨테이너로
   바꾸고 최소폭 예산을 테스트로 고정했다.

### 미검증·미구현 (완료로 표현하지 않는다)

- 실물 로봇팔·ROS·모터와의 연결과 모션은 **전혀 검증하지 않았다.** 통과한 것은
  fixture와 주입 상태에 대한 UI 동작뿐이다.
- `operator_console.runtime_smoke` / `integrated_runtime_smoke`는 이 호스트에
  `xvfb-run`이 없어 실행하지 못했다(기존에도 skip 상태). 신규 탭은 아직 `app.py`에
  등록돼 있지 않으므로 이 스모크의 대상이 아니며, 통합 시점에 스모크에 포함해야 한다.
- 기존 주행·영상·복구 회귀는 콘솔 스위트 통과로만 확인했다. 실기 운용 회귀는
  통합 이후 별도 확인 대상이다.
- 키 바인딩 정본 대조표(누락표)는 만들지 않았다. 화면에는 잠정 표기임을 명시했다.

---

## 6. 미리보기 실행

```bash
# 사용자 화면에 창을 띄우는 일반 실행
/usr/bin/python3 -m operator_console.arm_ui
/usr/bin/python3 -m operator_console.arm_ui --scenario dual

# 헤드리스 (이 호스트에 Xvfb가 없어 broadway 사용)
broadwayd :7 &
GDK_BACKEND=broadway BROADWAY_DISPLAY=:7 \
    /usr/bin/python3 -m operator_console.arm_ui --seconds 5 --cycle
```

시나리오: `disconnected` · `waiting` · `single` · `dual` · `cleaner` · `stale`.
우측 패널에 콜백 호출 기록이 남는다 — 드롭다운을 바꿔도 기록이 남지 않는 것이
"선택은 명령이 아니다"의 눈으로 보는 증거다.

미리보기는 `app.py`를 import하지 않으므로 GStreamer·영상 파이프라인 없이 뜬다.

---

## 7. 후속 연결에서 결정해야 할 것 (이 작업에서 확정하지 않음)

- `ArmUiState` 각 항목의 원천 토픽·필드·단위·신선도 기준.
- `ArmUiCallbacks` 각 의도의 인증 ops 표현과 누름 조그의 최신값·만료·명시 종료 계약.
- 키보드 텔레옵 경로의 모터 단일 소유권과 MANUAL 승인 순서.
- capability 토큰을 백엔드가 어떻게 보고할지. 보고되지 않으면 해당 조작은 계속
  `지원하지 않음`으로 비활성이며, 이는 의도된 기본값이다.
