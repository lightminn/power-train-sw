# 운용 콘솔 — PiP 크기 고정 · 거리 정합성 · 라이브 결함 수정 설계

- 작성일: 2026-07-29
- 대상: GitHub PR #3 `agent/operator-console-live-telemetry` (커밋 `00126f0`)
- 기준선: `operator_console/tests` 215 passed, `runtime_smoke` PASS (수정 전 확인)
- 라이브 관측: 노트북 192.168.8.163 ↔ Jetson 192.168.8.106, 2026-07-28 23:30 ~ 07-29 00:10

## 0. 배경과 범위

사용자 요구 3건:

1. 작은 카메라 화면(작업 카메라 PiP)의 크기를 그대로 유지할 것
2. 거리를 정확히 측정할 것 — **D435i SDK 의 depth 값을 정본으로 삼을 것**
   (2026-07-29 추가 지시; 기존 표시값이 이전 실측 테스트와 크게 달랐다)
3. 실시간 연결 상태에서 오류·동작 버그를 전반 검토할 것

3번은 이번 세션에서 실제 젯슨에 붙여 수행했고, 아래 결함은 전부 **재현된
관측 결과**다. 추정만으로 적은 항목은 없다.

범위 밖: 물리 줄자 기준 절대 캘리브레이션(SDK depth 자체의 정확도 검증은
별건), 모터 구동 검증, 로봇팔 파지 동작. 본 작업의 거리 판정 기준은
"콘솔 표시값이 SDK depth 와 일치하는가"이다.

## 1. 관측된 결함

### A. PiP 크기 (요구 1)

**근본 원인.** `OperatorConsole._on_video_area_allocated` 는 PiP 프레임에
`set_size_request(pip_width, pip_height)` 를 건다. GTK 에서 size request 는
**최소 크기**이고, 프레임 안의 `gtksink` 위젯은 수신 영상 해상도를 자신의
natural size 로 보고한다. `Gtk.Overlay` 는 overlay child 를 natural size 로
할당하므로 **PiP 는 스트림 해상도만큼 커진다.**

관측:

| 실험 | 결과 | 증거 |
|---|---|---|
| 합성 848×480 스트림을 PiP 슬롯에 주입 | 클릭 없이 PiP 가 ~850×480 으로 부풀어 전방 화면을 거의 덮음 | `pip1/02_both_live_later.png` |
| 젯슨 라이브에서 PiP 헤더 클릭(1280×720 전방이 PiP 로 강등) | 강등된 패널이 **스테이지 전체를 점거**, 헤더 2개 중첩, MAIN 패널 안 보임 | `act2/02_after_pip_header_click.png` |
| **실기**: 팔팀 `stream_node` 로 실제 작업 카메라 :5002 를 올린 상태 | PiP 가 **전방 화면을 완전히 덮음** — 가장자리 일부만 남음. 클릭 없음 | `live_full/shot_04.png` |

즉 팔팀 D435 스트림이 올라오는 순간 시연 화면 레이아웃이 깨진다. 클릭은
악화 요인일 뿐 원인이 아니며, 이는 합성 재현이 아니라 **실기에서 관측**됐다.

**클릭 정책.** 사용자 결정: 스왑 기능은 유지하되 **오조작 방지** — 영상/헤더
클릭으로는 바뀌지 않게 한다.

### B. 거리 정합성 (요구 2)

송신 계약은 확인됨: 팔팀 `metadata_sender_node` 가 `/detected_objects` 의
`pose.position` 을 그대로 `position_m` 으로 싣고, `frame_id` 는
`camera_color_optical_frame`, 즉 **컬러 카메라 광학 프레임**이다.

### B0 거리 정본 = D435i SDK depth (사용자 지시, 2026-07-29)

현재 콘솔은 √(x²+y²+z²), 즉 카메라 원점 ↔ 대상 **3D 직선거리**를 표시한다.
사용자 지시는 **D435i SDK 가 주는 depth 값을 정본으로 삼으라**는 것이며,
이는 이전 실측 테스트 결과와의 불일치가 근거다. 실기 대조로 정량 확인했다.

동일 대상(색 픽셀 612, 386), 20 프레임 평균:

| 값 | 측정 |
|---|---|
| SDK depth Z — 3×3 patch median | **0.2883 m** (sd 0.0005) |
| `rs2::depth_frame::get_distance()` | 0.2884 m |
| 컬러 프레임 Z (`position_m[2]`, d2c 변환 후) | 0.2882 m |
| 3D magnitude — 현재 콘솔 표시 | 0.3199 m |
| 차이 (magnitude − depth) | **+3.16 cm (약 11%)** |

magnitude = depth / cos θ 이므로 대상이 광축에서 벗어날수록(이 관측은
θ ≈ 25.7°) 오차가 커진다. 화면 가장자리·원거리에서는 더 벌어진다.

**결정.** 표시 거리 = **SDK depth**. PR #3 은 의도적으로 "depth 축 Z 대신
3D magnitude" 로 바꿨으나, 위 실측과 사용자 지시에 따라 이를 되돌린다.

**구현 근거.** `position_m[2]`(컬러 프레임 Z)는 위 측정에서 SDK depth 와
**0.2 mm 이내로 일치**한다(깊이→컬러 외부 파라미터의 평행이동이 거의 x 축
성분이라 Z 가 사실상 보존됨). 따라서 **프로토콜 변경 없이 콘솔에서
`position_m[2]` 를 표시**하면 정본을 만족한다. 송신부가 추후 명시적
`depth_m` 필드를 실어 주면 그 값이 우선하도록 선택적 경로만 열어 둔다.

아래는 그 위에서 발견된 나머지 정합성 결함이다. 계산식·단위·오버레이
스케일링 자체에는 오차가 없었다(0.309 m 계산 = 0.31 m 표시). 문제는
"어떤 규약의, 어떤 대상의, 언제 시점의 거리인가"이다.

- **B1 비-타깃 폴백.** `DisplayTargetTracker.update` 의
  `selected = explicit or continuous or max(detections, key=confidence)` 는
  송신자가 pick target 을 지정하지 않아도 **confidence 최고 검출**을
  "대상"으로 삼는다. 주석은 "초기 프레임 한정"이라고 하지만 코드는 매
  프레임 적용되고, 한 번 잡히면 IoU 연속성이 계속 그것을 유지한다.
  07-28 감사 실측에서 검출 56건 중 `is_pick_target` 은 1건뿐이었으므로
  실사용에서는 거의 항상 폴백 경로다. 결과적으로 rail 의 "대상 거리"가
  로봇팔이 겨냥하지 않는 물체의 거리일 수 있다. 같은 모듈의
  `pick_display_target` 는 "never invent one in the UI" 를 명시하고 있어
  모듈 내부에서 정책이 충돌한다.
  **실기 확인**: 젯슨 브리지가 보낸 datagram 은 `is_pick_target: false`
  였는데 콘솔은 "작업 대상 box-segmentation 96% / 거리 0.31 m" 를 표시했다.
  폴백이 실제로 발동한다.
- **B1-a 배포 격차(별건, 젯슨).** 위 datagram 이 `is_pick_target: false` 인
  이유는 젯슨에 배포된 `arm_console_bridge` 가 **PR #3 이전 버전**(이전
  프레임 bbox 완전일치 요구)이기 때문이다. `/pick_target` 은 같은 상자를
  bbox [506,293,215,187] 로, `/detected_objects` 는 [503,293,219,187] 로
  발행해 IoU 는 0.9 이상인데도 매칭에 실패한다. PR #3 의 IoU 0.5 + freshness
  0.75 s 수정이 배포되면 해소된다. 즉 이 항목은 코드가 아니라 **배포**
  문제이며, 수정본 배포 뒤 라이브 재확인이 필요하다.
- **B1-b 운용 전제(팔팀 파라미터).** `perception_node` 의 `pick_classes`
  기본값은 비어 있어 그대로 기동하면 `/pick_target` 후보가 아예 없다
  (기동 로그: "pick_classes 미설정 → /pick_target 후보 없음"). 시연 시
  `pick_classes:=box-segmentation` 지정이 전제 조건이다. 콘솔은 이 상태를
  "대상 미지정"으로 구분해 보여줄 수 있어야 한다.
- **B2 `frame_id` 미검증.** `parse_metadata` 는 `frame_id` 를 받아 보관만 하고
  검사하지 않는다. 팔팀이 좌표를 `base_link` 등으로 바꾸면 같은 계산식이
  "로봇 원점 기준 거리"로 의미가 바뀌는데 UI 에는 아무 신호가 없다.
- **B3 신선도 기준.** age 를 `received_monotonic_s`(콘솔 도착시각)로만 잰다.
  파싱된 `capture_stamp_ns` 는 쓰이지 않는다. 추론+전송 지연이 거리값에
  반영되지 않으며, 운용자는 표시값이 얼마나 과거인지 알 수 없다.
- **B4 dropout 후 필터 상태 잔존.** `update(None)` 은 `_cached` 만 비우고
  `_distances`/`_ema`/`_last_bbox` 는 유지한다. 임의 길이의 공백 뒤
  재획득 시 bbox IoU 가 우연히 겹치면 **공백 이전 값과 0.6/0.4 로 혼합된
  거리**가 표시된다.
- **B6 프레임 크기 신뢰.** 송신부(`metadata_sender_node`·`arm_console_bridge`)
  는 `frame_width/height` 를 **파라미터 기본값 848×480** 으로 싣는다(실제
  이미지에서 유도하지 않는다). 콘솔은 이 값으로 bbox 를 영상 위에 스케일
  하므로, 인식 해상도가 바뀌고 파라미터가 안 바뀌면 **박스가 조용히
  어긋난다**. 현재는 실제 스트림도 848×480 이라 일치한다(문제 없음, 잠재
  위험). 콘솔 쪽 완화책: 수신 영상 caps 와 metadata 프레임 크기가 다르면
  오버레이를 그리지 않고 경고한다.
- **B5 draw 핸들러에서 상태 변경.** `MetadataCanvas._on_draw` 가 상태를 가진
  `self._target_tracker.update(frame)` 를 호출한다. 같은 tracker 를
  `_sync_overlay_rail` 도 타이머로 갱신하므로, 필터 진행이 **렌더 빈도**에
  좌우되고 두 경로가 서로의 시퀀스를 소비한다. 창이 가려져 draw 가 줄면
  필터 거동이 달라진다.

### C. 라이브 결함 (요구 3)

- **C1 (블로커, 젯슨측)** `ops_broker` 프로세스 사망.
  `ModuleNotFoundError: No module named 'powertrain_msgs'`
  (`ops_broker_node.py:22`) 로 기동 직후 exit 1. 결과: **ops 채널 :9001 미개방
  → 콘솔의 모든 조작·복구·비상정지 전송 경로 불통**, `powertrain_control`
  컨테이너 healthcheck 실패(unhealthy). 같은 런치의 `teleop_command` 는 정상.
- **C2 (안전 UX)** 전역 E-STOP 버튼의 `set_sensitive` 는 **생성 시점 1회**
  `ops_panel.ops_available()`(토큰 파일 유무)로만 결정되고 이후 갱신되지
  않는다. 이 노트북에는 토큰이 있으므로(`~/.config/powertrain/ops_console.token`,
  젯슨 `/etc/powertrain/ops_console.token` 과 동일 크기) 버튼은 항상 활성이다.
  따라서 **브로커가 죽어 있던 그 1시간 동안에도 버튼은 눌리는 상태였고**
  명령은 허공으로 나갔다. 운용자 피드백은 접힌 서랍의 일반 이벤트 한 줄
  ("OPS 상태가 업데이트되었습니다")뿐이라 실패를 알 수 없다. 반대로 토큰이
  나중에 생겨도 재시작 전에는 계속 비활성이다.
- **C3 freshness 를 health 로 오표시.** `RobotStatusDashboard._state` 는
  datagram 이 신선하면 "정상"을 돌려준다. 실제 수신값은
  `rs485_state="ERROR"`, `rs485_consecutive_failures=3691`,
  `rs485_detail="could not open port /dev/powertrain-pdist80b"`,
  전압·SOC·보호상태 전부 `null` 인데 **전원 시스템 카드는 "정상"** 으로
  뜨고 "필수 시스템 준비" 카운트에도 포함된다. PR 자신이 내건
  "결측·정체를 정상으로 렌더하지 않는다" 규칙과 어긋난다.
- **C4 운용 IP 하드코딩.** 젯슨의 텔레메트리 송신기가
  `--operator-host 192.168.8.206` 로 고정되어 있고 그 IP 는 현재 죽어 있다
  (운용 노트북은 .163). 검증을 위해 노트북에 .206 을 임시 부여했고
  세션 종료 시 제거한다. 항구 대책이 필요하다.
- **C5 검증 게이트 결함.** `runtime_smoke` 는 "xvfb-run 으로 감싸 사용자
  화면에 창을 띄우지 않는다"고 주장하지만, Wayland 세션에서는 GDK 가
  `DISPLAY` 를 무시하고 Wayland 백엔드를 잡아 **실제 사용자 화면에 창이 뜬다**
  (이번 세션에서 재현). 격리 주장이 거짓이고, X11 전용 렌더 경로 결함을
  놓칠 수 있다.
- **C6 (정보)** SRT 조인 시 `h264parse ... broken/invalid nal ... will be dropped`
  경고가 다량 발생한다. 키프레임 수신 전 정상 동작이지만 로그 노이즈가 커
  실제 오류를 가린다.
- **누수 없음(음성 결과).** 10분 라이브 소크에서 RSS 159~190 MB 변동,
  fd 36±1, thread 20~22, traceback 0, 조기 종료 없음.

**크로스팀 위생 (보고 항목).** PR #3 의 `tools/deploy_d435_distance_fix.sh` 는
이미 젯슨의 팔팀 레포(`/home/zetin/extreme-robot`)에 커밋되지 않은 수정을
남겼다: `perception_node.py`·`stream_node.py` 수정 + `perception_quality.py`
신규. 팔팀 소유 코드이므로 이번 작업에서 되돌리지 않고, 사용자·팔팀에
알리는 것으로 처리한다.

## 2. 설계

### A. PiP 크기 고정 + 클릭 정책

**A-1. 고정 크기 슬롯.** PiP 프레임의 자식을, 자식의 natural size 를
전파하지 않는 **고정 크기 컨테이너**로 감싼다. 구현 수단은 자식의
`preferred width/height` 를 슬롯이 지정한 값으로 강제하는 컨테이너면 무엇이든
되지만(예: `Gtk.Bin` 파생 + `do_get_preferred_*` 오버라이드, 또는
`propagate_natural_*=False` 로 설정한 `Gtk.ScrolledWindow` + 스크롤바 비표시),
**계약은 하나다**: PiP 슬롯의 실제 할당 크기는 스트림 해상도와 무관하게
`_on_video_area_allocated` 가 계산한 값과 같아야 한다.

- PiP 크기 계산식 자체(`min(430, max(300, width*0.30))`, 848:480 비율)는 유지.
- 영상은 슬롯 안에서 letterbox 로 맞춘다(가로세로비 유지, 잘림 없음).
- MAIN 스테이지는 지금처럼 남는 공간 전부.

**A-2. 클릭 정책.** `VideoPanel` 헤더 EventBox 의 `button-press-event` →
스왑 경로를 제거한다. 대신 미션 화면 우측 "화면 표시" 패널에 **명시 버튼
`주/보조 화면 교체`** 를 두고, 단축키 하나를 붙인다. `swap_camera_views(...,
user_initiated=True)` 공개 동작과 기존 테스트는 그대로 유지한다(호출 지점만
바뀐다). 헤더의 "클릭하여 크게 보기" 안내 문구는 제거한다.

**A-3. 스왑 후에도 A-1 계약 유지.** 스왑으로 어떤 패널이 PiP 로 들어가든
슬롯 크기는 불변이어야 한다.

### B. 거리 정합성

- **B0 (최우선).** 표시 거리를 **SDK depth 규약**으로 바꾼다.
  `target_distance_m` 은 √(x²+y²+z²) 대신 **`position_m[2]`** 를 돌려주고,
  `position_m[2] > 0` 유효성 검사를 유지한다(z ≤ 0 은 기존 계약대로
  "depth 없음"). datagram 에 선택적 `depth_m` 이 있으면 그 값이 우선한다.
  3D magnitude 는 표시에서 제거한다. 함수 이름·docstring 도 규약을
  드러내도록 바꿔 다음 사람이 다시 뒤집지 않게 한다.
- **B1.** 표시 대상 선정에서 "최고 confidence 폴백"을 제거한다. 거리 숫자는
  **송신자 지정 pick target**(및 그 IoU 연속 추적분)에 대해서만 표시하고,
  그 외에는 숫자 대신 상태 문구(`대상 탐지 대기`)를 유지한다. 검출 박스
  자체는 지금처럼 계속 그린다(관측 정보는 잃지 않는다).
- **B2.** `frame_id` 화이트리스트(`camera_color_optical_frame`)를 둔다.
  불일치면 거리 숫자를 표시하지 않고 `거리 기준 불일치` 상태로 강등하며
  이벤트 로그에 1회 기록한다. `frame_id` 부재는 기존 호환을 위해 허용한다.
- **B3.** `capture_stamp_ns` 가 있으면 age 를 그것으로 계산한다. 송신·수신
  시계가 다를 수 있으므로 **오프셋을 첫 유효 프레임에서 1회 추정**해
  적용하고, 추정 불가하면 도착시각으로 폴백한다. rail 에 파이프라인 지연을
  ms 로 함께 표시한다.
- **B4.** 마지막 유효 프레임으로부터 `dropout_hold_ms` 를 초과한 공백이
  발생하면 `_distances`·`_ema`·`_last_bbox`·`_last_class` 를 리셋한다.
- **B5.** tracker 갱신 소유자를 **타이머 경로 한 곳**으로 정한다.
  `MetadataCanvas._on_draw` 는 tracker 를 갱신하지 않고 최근 스냅샷만 읽어
  그린다. 갱신 주기는 현재 rail 주기를 따른다.

### C. 라이브 결함

- **C1.** 젯슨의 `ops_broker` 가 `powertrain_msgs` 를 import 할 수 있도록
  기동 환경을 고친다(원인 확정 후: 오버레이 소싱 누락인지, 설치 누락인지
  판별). 판정 기준은 **:9001 LISTEN + `powertrain_control` healthy** 이며,
  이 수정은 콘솔 PR 과 별도 커밋으로 분리한다.
- **C2.** E-STOP 버튼 가용성을 **동적**으로 만든다: 토큰 존재 ∧ ops 링크
  생존일 때만 활성, 링크가 끊기면 비활성 + 사유 표시. 전송 실패·응답 없음
  (`OUTCOME_UNKNOWN` 포함) 시 접힌 서랍이 아니라 **상단에 즉시 보이는 경고**
  를 띄운다. 토큰이 없는 상태도 상단에 상시 표기한다.
- **C3.** freshness 와 health 를 분리한다. datagram 이 신선해도 해당
  서브시스템의 상태 필드가 오류/결측이면 "정상"으로 표시하지 않고
  "확인 필요"로 강등하며, 준비 카운트에서도 제외한다. 최소한
  `rs485_state`, 그리고 전압·SOC 전부 결측인 경우를 판정에 포함한다.
- **C4.** 운용 호스트를 고정 IP 대신 설정 가능한 값으로 정리한다(배포
  설정/DHCP 예약 중 사용자 결정 사항). 본 작업에서는 현상·영향·후보안을
  문서화하고, 콘솔 쪽에는 "텔레메트리 미수신"이 명확히 보이게 한다(C3 로
  일부 충족).
- **C5.** `runtime_smoke` 가 `GDK_BACKEND=x11` 을 강제하고 `WAYLAND_DISPLAY`
  를 제거한 환경에서 콘솔을 띄우도록 고친다. 창이 사용자 화면으로 새는지
  자체를 **음성 대조**(강제 제거 시 실패)로 증명한다.
- **C6.** 콘솔이 GStreamer 경고를 기본 수준에서 억제하거나, 최소한 조인
  구간의 NAL 경고를 이벤트 로그로 승격하지 않는다(현행 유지 확인).

## 3. 검증 계획

수정 뒤 아래를 **모두 통과**해야 완료로 본다.

1. `operator_console/tests` 전건 통과(현 215 + 신규).
2. `runtime_smoke` PASS (X11 강제 후).
3. **자동 회귀 테스트 신설**
   - PiP: 848×480 / 1280×720 두 해상도 스트림을 PiP 슬롯에 주입하고 슬롯
     할당 크기가 계산값과 같은지 단언. 스왑 후에도 동일.
   - 클릭: 헤더 클릭이 MAIN 을 바꾸지 않는지, 명시 버튼/단축키는 바꾸는지.
   - 거리 규약: `position_m=(0.0854, 0.0704, 0.2882)` 입력 시 표시값이
     **0.288 m**(depth) 이어야 하고 0.309 m(magnitude) 이면 FAIL.
     선택적 `depth_m` 이 있으면 그 값이 우선.
   - 거리: pick target 없음 → 숫자 미표시, frame_id 불일치 → 강등,
     dropout 초과 후 재획득 → 이전 EMA 미혼합, draw 호출이 필터 상태를
     바꾸지 않음.
4. **음성 대조**: 위 신설 테스트 각각에 대해 결함을 되돌리면 FAIL 하는지
   확인한다(게이트 자체의 증명).
5. **라이브 재검증**(젯슨 실기): 전방 :5000 + 팔팀 작업 :5002/:5003 을 모두
   올린 상태로 콘솔을 띄워 ① PiP 크기 불변 ② 거리 표시 ③ 무 traceback
   ④ 10분 소크 자원 안정 을 캡처로 남긴다.
   거리는 **SDK 원본과 직접 대조**한다: 같은 대상에 대해
   `/detected_objects` 의 z, `depth_frame.get_distance()`, 콘솔 표시값 세
   가지가 **±5 mm 이내**로 일치해야 한다(오늘 기준선: 0.2882 / 0.2884 /
   현재 0.3199 → 수정 후 0.288 대 예상).
6. C1 은 `:9001 LISTEN` 과 `powertrain_control healthy` 로 판정한다.

## 4. 작업 분담

설계·계획·라이브 검증은 Claude, 구현은 Codex(프로젝트 규칙). Codex 에는
파일 경로·계약·완료 조건을 명시해 위임하고, Claude 가 diff 와 실행 결과를
검증한 뒤 사용자에게 보고한다.

## 5. 미해결·후속

- 물리 줄자 기준 거리 정확도 검증(기준값 확보 후 별도 세션).
- C4 운용 IP 항구 대책(DHCP 예약 vs 배포 파라미터) — 사용자 결정 필요.
- 팔팀 레포에 남은 미커밋 수정의 처리 — 팔팀과 협의 필요.
