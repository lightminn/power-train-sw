# 운용 콘솔 라이브 검토 보고 (2026-07-28 ~ 07-29)

노트북 192.168.8.163 ↔ Jetson 192.168.8.106 실기 연결 상태에서 GitHub PR #3
(`agent/operator-console-live-telemetry`, 베이스 `00126f0`)을 검토했다.
아래 항목은 전부 **재현된 관측**이며, 추정만으로 적은 것은 없다.

## 1. 검토 환경

- 전방 카메라(L515) SRT `:5000` — 1280×720, 상시 송신 중이었다.
- 작업 카메라(D435i) SRT `:5002` 와 metadata `:5003` — 검토 시작 시점에는
  꺼져 있었다. 사용자 승인을 받아 팔팀 `ros2_humble` 컨테이너에서
  `perception_node`(`pick_classes:=box-segmentation`) + `stream_node` 를 띄우고,
  우리 `arm_console_bridge` 를 붙여 실제 검출·거리 경로를 살렸다. 로봇팔은
  움직이지 않았다.
- 콘솔은 Xvfb(`GDK_BACKEND=x11`) 위에서 실제 GTK/GStreamer 로 기동했다.

## 2. 확인된 결함과 처리

| # | 결함 | 근거 | 처리 |
|---|---|---|---|
| A | 작업 카메라 스트림이 살아나면 PiP 가 부풀어 전방 화면을 덮음 | `live_full/shot_04.png`(실기), `pip1/02_both_live_later.png`(합성) | 고정 크기 슬롯 |
| A' | PiP 헤더 클릭 시 강등 패널이 스테이지 전체 점거·헤더 2개 중첩 | `act2/02_after_pip_header_click.png` | 클릭 스왑 제거, 명시 버튼·`V` |
| B0 | 거리 표시가 SDK depth 가 아니라 3D 직선거리 | 아래 §3 대조표 | `position_m[2]` 정본화 |
| B1 | pick target 이 없으면 최고 confidence 검출을 '대상'으로 승격 | 라이브에서 `is_pick_target:false` 인데 "작업 대상 0.31 m" 표시 | 폴백 제거 |
| B2 | `frame_id` 미검증 | 코드 | 화이트리스트 |
| B4 | dropout 후 EMA/median 잔존 | 코드 | 공백 초과 시 리셋 |
| B5 | draw 핸들러가 상태 필터를 갱신 | 코드 | 갱신 소유자 단일화 |
| B6 | 송신 `frame_width/height` 가 파라미터 기본값 | 코드 | 영상 caps 와 불일치 시 오버레이 억제 |
| C1 | `ops_broker` 사망으로 ops 채널 `:9001` 부재 | 컨테이너 로그 + 프로세스 환경 | 아래 §4 |
| C2 | E-STOP 가용성이 **기동 시 1회** 토큰 유무로만 결정되고 링크 상태를 반영 안 함 | 코드 | 동적 판정 + 상단 경고 |
| C3 | PDIST 링크 ERROR 인데 전원 카드 "정상" | `:5004` 원문 | freshness 와 health 분리 |
| C4 | 텔레메트리 송신 대상이 죽은 IP 고정 | 아래 §5 | 결정 필요 |
| C5 | `runtime_smoke` 의 Xvfb 격리가 Wayland 에서 무효 | 재현 | 자식 환경 X11 강제 |

**C2 관련 정정.** 처음에 "이 노트북에 토큰이 없어 버튼이 영구 비활성"이라고
적었으나 경로를 잘못 봤다. 콘솔은 `~/.config/powertrain/ops_console.token` 을
읽고 그 파일은 존재한다(젯슨 `/etc/powertrain/ops_console.token` 과 같은
49 바이트). 따라서 실제 상황은 더 나빴다 — 브로커가 죽어 있던 1시간 동안에도
**버튼은 눌리는 상태였고 명령만 허공으로 나갔다.**

10분 라이브 소크에서 RSS 159~190 MB(상승 추세 없음), fd 36±1, thread 20~22,
traceback 0, 조기 종료 없음 — **자원 누수는 없다**(음성 결과).

## 3. 거리 정본 대조 (핵심)

동일 대상(색 픽셀 612, 386), D435i SDK 직접 측정 20 프레임 평균:

| 값 | 측정 |
|---|---|
| SDK depth Z — 3×3 patch median | **0.2883 m** (sd 0.0005) |
| `rs2::depth_frame::get_distance()` | 0.2884 m |
| 컬러 프레임 Z (`position_m[2]`) | 0.2882 m |
| 3D magnitude — 수정 전 콘솔 표시 | 0.3199 m |
| 차이 | **+3.16 cm (약 11%)** |

magnitude = depth / cos θ 이므로 대상이 광축에서 벗어날수록(이 관측은 θ ≈ 25.7°)
오차가 커진다. `position_m[2]` 가 SDK depth 와 **0.2 mm 이내**로 일치하므로
프로토콜 변경 없이 콘솔에서 정본을 낼 수 있었다.

## 4. C1 — ops_broker 사망 (해결)

**증상.** `powertrain_control` 컨테이너가 unhealthy 였고 ops 채널 `:9001` 이
열리지 않았다. 콘솔의 조작·복구·비상정지 경로가 전부 불통이었다.

**직접 원인.** `ops_broker` 가 기동 즉시
`ModuleNotFoundError: No module named 'powertrain_msgs'` 로 종료했다.
런치 프로세스의 환경을 읽어 보니
`AMENT_PREFIX_PATH=/workspace/ros2/install/powertrain_ros:.../robot_arm_msgs:/opt/ros/humble`
로 **`powertrain_msgs` 만 빠져 있었다**. 같은 런치의 `teleop_command` 는 그
메시지를 import 하지 않아 살아남았다. 사후에 같은 컨테이너에서
`source install/setup.bash` 를 다시 해 보면 `powertrain_msgs` 가 정상 포함되고
import 도 된다 — 즉 그 기동 시점의 install space 만 stale 했다.

**더 큰 문제.** 컨테이너는 죽지 않았고 런치도 계속 돌았다. 그래서 ops 채널만
없는 상태로 **약 1시간을 조용히** 버텼다. `restart: unless-stopped` 는
unhealthy 로 재시작하지 않는다.

**처리.** `control.launch.py` 의 두 노드에 `on_exit=Shutdown()` 을 걸어 노드
사망이 PID 1 종료가 되게 했다. 그러면 compose 가 컨테이너를 다시 세워
오버레이를 재빌드·재소싱한다.

**검증(실기).** 재배포 후 `:9000`·`:9001` LISTEN + healthy. 그 상태에서
`ops_broker` 를 강제 종료했더니 컨테이너가 자동 재시작(RestartCount 1)되고
`:9001` 이 복구됐다.

## 5. C4 — 운용 호스트 IP (결정 필요)

젯슨의 텔레메트리 송신기들이 `--operator-host 192.168.8.206` 으로 고정되어
있는데 그 IP 는 현재 사용되지 않는다. 운용 노트북은 192.168.8.163 이다.
따라서 `:5004`(전원)·`:5005`(차대) 텔레메트리가 운용 노트북에 도달하지 않았다.

이번 검토에서는 노트북에 `192.168.8.206/24` 를 임시로 부여해 우회했고
검토 종료와 함께 제거한다.

후보:

1. 공유기(GL-SFT1200)에서 운용 노트북에 DHCP 예약을 걸어 .206 고정 — 즉효,
   코드 변경 없음.
2. 송신기의 operator host 를 배포 설정으로 빼서 현장에서 바꾸기 쉽게 — 대회장처럼
   네트워크가 바뀌는 상황에 필요.

권고: 둘 다 해 둔다. 충돌하지 않는다. **사용자 결정 필요.**

## 6. 팔팀 관련 (협의 필요)

- PR #3 의 `tools/deploy_d435_distance_fix.sh` 가 젯슨의 팔팀 레포
  `/home/zetin/extreme-robot` 에 **커밋되지 않은 수정**을 남겨 두었다
  (`perception_node.py`·`stream_node.py` 수정, `perception_quality.py` 신규).
  팔팀 소유 코드이므로 이번 작업에서 되돌리지 않았다. 팔팀에 알려야 한다.
- 젯슨에 배포돼 있던 `arm_console_bridge` 는 PR #3 이전 버전이라 pick target 을
  이전 프레임 bbox 와 **완전 일치**로만 매칭했다. 실측에서 `/pick_target`
  [506,293,215,187] 과 `/detected_objects` [503,293,219,187] 이 어긋나
  `is_pick_target` 이 항상 false 였다. PR #3 의 IoU 0.5 + freshness 0.75 s
  판정을 배포하니 **74/74 true** 로 회복했다.
- `perception_node` 의 `pick_classes` 기본값은 비어 있다. 그대로 기동하면
  `/pick_target` 후보가 아예 없다(기동 로그에 경고가 찍힌다). 시연 시
  `pick_classes:=box-segmentation` 지정이 전제 조건이다.

## 7. 수정 후 실기 재검증 (A·B 완료분)

전방 `:5000` + 팔팀 작업 `:5002` + 브리지 `:5003` 이 모두 살아 있는 상태에서
Xvfb(`GDK_BACKEND=x11`) 위의 실제 콘솔로 확인했다.

| 항목 | 수정 전 | 수정 후 |
|---|---|---|
| 작업 카메라 라이브 시 PiP | 전방 화면을 완전히 덮음 | 우하단 소형 유지, 전방 온전 |
| 헤더 클릭 | 강등 패널이 스테이지 점거 | **스왑 없음** |
| 버튼 / `V` | 없음 | 스왑되고 **PiP 크기 불변**(1280×720 이 들어가도 안 부풂) |
| 거리 표시 | 0.3199 m (3D magnitude) | **0.29 m** — 동시 측정 pick-target depth 0.2862 m |

거리 3자 대조: SDK depth 0.2883 m / metadata z 0.2862 m / 콘솔 표시 0.29 m
(표시는 소수점 2자리 반올림). 목표였던 ±5 mm 이내다.

`is_pick_target` 은 브리지 배포 후 **74/74 true** 로 회복했다(배포 전 0).

자동 회귀 테스트는 232 passed(기준선 215), runtime smoke PASS,
PiP 고정 크기 게이트는 음성 대조(결함 재주입 시 2건 FAIL)로 증명했다.

### 관찰 — metadata 간헐 STALE (조치 판단 필요)

42초 동안 3초 간격 14회 표본에서 metadata 패널이 **13회 LIVE, 1회 STALE**
이었다. 브리지는 10.6 Hz 로 계속 보내고 있었으므로 500 ms 를 넘는 간헐적
공백(추론 지터 또는 WiFi)이 원인이다. 이때 콘솔은 숫자 대신 "거리 정보 지연"
을 띄우므로 **동작은 옳다**(정체된 값을 정상처럼 보여주지 않는다). 다만
시연 중 거리 표시가 가끔 깜빡인다. `OVERLAY_STALE_AFTER_S`(0.50 s) 를
올리거나 짧은 공백을 dropout hold 로 덮는 선택지가 있는데, 표시 안전성과
맞바꾸는 문제라 임의로 바꾸지 않았다. **사용자 판단 필요.**

## 7-1. 젯슨에 남긴 변경 (배포 기록)

`~/power-train-sw`(우리 레포, 체크아웃은 `be0b9a0` 기준) — 파일 3개를 직접
반영했다. `git checkout`/`reset` 은 쓰지 않았고 팀원의 다른 미커밋 작업은
건드리지 않았다.

- `ros2/src/powertrain_ros/launch/control.launch.py` — `on_exit=Shutdown()`
- `ros2/src/powertrain_ros/powertrain_ros/arm_console_bridge_node.py` — pick target IoU·freshness
- `ros2/src/powertrain_ros/powertrain_ros/arm_console_mirror.py` — 위와 짝

`~/extreme-robot`(팔팀 레포)에는 **이번 작업에서 아무것도 쓰지 않았다.**
거기 남아 있는 미커밋 변경 3건은 PR #3 의 `tools/deploy_d435_distance_fix.sh`
가 이전에 남긴 것이다(§6).

검토를 위해 띄운 팔팀 스택(`ros2_humble` 컨테이너 + `perception_node` +
`stream_node`)과 `arm_console_bridge` 는 현재 **계속 실행 중**이다. 필요 없으면
`docker stop ros2_humble` 로 내리면 된다.

## 7-2. 최종 통합 검증 결과

- `operator_console/tests`: **249 passed** (기준선 215 → 신규 34건).
- `runtime_smoke`: PASS. 외부에서 환경을 강제하지 않아도 스스로 X11 로
  격리한다(Task 11).
- 젯슨 `powertrain_ros` 스위트(중립 cwd): 665 passed / 12 failed. 실패 12건은
  전부 `test_autonomy_controller_node.py` 로, `ec3e484` 이전부터 red 인
  기존 상태다. 배포한 브리지 때문에 잠시 깨졌던
  `test_arm_console_bridge.py` 는 PR #3 의 갱신된 테스트를 함께 배포해
  **30 passed** 로 정상화했다.
- 전 스택 라이브 10분 소크(전방 + 작업 + metadata + 텔레메트리 동시):
  표본 30개/580초, RSS **83~100 MB**(단조 증가 없음), fd 37~40,
  thread 21~23, **traceback 0**, 조기 종료 없음. 10분 뒤 화면도 PiP 소형
  유지·레이아웃 정상.
- 기동 시 뜨던 `Negative content width -1 ... owner VideoPanel` GTK 경고는
  슬롯 초기값을 1×1 대신 최소 PiP 크기로 바꿔 제거했다(재확인: 경고 0).

음성 대조로 게이트 자체를 증명한 항목:

| 게이트 | 결함 재주입 | 결과 |
|---|---|---|
| 거리 정본 | magnitude 로 되돌림 | FAIL (0.3087 ≠ 0.2882) |
| PiP 고정 크기 | 크기 오버라이드 제거 | FAIL 2건 |
| 스모크 격리 | `env=` 제거 | FAIL |
| 전원 health | rs485 분기 제거 | 처음엔 **통과**(다른 분기가 대신 잡음) → 링크만 ERROR 인 케이스를 테스트로 추가해 분기를 독립 검증하게 만든 뒤 FAIL |

## 7-3. 종료 시 hang·세그폴트 (사용자 실사용 발견)

**증상.** 창의 X 를 눌러도 터미널이 안 돌아오고, Ctrl+C 를 누르면
`segmentation fault (core dumped)`.

**근본 원인(코어 덤프로 확정).** 크래시 스레드는 `SRT:RcvQ:w1`(libsrt 수신
워커)이고 메인 스레드 스택은
`exit() → srt::CUDTUnited::~CUDTUnited() → srt::CSndQueue::~CSndQueue()
→ srt::CSndUList::~CSndUList() → pthread_cond_destroy` 였다. 프로세스
`exit()` 시점에 **libsrt 전역 소멸자가 자기 워커 스레드가 살아있는 채로 큐를
파괴**한다. 메인 스레드는 `pthread_cond_destroy` 에서 막히고(=창은 닫혔는데
터미널이 안 돌아옴) 워커는 해제된 메모리를 건드려 SEGV_MAPERR 로 죽는다.
두 증상은 같은 원인이다.

기각한 가설: 콘솔의 teardown 지연(실측 d435 0.005 s / l515 0.011 s /
수신 소켓 0.374 s / ops 0.000 s — 문제 없음), GStreamer 자원 미해제
(파이프라인을 명시적으로 NULL·해제해도 크래시 동일).

**조치.** `_on_destroy` 가 이미 파이프라인·소켓·ops 를 정리하므로,
`Gtk.main()` 반환 뒤 `os._exit(0)` 로 libc 의 exit 전역 소멸자 경로를 아예
타지 않는다. Ctrl+C/SIGTERM 도 창 닫기와 같은 경로로 흐르게 GLib 신호
워치를 걸었다(기본 SIGINT 는 `Gtk.main()` 안의 C 프레임을 파이썬 예외로
깨뜨려 정리 없이 나간다).

**게이트.** `runtime_smoke` 가 이제 SIGINT 로 콘솔을 내리고 **15초 내 종료 +
종료코드 0** 을 단언한다(래퍼가 아니라 콘솔 자식 프로세스에 신호를 보낸다).
음성 대조: `os._exit(0)` 을 빼면 `rc=139 · Segmentation fault (core dumped)`
로 FAIL. 실사용 경로 재확인 — X 버튼 상당 `close()` 와 Ctrl+C 모두
종료코드 0, 코어덤프 0.

## 8. 남은 일

- C4 운용 IP 결정(DHCP 예약 / 파라미터화).
- §7 metadata 간헐 STALE 임계값(0.5 s) 유지 여부 결정.
- 팔팀 협의: `~/extreme-robot` 미커밋 변경, `pick_classes` 기동 파라미터.
- 젯슨 배포분(§7-1)은 현재 워킹트리 수정 상태다 — 정식 반영 경로 결정 필요.
- PR #3 갱신(푸시) 여부 결정.
