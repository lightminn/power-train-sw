# 프로젝트 상태·인수인계 정본 (2026-07-16)

`docs/reports/2026-07-10-project-and-jetson-state.md`를 대체하는 최신 정본.
**새 세션/새 사람은 이 문서 → 아래 "정독 순서"만 따라가면 이어서 개발 가능**하도록 쓴다.
기준 커밋: main `149302e`(로컬·GitHub·젯슨 3-way 동기, 워크트리 clean).

## 0. 정독 순서 (새 세션 부트스트랩)

1. 이 문서 전체.
2. 마스터 계획 `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md` —
   WP 정의·순서·완료 기준의 정본. §9 순서표와 §WP6-S 절이 지금 진행 중인 부분.
3. 직전 구현 보고 2건: `2026-07-16-wp53-observability-implementation.md`(WP5.3 Task 1~5 +
   팀원 PR 통합), `2026-07-16-full-hil-safety-fixes.md`(첫 FULL HIL — 안전 결함 2건 수정).
4. 작업별 세부는 각 패키지 README(`powertrain_sim/README.md`, `ros2/README.md`의
   WP6-A 절)와 `docs/plans/2026-07-13-*` 두 계획.
5. 착수 전 필수 습관: GitHub(우리 + 로봇팔 `extreme-robot`)와 젯슨 로컬의
   dirty/ahead/behind 확인. 팀원 미커밋·미추적 파일은 보존.

## 1. WP 상태 한눈표

| WP | 상태 | 증거·비고 |
|---|---|---|
| WP1~3 (CAN 구동·키네마틱스·ChassisManager) | ✅ 실기 10모터 4WS HIL | 2026-07-05, 육안 확인 포함 |
| WP4 (ROS2 왕복 DDS) | ✅ | 로봇팔 실물 그래프와 양방향 |
| WP5/5.1 (`/cmd_vel` 체인 + 안전 코어) | ✅ HIL | 50.000 Hz, US-100 latch 시나리오 통과 |
| WP5.2 (팔 협업 안전: 계약 v2·ArmInterlock·CommandAuthority·원격 gateway·MissionSupervisor) | ✅ Task 1~6 + 감사갭 4건 | 07-14. 실기 원격 E2E 스모크만 벤치 잔여 |
| WP5.3 (관측성: journal/데몬/CAN health/depth 품질/팔 결과 adapter) | ✅ Task 1~7 SW | 6-A `f9d01df`·6-B `e6a2b24`(gateway 배선은 팀원 WIP 뒤)·6-C `f41730f`+`0198830`·7 `0d28552`. Task 8(최종 HIL)과 fault matrix 실 kill은 벤치 |
| **WheelStopPredicate 실측 자격화** | ✅ 오늘 HIL | `wheel_stop.yaml qualified: true`, 임계 0.10 rev/s |
| WP6-A (wheel+IMU 상태 추정 코어) | ✅ SW 완료 | `dfdfb32`. 실측 5 m ±5%·90°는 **차체 조립 후** |
| WP6-S P0 1부 (scenario 계약·analytic fixture·recorded replay) | ✅ | `9a5f37f`. production 추정기 합성 5 m 오차 0%, 피벗 yaw 0.0008% |
| WP6-S P1 (hidden-seed 폐루프) | ✅ | `d30ace1`. production terrain+controller 폐루프, hidden_eval CLI, 정직한 완주 의미론(fail-closed 종단 정지) |
| WP6-S P0 2부 (절차 생성 트랙 + MuJoCo fast 브리지) | ✅ | `e22e364`. 헤드리스 mj_multiRay depth(광축 Z), production solve() 직결, MuJoCo→replay→추정기 flat 거리 0.106%·피벗 yaw 0.199%, CLI 3/3 PASS(기대 메트릭 물리 캘리브레이션) |
| WP6-B (bank-aware NumPy terrain 코어) | ✅ SW 완료 | `eba8b74`. 54 tests + 광FOV MuJoCo 정량 통합. corridor/FOV-한계 낙하 경계 의미론, fail-closed. JAX 커널+NumPy 동등성 `5a415e9`(x86 29.8 ms). **잔여 게이트**: 장착각 20/25/30° HIL, Jetson 전체부하 JAX 자격화·backend 선택 |
| WP7 (선도 로봇 추종) | ✅ SW+fake target | `158b863`. 실기 UGV/대역 HIL 잔여 |
| WP6-C (autonomy controller + command authority) | ✅ SW 완료 | `c744936`. 순수 `powertrain_autonomy/controller`(BLOCKED vs CONTROLLED_HOLD vs TRACKING) + 단일 프로세스 `autonomy_controller` 노드(`guidance:=terrain`), `/odom_diagnostics`. authority는 WP5.2 것 그대로. **잔여**: 프로파일 잠정값 HIL(제동·뱅크·경사), 실기 terrain 유도 스모크 |
| L515 경량 파이프라인 | ✅ | 29.91 fps RGB SRT, raw depth 10 Hz |
| 원격운용 (teleop 유/무선 + operator_console) | ✅ | 콘솔은 팀원 PR #2 병합·정합화 |
| WP9 (환경 degradation 정책: slip/stuck/depth 열화 → 감속·bounded 복구·핸드오버 대기) | ✅ SW (C1) | `degradation.py` 순수 FSM + 노드/시뮬 배선. **실한계**: 균일 저마찰 slip은 휠+IMU로 검출 불가(0% — B2 실측), stall형만 실질(지령속 계약 체인). HANDOVER_WAIT 탈출은 운영자 조작만(§11) |
| WP8 구간 supervisor | ✅ SW 골격 | `72ec7e4`. 5구간 profile+MarkerDedup 순수 코어 + fake `/section_events` 어댑터. 공통 상태머신은 기존 MissionSupervisor 재사용. **잔여**: 크로스팀 인식 이벤트 실토픽·`MISSION_STOP` 언락 순서·풀 핸드셰이크 1사이클 |

## 2. 2026-07-15~16 개발 로그 (커밋 체인, 전부 main 푸시·젯슨 배포)

| 커밋 | 내용 |
|---|---|
| `a16a5fe` `4c0885a` `12fe45d` `ce6368d` `8dd2b54`~`9aec6eb` | WP5.3 Task 1·2·3·5·4 (보고서 §1 참조) |
| `fd6aa09` + `a1977d0` | 팀원 PR #1 병합 + **pre-hold 명령 재생 버그 수정**(래퍼 우회 금지 계약 테스트) |
| `484530a`~`1f21752` | 팀원 PR #2 operator_console 병합 + 정합화 4건 + 레이아웃 재배치(`scripts/systemd/`) |
| `dfdfb32` | **WP6-A** `state_estimation.py` 순수 코어 + odometry/imu_tilt 노드 어댑터화 |
| `a191116` `149302e` | 블로킹 서비스發 거짓 safety-stale 래치 근본수정(`_refresh_safety_baseline`) |
| `0a89098` `fe67096` `dc7ebc8` | wheel-stop 자격화(YAML·고정 테스트·원천 bag) |
| `9a5f37f` | **WP6-S P0 1부** `powertrain_sim/`(scenario·fixtures·recording, 42 tests) |
| `09cb606` | US-100 발행-UART 결합 해소(리더 스레드) |
| `e22e364` | **WP6-S P0 2부** 절차 생성 고가 트랙 + 헤드리스 MuJoCo fast 브리지(광축 Z depth·production solve() 직결·metric 6종·기대 메트릭 물리 캘리브레이션, dev 이미지 mujoco 추가) |
| `eba8b74` | **WP6-B** bank-aware NumPy terrain 코어(고정 shape 5 cm grid·corridor/FOV-한계 낙하 경계·footprint erosion·fail-closed, 54 tests + 광FOV MuJoCo 정량 통합, autonomy 이미지에 chassis 동봉). ⚠️ Codex 위임분을 검토자가 낙하 경계 의미론·성능(107→31 ms) 근본 재설계 |
| `84a4dbd` | **원클릭 GUI 브링업 `scripts/jetson_gui_up.sh`(07-19)**: 콘솔 의존 젯슨측 전부를 한 명령으로 — 호스트 preflight(can0·런타임 dir·powertrain.env·ops 토큰) → 파워트레인 compose 5종 명시 기동(bare `up -d` 금지)+헬스 대기 → 텔레메트리 유닛 자가회복 대기(Restart=always 활용, sudo-free 경로) → 팔 스택(ros2_humble + perception/stream, 팔 레포 `run_perception.sh` 인자 그대로) → **:5003 단일송신 가드**(팔 metadata_sender 감지 시 브리지 스킵, 팔 프로세스 kill 금지) → `arm_console_bridge`(powertrain_ros 다운 시 powertrain_control 폴백, 양 컨테이너 pkill로 이중송신 방지) → SRT lazy-start 폴링 포트검증 + 한국어 요약표(exit 0/1/2). fake-PATH pytest 8종(`scripts/tests/test_jetson_gui_up.py`). **젯슨 실기 2회(07-19)**: 1차 exit 2(SRT 첫 프레임 전 lazy 포트) → 폴링 보강 후 2차 전항목 ✅ exit 0 + 노트북 UDP :5004/:5005/:5007 실수신 확인. 부수 발견: ①chassis-telemetry 유닛 5,904회 크래시루프는 powertrain_ros 정지가 원인(컨테이너 회복으로 자가치유 — 스크립트가 이 경로 자동화) ②**젯슨 체크아웃이 팀원 브랜치 `feat/l515-aligned-depth-slam`(a177fe2)로 전환됨** — main 미머지라 스크립트는 untracked 배치, 브랜치 무접촉 유지 |
| ~`9475ca5`+ | **KGUI 후속 2건 + 종합 재검증(07-18 밤)**: ①비상정지 원인 표시 — interlock first_source/detail → OpsState → 상태줄 한국어, 이후 사용자 지적으로 **활성 원인 우선**으로 의미론 교정(활성 나열 / 전부 해소 시 "원인(최초) … 초기화 가능") ②종합 적대 리뷰(Codex)에서 실결함 2건: **IMMEDIATE 비상정지가 상태 push 수신 전 거부 가능(높음 — 전제조건 제거로 무조건 발사)** + 원인 라벨 미매핑 5종. ③재검증 전체 green: 호스트 408→176(콘솔)·dev 1254+3skip·ros 546·smoke PASS + **젯슨 GRAND-E2E PASS**(활성/이력 원인 양쪽 실원인·멱등·토글 왕복·복구 7단계) + 배포 건강(컨테이너 5 healthy·유닛 3 active·크래시 0). ⚠️ 설계 유보 항목: 브로커 변이 직렬화 중 estop 최대 명목 대기 10.05 s(busy 거부·재시도 멱등, 1차 비상경로는 DualSense ○) — estop 직렬화 우회는 후속 결정 |
| `ed8a7a5`~ | **KGUI 배치(07-18 저녁)**: 콘솔 간소화+전면 한국어화(광운대 참조, 스펙 `2026-07-18-console-simplify-korean-design.md` D1~D4+보충). ①기본 화면 = **비상정지(신규 — 무확인 즉시, `estop` ops 액션·기존 `~/estop` 서비스 정비·CONSOLE_ESTOP 저널)**·경고 초기화·시동(1.5 s 홀드)/해제·모듈 스위치 4행(기본 전부 켜짐 = CMASK 무영속 계약); 권한·extraction·팔잠금·hold해제는 "고급" Expander ②OpsState `chassis_mode`(semantic) → 상태줄 모드 표시 + 모터 토글 IDLE 게이트 회색화(`not_idle` 혼동 해소) ③전면 한국어(labels.py 순수 모듈, 상태코드 병기)·배너·요약 1줄+상세 Expander(전원/차대/팔) ④runtime_smoke가 임시 토큰으로 ops 패널 활성 경로까지 실기동. 기준선: 호스트 146 / dev 1230+3skip / ros 540 + **smoke PASS** + **젯슨 비상정지 버튼 실전 E2E PASS**(estop 무확인 latch→멱등→us100 OFF→reset IDLE→원복). Notion §5/6 매뉴얼 한국어 기준 갱신 |
| `f9a8ac7`~ | **콘솔 실기동 사고 3건 + 브로커 영구 웨지 수정(07-18 오후, 사용자 실기 테스트發)** — ①환경 2건: conda `gi` 부재(→ `/usr/bin/python3` 고정), Arch가 gtksink 분리(→ `gst-plugin-gtk` 설치) ②**ChassisTelemetryPanel `_rss` AttributeError**: 타 클래스 staticmethod 호출 — 차대 텔레메트리가 LIVE일 때만 도는 경로라 스위트 전부 통과 후 실기동에서 발화(→ 포매터 4종 gi-무관 telemetry 모듈로 공유, 소스스캔 봉인) ③**정의: 실행 게이트 신설** — `operator_console/runtime_smoke.py`(Xvfb 실기동+4채널 LIVE 주입+STALE/sparse 페이즈+traceback 검출, **음성 대조로 게이트 증명**, 호스트 스위트 편입) + CLAUDE.md/AGENTS.md 「완료 선언 규칙」 ④Codex 적대 리뷰: 23클래스 self-attr AST·74필드×132접근·ops 키 패리티 — 잔여 0건 ⑤**브로커 영구 웨지**(CMASK 토글 실사용에서 발견): 부재 서버로의 call_async가 'late completion 대기'로 변이 슬롯 영구 점유 → 전 명령 busy 거부. `SERVICE_ORDER_ABANDON_S`(10 s) 2차 데드라인 — never-ready→FINAL_REJECTED(unavailable)/ready-무응답→OUTCOME_UNKNOWN, future cancel+클라이언트 파기+슬롯 해제+저널. 콘솔 Ops 패널에 최근 ACK 인라인 표시. ⑥벤치에 wp5_control 스택(us100_safety+chassis 실기, stop_mm 200) 세션 기동 — **실전 순서 E2E PASS**(us100 OFF→reset IDLE→drive OFF→원복, 전부 FINAL_SUCCESS). 기준선: 호스트 115(operator_console+tests)/ros 531 |
| `fdb0350`~`32d20a2` | **CMASK 배치**(스펙 `docs/superpowers/specs/2026-07-18-component-mask-design.md`, 계획 `2026-07-18-cmask-component-toggles.md`): 콘솔에서 구동·조향·US-100·로봇팔 개별 on/off — OFF = 미장착 모드(명령·감시 스킵, 해당 소스 estop/hold 비체결). ①순수 코어(`419ca6a`): 코너 enable 플래그 + `ChassisManager.set_component_enabled`(모터는 IDLE 한정, D3 무영속 — 재시작=전부 ON). **리뷰어 수정: command_recovery hold 자동 해제 제거**(pre-hold 명령 재생 방지 의미론 보존 — fresh set()만 해제) ②배선(`cbee989`): SetBool 서비스 4개 + 순수 초크포인트(update_external_safety·set_safety_link_stale·set_arm_motion_hold) + safety_state/OpsState semantic 필드 + COMPONENT_MASK 저널. **리뷰어 수정: us100 OFF 시 `safety_topic_stale` 활성 조건 동반 해제**(마스크가 갱신 경로를 막아 reset 교착) ③콘솔(`32d20a2`): 토글 4행(현재값 반전 제출·상태 전 비활성), US-100 해제 위험 문구, 주황 MASK 배너 + `SAFETY DISABLED (US-100 OFF)` 우선, :5005 미러. 기준선: 호스트 345 / dev 1192+2skip / ros 527. **젯슨 실기(07-18)**: parity 527 + 설치 엔트리포인트 FAKE 스모크 PASS(도메인 77 — 안전 미수신 stale ESTOP latch → us100 OFF 시 활성소스 소멸·latch 유지 → reset IDLE → drive IDLE 토글 왕복 → us100 ON 재latch, killpg 좀비 0). 참고: 벤치 상시 스택엔 chassis_node 없음(CAN 다운 정책) — 라이브 콘솔 토글 왕복은 다음 HIL 세션에서 확인 |
| `9cb8f3e` | **chassis telemetry 크래시 루프 수리(07-18 실사고)** — Gateway/Observability 클라이언트의 frozen(MappingProxyType) payload 중 `ros_topic_rates_hz`가 JSON에 직삽입돼 "mappingproxy is not JSON serializable" 크래시(03:03부터 5,586회 재시작; 게이트웨이가 응답을 시작하자 발화). 인코더(`chassis_telemetry.py`)가 경계에서 deep-thaw + 회귀 테스트(ros 512). **동반 발견 2건 수리**: ①젯슨 설치 유닛이 구버전(소스트리 직접 실행 — 금지 방식) → `install_chassis_telemetry_service.sh`로 정본 재설치 ②`powertrain_ros` 컨테이너가 2일 미재생성(pre-C1 install space, WheelState msg 불일치 위험) → force-recreate. 수리 후 30 s 창 실패 0·active. ⚠️ 교훈: 텔레메트리 유닛 상태는 배포 체크리스트에 포함할 것(§12 Notion GUI 페이지 트러블슈팅 반영) |
| `93e26db`~`d27e537` | **ARM-CON 배치**(계획 `docs/superpowers/plans/2026-07-18-armcon-console-bridge.md` r2): 로봇팔 팀 콘솔 요청 4종을 **팔 레포 diff 0**으로 구현. ①`arm_console_bridge` 노드(`3ed5610`) — `/dynamixel/state`·`/joint_states`·`/detected_objects`·latched `/pick_target` read-only 미러 → UDP :5007(5 Hz, 방어파싱: 5배수·8모터·범위, joints-우선 절단) + :5003 **superset**(팔 metadata_sender 스키마 + `yaw_rad`(2·atan2(z,w), (−π,π])·`is_pick_target` — 단일송신 원칙: 브리지 가동 시 팔 sender 미기동) ②콘솔 `ArmTelemetryPanel` + 검출 yaw·픽타깃 주황 오버레이(`d27e537`, 온도 임계 55/65 ℃ **임시 — 팔 팀 확인 대기**, `arm_telemetry.py` 송신봉인 목록 추가, :5003+:5007 크로스 계약 테스트) ③**r2 개정: 영상 브리지 폐기** — 팔 레포에 `stream_node`(:5002 SRT) 기존재 확인, 기존 콘솔 패널로 충족. 기준선: 호스트 332 / dev 1163+2skip / ros 511. **젯슨 실기(07-18)**: parity 511×2 클린 + 설치 엔트리포인트 fixture 스모크 PASS(도메인 77, ephemeral 포트, 2모터 90.0°/0.0°·온도·yaw 0.6·픽타깃 True 왕복, killpg 좀비 0). 라이브 체크: 팔 perception 미가동(`/detected_objects` 발행자 0)이라 실데이터 E2E는 협조 세션 이월. ⚠️ 감시: `/joint_states`는 현재 팔 `joint_state_broadcaster` 단독 발행 — 우리 스택이 동일 토픽을 발행하게 되면 브리지에 이름 필터 필요 |
| `6c47583`~직전 | **A/B/C 프로그램 C1 배치(최종)**(계획 `docs/superpowers/plans/2026-07-18-c1-wp9-degradation.md`): ①지령 휠속 계약 체인(`WheelState.msg` additive `command_turns_per_s` — snapshot→adapter→odometry seam, **stall형 slip/stuck 실질화**; msg 변경 = 3환경 재빌드) ②**WP9 순수 degradation FSM**(NORMAL→SLOWDOWN→HOLD_RECOVERY→HANDOVER_WAIT, depth hysteresis, 시도/시간/거리 bounded budget — §11, HANDOVER_WAIT 탈출은 운영자만) ③노드+시뮬 배선(`/autonomy/degradation_state`·DEGRADATION journal·smog seed-2 SLOWDOWN 1.42~2.40 s 실측·앵커 불변) ④**정본 개정 4건 이행**(콘솔 헌장·§10-4 TRACKING 시뮬 대체·US-100 extraction·캘리 NVM 경로)+WP표 WP9 행+Notion 콜아웃 동기. 기준선: 호스트 315 / dev 1146+2skip / ros 485, 캠페인 dev 8/8 exit 0. **젯슨 실기(07-18)**: msg 재빌드 후 parity 485×2 클린 + FAKE autonomy `degradation_state` 라이브 스모크(NORMAL/1.0, process-group kill 좀비 0) |
| `81c9996`~직전 | **A/B/C 프로그램 B2 배치**(계획 `docs/superpowers/plans/2026-07-18-b2-sim-portfolio.md`): ①마찰 patch fixture + `closed_loop` DriveDiagnostics 배선 — **정직 발견: production 추정기는 균일 저마찰 slip 검출 0%**(휠+IMU가 절대 종방향 지면속도를 못 봄 — C1 slip 정책의 실한계로 기록, 추정기 불변) ②`depth_degradation` 램프(결정론 dropout/noise, 열화 hold→dwell 복귀) ③**동적 선도 표적 폐루프**(detections_source 훅·표적 plant·가림/dropout — 5구간 추종 배점의 유일한 plant-급 경로. **정직 발견 2: WP7 P-제어 정상상태 오프셋** d=target+v/kp=2.625 m — 정지 표적용 1.5~2.5 밴드는 이동 표적에서 물리 불가, 평형 밴드 재정박·1.5 m 침범 0·5 s 가림 재획득 0.02 s. **lead-속도 feedforward = 벤치 개선 후보**) ④캠페인 러너(가족×시드, hidden=해시만, 가족별 보정 dev 시드) — dev 매트릭스 8/8 exit 0. 기준선: 호스트 302 / dev 1131+2skip / ros 480 |
| `1966510`+`a4ea2b7` | **A/B/C 프로그램 B1 배치**(계획 `docs/superpowers/plans/2026-07-18-b1-sim-families.md`, 트랙 영구 대체): 시뮬 S급 가족 — ①핀치포인트 폭 협착(footprint W=0.949 m 상수, 넓으면 완주·좁으면 핀치 앞 fail-closed 정지) ②클로소이드 곡률(|dκ/ds|≤0.08 연속 변화) ③기복 지형(±0.05 m/λ2 m, pitch 유한 — 종단 fail-closed 의미론상 '완주' 대신 ≥30% 주행 앵커 = P1 정직 의미론) ④**fixture_class 실행 계약화**(증거 있는 3종만 executable, 나머지 declared-only 명시 강등 — 라벨 과장 제거) + regression 러너 계약 검증. 전부 additive(기존 가족 RNG 순서 보존, 앵커 0.709/0.20 불변). 리뷰어 수정: A3 dwell로 stale해진 wide_fov_drop recovery 한도 0.0→0.2 s. 기준선: 호스트 278 / dev 1107+2skip / ros 480. 젯슨 = repo 동기+parity(시뮬은 dev 컨테이너 전용 — 실기 표면 없음) |
| `99e1bb3`~`4282aaa` | **A/B/C 프로그램 A3 배치**(계획 `docs/superpowers/plans/2026-07-18-a3-execution-wiring.md`): 실행 배선 — ①`/section/state` versioned(session/sequence/stamp/ttl, additive) ②순수 `SectionEnforcer`+chassis 집행(`section_enforcement` 기본 off — speed_hint v클램프·hold_hint v/ω 0·stale/역행/미래 fail-close·휴면 플로어 가드, 최종 `cm.set()` 직전, SECTION_ENFORCEMENT journal — **신호 20점 실패조건 폐쇄 경로**) ③마커 ledger 후보→확정 2단계(N=2·TTL 5 s·상한 16, unique는 확정만 — 3구간 50점 오확정 방어) ④teleop typed 큐(모션 latest-only+drop 카운터·수명 deque 8·violation 병합 64·E-stop 수신 즉시 락 latch·overflow→MOTION_HOLD) ⑤복귀 dwell = ticks AND ≥0.15 s AND 신선 표본 3(폐루프 앵커 정직 갱신: recovery 0.04→0.20 s, dev-seed 완주핀 0.75→0.70 — passed/fail_open 불변). 기준선: 호스트 264 / dev 1093+2skip / ros 480. 리뷰어 개입: §9-3 getattr 관례 위반 수정+pipefail 게이트 강화, T5는 Codex 중단으로 직접 구현. **젯슨 실기(07-18)**: parity 480(3회 중 2클린, 간헐 1회 감시 계열·좀비 0 확인) + FAKE(enforcement+authority on) 부팅 스모크 healthy·safety_state 발행·process-group kill 잔재 0 |
| `20df10d`+`4d30ce4` | **A/B/C 프로그램 C0 배치**(계획 `docs/superpowers/plans/2026-07-18-c0-extraction.md`, **WP5.1 US-100 의미론 개정 이행** — 스펙 §0 정본 개정 3): 후진 한정 EXTRACTION 상태 — US-100 **단독** latch에서만 콘솔 grant, 인터록 latch 보존한 채 코너 재-arm, 최종단 clamp −0.2≤v≤0·ω=0, TTL 3 s+에피소드 1.0 m+grant 3회 budget, us100 외 원인 발생 즉시 ESTOP 복귀. 배선 = `~/extraction_grant` 서비스 + ops action(console 전용) + 콘솔 패널 강확인 항목. §6.1 상태표 12계약 순수 테스트. **`extraction_enabled` 기본 False — 실모터 후진 풀사이클 벤치 HIL 통과 전 실기 활성 금지**. **젯슨 실기(07-18, 비회전 FAKE)**: 거부 경로 2종 라이브 확인(`estop_not_latched`·`active_estop_sources_not_us100_only`) + broker force-recreate 후 `extraction_grant` action 수락(PENDING) + parity 470×2(중간 실패는 프로브 좀비 §9-5가 원인 — 정리 후 클린) |
| `d1253d7`~`7fac0a4` | **A/B/C 프로그램 A2c 배치**(계획 `docs/superpowers/plans/2026-07-18-a2c-bringup.md`): 무SSH 브링업 — `preflight`(env·stop_mm provenance·토큰 검증 + `__main__`), 순수 `boot_qualification`(플래그+보드 지문+전압), **chassis 소유 자격화 게이트가 모든 arm 경로 통과**(미자격→estop fail-closed, 기본 미설정=현행), `board_registry`(시리얼→노드쌍)+`bl70200_setup --serial/--axis/--persist-calibration`(fw 0.5.1 전제), `can_calibrate_all` 라이브러리화+`CalibrationJob` 상태기계, 비콘(pdist 센더 unit/compose/journal_tail 필드+토큰 redaction), oneshot preflight 유닛+installer, compose 기동 전 preflight 재검증. 기준선: 호스트 259 / dev 1063+2skip / ros 466. **젯슨 실기(07-18, 비회전)**: `/etc/powertrain/powertrain.env` 프로비저닝(STOP_MM=200·BENCH) → preflight 통과+BENCH 경고 실확인 → oneshot 유닛 설치·기동 SUCCESS(enabled, 부팅 자동) → control `--force-recreate`로 **컨테이너 내 preflight 재검증 라이브 확인** 후 healthy → pdist 유닛 `--include-unit-status` 갱신·재시작 → 비콘 payload 라이브 수신(unit_status에 신설 유닛 active·compose_status control healthy·journal_tail) → parity 466(간헐 1회 비재현). **벤치 이월(실모터)**: NVM 영속화 실행·캘리 lifecycle 실행·조립 전 3회 전원사이클 게이트 |
| `196d856`~`8f15e72` | **A/B/C 프로그램 A2b 배치**(계획 `docs/superpowers/plans/2026-07-18-a2b-console-panel-haptics.md`): 콘솔 운용 패널 + 햅틱. `arm_lock_override`(SetBool) 계약 추가, `ConsoleOpsClient`(laptop 패키지화·스레드 래퍼·bounded queue·A2a 채널 재사용), `ops_panel.ConfirmFlow`(2단 확인·revision 재검증 TOCTOU·estop_reset↔arm STRIP/1.5 s HOLD 분리+spacer·16 action), GTK `OpsPanel`(인라인 확인·토큰 부재 비활성)·**헌장 개정**(배너 OBSERVE:RX-ONLY·OPS:TOKEN-GATED, README/독스트링, **송신 표면 계약 테스트**로 "ops 클라이언트 외 제어 송신 없음" 봉인), `haptic_arbiter`(단일 우선순위·stale>0.5 s 강제 link-loss·전이 1회 펄스·lightbar 색), `dualsense_output`(pydualsense lazy·격리 스레드·예외 시 영구 비활성·입력 무영향·opt-in trigger-fx). 기준선: 호스트 254 / dev 1026+2skip / ros 458. **젯슨 실기(07-18)**: 458 parity(2연속; 첫 실행 웜업 플레이크 1회 — A1과 동일 패턴, 재현 안 됨) + `powertrain_control` --force-recreate로 A2b broker 반영(⚠️ 코드는 컨테이너 **시작 시** colcon 빌드로만 반영 — 설정 불변 시 `up -d`는 no-op, `--force-recreate` 필요) + 콘솔 토큰 라이브 핸드셰이크 `role=console` + `arm_lock_override`(data 부재)→`FINAL_REJECTED "params.data must be bool"` 왕복 확인. **벤치 이월**: 콘솔 GUI 육안(2단 확인 UX)·햅틱/트리거 체감·chord 실감 |
| `399d485`~`930f37e` | **A/B/C 프로그램 A2a 배치**(계획 `docs/superpowers/plans/2026-07-18-a2a-ops-broker.md`): ops 채널 :9001 — 와이어 계약·action 표(`ops_contract`), 순수 코어(역할 토큰 인가·세션 단조 sequence·멱등 pending/final 캐시·단일 mutation 직렬화·rate limit·**비상 2단계 서버 시간 검증**(reset 5 s/arm 3 s + neutral/fresh/stopped 게이트)·authority 전이표·stale 상태 거부), `ops_broker` 노드(TCP·call_async 1 s 타임아웃=PENDING 유지+late final push·composite 부분 성공 분리 보고·5 Hz ops-state push(의미 전이만 revision)·OPS_COMMAND journal), 상태 소스 `/teleop/gateway_state`·`/chassis/safety_state`(+공개 `safety_snapshot()`), 배포 `control.launch.py`+compose healthcheck :9000+:9001, 노트북 `ops_channel_client`+복구 chord(전부 임시 후보). 리뷰어 수정: rclpy `Node._clients` 섀도잉 버그 + `/etc/powertrain` ro 마운트 갭(`eeaf2e2` — 실기 스모크가 발견). **젯슨 실기 검증(07-18)**: 스위트 456 parity + 토큰 프로비저닝(0600) + `powertrain_control` 재생성 healthy(:9000+:9001) + 노트북 발 라이브 핸드셰이크 `FINAL_SUCCESS role=controller`·status_query OUTCOME_UNKNOWN·push revision 수신. **벤치 이월**: chord 실감·비상 hold 체감(DualSense 물리) |
| `7be6f7e`~`18cc97b` | **A/B/C 프로그램 A1 배치**(스펙 `docs/superpowers/specs/2026-07-17-abc-program-design.md` r6, 계획 `docs/superpowers/plans/2026-07-17-a1-estop-floor-ff.md`): ①`DriveOdriveCan` friction_ff/v_knee 저속 마찰보상 노브(D4, 기본 off — 값 튜닝은 벤치) ②build_real_corners→teleop CLI/chassis_node 파라미터 배선 ③**min_rev 플로어 폐지**(D3 — 기본값 전면 0, 메커니즘은 opt-in 보존) ④`/teleop/estop` TRANSIENT_LOCAL latched 발행(event_id, 1 s 재발행) ⑤chassis_node 멱등 dedup→`cm.estop("remote_operator")` 전역 latch(○=전역 E-stop 계획 정합). A1 후 기준선: 호스트 240 / dev 991+2skip / ros 418 → A2a 후: 호스트 240 / dev 998+2skip / ros 456 |
| `d665228`+`0cff49e`+`42800f4` | **2차 렌즈 리뷰 하드닝 10건**(실입력/네트워크/executor/복구 렌즈 — production 표면 전체): 레거시 NaN 파서·Pi 워치독+배너·레거시 NODELAY·:9000 half-open 5 s 종료·이벤트 큐/violation 캡·콘솔 송신자 고정+seq 검증+LIVE 동결 수정·payload 4096/8192 계약·sender poll 워커 분리·controller 복귀 dwell(3 tick, 임계 불변)·GatewayClient 예외 봉쇄. + 테스트 하네스 스트림화 + **DDS 도메인 격리 conftest**(함정 §9-0). 렌즈 D(현장 복구 입력 부재)는 설계 안건으로 이관 |
| `1ec012c` | **HIL 결함 4호 수정** — autonomy 노드 terrain 처리를 latest-only 슬롯+워커로 분리(executor 기아 해소), 발행 케이던스 회귀 테스트 |
| `2c30abc`+`6bb4029`+`6f64b98` | **07-17 벤치 HIL 수정 3건+플래그** — 스틱 데드존, TCP RST 서버 사살, 전 원격 엔드포인트 NODELAY, wp5 launch authority_enabled 플래그 |
| `dcd5d11` | **독립 리뷰 하드닝 9건** — Codex 독립 리뷰(H2/M6/L2)+검토자 이음새 패스. H: follow 비유한 검출 차단, odometry stale 재스탬프 금지(발행 생략으로 freshness 전파). 검토자 발견: **assist operator-중립 게이트**(중립 조종자에 ω 주입 금지 — 의도 없는 피벗·wheel-stop 차단 방지). M: 클럭 롤백 dt 클램프, 재획득 identity 귀속, 전이 에지 zero 1회, hidden_eval no_progress 게이트. L: 섹션 이벤트 역행/EXIT 가드, overlay seq/age. 유지 판정: bypass-unknown→raw(advisory 설계), overlay 프레임 상관(물리 불가) |
| `72ec7e4` | **WP8** 5구간 section supervisor SW 골격 — 순수 `section_profiles.py`(스모그/구호/마커5종 dedup/빙판 stuck/추종) + fake 계약 어댑터 노드, 모터 명령 없음. 크로스팀 계약 확정 뒤 배선 |
| `5a415e9` | **WP6-B JAX** 고정 shape terrain 커널(`terrain/kernel.py` 공용 경계 + `jax_backend.py` JIT·warmup·재컴파일 가드·CPU 경계 검증) + NumPy 동등성 테스트(importorskip). NumPy 수치 불변(x86 29.8 ms/프레임), Jetson 자격화·backend 선택은 실기 게이트 |
| `158b863` | **WP7** 선도 추종 완성 — 팀원 코어 확장(2.0 m 목표·1.5~2.5 band·1.5 m hard stop·가림 예측 감속·예측 한계 1회 0 명령·2-frame 재획득 게이트), 노드 TF 게이트(base_link 변환, frame/TF 부재·stale 시 미발행 — 광학 프레임 가정 제거) |
| `d30ace1` | **WP6-S P1** hidden-seed 폐루프 — MuJoCo fast → production TerrainEstimator → AutonomyController → plant. `python -m powertrain_sim.hidden_eval <seed> <dir>`(sha256 기록, exit=passed). 폐루프 생성 문서는 `expected_completion=False`(fail-closed 정지가 전방 코너 반경 0.55 m 앞 = 정답, 95% 완주 물리 불가), hold 계측은 결정이 소비한 terrain 기준(1-tick 위상 아티팩트 제거). ⚠️ Codex 태스크 중지→검토자 인수 마무리 |
| `0d28552` | **WP5.3 Task 7** 환경 regression manifest(9항목, sha256) + `run_autonomy_regression.py`(backend 교차비교·명시적 SKIPPED) + 채널 9종 fault matrix 계획 코어·벤치 래퍼(승인 게이트, 실 kill은 벤치 몫) |
| `3c1e098` | **health matrix 유휴 플래핑 근본수정** — 비ARMED `CornerModule.tick()`이 반응 없는 RX 서비스 수행(캐시 실시간화). 실기 유휴 확인만 잔여 |
| `0198830` | **WP5.3 Task 6-C 2부** 노트북 recv_remote_operation 듀얼영상 뷰어 — SRT 2채널 stall-재기동, OVERLAY_STALE 규칙, :5006 역방향 피드백(1 Hz, Jetson 파서와 왕복 테스트), DualSense 단일 오픈·요청/ACK 분리 표시 |
| `e6a2b24` | **WP5.3 Task 6-B** network profile·receiver feedback·remote_video 순수 코어(신규 파일만 — gateway 배선은 팀원 l515 WIP landing 뒤). NORMAL/CONGESTED/EMERGENCY_REMOTE 고정 프리셋+불변식, receiver-authoritative 상태머신, D435i metadata 수신 계약(OVERLAY_STALE=로컬 monotonic TTL만). 부수: 콘솔 재배치 때 깨진 stale 테스트 2건 수리(tmpfiles 경로, docker CLI 호스트 전용 skip) |
| `f41730f` | **WP5.3 Task 6-C 1부** remote assist — 순수 `chassis/remote_assist.compose()`(TELEOP 선택 후 합성, \|v\| 불증가·부호 불변, bypass/correction stale=fail-closed), 원격 입력 **스키마 v2**(`assist_bypass`, R1 hold 초기 후보, v1 거부), `/autonomy/assist_correction` 발행, chassis_node `assist_enabled`(기본 off)+REMOTE_ASSIST 이벤트 |
| `f9d01df` | **WP5.3 Task 6-A** 콘솔 CAN 표시 일원화 — sender의 passive can0 RX/sysfs 제거, daemon 캐시 `CAN_HEALTH`(owner 측정) → 순수 `console_can_status.can_status_text`, allowlist 축소. ⚠️ 젯슨 ros 스위트에서 1/366 비재현 타이밍 플레이크 1회 관측(재실행 2회 GREEN — 감시 항목) |
| `c744936` | **WP6-C** terrain autonomy controller: 순수 코어(`controller/core.py` — BLOCKED=팔 게이트 즉시 0+slew 리셋 / CONTROLLED_HOLD=stale·경로상실 profile 감속 ramp·자동복구 / TRACKING=clearance·bank·slope·confidence·slip·speed-cap 스케일+곡률 감속+slew, EMPTY_STOWED·CARRYING_LOCKED 잠정 프리셋+보수 불변식) + 단일 프로세스 ROS 어댑터 `autonomy_controller`(첫 CameraInfo에 60×80 중앙크롭 격자 고정, odometry delta는 terrain 성공 후에만 전진(검토자 수정), hold도 0 발행 유지, 첫 estimate 전 발행 금지) + `/odom_diagnostics` + WP6-C 소유권 계약 테스트 |

WP6-A 코어의 검토자 수정 2건(다른 세션에서 알아야 할 설계 결정):
- `max_bias_rad_s=0.05` **bias 타당성 게이트** — 바퀴 정지 중 큰 gyro는 bias가 아니라
  실회전(예: 빙판 미끄러짐)으로 적분한다. 테스트
  `test_large_gyro_during_wheel_stationary_integrates_as_rotation`.
- **클럭 도메인 분리** — freshness는 수신 클럭(`_last_wheel_seen_s`/`_last_imu_seen_s`),
  적분은 stamp 도메인. WP5.2 `stamp_domain` 결함 클래스와 동일 원칙.

## 3. 아키텍처 맵 (패키지 → 역할)

- `motor_control/` — 하드웨어 계층. `chassis/`(ChassisManager+키네마틱스+wheel_consistency),
  `corner_module/`, `safety_us100/`(순수 판정), `steering/`·`drive/bl70200/`(드라이버).
- `ros2/src/powertrain_ros/` — 얇은 ROS 어댑터 계층. 순수 코어(arm_interlock, wheel_stop,
  remote_input, command_authority, mission_supervisor, **state_estimation**) + 노드(chassis_node,
  us100_safety_node, odometry_node, imu_tilt_node, l515 gateway). **패턴: 정책은 순수
  파이썬, 노드는 I/O만.**
- `powertrain_observability/` — WP5.3. journal/health 코어 + 데몬(abstract socket
  `@powertrain-observability-events/status`, SCM_CREDENTIALS, flock 싱글턴,
  `/var/lib/powertrain/runs`) + TUI.
- `powertrain_autonomy/` — WP5.3 Task 4. depth 품질/sensor-TF qualification(NumPy 전용),
  `l515_commissioning` CLI(YAML SHA-256 동결, repo YAML은 fail-closed unapproved).
- `powertrain_sim/` — WP6-S. scenario.yaml 계약(SI·frame·PCG64·seed 분류
  dev/regression/hidden/stress)·결정적 fixture·JSONL+NPZ 기록/재생(torn-tail 복구,
  `/sim` ground-truth 격리). production 수정 금지 — 소비만.
- `motor_gui/`, `operator_console/` — 진단 GUI(FastAPI)와 read-only 운용 콘솔(GTK).
- 의존 방향: `powertrain_ros → motor_control`, `motor_gui → motor_control` (역방향 금지),
  `powertrain_sim → (powertrain_ros 코어·powertrain_autonomy·powertrain_observability.events 소비)`.

## 4. 개발 파이프라인 (재사용할 것)

**Codex 위임 패턴**: 스펙 파일 작성(정본 계획 절 인용·Files·계약·제약·실행 명령 명시) →
`node ~/.claude/plugins/cache/openai-codex/codex/1.0.6/scripts/codex-companion.mjs task
--write "$(cat spec)"` → 검토자(Claude)가 diff 정독 + **3환경 검증** + 결함 수정 후 커밋.
Codex 샌드박스는 docker/rclpy/abstract-socket bind 불가(EPERM) — 통합 검증은 반드시 검토자 몫.

**3환경 검증 명령 원문**:

```bash
# ① 호스트(실소켓·SCM_CREDENTIALS 필요분)
PYTHONPATH=ros2/src/powertrain_ros:motor_control /home/light/anaconda3/bin/python -m pytest <dirs> -q
# ② dev 컨테이너(python-can 포함 전체 회귀)
docker run --rm -v "$PWD:/workspace" -w /workspace -e PYTHONPATH=/workspace/motor_control \
  powertrain-sw:dev python3 -m pytest motor_control motor_gui powertrain_observability powertrain_autonomy powertrain_sim -q
# ③ ros 컨테이너(colcon /tmp install-space — 엔트리포인트 검증 포함)
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace/ros2 powertrain-sw:ros -lc '
  set -e; source /opt/ros/humble/setup.bash
  colcon --log-base /tmp/log build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros \
    --build-base /tmp/b --install-base /tmp/i
  source /tmp/i/setup.bash && python3 -m pytest src/powertrain_ros/test -q'
# ④ 콘솔 런타임 스모크(operator_console 변경 시 필수 — 순수 테스트로는
#    Gtk 콜백 크래시를 못 본다. 07-18 _rss 실사고의 재발 방지 게이트.
#    실제 GUI를 Xvfb에서 띄우고 4채널 LIVE 데이터를 주입해 traceback 검출;
#    호스트 스위트(test_runtime_smoke)에도 편입되어 자동 실행됨)
/usr/bin/python3 -m operator_console.runtime_smoke
```

기준선(07-17 심야, 2차 하드닝 `42800f4` 후): 호스트 240(autonomy/console/sim/tests) /
dev 컨테이너 979+2skip(+operator_console 추가) / ros 컨테이너 410 / **젯슨 ros 410**
(도메인 격리 후 parity 완전 복구 — 과거 "플레이크"는 §9-0 도메인 누수였음) /
젯슨 autonomy 이미지 108+3skip / l515+remote 332+2skip.
(이전 기준선, WP8 `72ec7e4` 후: dev 컨테이너 894+2skip(표준 목록 =
motor_control motor_gui powertrain_observability powertrain_autonomy powertrain_sim
remote_video **tests**, 이미지에 mujoco 포함; skip=jsonschema·jax 호스트 전용) /
ros 컨테이너 390 + l515_dashboard·remote_video·tests 362+3skip / 젯슨 ros 컨테이너
390(일회용 컨테이너, 라이브 스택 무중단; 07-16 1회 비재현 플레이크 감시) /
젯슨 autonomy 이미지 99 passed + 3 skipped — skip은 정상(MuJoCo 통합·이미지 계약·
JAX 미설치). 환경 회귀 러너:
`scripts/run_autonomy_regression.py --manifest tests/fixtures/environment/manifest.yaml`
= 9 PASS / 0 FAIL / 0 SKIPPED. 호스트 conda base에 CPU jax 0.10.2 설치됨(동등성
테스트용).)
MuJoCo CLI 스모크: 3 시나리오 전부 PASS(exit 0). ⚠️ 젯슨 docker build 가 buildkit
snapshot 오류를 내면 `docker builder prune -f` 후 재시도(07-16 실측 복구). **테스트 실행과 commit/push는 반드시 `&&`
체인**(0a89098에서 비체인 스크립트가 1 failed를 그대로 커밋한 사고 있음 — fe67096로 수습).

**젯슨 배포 절차**: `git pull --ff-only` →
`docker compose -f docker/docker-compose.jetson.yml up -d --force-recreate powertrain_ros powertrain_control powertrain_chassis`
(엔트리포인트 colcon 재빌드 ~5 s) → 90 s 대기 → 헬스 확인. 서비스명 생략 `up -d` 금지
(`/run/powertrain` fail-closed와 결합해 미프로비저닝 보드에서 실패).
`powertrain_chassis`(07-18 사용자 결정, 상시): us100_safety+chassis 실기 —
stop_mm은 `/etc/powertrain/powertrain.env`의 STOP_MM만(부재 시 기동 거부),
healthcheck는 실제 노드 프로세스 확인. ⚠️ 크로스팀 조율 항목: 팔팀이
`powertrain-chassis-telemetry` 유닛의 OPERATOR_HOST를 자기 콘솔(.163)로
재설정함 — 현재 .158(우리)로 보내는 비관리 스트레이 프로세스와 공존해
우연히 양쪽 콘솔이 다 수신 중. 다중 콘솔 텔레메트리 정책(유닛 인스턴스화
또는 브로드캐스트) 합의 필요.

## 5. HIL 운용 모드·벤치 제약 (2026-07-16 확립)

- **FULL HIL 모드**: 사용자는 물리 조작(전원·리프트·육안)만, 모든 명령은 에이전트가 SSH로
  직접. **모터 회전 전 물리 확인(바퀴 리프트·클리어) 필수**, 실물 거동은 육안 확인을 HIL
  통과 조건에 포함(텔레메트리만으로 판정 금지 — 코깅존 교훈).
- **실차체 미조립**: 모터 벤치 배열 상태. 로봇팔 미장착(`arm_absent_field`).
  → 차체 조립 후로 이월된 항목: WP6-A 실측(5 m ±5%·제자리 90°), 지상 `stop_mm` 커미셔닝,
  경사로 시험.
- 벤치 함정: ros2 CLI 데몬 불안정 → rclpy 직접 클라이언트로; 좀비 teleop/제어루프 확인
  (`docker exec ... ps | grep teleop`); 컨테이너 root 소유 파일은 컨테이너 안에서 rm.

## 6. 다음 작업 (우선순위 순)

1. **(07-18 기준) A/B/C 개선 프로그램 9배치 SW 완주**(스펙
   `docs/superpowers/specs/2026-07-17-abc-program-design.md` r6, §2 커밋 체인) —
   벤치 이월 통합 목록: ⓐfriction_ff 값 튜닝+tq_ff 단위 확정(wheels-up 저속 추종)
   ⓑchord 실감·비상 chord 5 s/3 s·햅틱/트리거 체감·콘솔 GUI 육안(2단 확인 UX)
   ⓒNVM 캘리 영속화 실행+전원사이클 3회×6축 게이트(조립 선행조건)
   ⓓ후진 extraction 풀사이클 HIL(통과 전 실기 flag on 금지) ⓔWP7 lead-속도
   feedforward 개선 후보 ⓕ균일 slip 실측정 수단(휠+IMU 한계 — 후순위)
   ⓖsection_enforcement 벤치 검증 후 활성. ARM-CON(로봇팔 콘솔 4기능)은 계획서만
   완료(`docs/superpowers/plans/2026-07-18-armcon-console-bridge.md`, ⏸️ 착수 대기).
   (이하 07-17 이력) 실차체 불필요 SW 백로그는 전부 소진됨 — Task 6(A/B/C)·7,
   WP6-S P1, WP6-B JAX 커널, WP7, WP8 골격, 플래핑 수정까지 완료(§2 커밋 체인).
   남은 것은 전부 ①실기/벤치 게이트(아래 2·3·4) ②크로스팀 계약 대기
   (gateway 배선=팀원 l515 WIP landing 뒤, WP8 인식 이벤트 실토픽·`MISSION_STOP`
   언락 순서, D435i sender) ③JAX Jetson 전체부하 자격화·backend 선택.
   `powertrain_sim/README.md`·`powertrain_autonomy/README.md`가 시뮬레이션·
   terrain+controller 계약 정본.
2. ~~D 런북~~ → **07-17 벤치 HIL 완주** (`docs/reports/2026-07-17-bench-hil-remote-e2e.md`
   정독): v2 E2E·주행 중 qualified 핸드오버(0.78 s)·R1 bypass·E-stop 엣지 전부 실증,
   **실결함 4건 발견**(데드존 `2c30abc`·TCP RST `2c30abc`·Nagle `6bb4029`·executor 기아
   — 마지막 건 수정 진행 중: autonomy 노드 latest-only 슬롯+워커 스레드).
   ⚠️ **트랙 정책(07-17 사용자 확정)**: 실전/모의 트랙은 앞으로도 없음 — TRACKING류
   검증은 시뮬레이터가 영구 정본, 실기는 트랙 불필요 항목만.
3. 기술 백로그: ①~530 ms 스톨 — **미재현 종결 강등**(격리 30분 + 라이브 ARMED 15분
   모두 갭 0; 07-16 관측은 세션 특이 요인. 로그 젯슨 `~/wp53_soak/`). ②유휴 플래핑 —
   수정 `3c1e098` + **07-17 실기 10분 소멸 확인 완료(종결)**. ③autonomy 노드 executor
   기아(신규, HIL 실증) — 수정 사이클 진행 중. ④뷰어 실영상·fault matrix 잔여 채널
   kill — 다음 벤치(게이트웨이 SRT는 팀원 WIP landing 뒤).
4. WP8·`MISSION_STOP`·언락 순서·풀 핸드셰이크 1사이클(크로스팀).

## 7. 크로스팀 상태 (로봇팔, `extreme-robot`)

- 그들 main에 PR #17 병합: **ipc:host 실증**(0→37건) + `/arm_status` 10 Hz heartbeat.
  ⚠️ 그들 `ros2_humble` 컨테이너 **재기동해야 ipc:host 실효** — 아직 안 됨.
- 잔여 합의 3건(LOWER_RELEASE·접힘 근접·controller_fault)은 팔 젯슨 워킹트리 미커밋 —
  WP5.2 Task 7 합동 HIL의 전제. + 접힘 캘리브레이션, DualSense 키매핑 v1a 확정 대기.
- 팔 젯슨 체크아웃(`~/extreme-robot`)은 **read-only** — 우리가 수정하지 않는다.
- **ARM-CON(07-18)**: 팔 요청 콘솔 4종(다이나믹셀 전류·온도/조인트/YOLO class·yaw/실시간
  화면) 구현 완료 — 팔 레포 무수정, `arm_console_bridge`가 팔 토픽을 :5007/:5003으로 미러.
  영상은 팔 `stream_node`(:5002)+기존 콘솔 패널. **팔 팀 확인 대기**: 다이나믹셀 모델별
  온도 한계(현재 55/65 ℃ 임시), current raw 스케일(현재 raw 표기), :5003 단일송신 조율
  (브리지 가동 시 그들 `metadata_sender_node` 미기동), 합동 라이브 검증 세션.

## 8. 미커밋·로컬 전용 항목 (분실 주의)

- `docs/defence_docs/초안.docx`(팀 원본)·`초안_v2.docx`(07-16 갱신본 — wheel-stop 자격화,
  안전 장기시험, WP6-A/S 반영) — **의도적 미커밋**(원본이 사용자 영역). 커밋 여부는 사용자 판단.
- **젯슨 워킹트리 미커밋 WIP(2026-07-16 새벽 발견, 팀원 작업)**: L515 정렬 depth의 ROS
  발행 경로(`l515_dashboard/gateway*.py` + 테스트, opt-in `L515_ALIGNED_DEPTH_ROS=1` —
  RTAB-Map RGB-D SLAM 준비) + `urdf/jetin_rover.urdf.xacro` 확장. 방향은 타당해 보이나
  진행 중 — **보존하고 임의 커밋 금지**, 완료 시 검증(3환경) 후 통합.
- 팀원 노트북에만 있는 미푸시 4종: `scripts/extract_geometry_from_cad_urdf.py`,
  `parameter_calc/python_gpu_triangle/export_chassis_geometry.py`,
  `docs/specs/2026-07-13-min-rev-speed-range.md` 등 — `kinematics.py` docstring이 첫
  파일을 provenance로 인용하는 재현성 갭. push 요청 필요(노션 배너 처리됨).

## 9. 반복 함정 모음 (누적)

0. **로봇에서 테스트 스위트 = DDS 도메인 격리 필수** (07-17 근본 규명): 도메인 0으로
   돌리면 라이브 게이트웨이의 실카메라 토픽이 테스트 구독을 선점(합성 fixture 전부
   무시 → 순서 의존 실패)하고, 테스트의 가짜 /arm_status·/odom이 라이브 그래프를
   오염한다. `test/conftest.py`가 ROS_DOMAIN_ID=77을 강제(`42800f4`) — 그동안의
   "비재현 플레이크"도 대부분 이 누수였을 것. 라이브 그래프 대상 테스트만
   `POWERTRAIN_TEST_DOMAIN`으로 재정의. **같은 계열 TCP판(07-17 A1 점검에서 발견):
   라이브 `powertrain_control`이 :9000을 점유하면 테스트 노드 bind가 EADDRINUSE로
   죽고 테스트가 라이브 서버에 붙는다** — teleop 노드 테스트는 autouse fixture로
   `DEFAULT_PORT`를 에페메랄 포트로 격리(도메인 77이 DDS만 막고 TCP는 못 막는다).
1. dustynv/l4t 베이스 pip = 죽은 미러 → 파생 이미지 `--index-url https://pypi.org/simple` 필수.
2. rclpy 로거에 %-스타일 포지셔널 인자 = TypeError (3회 발견 — f-string으로).
3. SimpleNamespace 레거시 픽스처 계약 — chassis_node 신규 속성 접근은 getattr 가드
   (`test_legacy_simplenamespace_tick_fixture_stays_usable_without_arm_fields`).
4. zsh: 매치 없는 glob·`=word` 확장이 명령 전체를 죽임 — 스크립트는 bash로.
5. **원격 프로브의 `proc.terminate()`는 `ros2 run` 래퍼만 죽인다**(자식 노드가
   고아로 생존 → 도메인 77에서 실서비스 응답·상태 발행 → 이후 스위트가
   "간헐 실패"로 오진, 07-18 C0 실측 — pkill 자기매치 함정도 동반). 프로브는
   `start_new_session=True`+`os.killpg`로 프로세스 그룹을 죽이고, 스위트 실패
   진단 전에 `ps | grep powertrain_ros/lib` 좀비 확인이 1순위.
5. 바퀴 지령 <0.3 rev/s = HALL 코깅존, 실물 정지 + 그럴듯한 텔레메트리.
6. 캘리는 RAM-only — 전원 사이클마다 `can_calibrate_all.py`.
7. 노션 SW 문서는 표준 템플릿 + 「주제 — 부제」 제목 컨벤션 + 복붙 완주 가능해야 함;
   제목 변경 시 README 매핑표 동기화.
