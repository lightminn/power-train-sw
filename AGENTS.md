<!-- BEGIN CLAUDE_TO_CODEX_MEMORY -->
# Migrated Claude Code Memory

Source memory directory: `/home/light/.claude/projects/-home-light-ZETIN-robotics-power-train-sw/memory`
Imported: `2026-07-10T02:30:45+09:00`
Reconciled with repository, Jetson, and Notion state: `2026-07-10T19:34+09:00`

These notes were migrated from Claude Code project memory. Treat them as durable project context unless the user gives newer instructions.

## CURRENT STATE OVERRIDE — 2026-07-10 WP5.1

This WP5.1 override is newer than the 2026-07-10 audit override and every migrated memory entry
below. Preserve the older text as history, but use this section when it conflicts.

- The original WP5 `/cmd_vel → ChassisManager → 10 motors` HIL remains a valid historical result.
  **WP5.1 Tasks 1–8 software are complete, but final Jetson/10-motor/US-100 HIL is NOT RUN.**
  Observed local evidence is motor_control 189 passed, motor_gui 91 passed, and a temporary
  read-only ROS workspace building all 3 packages with 23 powertrain_ros tests passed. FAKE,
  Jetson, and HIL remain pending; none of the local results is hardware evidence.
- Current architecture is a pure-Python control/safety core with thin internal ROS nodes:
  `us100_safety_node` isolates blocking UART at 5–10 Hz, while `chassis_node` checks the latest
  `/safety_verdict`, owns the final E-stop decision, runs the 50 Hz chassis loop, and publishes
  `/wheel_states`. `ChassisManager` alone owns one `can0` at 500 kbps and AK45-36 ×4 plus
  ODrive/BL70200 ×6.
- Current US-100 states are `VALID`, `INVALID_READING`, `CHECKING`, and `NO_RESPONSE`.
  `INVALID_READING` is normal operation because 0x50 proves only MCU/UART liveness, not the
  ultrasonic transmitter/receiver. `CHECKING`, the 0.5 s `/cmd_vel` watchdog, and connection loss
  are auto-recovering `MOTION_HOLD`. The command watchdog is distinct from safety-topic freshness.
  A valid near reading or three consecutive distance-and-liveness misses is a latched `ESTOP`.
- E-stop reset returns only to `IDLE`; a separate arm action is mandatory. Production
  `safety_topic_timeout` is 0.75 s (`age > threshold`, then the next 50 Hz tick, nominally
  0.75–0.77 s), startup timeout is 1.0 s, and `safety_required=false` is BENCH/FAKE only.
- The migrated `safe/warn/stop`, `Verdict.level`, startup/`None`→`stop`, and auto-clearing safety
  gate descriptions are **explicitly superseded historical memory**. Do not use them for current
  code, tests, operations, or documentation. Current authorities are
  `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`,
  `docs/plans/2026-07-10-wp5-control-safety-hardening-plan.md`, and
  `docs/reports/2026-07-10-wp5-control-safety-hil.md`.
- After the final WP5.1 HIL, proceed in this order: command-authority spec → L515 lightweight
  color-image + depth-image + IMU pipeline (PointCloud2 optional) → WP6 odometry. WP8 and the
  remaining `MISSION_STOP`, unlock-ordering, and full arm-handshake work remain open in parallel.

## CURRENT STATE OVERRIDE — 2026-07-10

This section is newer than every embedded migrated note below and overrides conflicting historical text.

- Current source of truth: `docs/reports/2026-07-10-project-and-jetson-state.md` and
  `docs/plans/2026-07-02-autonomous-driving-kickoff.md`.
- **WP1–WP5 are complete.** WP4 passed bidirectional DDS delivery against the robot-arm graph;
  WP5 passed real HIL from `/cmd_vel` through `ChassisManager` to all 10 motors. Next work is
  WP6 odometry or WP8 mission sequencing. Remaining cross-team items are `MISSION_STOP`, unlock
  ordering, and one full `ARRIVED_* → arm work → DONE → resume` handshake.
- Sensors: **L515 = powertrain RGB/depth/IMU**, **D435i = robot-arm exclusive**, **US-100 =
  independent collision safety**. Both RealSense devices were present on Jetson USB on 2026-07-10.
- Motor bus: one `can0` at 500 kbps, AK45-36 ×4 plus ODrive/BL70200 ×6. ADM3053 isolation removed
  the PWM-noise coupling; the CAN watchdog remains insurance.
- ODrive authority: pp=10, cpr=60, bandwidth=30, vel_gain=0.12, vel_integrator_gain=0.2,
  node 11–16. Use `bl70200_setup.py` and `can_calibrate_all.py`. Never use legacy
  `odrive_calibration.py` on real BL70200 hardware; it still forces single-axis pp=5 settings.
- Parameter-optimization v4 remains final at **50 kg**; do not plan an 86 kg rerun. Treat 86 kg as
  an old design estimate, not the v4 calculation mass.
- Jetson audit on 2026-07-10: powertrain had no unpushed commits but contained untracked
  `motor_control/vision/tests/`; `extreme-robot` was on dirty `Gripper_YOLO_FSM`; vendored
  `robot_arm_msgs` matched; `powertrain_ros` and `powertrain_canwatchdog` ran, while
  `powertrain_jetson` and `ros2_humble` were stopped; can0 was down. Preserve all teammate changes.
- The active Notion Software pages were reconciled on 2026-07-10. Apply the standard Software
  template to plans and analyses too. Existing pages may be edited only when the user explicitly
  authorizes it; always re-fetch after writes.

## MEMORY

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/MEMORY.md`

# Memory Index

- [project state 2026-07-10](project-state-2026-07-10.md) — 최우선 최신 정본: WP1~5 완료·다음 WP6/WP8, 센서분리, 10모터/ADM3053, ODrive 정본·레거시 금지, Jetson/Notion 감사 결과
- [patent whitepaper](patent-whitepaper.md) — 파워트레인 SW 전체 특허 출원 의도 + 기술차별성 백서(docs/patent/, 작성됨). 프레이밍=균형·통합1건+요소랭킹. 전략결론: 개별 메커니즘은 prior art→조합/시스템 청구. E1≈E2≈E3>E4>E5, 0xFF UART=영업비밀
- [robot-arm team resources](robot-arm-team-resources.md) — WP4 양방향 DDS·WP5 `/cmd_vel→10모터` HIL 완료, msg 5종 드리프트 없음. 다음 WP6/WP8; MISSION_STOP·락 해제·풀 핸드셰이크 미결. L515=우리 RGB/depth/IMU, D435i=로봇팔 전용
- [FSM competition track](fsm-competition-track.md) — 국방(9월)·극한(10월) 규정 핵심수치·초안 대조(문 PUSH, 결과지 수기, 지형순서 고정)·3레이어+behavior 8개 방향, **센서배치 확정(2026-07-07): L515=우리 카메라/depth/IMU, D435i=로봇팔, US100=안전 — D435i독점·라이다역할 해소(센서분리), 연막 잔존**
- [motor-gui build](motor-gui-build.md) — motor_gui 웹 진단 GUI: 코드(Task 1-10)·25테스트 완료·push. GUI HIL 진행(AK/ODrive CAN 트랙 HIL후 main병합), 이제 10모터 단일 can0 동시(구동 CAN 통일)
- [AK-CAN GUI](ak-can-gui.md) — --track ak 컴포저블 디바이스 구조·HIL 학습(spd÷10, brake/current 폭주 삭제, pos int16 한계)·main 병합
- [ODrive-CAN GUI](odrive-can-gui.md) — --track odrive_can 컴포저블·1Mbps 종단저항/TX견고성·소프트영점·USB설정필요·main 병합
- [Jetson deploy/CAN](jetson-deploy-can.md) — Jetson HIL 워크플로: sshfs 마운트(끊기면 scp 폴백)·can_setup·컨테이너 exec·host-network curl
- [Jetson CAN loopback 함정](jetson-can-loopback-footgun.md) — can0 loopback 모드는 down/up에도 sticky(명시적 off/리부팅 필요). 모든 baud에서 ACK 성공=loopback 의심(가짜 self-ACK, 버스 무음). 진단 통째로 오염됨
- [Jetson docker-only CAN/motor](jetson-docker-motor-control.md) — Jetson 모터/CAN python은 host 아닌 컨테이너 powertrain_jetson 안에서 실행(python-can은 컨테이너에만, host엔 없음). host net+privileged로 can0 공유, repo=/workspace. 컨테이너에 ip/busybox/can-utils/sudo 없음(필요시 apt 설치). ★함정: 안 끈 좀비 teleop/제어루프가 v=0을 계속 명령→새 모터테스트와 싸워 surging/undershoot/멈춤 오진단(2026-07-05 반나절 날림); 테스트 전 `docker exec ps|grep teleop/chassis` 확인·pkill
- [AK CAN 500k/50Hz 확정](ak-can-500k-50hz.md) — AK45-36 다중모터 버스=500kbps+50Hz 주기피드백(HIL: 1M는 드롭 24~52%/bus-off 폭주, 500k는 동시회전 10회 드롭 0%). 긴 케이블 못 줄임→비트레이트로 마진. 종단60Ω·솔더·restart-ms100, Δ읽기 아티팩트 주의
- [Notion team hub](notion-team-hub.md) — 팀 Notion 허브(기존페이지 무단수정금지, 신규 SW문서 생성OK)·대조: AK45-36 조향·ODrive(VESC아님)·BL70200S·단일CAN 4WS(구동 CAN 통일). 무게=50kg v4 그대로 확정(86kg 재최적화 안 함). 신규 SW문서=표준템플릿 필수
- [US-100 safety module](us100-safety-module.md) — 충돌방지 판정 모듈(#3, publish-only safe/warn/stop), cliff·다센서 제외, 인계 팀원 초보→문서 쉽게
- [corner module + Ackermann](corner-module-ackermann.md) — WP1~3·10모터 4WS HIL·유무선 텔레옵 완료, WP4 DDS·WP5 chassis_node HIL까지 완료. 다음 WP6/WP8; 저속 플로어1.0·조향슬루4500·node12/16 HALL 주의
- [param-calc v4 track](param-calc-v4-newtrack.md) — v4 권위본=parameter_calc/python_gpu_triangle(6/4 평탄화로 new_parameter_calc 중첩 제거), 면-기준 물리 수정·envelope 캐시·test_v4 재작성(48검증)·도구버그 완료
- [Jetson RealSense D435i](jetson-realsense-d435i.md) — 역사적 D435i 실험/로봇팔 참고 자산. 현재 D435i=로봇팔 전용, L515=우리 자율 RGB/depth/IMU. SDK·RSUSB·3D좌표·SRT 학습은 유효
- [docker dev env for tests](docker-dev-env-for-tests.md) — 실행·검증은 Jetson 직접 우선, x86 dev 컨테이너(powertrain_dev) 테스트는 Jetson 접근 불가 시 차선·CPU전용 슬림(3.3GB, gpu.yml 삭제), pip 의존성은 docker/Dockerfile
- [conda base env for python](conda-base-env-for-python.md) — 범용/일회성 파이썬 도구(python-pptx 등)는 conda base(/home/light/anaconda3), 프로젝트 런타임 deps는 Docker dev 컨테이너
- [CAN Isolator Click (ADM3053)](can-isolator-click-adm3053.md) — 신형 조향 트랜시버, Jetson 40핀 5V로는 isoPower 딸려 2.5V 처짐→외부5V 필수, TX/RX 스왑·공통GND·VISO_OUT 측정·절연측 GND로 재면 가짜2.5V
- [BL70200 ODrive Jetson bringup](bl70200-odrive-jetson-bringup.md) — 실측: 보드 실제 pp=10/cpr=60(레포 pp=5와 충돌), odrive enum은 .value 필요(IntEnum 아님), 오프셋스캔 ~55s(타임아웃≥90s), 포지션 정지=게인 마찰(pos_gain2.0+적분0.2로 1바퀴 성공), 캘리 RAM only. 듀얼축 M0+M1 독립제어 OK(calib_scan_omega=6.0 필수, 캘리 반복실패로 shadow_count 폭주시 파라미터 말고 전원사이클이 답, ILLEGAL_HALL_STATE 트립은 ignore_illegal_hall_state=True+HW 접지/필터캡). 2026-07-04 자유회전 재튜닝: vel_gain 0.06→0.12(무부하 연속회전이라 ±1바퀴 제약 벗어나 상향, 정속2×·오버슈트4×·방향전환6× 개선, 0.14=천장), bw30/vi0.2 유지, 레포+노션 반영
- [motor_gui→BL70200 config 오염](motor-gui-bl70200-config-contamination.md) — motor_gui usb 트랙 게인/리밋 기본값=X2212 → BL70200 ODrive NVM 오염(current_lim 100A·pos8·vel_int0·ifbw50·vel_gain0.015). 캘리OK·state8인데 vel_int0이라 부하 시 안 돎(Iq 안 감김). BL70200 셋업 §2 전수대조로 복구. Never mix tracks 실사례
- [CAN multi-device topology](can-bus-multidevice-topology.md) — 단일 can0 500k, AK ×4(id1-4 조향)+ODrive 구동(검증 8모터 node 11/12/15/16→2026-07 사용자: 구동 6개로 10모터 확장, 전부 CAN 개별제어; 07-04 전 구동계 bring-up 완료=6축 전부 게인0.12 NVM+CAN 풀캘리 6/6(state3, 55s, 3보드 serial 3352/336a/3377)+10/10 ID 인식·버스 에러0; 6축 CAN 동시주행 5/6 즉시+node12(board A axis1) marginal HALL은 ignore_illegal_hall_state=True로 트립해결(전진OK, 역방향 피드백은 HALL HW 필요), 도구=can_calibrate_all.py/can_drive_test.py 커밋 d416e60). 함정: 공장기본 ODrive 250k·node0가 500k 버스 ERROR-PASSIVE 깸(가짜 "node0 cmd4 garbage")→USB bl70200_setup --node N --apply로 baud+node 동시교정. CAN FULL_CAL=state3(HALL), 캘리 RAM-only, shadow_count 폭주=전원사이클. Notion 정본=「8모터 독립제어」
- [GL-SFT1200 robot AP](gl-sft1200-robot-ap.md) — 로봇 전용 WiFi AP(GL.iNet 4.3.28, standalone — 대회장 NITEZ 없음 대비). 젯슨 유선 LAN 192.168.8.106 고정예약 + 노트북 5GHz(ZETIN-ROBOT-5G, country KR). 설정=SSH+uci(dropbear ssh-rsa 호스트키 필수, NITEZ시 젯슨 점프 터널). 자격증명은 env($ROUTER_ADMIN_PASS/$ROUTER_WIFI_PASS)만 참조. 전원 5V2A 자체어댑터(젯슨USB X)
- [check GitHub before work](check-github-before-work.md) — 작업 착수 전 GitHub(우리+로봇팔 extreme-robot)와 **Jetson 로컬(미커밋/미푸시)** 둘 다 확인 후 진행(2026-07-03 지시). 팀원이 젯슨에서 직접 작업하고 안 올렸을 수 있음. "머지/작업했다" 구두정보는 실제 검증. git fetch+log origin/main / ssh 젯슨 git status·@{u}.. / gh pr list
- [doc scope: powertrain SW only](doc-scope-powertrain-only.md) — 문서는 파워트레인 **SW 담당** 관점(사용자 역할): CAD·파워(전장) 소관은 인계 요구사항으로만, 팀 전체 발표전략·일정 확장 금지(2026-07-07 두 차례 교정)
- [notion SW template required](notion-sw-template-required.md) — 노션 SW 페이지는 성격 불문(계획·로드맵 포함) SW 표준 템플릿 구조·번호·컨벤션 반영. 성격 이유로 건너뛰지 말 것(키네마틱스 부분반영·계획 미반영으로 지적받고 재작성). 안 쓰는 섹션 삭제+순차재번호, 스펙원문=프로젝트 CLAUDE.md

## ak-can-500k-50hz

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/ak-can-500k-50hz.md`

---
name: ak-can-500k-50hz
description: "AK45-36 다중모터 CAN은 500kbps + 50Hz 주기피드백으로 확정 — 1Mbps는 긴 로봇 버스에서 드롭율 24~52%/bus-off 폭주, 500k로 내리니 10회 동시회전 드롭 0.00%"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**결정 (2026-06-23 HIL 검증):** 로봇 CAN 버스(AK45-36 조향 모터들)는 **500kbps + 모터 주기피드백 50Hz**로 운용한다. (모터 CAN Bitrate는 R-LINK에서, Jetson `can0`도 500k로 — 반드시 일치.)

**왜 (실측 비교, 2모터 ID1/2 동시 10rpm 2초 × 10회):**
- **1Mbps + 100Hz 피드백**: 명령 드롭율 **24~52%**(시도마다 들쭉), bus-off 15~24회, run 후반 TX wedge. 사용 불가.
- **500k + 50Hz 피드백**: 드롭율 **0.00%**(2100/2100 ACK), bus-off 0, 에러프레임 0. 완벽.
- 물리층은 그대로(마진 부족)인데 **비트레이트 비트타임 2배 + 피드백 트래픽 절반**이 마진을 만들어 에러를 없앰. 로봇이 커서 **케이블 못 줄임** → 길이 대신 비트레이트로 마진 확보가 정답이었음.

**대역폭 여유:** 500k에서 10모터(명령+피드백 50Hz) ≈ 130 kbit/s = **~26% 부하**. 충분. 100Hz 필요해지면 빡빡(~52%) → 그땐 조향/구동 **CAN 버스 2개 분리**.

**물리층 교훈 (이번 디버그):**
- 긴 untwisted 케이블 = 노이즈. CANH/CANL **트위스트 필수**(했음).
- 드롭율이 시도마다 5.9→24→52%로 **변동 = 간헐 접촉(커넥터)**의 신호. 트위스트로 못 고침 → 듀폰점퍼 말고 **솔더/크림프/락킹**. 특히 **공통 경로(Jetson J17↔트랜시버 배선, 종단)** 가 의심 1순위(단독 측정 시 ID1·ID2 둘 다 7~9% 드롭 = 모터 아닌 공통버스).
- **종단 60Ω, 버스 양 끝 2곳만**(중간 모터 종단 OFF). 전원 끄고 측정.
- 측정 위치 주의: 모터 주기피드백이 흐르면 `set_origin`+`poll` 위치읽기가 **stale 프레임**에 오염돼 Δ가 가짜로 들쭉여 보임(실제 모션 아님). 신뢰할 지표 = **드롭율(tx_packets delta)·bus-off·에러프레임**.

**bus-off 함정:** restart-ms 0(노션 기본)은 bus-off 시 latch → 복구 안 됨. **restart-ms 100** 써야 자동복구. 단 bus-off 폭풍이 심하면 컨트롤러 TX가 wedge될 수 있어 `can_setup`(down/up) 재실행 필요할 때 있음.

관련: [[jetson-docker-motor-control]] · [[jetson-can-loopback-footgun]] · [[corner-module-ackermann]]

## ak-can-gui

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/ak-can-gui.md`

---
name: ak-can-gui
description: "motor_gui AK-CAN 트랙(--track ak) — 컴포저블 디바이스 구조, HIL 학습, brake/current 폭주"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

motor_gui 에 AK40 조향 모터 CAN 제어 트랙 추가 완료 (2026-05-21, main 병합 `bf83093`).
spec/plan: `docs/specs/2026-05-21-ak-can-gui-design.md`, `docs/plans/2026-05-21-ak-can-gui-plan.md`.

**구조 (컴포저블, 미래 ODrive-CAN+AK 동시제어 대비)**: `transport/can_device.py` = `CanDevice` ABC +
`CanTransport`(버스1개 + 디바이스 리스트 집계, 단일 recv 드레인 → 모든 디바이스 on_rx 분배,
워치독 tick). `transport/ak_device.py` = `AkDevice`(AK40 래핑). 런처 `--track ak` →
`CanTransport([AkDevice()])`. 다음 단계는 `OdriveCanDevice` 추가 후 `CanTransport([Odrive, Ak])`.
기존 `can_bus.py`(CanBackend, ODrive+AK 모놀리식)는 OdriveCanDevice 추출 원본으로 보존.

**모드**: position/velocity/duty 만. **brake·current 모드는 삭제** — HIL에서 둘 다 폭주
(2A 명령 → speed ~-380). 이 AK 펌웨어가 `SET_BRAKE(2)`/`SET_CURRENT(1)` 패킷을 기대대로
처리 안 함. duty(0)/rpm(3)/pos_spd(6)는 정상.

**HIL로 찾은 핵심 버그/한계 (실 AK 검증)**:
- `send_pos_out` 의 spd/acc 필드는 AK가 **×10으로 해석** → ERPM 그대로 보내면 속도 10배. **÷10 전송**으로 수정(`ak_control.py`). 안 그러면 position 속도제한 무시하고 무부하 최대까지 감.
- velocity 모드: 입력 RPM이 곧 목표속도, **무부하 최대 ~43 출력RPM**(NOLOAD 6090erpm÷140). 속도제한 튜닝은 position 전용.
- AK STATUS 위치는 **int16(×10도) → ±3276.7° 표시 한계**(3200에서 막힘). 명령은 int32라 멀티턴 OK, 모터 내부 추적 정상. 조향엔 충분 → 그대로 둠.
- AK는 idle에도 50Hz 자동 broadcast. 워치독: 활성 명령 20Hz 재전송.
- "CAN 죽음" 진단: `ip -details link show can0` 에서 ERROR-PASSIVE + berr **tx만 오르고 rx=0** = AK가 ACK 안 함 = AK 전원/케이블 문제(소프트 아님). 복구: AK 전원 확인 → `scripts/can_setup.sh` → 재연결.

**기타 기능**: setpoint 오버레이(ak.pos_cmd/speed_cmd, 점선 같은색), 튜닝 ERPM/RPM 듀얼 칸,
하드웨어 재연결(`/api/reconnect`, worker 스레드에서 close+connect), 과전류 자동정지(max_cur_a),
fault 디코드, 모드별 help, 정적 no-cache 헤더(브라우저 stale JS 방지).

관련: [[motor-gui-build]] [[docker-dev-env-for-tests]]. HIL/배포는 [[jetson-deploy-can]] 참고.

## bl70200-odrive-jetson-bringup

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/bl70200-odrive-jetson-bringup.md`

---
name: bl70200-odrive-jetson-bringup
description: BL70200 ODrive 젯슨 USB 캘리·포지션 브링업 실측(pp=10/cpr=60 실제값·enum.value 함정·오프셋스캔 55s·게인 마찰)
metadata:
  node_type: memory
  type: project
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

젯슨 컨테이너 `powertrain_jetson` 안 USB ODrive(fw 0.5.1, axis1, 36V/UV24)로 BL70200 캘리+포지션 한바퀴 실측(2026-06-23). 직전 세션이 설정을 NVM에 박은 상태에서 풀캘리→포지션 1바퀴 성공.

- **실제 보드 NVM = pp=10 / cpr=60 (HALL).** 레포 `odrive_calibration.py`와 CLAUDE.md는 pp=5/cpr=30이라 적혀 있지만 이 물리 모터는 pp=10이 맞다(커뮤테이션 깨끗·정확히 1바퀴 도달 = pp 검증됨). 레포 캘리 스크립트는 pp=5를 강제하므로 **그대로 쓰지 말 것** — 보드 현재 pp/cpr 유지하고 캘리할 것.
- **이 odrive 파이썬 빌드의 enum(AxisState/ControlMode/InputMode)은 IntEnum 아님.** `ax.requested_state = AxisState.X` 하면 `TypeError: int() argument ... not 'AxisState'`. → **`.value` 붙여 정수로 넘겨야** 함(예: `AxisState.FULL_CALIBRATION_SEQUENCE.value`). `ax.current_state`는 정수 반환하므로 비교도 정수로.
- **오프셋 캘리(`calib_scan_distance=150`) 스캔에 ~55s 걸림** → wait 타임아웃 ≥90s 둘 것. 45s면 스캔 중간에 잘려 "타임아웃"처럼 보이지만, 연결 끊기면 펌웨어가 알아서 끝내 eready=True 됨(가짜 실패).
- **POSITION_CONTROL 이동이 목표 못 채우고 중간 정지하면 게인 문제(마찰).** 캘리 스크립트 기본값 `pos_gain=0.5, vel_integrator_gain=0`이면 0.6바퀴쯤에서 Iq 0.2~0.5A로 정지. **`pos_gain=2.0, vel_integrator_gain=0.2`**로 올리면 적분기가 마찰 보상해 끝까지 도달(무부하 1바퀴→+1.012, 최대 Iq 0.84A).
- 캘리 결과(offset=-44, dir=-1, R=0.279Ω, L=362µH)는 **RAM에만** 있음 — 리부팅 시 소실. 영구화하려면 `save_configuration()` + `pre_calibrated=True`.
- 실행 패턴: `sshpass ... ssh zetin@jetson-orin.local "docker exec -i powertrain_jetson python3 -" <<'PYEOF' ...`. find_any(timeout=20)로 행 방지. 직전 점유 프로세스 있으면 `-6 Could not claim interface` → 컨테이너 python pkill 후 재시도.
- **CAN 설정(2026-06-23 NVM 저장): node_id=11, baud=500000(500kbps).** AK 조향 버스(500k)와 맞춤. (이전엔 x2212 잔재로 node=1/250k였음.)
- **CAN baud_rate는 `can.config.baud_rate` 직접 쓰기 불가**("this attribute cannot be written to") → **`drv.can.set_baud_rate(500000)` 메서드** 사용. node_id 쓰기는 `axis1.config.can.node_id`가 AttributeError → **`axis1.config.can_node_id = 11`** 폴백 경로로 써짐(읽기는 nested 경로 됨).
- **NVM 설정만 저장하고 RAM 테스트 잔재 안 묻히려면: 먼저 `drv.reboot()`로 NVM 복원 → 원하는 것만 얹어 save.** reboot()는 ObjectLostError 던짐(정상), 5s 후 find_any 재시도. 컨테이너에서 reboot 후 USB 재열거 정상 동작.

## CAN 구동 (node 11 / 500k, 2026-06-23 실측 성공)
- can0는 보통 이미 500k UP(`ip -details link show can0`). 아니면 `scripts/can_setup.sh`(1Mbps 하드코딩이라 bitrate만 500000으로 바꿔 실행, sudo 비번 필요=$JETSON_SSH_PASS). 호스트에 candump/cansend 있음(컨테이너엔 python-can 4.6.1).
- node 11 heartbeat = arb `0x161`(=11<<5|0x01), `[err uint32 LE][state byte4]`. axis0(미사용)은 node 0 기본이라 `0x001`로도 heartbeat 나옴(무해, 필터 `can_id=node<<5, mask=0x7E0`로 분리).
- **CAN 풀캘리 함정: `Set_Axis_State(0x07)=FULL_CALIBRATION_SEQUENCE(6)`는 미캘리+HALL에서 `axis_err=0x1 INVALID_STATE`로 즉시 거부됨**(내부 HALL polarity 단계 거부). **우회=분리 요청: `MOTOR_CALIBRATION(4)`(~4s) → `ENCODER_OFFSET_CALIBRATION(7)`(~51s 스캔) 각각 보내고 heartbeat에서 state가 해당값→IDLE(1) 복귀 + err==0 확인.** USB에서 FULL_CAL 통했던 건 이미 eready라 polarity 건너뛴 거였음.
- **게인/리밋/모드도 CAN으로 설정 가능**(노션엔 USB 굽기만 있지만 이 fw 작동 확인): `Set_Limits(0x0F)`=`<ff`(vel_lim,cur_lim), `Set_Pos_Gain(0x1A)`=`<f`, `Set_Vel_Gains(0x1B)`=`<ff`(vel_gain,vel_int), `Set_Controller_Mode(0x0B)`=`<ii`(ctrl,input). → NVM 안 건드리고 런타임 주입 가능.
- **위치 골든게인(노션 "Odrive CAN 제어" 기준, 우리 BL70200에도 OK)**: pos_gain 2.0 / vel_gain 0.015 / vel_integrator 0.15 / vel_limit 5.0 / POSITION(3)+POS_FILTER(3). 점프방지=폐루프 진입 전 `Set_Input_Pos(0x0C)`=`<fhh`로 현재위치 시드. 1바퀴(+1.0) 2.7s 매끈 도달(오차 0.015).
- **노션 "Odrive CAN 제어" 문서는 x2212/D6374 + TLE5012B(16384CPR, pp7) 리그용** — 프로토콜/시퀀스/게인구조만 차용. BL70200 현행 정본은 pp10/cpr60/HALL/**bandwidth30**이며, 고해상도 엔코더용 bandwidth1000 지침은 부적용.
- **3모터 동시 구동 실증(2026-06-24): ODrive node11(표준 프레임) + AK id1·id2(확장 프레임)가 단일 can0(500k)에서 충돌 없이 공존.** 프레임 타입(std vs ext)이 달라 arb-id 겹쳐도 무관. 단일 `bus.recv` 드레인 후 `is_extended_id`로 분배(ODrive 0x161/0x169 / AK pkt41) = 메모리 [[ak-can-gui]]의 CanTransport 패턴 그대로. 12초 동시 사인 진동(ODrive ±0.5turn 실측[POS_FILTER 감쇠로 명령 ±0.8보다 작음], AK ±28° 역위상) 드롭/bus-off/에러 0, fault 0 → 미래 4WS 단일 CAN(조향 AK + 구동 ODrive) 아키텍처 검증됨. 코드 패턴: [[corner-module-ackermann]].
- **함정: `save_configuration()` 직후 axis err=0x8(CURRENT_MEASUREMENT_TIMEOUT)가 latch될 수 있음** — clear_errors로 안 빠지고 모터캘리(state4)도 0x8로 실패. **해결=ODrive 리부팅**(d.reboot 또는 CAN 0x16). 리부팅하면 깨끗(axis 0x0)해지지만 pre_calibrated=False라 캘리 소실 → 재캘리 필요(오프셋 스캔 다회전이므로 샤프트 자유 필요). bandwidth 등 NVM 설정은 리부팅 후 정상 로드됨.
- **ODrive 속도제어 over CAN 검증(2026-06-24): `Set_Input_Vel(0x0D)` = `<ff`(vel, torque_ff).** VELOCITY/VEL_RAMP 모드(0x0B `<ii` 2,2)로 정속 1.0 rev/s 지령 → 평균 1.001, 리플 0.073, 10.2회전(고속이라 매끈). AK 위치(±30° 사인)와 동시 구동 정상. ⚠️ **런어웨이: ODrive 속도모드에서 IDLE 안 내리고 스크립트가 죽으면 마지막 input_vel로 무한 회전**(ODrive가 마지막 지령 유지) → `try/finally`로 `Set_Axis_State=IDLE(1)` 필수. 노션 새 페이지 "AK + ODrive 동시 CAN 제어"(3882d27b...efa56b) = 이 검증 기반 작성.
- **ODrive heartbeat 주기 50Hz 설정(2026-06-24, NVM 저장): `ax.config.can_heartbeat_rate_ms = 20`** (flat 경로 — 이 빌드엔 `config.can` nested 객체 없음, `can_heartbeat_rate_ms`/`can_node_id`/`can_node_id_extended`만 존재). 라이브 즉시 50.3Hz 반영, 리부팅 후 50.4Hz 유지(AK 50Hz와 일치). ⚠️ **이 fw엔 `encoder_rate_ms`(pos/vel 주기 방송) 속성이 아예 없음** → ODrive 자동 주기 방송은 **heartbeat(에러+상태)뿐**, pos/vel/Iq 등은 **RTR 폴링(0x09 등) 전용**. 수신측에서 0x09를 원하는 Hz로 RTR하면 됨.
- **CAN 에러율 실측(2026-06-24, 3디바이스 부하 15s): 명령 드롭 0.000%**(의도 1428 전부 송신, send실패 0, 에러프레임 0, tx_errors 0, bus-off 증가 0). 측정법: `/sys/class/net/can0/statistics/tx_packets` 델타 vs 의도 송신수, **루프 후 0.6s 드레인 필수**(안 하면 in-flight 프레임이 가짜 ~1% 드롭으로 보임). bus-off/error-pass는 호스트 `ip -details -statistics link show can0`. 주의: 그 카운터의 restarts129/error-pass191/bus-off130은 **과거 1M 디버깅 누적치**(절대값 보지 말고 전후 델타만). → [[ak-can-500k-50hz]]의 500k 무손실, 3디바이스로 재확인.
- **BL70200 HALL 최적 튜닝(2026-06-24 실험 A~F 수렴, NVM 저장됨): `encoder.config.bandwidth=30`, `pos_gain=2.0`, `vel_gain=0.06`, `vel_integrator_gain=0.2`** (POS_FILTER/ifbw2.0). 원본(노션 bw100/vg0.015/vi0.15) 대비 **저속 회전 진동 vel_std 0.338→0.193 (-43%)**. 핵심 레버 = **encoder bandwidth 100→30**(단독 -38%, HALL 속도추정 평활화); 낮춘 bandwidth 덕에 vel_gain을 0.06까지 올려 댐핑 확보 가능(고대역폭에선 노이즈로 불가). **vel_gain 상한(±1바퀴 포지션 제약下) ~0.10, 0.12=트립** — ⚠️단 이 상한은 포지션 ±1바퀴 제약 안에서 잰 값이라 틀림: 무부하 자유회전에선 0.12가 되레 최적(아래 2026-07-04 재튜닝). bandwidth는 **CAN 명령 없음→NVM 저장 필수**(CAN 주행에서 먹히려면). 트레이드오프: 정지 hold 버징은 약간↑(Iq_std 0.103→0.207)이나 드라이브 휠(주로 회전)엔 회전 매끈함이 우선. 측정법: **양방향 등속 vel_std + 반복평균**(HALL 리플은 코깅위상·방향 의존이라 단발측정 2배 편차, 양방향 풀링하면 안정). save_configuration은 리부팅 안 함(캘리 RAM 생존)·사용자 VELOCITY/vlim50/cur9 프레임 보존·pre_calibrated=False 유지.
- **★자유회전 재튜닝 (2026-07-04, 무부하 연속회전, 신규 중간 듀얼보드 node13/14 NVM 저장): `vel_gain 0.06→0.12`** (bw30/vi0.2 유지). 기존 0.06은 포지션 ±1바퀴 제약 안에서만 잰 값 — 모터 free하게 연속회전시키며 다중 시나리오(정속 0.5~2.0·가감속 ramp2.5/10·방향전환 +1→-1) 종합비용 스윕하니 vel_gain을 0.12까지 올릴 수 있었고 전 구간 개선: 정속 리플 ~2×↓(0.09→0.05@0.5, 0.018→0.009@2.0rev/s), 가감속 오버슈트 ~4×↓(1.04→0.23@ramp2.5), 방향전환 리플 ~6×↓(0.19→0.03). **0.14는 다시 악화 → 0.12가 천장.** vel_int는 0.15가 순수 정속 최저(0.029)지만 방향전환 회복 느려 리플↑ → 0.20이 전 시나리오 균형 1위. bw는 30 재확인(22/26/35/45 다 나쁨). 노이즈 큼(짧은 settle시 steady 0.04~0.22 요동) → 긴 측정 2초×2반복 평균으로 확정. ⚠️무부하 최적이라 실차 바퀴+지면 부하 시 재확인(부하는 보통 감쇠 추가→오버슈트↓ 안전쪽). 절차: 양축 vel_gain=0.12 set→save_configuration→reboot(0x8 latch 해소)→FULL_CAL 재캘리(56s×2, RAM-only)→검증스핀(a0 0.96/a1 1.01 rev/s err0). 레포 `bl70200_setup.py` CFG 0.12로 커밋(23ae99d)·노션 셋업가이드 §2/§6/§8 반영 완료.
- **★HALL 고속 커뮤테이션 천장 (2026-07-05 실측): 구동 실용 최대 ~10~12 rev/s**. 단일 node13(깨끗) 12 rev/s까지 클린(Iq 1.3A 무부하), 15 지령 시 실제 3으로 붕괴+Iq 6.8A(토크부족 아니라 HALL 엣지 과속→미스카운트→동기상실). 6모터 동시 12 rev/s에선 marginal 보드(11/12/15/16) `0x2 CPR_POLEPAIRS_MISMATCH` 트립, 13/14만 버팀. **HALL(cpr60)이 속도 양끝 제약: <0.3(저속코깅)·>~10~12(고속미스카운트) → 깨끗한 대역 ~0.5~10 rev/s.** 실주행 0.8m/s≈1.3rev/s라 여유 충분. 더 필요하면 HALL 접지/필터캡 or 고해상도 엔코더. **내구 60초(6구동@10rev/s+AK4@최대 동시): 전부 완주**, 전류 0.3~0.7A·AK 57→59℃(문제0). 단 ①0→10 PASSTHROUGH 즉시지령의 급가속 startup에서 전 노드 0x2 순간트립(램프업하면 없음), ②node14가 중간 41·57s에 0x40 추가트립(marginal 기미, node12·16과 함께 HALL HW 후보). **auto clear+재arm(노드당 최대3회)이 startup·중간 글리치 다 흡수해 완주** — 이 자가복구를 실주행 드라이버(DriveOdriveCan)에 넣을 가치 있음(현재 미구현).
- **★`enable_phase_interpolation=False` 저속 안티코깅 효과 = 없음 (2026-07-06 4라운드 A/B 수치검증).** ODrive 공식이 저속 HALL 툭툭거림 완화로 권고하는 무료 SW 레버(HALL은 전기주기당 6점만 알아 그 사이 속도추정으로 보간 → 저속 노이지). node13 USB, 코깅존 0.3~0.5rev/s 양방향 등속 2/3/5회 반복 A/B: True vs False **Iq_std(공정 토크리플 지표) 차이 +2%~−8%로 전부 반복편차 안(무의미)**. ⚠️**vstd는 비교에 쓰면 안 됨** — phase_interp 가 vel_estimate readout 자체를 smoothing 하므로 True 가 구조적으로 낮게 나옴(실제 매끈함 무관). 결론: 이 모터(BL70200+HALL cpr60)엔 실효 없음, NVM 저장 불필요. 저속 대책은 min_drive 속도플로어(기동존 회피)+bandwidth30 이 사실상 최선, 진짜 해법은 고해상도 절대엔코더(AS5047 등, HW). ⚠️테스트 함정: enable_phase_interpolation/bandwidth 등 encoder.config 은 CAN 런타임 명령 없음 → USB로만 A/B·NVM 저장(bandwidth 동일). node13/14 보드는 전원이벤트로 캘리 소실돼 있어 arm 실패 → USB FULL_CAL 선행 필요했음.
- **48V 실전 전환(2026-06-24, vbus 47.6V, NVM 영구 저장 완료): 전압 영향값은 UV 트립뿐 — `dc_bus_undervoltage_trip_level` 24→40.** OV=56(보드 한계·12S 50.4V 위 마진)·brake_resistance=2.0(물리 저항값)·게인·bandwidth·current_lim·cpr 전부 전압 무관이라 그대로 유지. NVM 검증값: UV40/OV56/brake2.0/bw30/pos2.0/vg0.06/vi0.2/node11/baud500k(pre_calibrated=False 유지). 0.5 rev/s×5s·1 rev/s×3s 둘 다 36V와 동일(평균=목표±1%, 트립 0, 고속일수록 리플↓). 노션 셋업페이지도 48V로 갱신. ⚠️ UV=40은 실제 vbus가 48V일 때만 적용(36V에 적용하면 즉시 트립) → 스크립트에서 vbus<42면 게이트로 중단. UV가 부하 sag로 트립하면 36~38로 낮춤.
- **ODrive HALL "이동 중 진동"은 게인 문제 아님 — HALL(cpr60) 저속 속도추정 코깅.** 실측: 정지홀드 Iq_std는 노션게인(pos2.0/vel0.015/int0.15)이 최저(0.033), vel_gain 낮추면 hold 10배 악화(0.38). 저속등속(0.16~0.3turn/s)에선 모든 게인 vel_std ~0.32~0.36(평균보다 큼)=게인 무관. **실제 레버=`encoder.config.bandwidth`(USB 전용·CAN명령 없음) 100→30~50, 또는 고속운전**(HALL은 고속에서 매끈). 게인으론 못 잡음. 남의 "설정+save" 레시피는 컨트롤러 게인 0줄=공장기본(pos20/vel0.16/int0.32, HALL엔 과함)+캘리/폐루프 없음 → "안움직임"의 원인.
- **★안티코깅 조사 결론 (2026-07-06, 웹+레포 조사): 내장 anti-cogging = 지금 HALL(cpr60)/fw0.5.x 구성에선 불가(이중 배제).** ①원리: ODrive cogging_map은 **absolute/인덱스 엔코더에서만 저장·로드**, HALL 미지원(60포인트/rev로 코깅맵 못 그림 + RAM-only 재캘리라 절대위치 재현성 없음). ②실측: fw0.5.1 `start_anticogging_calibration`이 불완전 종료→`anticogging_enabled=True`+무효맵으로 **폐루프 모션 brick**(X2212 때 실측, `docs/motor-gui-tuning-guide.md` §6에 복구법). fw0.6.x 신형 안티코깅도 HALL 해상도 제약 불변. → **저속코깅 대체 레버**: (a)bandwidth30·(b)최저속도플로어(커밋2291774)=둘 다 적용완료+방향맞음, (c)**미시도 신규 SW 레버 = `encoder.config.enable_phase_interpolation=False`**(현재 CFG에 없어 기본 True; 저속 HALL 보간 노이즈 제거→툭툭 감소 가능, 고속은 반대라 양방향 vel_std A/B 필수, CAN명령 없어 USB NVM 저장), (d)진짜 해법=AS5047P 등 고해상도 절대엔코더 교체(안티코깅도 개방). enable_phase_interp는 이동중 리플(②)만·정지마찰 기동(①)은 여전히 속도플로어 담당.

## 듀얼축 M0(axis0)+M1(axis1) 동시 브링업 (2026-06-24/25, USB 48V)
M0·M1 둘 다 BL70200 동일 모터·동일 결선. 같은 설정 주입하면 둘 다 독립 동작함(아래 함정만 피하면).
- **`encoder.config.calib_scan_omega=6.0` 필수 (기본값 12.566이면 OFFSET 캘리 실패).** M1은 원셋업 때 6.0이었는데 새로 단 M0은 기본 12.566 → M0만 OFFSET에서 `CPR_POLEPAIRS_MISMATCH(enc 0x2)`로 15~30s에 조기중단. omega 낮추면 오픈루프 스캔이 느려져 카운트 정합. **신규 축 설정 시 calib_scan_omega를 M1값(6.0)에 맞출 것.** (HALL polarity 단계는 통과·offset 단계만 실패 → calib_range 늘려도 무효, omega가 진짜 레버.)
- **★최대 함정: 캘리 반복 실패하면 ODrive가 progressively 나빠지는 latched 상태에 빠져 HALL 카운터 폭주.** 증상: `shadow_count` 변동폭이 실 HALL 전이수의 49→58→231→452배로 **매 실행 단조 증가**(실전이 ~145인데 shadow 7천~6.5만). **무전원 IDLE에선 shadow 변동 0(깨끗)** → 노이즈는 전류 흐를 때만. 전류↑↓·스캔·omega·range·공장초기화·동일설정 **전부 무효**. **해결=ODrive 전원 사이클 1회**(reboot 아니라 물리 전원 OFF/ON). 전원 사이클 후 양축 즉시 깨끗 캘리(55s, err 0x0, shadow변동 205≈실전이 290). → **shadow_count 폭주/CPR mismatch가 파라미터 무관하게 지속되면 파라미터 만지지 말고 전원부터 내릴 것.** (반복 캘리 ~12회의 열·에러누적이 원인 추정.)
- **폐루프 회전 중 간헐 트립 `axis 0x100(ENCODER_FAILED)` = `encoder.error 0x10(ILLEGAL_HALL_STATE)`** — HALL이 회전+전류 스트레스에서 순간 불법상태(000/111) 읽음. 간헐적(클리어 후 재시도하면 됨). **완화=`encoder.config.ignore_illegal_hall_state=True`**(직전 유효상태 유지, 트립 안 함) → 적용 후 ±1바퀴 위치제어 3종(M0단독/M1단독/동시반대) 전부 err 0x0. **근본 신뢰성은 HALL 접지/필터캡(22~47nF 라인→GND) 하드웨어 보강이 정답**(밴드에이드임). 어느 축이 트립할지는 고정 아님(marginal HALL 공통).
- **진단 기법**: ①`shadow_count` 변동폭 vs 폴링 hall 전이수 비교(폭주=노이즈). ②무전원 IDLE vs 통전 비교(IDLE만 깨끗=PWM/모션 유발). ③`AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION`/`ENCODER_OFFSET_CALIBRATION` 분리 실행으로 어느 단계 실패인지 격리. ④손회전으로 hall 시퀀스 찍으면 pp/배선 직판정(1,3,2,6,4,5 Gray=정상). ⑤양축 motor.config+encoder.config 전체 diff로 "동일 설정" 검증.
- **이 빌드엔 `odrv.config.gpioN_mode` 없음**(AttributeError) — GPIO 모드 설정 불가/불필요. 레포 diff_drive의 gpio9_mode 출력 코드는 이 보드에서 에러남.
- 독립 제어 검증값: 위치모드 ±1바퀴, Δ도달 ~0.95(HALL 저해상도 정상오차 ~0.05). 폐루프 진입 전 `input_pos=pos_estimate` 점프방지. 캘리·ignore_illegal_hall_state는 RAM(저장 안 함, 리부팅 시 재캘리). 영구화하려면 save→리부팅→재캘리.

관련: [[jetson-docker-motor-control]] [[jetson-deploy-can]]

## can-bus-multidevice-topology

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/can-bus-multidevice-topology.md`

---
name: can-bus-multidevice-topology
description: 실전 단일 CAN 버스 8모터(AK ×4 조향 id1-4 + ODrive ×4 구동 node 11/12/15/16 = 2보드×듀얼축) + 새 ODrive 공장기본 250k가 500k 버스 깨는 함정. 2026-07 사용자 구두: 10모터로 확장(구동 ODrive ×6, 중간 2바퀴 node13/14 셋업 확정 07-04)
metadata:
  node_type: memory
  type: project
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

단일 can0 500kbps 버스에 **8모터 동시·독립 제어** (2026-06-25 토폴로지 / 2026-06-29 8모터 스핀 검증):
- **조향 AK45-36 ×4** = id **1·2·3·4** (VESC 확장프레임, pkt41 STATUS_1 50Hz 자동 브로드캐스트, 패시브 스캔으로 인식). pos=출력축 deg/10, fault 바이트.
- **구동 ODrive ×4** = BL70200 **2보드 × 듀얼축**(M0=axis0, M1=axis1 둘 다 사용) = node **11·12**(보드A) · **15·16**(보드B). CANSimple 표준프레임, heartbeat(cmd1). node 번호는 보드 스왑하며 재배정해온 값(이전 11·13에서 변경).
- **CAN으로 FULL_CALIBRATION = state 3** (HALL은 CAN만으로 캘리 OK — 4→7 따로 안 해도 됨). 캘리 RAM-only(전원 사이클마다 재캘리). 연속 캘리 실패로 **shadow_count 폭주(CPR_POLEPAIRS_MISMATCH)** 시 파라미터 말고 **전원 사이클**이 답(shadow_count 리셋).

⚠️ **함정**: 공장출고/리셋 ODrive = **baud 250k + node_id 0** 기본. 이걸 500k 버스에 꽂으면 프레임이 깨져 들어와 **can0 ERROR-PASSIVE(berr rx 127)** + 가짜 "node 0 / cmd4 / dlc8 garbage 120Hz" 로 보임(250k 프레임 오독). → USB로 `bl70200_setup.py --node N --apply` 하면 baud 500k 까지 같이 교정돼 버스가 ERROR-ACTIVE로 복구. cal 은 RAM-only(전원 사이클마다 재캘리, save 시 리부팅→cal 소실).

진단 절차: 호스트 `ip -details link show can0` 로 에러상태 확인 + 컨테이너 python-can 패시브 스캔(extended=AK / standard=ODrive 분기). 보드 식별은 USB `odrive.find_any().serial_number`(CAN 전용이면 USB 없어 introspection 불가). `bl70200_setup.py --node` 추가됨(보드별 지정). **2026-06-29 역사 검증**: 당시 AK 1-4 + ODrive 11/12/15/16 8/8 정상. 현재 Notion 정본은 「단일 CAN 버스 10모터 독립제어」.

**2026-07-02 (사용자 구두 갱신)**: 토폴로지가 **10모터로 확장** — 조향 AK ×4 + **구동 ODrive ×6**(중간 2바퀴 구동이 위 8모터에 추가; 중간 듀얼보드 **node 13/14 셋업 확정 — 2026-07-04** 신규 ODrive USB 초기셋업+양축 NVM 저장(최적게인 vg0.12, [[bl70200-odrive-jetson-bringup]] 자유회전 재튜닝)+FULL_CAL 재캘리+검증스핀 완료, serial 0x336a33523235, WP3 `DEFAULT_WHEEL_MAP` 13/14와 일치 = placeholder 아님. 단 CAN 버스 합류 HIL은 미실행). 조향+구동 **전부 CAN으로 개별 제어 가능**(커스텀 케이블로 구동도 CAN 통일 — 이전 "구동 USB/조향 CAN" 상태 해소). ⚠️ WP3 `ChassisManager` 실기 4WS(HIL)를 이번에 **안 돌린 건 하드웨어 한계가 아니라 안전 판단**(10모터 통합 첫 구동 위험). ChassisManager 실구동에 남은 유일한 코드 = `corner_module/drive_odrive_can.py` 스텁 채우기(검증된 CAN 프레임 이식 = WP1). Notion「8모터 독립제어」정본 페이지는 이 확장분(10) 미반영—stale 가능.

**2026-07-04 (전 구동계 CAN bring-up 완료)**: ① 구동 6축 전부 궁극최적 게인 `vel_gain=0.12`(vi0.2/bw30) NVM 저장 — 3보드 serial A(11/12)=`0x335233643235` / 중간(13/14)=`0x336a33523235` / B(15/16)=`0x337733643235`, node별 안전확인(set{n0,n1}==기대) 후 각각 USB로 set→save_configuration→reboot→저장확인. ② 10모터 전부 단일 can0(500k) 패시브 스캔 인식: ODrive heartbeat node 11~16(표준프레임 0x161/181/1A1/1C1/1E1/201) + AK STATUS id 1~4(확장 pkt41, 50Hz, 실제 조향각까지 읽힘). 버스 ERROR-ACTIVE·에러카운터 전부 0·**250k 불량장비 없음**(그게 있으면 error-pass로 튐). ③ 구동 6축 **CAN 풀캘리 6/6 성공**: `Set_Axis_State`(cmd0x07)=**state 3**(FULL_CAL, HALL OK over CAN 재확인), heartbeat 상태전이 4→7→1, 각 55s, err0. ⚠️캘리 RAM-only=전원 사이클마다 재필요 → 레포 `can_calibrate_all.py`가 6축 순차 일괄(한 축씩=전류 스파이크 방지). ④ **6축 CAN 동시주행**(`can_drive_test.py`, 전진·제자리선회·정지): 5/6 즉시 정상, **node12(board A=axis1)만 marginal HALL**로 폐루프 진입 시 0x100 ENCODER_FAILED 트립 → `ignore_illegal_hall_state=True`로 해결(board A NVM 영구저장 + CFG 표준화, 커밋 d416e60). ⚠️단 플래그는 **트립만** 막고 node12 **역방향 속도피드백은 여전히 불안정**(제자리선회서 −1.0 미추종, 정지 시 유령 −0.7)=근본은 HALL 접지/필터캡(라인→GND 22~47nF) HW. node16도 역방향 노이즈 큼(부호는 맞음), node14는 깨끗. 전진은 6축 다 견고. **구동 6축 전부 flag NVM 반영완료**(각 USB로 set→save→reboot→재캘리, 11/12=07-04 / 13/14·15/16=07-05) — 3보드 모두 `vel_gain=0.12·vi=0.20·bw=30·ignore_illegal_hall_state=True` NVM 통일. (플래그는 트립만 막음; node12·16 역방향 피드백 품질은 여전히 HALL 접지/필터캡 HW가 근본.) **WP1 완료(2026-07-05, 커밋 8453866)**: `corner_module/drive_odrive_can.py` 구현(검증프레임 이식)+단위테스트 32+실기 HIL(node11·12). 남은 것=node12/16 HALL HW 보강·**WP3 ChassisManager 실기 4WS HIL**(이제 언블록됨).

**2026-07-06~07 ★모터 PWM 노이즈 → 젯슨 CAN TX 오염 (실측 확정)**: 무선 텔레옵 중 "잘 되다 갑자기 안 됨" 반복 + bus-off 674회/error-passive 1199회 누적의 근본원인. **판별 실험**: ①젯슨 TX 시 에러 11~75%(BIT1 97% = 수신측 에러플래그) vs 젯슨 침묵 시 노드 2881프레임 에러 0 → 젯슨 TX만 오염 ②RTR/데이터 무관(RTR 응답충돌 가설 기각), 프레임 길수록 에러↑(비트단위 오염) ③**모터 4축 CLOSED_LOOP(게이트 스위칭) 시 28~75% ↔ IDLE 시 0.0%(4582프레임 에러 제로)** = 완전 판별. 메커니즘(사용자 확인): **젯슨 CAN 트랜시버가 절연형(ADM3053)이 아니라 일반 비절연** + 젯슨 GND↔모터 파워서플라이 GND **미연결(플로팅)** → PWM 커플링으로 공통모드 출렁 + 젯슨 송신 시 버스 구동 리턴 전류가 고임피던스 그라운드 경로에서 자기 기준전위를 흔들어 자기 TX 되읽기 오염(BIT1) → 에러프레임이 자기 프레임 파괴 → 재전송. RX는 고임피던스 차동 감지(리턴 전류 없음)라 무결 — TX만 깨지는 비대칭 설명. ODrive/AK 자신들의 TX도 무결(자기 GND=서플라이 GND). **에러율이 TEC 산식(+8/-1) 폭주 문턱(11%≈칼끝)을 넘나들어** 부하↑(주행)=폭풍(bus-off 연쇄, 조향 stale→FAULT)/부하↓(정지)=회복 — "껐다켜니 됐다"는 모터 IDLE화 효과지 전원 자체가 아님. 배선/커넥터 무죄(사용자 주장 옳았음). **진단 도구**: raw CAN 소켓 + `CAN_RAW_ERR_FILTER`(0x1FFFFFFF)로 에러프레임 클래스/PROT상세(BIT1/STUFF/위치) 캡처, `berr-reporting on`(ip link, down 필요), `ip -s -details link show can0`의 bus-off/error-pass 누적카운트. 대책 경과: **GND 공통화(07-07)는 효과 미미**(전축 28.8→19.6% — 저주파만 잡고 HF 공통모드는 GND선 인덕턴스로 못 잡음; 단 IDLE 0%는 유지, 그라운드 루프 악화는 없음·USB 미연결 확인). **★per-node "채널 가설"은 이후 기각됨(07-07)**: 첫 격리측정(node16=11%·11=7%·15=1.6%·12=0.1%)은 세션 내 재현됐지만 **세션 간 붕괴**(케이블 이격 무효: node16 11%→10~13%, 무변경 대조군 node12 0.1%→31%!) + 유지전류(Iq) 전 노드 0.05A 동일로 전류 가설도 기각. **진짜 지배 변수 = 모터 상태**: 4축 **정지 폐루프 유지(vel=0) 27.9%/27.1% ↔ 회전(2rev/s) 3.5→1.6%** (가역·즉시 전환). 메커니즘(실험 14~15로 확정): (a)센터정렬 SVM에서 vel=0→3상 듀티 전부 ~50%→에지 동시발화→CM 풀스윙 한 방(회전하면 역기전력∝듀티 분리→에지 분산→붕괴; 속도 스윕 5.1→0.2%@0.6rev/s→0.0%@4 **매끈한 감소**로 전류클램프 가설 기각) (b)각도 스캔 3.9~10.5% = 잔여 전압벡터 방향이 상 정렬 결정=세션 복불복의 정체 (c)젯슨 송신만 깨지는 이유=그라운드 도메인 비대칭 — ODrive 트랜시버들은 PWM과 같이 출렁이는 모터 GND 위(서로 무결), 젯슨이 구동하면 버스 CM이 조용한 도메인에 클램프→ODrive 수신기들이 CM 요동으로 recessive 오독→에러플래그→젯슨 BIT1. **⭐0.6 rev/s 이상이면 이미 0.2% = min_drive 플로어(1.0) 덕에 '움직이면 청정, vel=0 유지만 더러움'**. **실용 결론: 실주행 ~2%=TEC 안전권(폭풍 없음), 위험 구간=armed 정차 대기(27%)** — "주행 잘 되다 세워두면 죽는" 실사용 패턴과 정합. **✅✅ 종결(2026-07-07): 절연형 CAN 트랜시버 교체로 노이즈 완전 해결.** 클린 재기동(docker down/up+can_setup)+재캘리 후 동일 A/B: **4축 정지 유지 27.9%→0.0%(0/1,482)·회전 0.0%·30s 데이터프레임 폭격 74.6%→0.00%(0/13,205, 송신실패 0)·bus-off 0·워치독 무발동** — 메커니즘(c) 그라운드 도메인 비대칭 = 결합 경로 확정. 정차 대기 폭풍·웻지 트리거·경사 estop 리스크 소멸, 정차 CAN 침묵 모드·코스트·3선화·초크 전부 불필요(기록만 보존). **웻지 워치독(canwatchdog 상주)은 보험으로 계속 유지**(트랜시버 전원 문제 등 회귀 대비). 절연 트랜시버 정상 동작 확인: RX 전 노드·TX 20/20·berr 0. txqueuelen 1000 필수(기본10, can_setup.sh 반영). 측정도구=/tmp/abc_test.py(에러프레임 캡처+프레임종류/노드별 격리).
**★"잘 되다 아예 안 됨" 상전이의 정체 = mttcan TX 웻지 (07-07 재현·해결)**: 사용자 관찰("에러는 일정한데 증상이 변함")이 결정타 — 노이즈로 bus-off 반복 누적 후 **mttcan 드라이버가 TX 큐를 영구 정지**(berr 0·ERROR-ACTIVE로 멀쩡해 보이는데 qdisc 백로그 139p 고착·전 send ENOBUFS). 전원사이클/프로그램 재시작 무관, **매번 같이 돌린 can_setup(down/up)이 진짜 복구 요인**이었음. 재현=45s TX폭격(bus-off +122)→프로브 0/30, down/up만으로 30/30. **해결=워치독 3형태 (정본=① 상주 서비스)**: ①**compose `canwatchdog` 서비스(eb780aa)** — `docker-compose.jetson.yml`, powertrain-sw:jetson 이미지 재사용·host net·privileged·restart unless-stopped, 컨테이너 스택 올리면 자동가동·재부팅 생존, `docker logs powertrain_canwatchdog` 로 리셋 이력. can_watchdog.py 에 __main__ 러너+상주 견고화(can0 없으면 5s 재시도, 리셋 후 소켓 재오픈). ②`corner_module/can_watchdog.py` CanWatchdog(f9c87f2) — 텔레옵 진입점(chassis.teleop_server·chassis/corner teleop_dualsense) 인프로세스 내장(①과 중복 무해). ③`scripts/can_watchdog.sh`(e729f92, 호스트판 비상용). **문서**: `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md`(판별실험 9종 전말) + 노션 「CAN 자동복구 워치독」(3952d27b08d381308d0eeafa8242e509, 표준템플릿·README 매핑표 등재). 감지=1s 프로브(빈 노드 21 RTR, 자체 raw소켓, 빈 CAN_RAW_FILTER=수신차단) 실패+`/sys/.../tx_packets` 정지 2연속(일시 폭주는 tx 증가로 구분→오탐0 검증), 복구=순수 ioctl(SIOCSIFFLAGS) down/up+txqueuelen — ip 바이너리 없는 컨테이너 OK, 기존 SocketCAN 소켓 리셋 후 유지(ifindex 불변). 실기검증: can0 강제down→2s 자가부활·서버 무중단, 폭격 3라운드(bus-off +664) 불필요 리셋 0. 웻지 형성은 확률적(폭격마다 안 생기기도 함). 리셋 순간 코너 stale→FAULT 가능(□ 재무장). ⚠️pkill -f can_watchdog 을 ssh 원격 복합명령 안에서 쓰면 자기매치로 셸 자살 — pidfile(/tmp/can_watchdog.pid) 사용.

링크: [[bl70200-odrive-jetson-bringup]] [[ak-can-500k-50hz]] [[jetson-docker-motor-control]] [[motor-gui-bl70200-config-contamination]] [[corner-module-ackermann]] [[can-isolator-click-adm3053]]

## can-isolator-click-adm3053

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/can-isolator-click-adm3053.md`

---
name: can-isolator-click-adm3053
description: "신형 조향 CAN 트랜시버=MikroE CAN Isolator Click(ADM3053). Jetson 40핀 5V로는 isoPower 부족→외부5V 필수, TX/RX 스왑·공통GND 주의"
metadata:
  node_type: memory
  type: reference
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

신형 CAN 트랜시버 = **MikroE CAN Isolator Click (ADM3053BRWZ — 절연 + isoPower DC-DC 내장)**. 2026-06-24 도입, AK45-36 조향 버스에 사용. 교체 후 버스 완전 무음으로 한참 헤맴 → 원인 2개:

- ⭐ **isoPower(절연측 버스전원 VISO)는 VCC=5V로 생성** → **5V 부실하면 CANH/CANL 죽어 버스 완전 무음**(ODrive heartbeat·AK status 둘 다 0, 내가 TX하면 무ACK→bus-off). **Jetson 40핀 5V는 isoPower 부하(~100mA+ DC-DC inrush)에 딸려 2.5V로 주저앉음**(무부하 5V, 물리면 분압). → **외부 5V 공급 필수**(그리고 외부 5V의 GND를 로직 GND와 **공통**으로 묶을 것).
- **TX/RX(CTX/CRX) 스왑 주의** — 실제로 한 번 바뀌어 있어 양방향 무음. 결선: VCC=외부5V, VIO=Jetson 3.3V(**VIO SEL 점퍼=3V3**), GND 공통, **CTX→클릭 TX / CRX→클릭 RX**(직결, 클릭이 내부에서 ADM3053 TXD/RXD로 연결). **종단 120Ω 내장**(R2+R3=60.4Ω×2 split) — 클릭이 버스 한쪽 끝이면 반대 끝에만 120Ω.
- **진단 키 = `VISO_OUT`(ADM3053 pin12, 절연측 5V) 측정**: ~5V면 isoPower 정상(전원부 OK)→남은 건 CANH/CANL 결선·RS standby; 0V면 5V/공통GND 문제 or 칩 손상. ⚠️ **절연 보드라 +5V를 GND_ISO(버스측 GND)로 재면 절연막 때문에 떠서 가짜 ~2.5V(절반)** 찍힘 → 반드시 로직 GND(3.3V 잴 때 그 GND) 기준으로 측정.
- 복구 확인: AK id1 ±30° 스윕 정상(pos·fault 0, cur ~0A, status 50Hz). 진단 패턴: 버스 무음 시 `candump can0`로 ODrive heartbeat(0x161) 자동방송 뜨는지 보면 Jetson 수신경로 정상여부 격리됨.

관련: [[ak-can-500k-50hz]] · [[jetson-docker-motor-control]]

## check-github-before-work

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/check-github-before-work.md`

---
name: check-github-before-work
description: 작업 착수 전 GitHub(우리+로봇팔 레포)와 Jetson(미커밋/미푸시 로컬 변경) 둘 다 확인하고 시작 — 남이 해둔 것 놓치지 말 것
metadata:
  node_type: memory
  type: feedback
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

**작업을 시작하기 전에 항상 GitHub와 Jetson을 둘 다 확인**해서 다른 사람이 해둔 것·최근 변경을 파악한 다음 진행한다 (사용자 2026-07-03 지시).

**Why:** 팀 협업 프로젝트 — 남의 작업과 충돌·중복을 피하고 최신 상태 위에서 작업하기 위함. **GitHub에 없는 변경이 Jetson 로컬에만 있을 수 있음** — 팀원이 젯슨에서 직접 작업하고 아직 커밋/푸시 안 했을 수 있어서(사용자가 명시). 로봇팔 팀 레포도 우리 인터페이스(robot_arm_msgs 등)의 근거라 상태가 자주 바뀜.

**How to apply — 착수 전 3곳 확인:**
1. **우리 레포(GitHub)**: `git fetch && git log origin/main --oneline -5` + 관련 `gh pr list`.
2. **Jetson 로컬**: SSH로 `git -C ~/power-train-sw status --short` + 미푸시 커밋 `git -C ~/power-train-sw log --oneline @{u}..` (젯슨=192.168.8.106[라우터] 또는 192.168.50.98[NITEZ], 유저 `zetin`). extreme-robot 체크아웃(`~/extreme-robot`)도 동일 확인.
3. **로봇팔 레포(GitHub)**: `gh pr list --repo ksp118/extreme-robot` + main 최신 커밋·`robot_arm_msgs` 머지 여부.

"머지/작업 했다"는 구두 정보는 GitHub·Jetson으로 실제 검증할 것 (과거: PR #11 미머지인데 머지됐다고 들음 / 2026-07-03 확인 시 WP3·PR#11이 이미 반영돼 내 Notion 플랜이 stale이었음). [[robot-arm-team-resources]]

## conda-base-env-for-python

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/conda-base-env-for-python.md`

---
name: conda-base-env-for-python
description: 범용/일회성 파이썬 도구는 conda base(/home/light/anaconda3)에 설치, 프로젝트 런타임 deps는 Docker dev 컨테이너
metadata:
  node_type: memory
  type: feedback
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**2026-06-09 갱신.** 사용자 지시: "python 패키지는 conda base에 깔아." 두 갈래로 나뉜다:
- **범용·일회성 도구**(예: 발표자료 생성용 `python-pptx`) → **conda base** 에 설치·실행.
  실행 바이너리는 `/home/light/anaconda3/bin/python` (conda 함수는 비대화형 셸에서 안 잡힐 수 있어 절대경로 사용). 설치: `/home/light/anaconda3/bin/python -m pip install <pkg>`.
- **프로젝트 런타임/테스트 deps**(jax, Defence_Robot 코드 실행·pytest 등) → 여전히 **x86 dev Docker 컨테이너** ([[docker-dev-env-for-tests]]).

**Why:** 프로젝트 환경은 컨테이너로 격리·재현하되, 레포와 무관한 잡일 도구까지 컨테이너에 넣지 않고 conda base로 빠르게 처리. (이전 "conda 전면 폐기" 기록은 이 구분으로 정정.)

설치 이력: `python-pptx 1.0.2` (PowerTrain 발표 3장 .pptx 생성용).

## corner-module-ackermann

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/corner-module-ackermann.md`

---
name: corner-module-ackermann
description: "코너 모듈 컨트롤러(#1)의 설계 의도와 미래 애커만 4WS 확장 제약"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**⚠️ 2026-07-03 갱신 (이 메모=2026-05 작성 — 아래 변경분 우선):**
- **① 미래로 적힌 4WS 애커만이 이제 구현됨** — 신규 패키지 `motor_control/chassis/`: `kinematics.py`(차체 (v,ω)→바퀴별 조향각·속도, WP2) + `chassis_manager.py`(코너 **6개**=조향4+고정2 통합·estop전파·US-100게이팅·차체워치독, WP3). pytest 53 통과. 이 CornerModule이 그 하부 실행기(코너 1개).
- **② 구동 배선 CAN 통일** — 구동USB→CAN(커스텀 케이블). 단일 can0 500k에 10모터(AK×4 id1-4 조향 + ODrive×6 구동 node 11/12/13/14/15/16).
- **③ corner_module은 main에 있음** — 배포=`git pull`(rsync 아님, 젯슨 origin/main 싱크).
- **④ active 조향 = AK45-36 id1-4** (AK40-10 id10은 레거시 테스트).
- **⑤ WP1 완료 (2026-07-05, 커밋 8453866)** — `drive_odrive_can.py` 구현+단위테스트 32개+**실기 HIL**(node 11·12 각 1.0rev/s: actual_vel~1.0·cur_a 실측·stale 정상, node12는 flag로 무트립). DriveOdriveUsb와 동일 계약을 CANSimple로: arm=Set_Controller_Mode(VELOCITY/PASSTHROUGH)+Input_Vel0+Axis_State8(8B패딩), tick=Set_Input_Vel+RTR(0x09 Enc/0x14 Iq)폴링, state={target/actual_vel,cur_a}+stale/axis_error, 노드별 socketcan+node필터(단일 can0 다중공존), bus 주입=무하드웨어 테스트. `chassis_manager.build_hardware_corners` 언블록. [[can-bus-multidevice-topology]]
- **⑥ WP3 실기 4WS HIL 통과 (2026-07-05, 실물 육안 확인, 문서커밋 9691310)** — `build_real_corners("can0")` 10모터 협조: 조향 홈→전진(6바퀴 0.8rev/s)→좌/우선회(**애커만** 안쪽 +31.5°>바깥 +16.5°·**뒤축 역위상**·차동, kinematics 실측 일치)→정지. 조향 꺾임+바퀴 회전 동시 실물 확인. HIL이 잡은 통합버그 2건 수정: ① SteerAk40 소켓 무필터→다중모터 버스에서 AK status 굶음(`4e5cf1c`, STATUS_1 필터 추가), ② **CornerModule.tick이 steer.tick 전에 state()로 stale 판정**하는데 6코너 순차 arm ~1.2s 걸려 첫 tick 무조건 false-estop → `state()`가 stale 판정 전 poll(0)로 커널버퍼 드레인(자가회복, `91c71e8`). **★HIL 교훈: 바퀴 지령 <0.3rev/s(HALL 코깅존)면 실물은 정지한 채 텔레메트리(순간 RTR 샘플)만 그럴듯함** — 첫 "성공" run(v=0.15→0.24rev/s)이 실제론 바퀴 안 돌았음(사용자 육안 지적으로 발각). 테스트 v≥0.4m/s(바퀴≥0.6rev/s) + **실물 육안 확인을 HIL 통과조건에 포함**할 것.
- **⑦ 차체 4WS 텔레옵 완료 (2026-07-05, 커밋 850e64e)** — `chassis/teleop_dualsense.py`(`python3 -m chassis.teleop_dualsense --no-us100`; DualSense→(v,ω), RT/LT=전후진·좌스틱X=회전·트리거0+스틱=피벗). ⭐**저속 코깅존 툭툭끊김·기동지연 제각각 해결** = `ChassisConfig.min_drive_turns_per_s` 최저 구동속도 플로어(기본 1.0rev/s; 0<|명령|<이값이면 부호유지 상향). **조향 슬루 4500 erpm**. 당시 다음은 WP4였고, 현재는 WP4·WP5 완료 후 WP6/WP8이 다음이다. ⚠️can0 LOOPBACK sticky·좀비 teleop kill 재확인.
- **⑧ 무선 차체 4WS 텔레옵 완료 (2026-07-06, 커밋 46f1fb1+dedb2fc)** — 유선 teleop_dualsense를 무선 분리: `chassis/teleop_server.py`(젯슨, `python3 -m chassis.teleop_server --no-us100`) ↔ `laptop/laptop_client_chassis.py`(노트북). x2212 무선 텔레옵(laptop_client_velocity↔pi_server_velocity, TCP:9000)을 참고해 만듦. **클라=매핑 안 하고 raw 입력만**(`"left_x rt lt sq ci\n"` 30Hz), **map_chassis_input·속도한계·min_drive·피벗·US-100은 전부 서버쪽**(클라 범용화). 구조=수신스레드(소켓)가 최신입력 공유상태 갱신→제어스레드(50Hz)가 edge판정·map·ChassisManager.tick(ChassisManager는 제어스레드만 만짐). □rising=arm토글·○rising=estop·클라끊김=구동0. **강건화(dedb2fc)**: 빈/죽은 CAN버스면 ACK없어 TX큐 참→ENOBUFS(CanOperationError)로 크래시하던 것 → `DriveOdriveCan._send`가 can.CanError 흡수(프레임 드롭)+제어루프 try/except. **★무선 엔드투엔드 HIL 통과(2026-07-06, 유선과 동일 코드경로)**: 캘리(RAM-only, 전원사이클로 날아감→`can_calibrate_all.py --nodes 11 12 15 16` 재캘리, 13/14 mid는 없어서 제외) 후 실제 서버+클라 경로로 CAN 스니핑 검증 — 전진 RT full v=1.5·**4축 2.40~2.42 rev/s 균일**, 좌회전 **애커만 차동 좌(안쪽)1.46<우(바깥쪽)2.01**·ω=+0.72(REP-103 좌회전 부호), arm/estop/끊김 다 동작. mid(13/14=NullSteer)는 없어도 chassis FAULT 안 됨(CornerModule은 steer state만 트립 판정, drive 부재는 무해). 실행=젯슨 `-m chassis.teleop_server --no-us100`+노트북 `laptop/laptop_client_chassis.py --host 192.168.8.106`.
- **⑧-b DualSense 축/버튼 매핑은 컨트롤러/연결(USB↔BT)/SDL 버전마다 다름 (2026-07-06, 커밋 9b2a346)** — 5월 corner_module HIL 때 "검증"이라 적힌 `RT=axis4/LT=axis3/□=btn0/○=btn2`(위 §31)가 **현재 노트북 DualSense(SDL 2.28)와 불일치** → 원격조종 축 이상 증상. **finder 도구 신설** `laptop/dualsense_axis_finder.py`(가이드형 — 시키는 대로 조작하면 메인스레드 타임드 캡처로 축/버튼 자동 판별 + 붙여넣기 블록 출력; `--monitor` 원시뷰. ⚠️ pygame은 conda base에만 설치됨, `python`으로 실행). **실측 현재값 = LX=axis0·RT(R2)=axis5·LT(L2)=axis2·□=btn3·○=btn1** (극성: 스틱 오른쪽=+, 트리거 뗌−1→당김+1이라 `trig()=(raw+1)/2` OK). 정지 시 트리거만 -1 → 축 종류(트리거 vs 스틱) 사전 판별 가능하나 L2/R2·버튼은 조작 필요. 액티브 텔레옵 5개(chassis/·corner_module/ 유선, laptop_client_chassis 무선, safety_us100/teleop_odrive_only, laptop_client_velocity) 전부 이 값으로 통일. laptop_client_basic/video는 이미 2/5. **컨트롤러/연결 바뀌면 finder 재실행 후 상수 교체.**

motor_control/corner_module/ (신규, 접근 A) = 로커보기의 "코너 1개" 제어기 = 조향 액추에이터(AK40, CAN) + 구동 액추에이터(ODrive 3.6) 협조 제어. 1차 목표는 재사용 라이브러리 + DualSense 텔레옵 데모. 단위는 액추에이터 네이티브(조향 °, 구동 turns/s), m/s 전환은 나중에 바퀴반경 0.1m 상수로.

현재 배선: 구동 USB + 조향 CAN(혼합), 추후 CAN-only 전환 예정 → 컨트롤러는 트랜스포트 무관하게 설계(DriveActuator 구현 교체만으로 USB→CAN).

**미래 핵심 제약: 4개 조향 모터를 애커만(Ackermann) 조향으로 동시 제어**해야 함. 미래 키네마틱스 레이어가 (차체속도, 조향반경/yaw) → 각 코너 (steer_deg, drive_vel) 변환 후 여러 CornerModule.set() 호출하는 소비자가 됨. 코너의 기하 위치·애커만 계산은 그 레이어 몫 → 현재 CornerModule 인터페이스(코너당 steer°+drive turns/s)는 변경 없이 확장 수용.

협조 로직(steer gate: 조향 따라오기 전 구동 자제)은 지금은 바퀴가 땅에 안 닿고 1개만 굴려서 OFF(hook만). motor_gui Transport는 역의존 방지 위해 import 안 함(motor_gui가 motor_control을 import하는 방향 유지).

**구현+HIL 완료 후 main 머지 완료(2026-05-25, 머지커밋 7736f60, 로컬·미푸시).** 9태스크 subagent-driven, 24 단위테스트, 실하드웨어 HIL(조향·구동·통합·텔레옵) 전부 통과. feature/corner-module 브랜치는 머지 후 잔존(삭제 안 함). 순수로직은 x86 dev 컨테이너 pytest(24개)로 검증.

**HIL 완료(2026-05-25, Jetson: AK40-10 id10 CAN + ODrive USB, powertrain_jetson 컨테이너).** 조향 20°/구동 1.46t/s, CornerModule 협조명령·워치독·estop 전부 정상. HIL이 버그 2건 잡아 수정(커밋 8e10341): ① drive_odrive_usb 클래스 enum→평면 int 상수(이 odrive lib는 클래스 enum 대입 시 TypeError), ② steer_ak40 arm()이 _last_rx_ms 미기록→arm 직후 stale 오판으로 첫 tick estop. **AK는 node id=10**(motor_gui --track ak와 동일; corner_module 기본 motor_id=1이라 HIL은 10 지정). ODrive 저속(setpoint<~1.5) 언더슈트는 NVM vel_integrator_gain=0 튜닝 이슈(드라이버 무관).

**텔레옵 HIL도 완료(DualSense, 커밋 6d72406): □arm→좌스틱 조향±45°, RT/LT 구동 전후진±4.4t/s 정상.** HIL이 잡은 텔레옵 버그 수정: DualSense 매핑 좌스틱X=axis0/RT=axis4/LT=axis3/□=btn0/○=btn2(초안 전부 틀림), 블로킹 sleep(0.3) 디바운스→상승엣지(폴링끊김→false-stale 유발), config stale_ms 200→500, motor_id=AK_MOTOR_ID(기본10), SDL더미 자동, 실행은 `python3 -m corner_module.teleop_dualsense`(직접 .py 실행은 import 깨짐). 남은 일: steer_current_limit_a 5.0A 현장 튜닝, drive_odrive_can 실구현(CAN전환), ODrive vel_integrator 게인 튜닝(저속 언더슈트).

**Jetson HIL 운영 메모: 비밀번호 문자열을 파일에 쓰지 말고 `$JETSON_SSH_PASS`/`$SUDO_PASS`만 사용. `ssh jetson`(키 등록됨), 현재 배포는 git pull 우선. 컨테이너 `sudo docker compose -f docker/docker-compose.jetson.yml exec -T powertrain`, can0는 `sudo bash scripts/can_setup.sh`, odrive/can은 컨테이너에만 있음(네이티브X), GUI는 setsid로 띄워야 exec 종료에도 안 죽음 http://jetson-orin.local:8000.** 관련 [[jetson-deploy-can]] 테스트는 컨테이너에서 `docker compose -f docker/docker-compose.yml exec -T powertrain bash -c "cd /workspace/motor_control && python3 -m pytest corner_module/tests/ -v"`. 관련: [[motor-gui-build]] [[ak-can-gui]] [[odrive-can-gui]] [[us100-safety-module]] [[docker-dev-env-for-tests]]

## doc-scope-powertrain-only

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/doc-scope-powertrain-only.md`

---
name: doc-scope-powertrain-only
description: "문서·자료 정리는 파워트레인 SW 담당 관점으로 — CAD·파워(전장) 소관은 인계 사항으로만, 팀 전체 발표전략·일정 확장 금지"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 7030ee16-240e-45ae-9fa1-f1a757a7dfc2
---

창공설 자료 정리(2026-07-07)에서 두 번 교정받음: ① 팀 전체 발표 전략·일정·심사 대응까지 넣었다가 "오지랖" 지적 → 파트 몫만. ② 사용자는 파워트레인 중에서도 **SW 담당** — CAD(기구 설계·모델링, 차체 악세사리, E6 브라켓/차동바)와 파워(전장, 멀티레일·전원 분기)는 별도 담당자가 있으므로 그쪽 내용은 우리가 쓰지 말고 넘기는 게 맞음.

**Why:** 문서 소유권 = 담당 영역. SW 담당이 CAD/전장 내용을 상세히 쓰면 소유권이 흐려지고 남의 파트 작업을 대신 정의하게 됨.

**How to apply:** 파워트레인 문서·자료 정리는 SW 스택(제어·통신·인지·안전·시뮬레이션 SW — 형상 최적화 포함) 중심으로 쓰고, CAD·파워 관련 항목이 걸리면 "인계 요구사항" 형태로만 정리해 해당 담당자 몫임을 명시. 전략 제안·비중 조정 코멘트 덧붙이지 않기. [[notion-team-hub]]

## docker-dev-env-for-tests

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/docker-dev-env-for-tests.md`

---
name: docker-dev-env-for-tests
description: x86 dev 컨테이너(powertrain_dev)는 "Jetson 사용 불가 시 차선 환경" — 무하드웨어 pytest뿐 아니라 ODrive USB 직결 실모터 구동까지 포함(--privileged 필요, odrive는 Jetson과 동일 git fw-v0.5.6=0.5.6). 실검증은 Jetson 우선. pip 의존성은 docker/Dockerfile
metadata:
  node_type: memory
  type: feedback
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**우선순위 (2026-06-16 사용자 지침):** 실제 실행·검증은 **Jetson Orin Nano 에서 직접** 해보는 것을 우선한다 (런타임 타깃이 Jetson). x86 노트북 컨테이너는 **Jetson 을 쓸 수 없을 때의 차선 환경**이다.

**범위 정정 (2026-06-24 사용자):** "무하드웨어 전용"이 아니다 — 무하드웨어 pytest·코드작성에 더해, **Jetson 없이 ODrive 를 노트북 USB 에 직결해 실제 모터를 굴리는 작업까지 포함**한다(이름도 "무하드웨어"보단 "Jetson 사용 불가 시"가 적절).

**x86 에서 실 ODrive USB 구동:** compose 가 `/dev` 마운트 + `SYS_RAWIO` 제공하지만, odrive 가 연결 시 USB reset(`USBDEVFS_RESET` ioctl)을 하는데 `cap_add: SYS_RAWIO` 로는 막혀 `[Errno 5] I/O Error` → **`docker run --rm -it --privileged -v /dev:/dev powertrain-sw:dev odrivetool`** 로 띄워야 연결됨. odrive 파이썬 라이브러리는 **Jetson 과 동일하게 git `fw-v0.5.6`(=0.5.6) 소스**로 설치(PyPI 는 0.5.4 다음 0.6.x 로 점프 → 0.5.6 미배포, 0.6.x 는 fw 0.5.x 와 프로토콜 비호환). fw 0.5.1 보드는 **device-level `odrv.clear_errors()` 없음(firmware-reflected) → axis-level `odrv.axis1.clear_errors()` 사용**(검증 스크립트·노션 컨벤션). 이 odrive git-source 레이어의 `build-essential` 때문에 이미지 ~3.3→3.73GB 불었는데 **build-essential 은 불필요**(odrive tools = 순수 파이썬 휠, libfibre 는 LFS prebuilt .so) → 빼면 회수 가능. 관련 보드 사실 [[bl70200-odrive-jetson-bringup]].

(Jetson 못 쓸 때) 이 노트북에서 파이썬 테스트·실행은 **x86 dev Docker 컨테이너 안에서** 한다 (conda 아님; 원 지침 2026-05-20). x86 이미지는 **CPU 전용 torch 로 슬림화**(9.9GB→3.3GB, commit `5bade45`) — GPU 추론은 Jetson 전용(`Dockerfile.jetson`). `docker-compose.gpu.yml` 은 CPU 전환으로 무의미해져 **삭제됨**.

**컨테이너**: `powertrain_dev` (이미지 `powertrain-sw:dev`), `docker/docker-compose.yml` 로 기동. 저장소 루트가 컨테이너 `/workspace` 에 bind mount → 호스트 코드 수정 즉시 반영. `network_mode: host` 라 GUI 포트(8000 등) 매핑 불필요.

**테스트/실행 incantation**:
```bash
docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/ -q"
```
컨테이너 꺼져 있으면: `docker compose -f docker/docker-compose.yml up -d`.

**Why / How to apply**:
- 새 pip 의존성이 필요하면 **`docker/Dockerfile`(x86) 과 `docker/Dockerfile.jetson`(Jetson) 에 추가** 후 `docker compose -f docker/docker-compose.yml build powertrain && up -d --force-recreate` 로 재빌드. 컨테이너에 직접 `pip install` 하지 말 것 (재생성 시 소실 + Dockerfile 이 source of truth).
- 2026-05-20 시점 x86 Dockerfile 에 추가됨: `python-can`, `fastapi`, `uvicorn[standard]`, `httpx`, `pytest` (motor_gui 용).
- subagent 에게 테스트 명령 줄 때 위 docker exec prefix 사용.
- Jetson 쪽은 `docker/docker-compose.jetson.yml` 의 `powertrain_jetson` 컨테이너 (별도).

관련: [[motor-gui-build]].

## fsm-competition-track

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/fsm-competition-track.md`

---
name: fsm-competition-track
description: 대회 2개(국방·극한) 출전용 FSM 설계 트랙 — 규정 핵심 수치·초안 대조 결과·3레이어 방향
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

2026-06-13 시작: docs/의 규정집 2개(HWP)·FSM.json 초안(드로우.io, 극한4+국방5 구간 플로우차트)을 대조해 FSM 설계 방향 수립. HWP 표 추출은 conda base의 pyhwp(`hwp5proc xml`)로 함 — `hwp5txt`는 표를 `<표>`로 누락.

**규정 핵심 (FSM에 직결):**
- 국방(9월, 리허설 9/12): 5구간×20분, 자율130+원격65=195점, 트랙폭 914.4mm. 극한(10/1~2): 4구간×10분, 구간당 자율60+원격40=400점, 트랙폭 ≥1000mm.
- 공통: 미션 도전 무제한·부분점수·구간포기 가능(0점, 동점 후순위)·작동불능→정비 후 재시작(완료미션 인정, 시간 계속)·조종자는 로봇 비전으로만(원격도 FPV)·자율 중 조종자가 화면 보고 인식 결과지 수기 작성.
- 원격이 전체 점수 33~40% → TELEOP 모드 + 스트리밍 UI가 1급 시민.

**초안 대조에서 찾은 단순화/오류 (팀 논의 필요):**
- 극한3 출입문 = 규정상 "좌측 PUSH(미시오)" → GRASP_HANDLE/PULL_DOWN 레버 조작 불필요 가능성. 주최 확인 1순위.
- 극한4 REPORT_DATA 무선전송+ACK는 과설계 — 결과지는 조종자 수기, 로봇은 화면 오버레이만 (yolo_depth_3d 연장).
- 지형 순서 규정 고정(극한1 자갈→파쇄석→나무, 국방2 사구→자갈→수중) → YOLO 지형분류 대신 odometry 전환이 베이스라인.
- 타임아웃이 구간 내부 상태로 산발(극한 1·3·4 누락) → 전역 supervisor 타이머+미션별 시간예산으로 승격.
- FELLOVER(팔로 86kg 일으키기) 비현실 → MANUAL_RECOVERY 요청으로 단순화.
- 리스크: 연막은 LiDAR/RealSense IR 모두 취약한데 초안 절반이 LiDAR 가정 — ~~센서 스택 미결정이 선행 블로커~~ (**2026-07-07 갱신: 자율 센서 배치 확정** — L515 3D LiDAR가 RPLIDAR 대체=**우리 카메라**(RGB 레인+depth 벽추종+**내장 IMU** 오도메트리), **D435i=로봇팔 전용**, US100=독립 안전. **D435i 독점·라이다 역할분담 해소**(센서 분리, 자율구간 실내 확정). 연막은 별건 잔존. 상세=docs/plans/2026-07-02-autonomous-driving-kickoff.md 센서배치 배너 — [[robot-arm-team-resources]]). 차폭 914mm 통과 여부 설계팀 확인 필요. LiDAR·로봇팔 코드 repo에 없음(팔은 블랙박스 액션 인터페이스로 추상화).

**합의 제안한 구조:** 3레이어 — L0 Supervisor(BOOT/IDLE/TELEOP/AUTO/FAULT/RECOVERY + 전역타이머·estop·스트리밍, [[corner-module-ackermann]] 빌딩블록), L1 Mission Sequencer(대회별 yaml: 미션=행동시퀀스+배점+시간예산+on_fail), L2 공용 behavior 8개(DRIVE_TERRAIN/NAV_WAYPOINT/DETECT_CLASSIFY/ALIGN_TO/ARM_ACTION/CARRY_MONITOR/FOLLOW_TARGET/WAIT_SIGNAL, 공통 modifier timeout·retry_max·on_fail). 구현 순서: 센서스택 확정 → 스펙 → DRIVE_TERRAIN+DETECT/ALIGN → 국방 먼저 조립, 극한(계단·문·연막) 추가. [[jetson-realsense-d435i]]

## gl-sft1200-robot-ap

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/gl-sft1200-robot-ap.md`

---
name: gl-sft1200-robot-ap
description: "GL-SFT1200(Opal) 라우터 = 로봇 전용 WiFi AP (대회장 NITEZ 없음 대비 standalone), 젯슨 유선 LAN + 노트북 5GHz, 설정은 SSH+uci"
metadata:
  node_type: memory
  type: project
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

GL-SFT1200 "Opal" (GL.iNet fw **4.3.28** / OpenWrt 18.06) = 로봇 **전용 WiFi AP**. 대회장엔 기존 공유망(NITEZ)이 없어서 도입 — **대회장=standalone**(WAN 미사용). **개발 중=WAN을 외부 업링크(예: 172.16.x 이더넷 드롭)에 꽂아 인터넷+로봇링크 동시**(라우터 NAT) — 노트북이 `ZETIN-ROBOT-5G` 한 연결로 둘 다 씀, NITEZ 불필요. 라우터는 로봇에 장착.

**토폴로지:** 젯슨 유선 `enP8p1s0` → 라우터 **LAN 포트(WAN 아님!)** = `192.168.8.106` DHCP 고정예약(SSH 타깃 고정, MAC 3c:6d:66:f8:8f:8a). 젯슨은 NM 관리 — `Wired connection 1`을 `ipv4.route-metric 100`+`connection.autoconnect-priority 10`으로 pin → 부팅 시 **라우터 유선 우선**(NITEZ WiFi=metric600 폴백), 인터넷도 라우터 WAN NAT 경유(ping ~33ms). 젯슨 도달 = `192.168.8.106`(우선) 또는 `192.168.50.98`(NITEZ, 둘다 살아있음). 노트북 → **5GHz WiFi**(움직이는 로봇 무선 추종). 라우터=`192.168.8.1`. 젯슨 SSH 유저=`zetin`(root 아님)·비번 `$JETSON_SSH_PASS`.

**WiFi:** SSID `ZETIN-ROBOT`(2.4G ch6) / `ZETIN-ROBOT-5G`(5G ch36 VHT80, 주력), WPA2, country KR. 자격증명은 `.claude/settings.local.json`에서 주입되는 `$ROUTER_ADMIN_PASS` / `$ROUTER_WIFI_PASS`만 참조하며 실제 문자열을 문서에 기록하지 않는다.

**설정법 = SSH+uci** (GL `/rpc` JSON-RPC 추측 불필요). 라우터 dropbear가 **ssh-rsa 호스트키만** 제시 → ssh에 `-o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa` 필수. 노트북이 NITEZ에 있을 땐 젯슨 경유 터널: `ssh -L 2222:192.168.8.1:22 zetin@<jetson>` 후 `root@127.0.0.1:2222`. 노트북이 AP에 직접 붙으면 `root@192.168.8.1` 직접. uci 키: `wireless.default_radio{0,1}.ssid/key`, `wireless.radio{0,1}.country`, 정적예약 `uci add dhcp host` + `.mac/.ip`. 적용 `uci commit wireless; uci commit dhcp; wifi reload`. (`/rpc` 로그인은 challenge→`crypt(pw,"$5$"+salt)`→`sha256(user:crypt:nonce)`→login, alg5=sha256crypt — 참고용, 안 씀.)

**검증(2026-06-29 HIL):** 노트북↔젯슨 ping — NITEZ 경유 avg **4.9ms** vs 전용 5G링크 avg **2.19ms**(jitter 0.6ms, loss 0%). ~2.2× 저지연 + 격리 + 대회장 가용. 노트북 전환 `nmcli dev wifi connect ZETIN-ROBOT-5G password $ROUTER_WIFI_PASS` + 절전OFF `nmcli c modify ... 802-11-wireless.powersave 2`; 복귀 `nmcli c up NITEZ`. WAN 업링크 추가 후(2026-06-29): 노트북 `ZETIN-ROBOT-5G` 한 연결로 다운 **346Mbps**·인터넷 ping 15~37ms·**젯슨링크 1.9ms 영향0**(동시) 확인. 노트북 프로파일 `connection.autoconnect-priority 10` → NITEZ보다 우선(AP 꺼지면 NITEZ 폴백).

⚠️ **함정:** 라우터 전원은 **자체 5V2A** 어댑터로 줄 것(젯슨 USB로 전원따면 전류부족 → WiFi 미부팅/리부팅반복, SSID 안 뜸). country 변경 직후 2.4G 라디오는 ACS settle에 수 초 걸림(Tx 0dBm·channel unknown 일시 → 정상화). nmcli 상태변경(connect/modify/up)은 non-interactive 셸에서 `echo "$SUDO_PASS" | sudo -S nmcli ...`로.

링크: [[jetson-deploy-can]] [[jetson-realsense-d435i]] [[jetson-docker-motor-control]] [[can-bus-multidevice-topology]]

## jetson-can-loopback-footgun

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/jetson-can-loopback-footgun.md`

---
name: jetson-can-loopback-footgun
description: Jetson can0(mttcan/SocketCAN) loopback 컨트롤 모드는 down/up에도 sticky — 명시적 loopback off 안 주면 남아서 모든 진단 오염. ACK가 모든 baud에서 성공하면 loopback 의심
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**증상 (실제로 당함, 2026-06-20 AK45-36 HIL):** Jetson `can0`를 `ip link set can0 up type can ... loopback on` 으로 한 번 켜면, 이후 `ip link set can0 down; ... up type can bitrate ...` 로 재기동해도 **loopback 플래그가 그대로 남는다**(SocketCAN 컨트롤 모드는 sticky — 명시적으로 `loopback off` 줘야 꺼짐). 리부팅하면 깨끗해짐.

**loopback이 켜져 있을 때의 가짜 신호 (이게 진단을 통째로 오염시킴):**
- 우리 송신이 **자기 자신에게 self-ACK** → `cansend` 후 TX 카운터가 **모든 bitrate(1M/500k/250k/125k)에서 완료**됨. → **"ACK가 모든 baud에서 성공"하면 100% loopback 의심**(정상 모터는 자기 baud에서만 ACK).
- candump는 **우리 echo만** 보임, 버스 실트래픽(모터 status 100Hz)은 **안 들림**(can0가 외부 버스에 귀를 닫음).
- 모터에 명령 보내도 **버스로 안 나가서 모터 무반응** → "MIT 모드 의심" 같은 **틀린 결론**으로 샘.

**구분법:**
- `ip -d link show can0` 에 `<LOOPBACK>` / `can <LOOPBACK>` 떠 있으면 loopback 모드.
- **진짜 죽은 버스**: TX=0 + bus-off/error-warn 카운터 상승 (아무도 ACK 안 함).
- **loopback 박힌 상태**: TX 완료됨 + passive candump 무음 + 어느 baud든 ACK.

**처방:** 진단 전 항상 `ip link set can0 up type can bitrate 1000000 loopback off restart-ms 100` 로 **loopback off 명시**, 또는 **리부팅**으로 클린 스타트. 리부팅 후 can0는 DOWN/기본값(loopback 없음)이라 노션 can_setup 그대로 재현됨.

관련: [[jetson-deploy-can]]. AK CAN 인식 확인은 모터가 status broadcast(100Hz) 켜져 있으면 **패시브 candump만으로 노드 ID(pkt 0x29) 뜸**; 안 켜졌으면 RPM=0(패킷3) "모닝콜" 후 STATUS_1 응답으로 확인.

## jetson-deploy-can

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/jetson-deploy-can.md`

---
name: jetson-deploy-can
description: "Jetson HIL 배포/실행 워크플로 — sshfs 마운트, can_setup, 컨테이너 exec, host-network 접근"
metadata:
  node_type: memory
  type: reference
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

Jetson Orin Nano 실하드웨어 작업 워크플로 (motor_gui / AK-CAN HIL).

- **접속**: `sshpass -p 0000 ssh zetin@jetson-orin.local`. sudo는 `echo 0000 | sudo -S`.
- **파일 배포**: 사용자가 `sshfs zetin@jetson-orin.local:/home/zetin ~/orin_mount` 해둠 →
  로컬에서 `cp <파일> ~/orin_mount/Defence_Robot/<경로>` 면 Jetson `~/Defence_Robot` 에 바로 반영.
  컨테이너가 그 디렉터리를 `/workspace` 로 마운트하므로 코드 즉시 반영.
  **마운트가 끊겨 있으면**(`~/orin_mount` 비어있음) `sshpass -p 0000 scp <파일>
  zetin@jetson-orin.local:~/Defence_Robot/<경로>` 로 직접 복사(폴백).
- **컨테이너**: `powertrain_jetson` (docker compose -f docker/docker-compose.jetson.yml).
  `network_mode: host` + privileged → can0/USB 공유. 죽어있으면 `docker compose ... up -d` 또는
  `restart powertrain`.
- **CAN 준비**: `cd ~/Defence_Robot && echo 0000 | sudo -S bash scripts/can_setup.sh` (1Mbps,
  can0 down→up). 리부팅/에러 후 필요.
- **서버 기동**: `docker exec -d powertrain_jetson bash -lc "cd /workspace &&
  python3 -m motor_gui.backend.server --track ak --port 8000 >/tmp/mg_ak.log 2>&1"`.
- **host-network 덕에 노트북에서 직접 접근**: `curl http://jetson-orin.local:8000/api/...`
  (명령 POST 가장 확실). 텔레메트리(WS)는 컨테이너 안 python 스크립트가 안정적.
- **함정**: `docker exec` 안 inline python 의 중첩 따옴표/stdout 이 ssh 파이프로 자주 유실됨
  → 스크립트를 마운트에 쓰고 결과를 `/workspace/out.txt` 로 받아 마운트로 읽는 방식이 안정적.
  컨테이너엔 `ip`/`node` 없을 수 있음(호스트엔 있음).

관련: [[ak-can-gui]] [[docker-dev-env-for-tests]] (테스트는 x86 powertrain_dev, HIL은 Jetson).

## jetson-docker-motor-control

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/jetson-docker-motor-control.md`

---
name: jetson-docker-motor-control
description: Jetson 모터 작업 분담 — can0 링크 셋업은 HOST(sudo, 노션 can_setup), python(ak_control/motor_gui)은 docker 컨테이너 powertrain_jetson에서 실행(python-can은 컨테이너에만). can0는 host net으로 공유
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**규칙 (사용자 명시, 2026-06-22):** Jetson Orin Nano 에서 CAN·모터 제어 관련 작업은 **무조건 docker 컨테이너 안에서** 한다. host 에서 python 으로 돌리지 말 것.

**왜:** `python-can`(4.6.1)·프로젝트 런타임 의존성은 **컨테이너 `powertrain_jetson` 에만** 설치돼 있고 **host python3 엔 `import can` 안 됨**(`ModuleNotFoundError: No module named 'can'`). host candump/cansend(can-utils)만 있음.

**컨테이너 사실:**
- container_name `powertrain_jetson`, image `powertrain-sw:jetson`, compose `docker/docker-compose.jetson.yml`(service `powertrain`).
- `network_mode: host` + `privileged: true` → **can0 를 host 와 공유**(컨테이너에서 can0 그대로 보임/제어 가능).
- repo 가 컨테이너 `/workspace` 에 마운트 → 스크립트 경로 예: `/workspace/motor_control/steering/ak_control.py`, `/workspace/scripts/can_setup.sh`.
- 컨테이너 user = root (sudo 불필요·애초에 sudo 없음).

**올바른 작업 분담 (사용자 확정 2026-06-22): ① CAN 셋업은 HOST → ② docker 들어가서 python.**
- ① **HOST**: 노션 `CAN 모터 제어 on Jetson` 절차 그대로 = `sudo ip link set can0 down; modprobe can/can_raw/mttcan; busybox devmem 0x0c303018 w 0xc458; busybox devmem 0x0c303010 w 0xc400; ip link set can0 up type can bitrate 1000000` (= `scripts/can_setup.sh`). 호스트엔 ip/busybox/modprobe 있음. host sudo 비대화형은 `echo "$PW" | sudo -S bash <script>` (한 번에 root로 스크립트 실행 — 내부 bare sudo는 root라 통과; `sudo -v` 캐시는 무 tty라 내부 sudo로 안 이어짐).
- ② **DOCKER**: `docker start powertrain_jetson` → `docker exec powertrain_jetson python3 /workspace/motor_control/steering/ak_control.py`. `network_mode host`라 호스트가 올린 can0 그대로 보임. **(can0를 컨테이너에서 올리려 iproute2 apt설치하던 건 잘못된 우회였음.)**
- 컨테이너엔 sudo/ip/busybox 없지만 python-can 4.6.1 있음. AK40-10 테스트모터=CAN ID 10, 데모=`demo_core_features`(모닝콜→영점→+90°→40rpm 3초→홀딩).

관련: [[jetson-deploy-can]] · [[jetson-can-loopback-footgun]] · [[docker-dev-env-for-tests]]

## ★함정: 좀비 제어루프 프로세스가 모터 테스트 오염 (2026-07-05, 디버깅 半日 날림)
**모터가 갑자기 거칠게 진동/undershoot(지령의 77%)/간헐 멈춤 → 하드웨어·캘리·게인 의심 전에 젯슨에 떠있는 백그라운드 제어루프부터 확인·kill할 것.** 실사례: `chassis.teleop_dualsense --no-us100`를 켜놓고 안 꺼서 58분째 백그라운드 실행 → 매 tick `ChassisManager.set(v,ω)`를 (스틱 입력 0이라) **v=0으로 6모터에 계속 명령** → 새 테스트 스크립트의 지령과 **한 버스에서 싸움**. 증상: ①0↔지령 surging(=거친 진동) ②0으로 잘려 평균 undershoot(~77%) ③재수없으면 그놈이 이겨 아예 멈춤(모터마다 간헐 다르게). 오진단 유발: config/캘리/게인/HALL/전원까지 다 정상인데 원인 못 찾음(전부 정상이니까). **결정타 진단**: USB로 `ax.controller.input_vel=2.0` 쓰고 0.3s 뒤 read하면 **0.00으로 리셋**돼 있음(setpoint=0, Iq≈0). 워치독 off인데 리셋 = 다른 writer 존재 확정. **찾기**: `docker exec powertrain_jetson ps -eo pid,etime,args | grep -iE 'teleop|chassis|corner|motor_gui|server'`. **kill**: `docker exec powertrain_jetson pkill -f teleop`. 죽이면 즉시 100%·std 0.017(매끈) 복구. ⇒ 모터 실기 테스트 전 루틴으로 백그라운드 프로세스 스캔. [[bl70200-odrive-jetson-bringup]] [[corner-module-ackermann]]

## jetson-realsense-d435i

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/jetson-realsense-d435i.md`

---
name: jetson-realsense-d435i
description: Jetson에 Intel RealSense D435i RGB-D 카메라 + SDK 설치(Dockerfile baking)·검증·사용법
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

2026-06-02 역사 기록: Jetson Orin Nano에 D435i USB3 연결. 당시 메인 비전 계획은 7/7 센서분리로 대체됨 — 현재 D435i=로봇팔 전용, L515=파워트레인, US-100=독립 안전. [[corner-module-ackermann]]

**SDK 설치 = `docker/Dockerfile.jetson` 에 librealsense v2.55.1 + pyrealsense2 소스 빌드 baking (커밋 fa75674 + fix 019e186 + 테스트 b40fc57, main).** 핵심 플래그·함정:
- **`-DFORCE_RSUSB_BACKEND=ON`** — 컨테이너라 커널 패치 불가 → libusb 유저스페이스 백엔드. compose 가 privileged + `/dev` 마운트 + network host 라 카메라 접근됨.
- **`-DIMPORT_DEPTH_CAM_FW=OFF`** — 안 끄면 cmake configure 가 펌웨어 바이너리를 다운로드하다 **도커 빌드 네트워크에서 SSL connect error 로 실패**(실행 컨테이너는 host net 이라 됐지만 buildkit 빌드 net 은 다름). 카메라에 펌웨어 이미 있어 스트리밍엔 불필요.
- **make install 버그**: 메인 `pyrealsense2.cpython-310-aarch64-linux-gnu.so` 와 `__init__.py` 를 site-packages 에 안 넣고 `pyrsutils` 만 넣음 → `import pyrealsense2` 가 빈 네임스페이스 패키지(`__file__=None`, `rs.context` 없음). **해결: `build/Release/` 에서 메인 .so 복사 + `printf 'from .pyrealsense2 import *' > __init__.py`** (Dockerfile 에 포함). 설치경로 `/usr/local/lib/python3.10/dist-packages/pyrealsense2/`(기본 path, PYTHONPATH 불필요).
- `make -j4`(8GB OOM 회피). 베이스 py3.10/aarch64 고정 가정으로 경로 하드코딩. 빌드 ~30~45분.

**검증법/점검 스크립트**: `motor_control/vision/realsense_test.py` — 컨테이너에서 `python3 motor_control/vision/realsense_test.py` → 장치/FW, depth·color 640x480, 중앙거리(m), 유효픽셀% 출력. 재빌드 후 새 이미지 컨테이너에서 depth+color 정상 확인(유효픽셀 ~46~81%). RGB 단독은 SDK 없이도 `/dev/video2` UVC 로 OpenCV 캡처 가능하나 depth 는 SDK 필수(video0/1 등 raw 스트림은 OpenCV 로 못 엶).

운영: 빌드/재빌드는 detached + 로그(`~/rs_image_build.log`) 폴링으로 진행. `up -d --build` 가 이미지 baking + 컨테이너 재생성. 관련 [[jetson-deploy-can]] [[docker-dev-env-for-tests]].

**2026-06-10 — YOLO+Depth 3D 좌표(`vision/yolo_depth_3d.py`, commit 17e3b96) + 스트리밍 랙 디버깅 학습:**
- **랙 1순위 용의자 = 수신 노트북 WiFi 절전모드.** AP 가 패킷을 버퍼링했다 몰아줌 → 초 단위 랙. **저fps 스트림일수록 악화**(프레임 간 공백에 라디오 doze 발동 — 30fps raw 는 멀쩡한데 YOLO 15fps 만 랙 걸리는 미스터리의 정체). 노트북 NM 프로파일 `802-11-wireless.powersave 2` 영구 반영. 전원 프로파일 전환 시 다시 켜질 수 있어 재발 시 첫 점검 항목.
- `avdec_h264` 기본 멀티스레드는 프레임 **개수** 단위 고정지연(코어수만큼) → 저fps 에서 초 단위 증폭. recv_stream.sh 에 `max-threads=1`.
- 인코더 파이프 write 는 인코딩 끝까지 블록 → 검출 루프와 직렬화돼 fps 절반 이하. **AsyncWriter 스레드(최신 1장만, drop)** 패턴으로 해소(5.5→21fps). openh264 `complexity=low scene-change-detection=false`.
- TRT FP16 단독으론 효과 미미(추론은 원래 GPU) — 병목은 대개 인코딩/네트워크/수신. TRT 엔진은 `/workspace/yolov8n_480x640_fp16.engine` 캐시됨. 컨테이너 pip 미러(jetson.webredirect.org) 죽어 있음 — onnxslim 등 옵션 deps 설치 실패해도 export 는 진행됨.
- 젯슨 WiFi(RTL8822) TX 출력 7dBm 드라이버 하드락(KR regdom·txpower 설정 무효) — 현장 RF 대비는 USB 어댑터/유선 고려.
- 진단 도구: `--tx-stamp`(송신시각 워터마크) + 스크린샷 대조로 종단 지연 실측(전송+표시 ~40ms), `frame_age`(rs 글로벌 타임스탬프 vs now)로 카메라 큐 적체 측정.

**2026-06-13 — 스트리밍 SRT 전환 (commit b0952b1, 대회 원격주행 대비):**
- **Orin Nano엔 NVENC 하드웨어가 아예 없음**(Orin NX/AGX만) — 과거 "ABI 불일치" 주석은 오진이었음. SW 인코딩 전제: 공용 `vision/gst_stream.py`, x264 zerolatency 기본 + openh264 폴백(`--encoder`). x264는 이미지 재빌드 필요(Dockerfile 말단에 plugins-ugly 레이어 추가 — librealsense 캐시 안 깨짐).
- 전송 UDP RTP→**SRT**(ARQ, latency=120ms): 송신 listener/수신 caller라 --host 없이 송신, 수신기 재시작 무관. srtsrc caller는 접속실패/링크사망 시 **EOF 없이 무한대기** → 수신측 select 기반 stall 워치독 필수.
- yolo_depth_3d: 오버레이 굽기 제거, 좌표는 UDP JSON(:5001) 분리 → 노트북 `scripts/recv_yolo3d.py`가 합성(시간기반 느슨한 sync). 연막 구간 주 정보원.
- 디버깅 함정 재확인: ① pkill 자기매치 — kill과 start/sed를 **반드시 별도 Bash 호출로**(같은 명령에 plain 파일명 있으면 bracket 트릭 무력화) ② 파이썬 stdout 리다이렉트 블록버퍼링 — 장기실행 로그는 flush=True 없으면 유령 "무응답" ③ SIGTERM으로 죽인 수신기의 gst 고아가 다음 listener 오염 — signal 핸들러로 자식 정리 ④ 수명 제한 테스트 송신기는 진단 중 만료되어 오진 유발.

**2026-06-14 — Jetson HIL 실기 검증 완료 (commit 0ad2e53):** Orin Nano(192.168.50.98)+RealSense D435i, 640x480 TRT(캐시 엔진 yolov8n_480x640_fp16.engine), 노트북 192.168.50.203.
- **결과: SRT+좌표분리 end-to-end 동작. coord_age 0.00~0.03s(영상-좌표 1프레임 내 동기), 재접속 0. person을 d=0.67m XYZ=(+0.11,+0.01,+0.66)m로 실측. 노트북창 오버레이(좌표채널로 그린 박스+3줄 라벨) 시각 확인.**
- **인코더 실측: x264 zerolatency 17.4fps/frame_age~107ms vs openh264 8.8fps/~170ms — x264 fps 2배·지연 낮음(분석 적중). 기본값 x264 타당.** x264는 `gstreamer1.0-plugins-ugly` 필요 → Dockerfile 말단 레이어라 **재빌드 1~2분(librealsense 레이어 CACHED).** `docker compose build`(빌드만) 후 `up -d`(컨테이너 recreate)로 신이미지 반영. srtsink/srtsrc/mpegtsmux는 **기존 이미지에 이미 있었음**(x264만 없었음).
- **새 함정: `jetson-orin.local` mDNS가 IPv6 link-local(fe80::)로 먼저 풀림 → gst SRT URI는 scope id 못 실어 무한 접속실패.** 명시 IPv4(192.168.50.98)로는 정상. 수신기 양쪽에 IPv4 강제(getaddrinfo AF_INET / getent ahostsv4) 박아 기본 호스트명만으로 즉시 부착. SRT는 listener(Jetson)/caller(노트북).
- **rs.align 병목 (commit 18ec0e5, 사용자 "화면 작고 느림" 보고 진단):** 단계별 프로파일링 결과 `rs.align(depth→color)` 전체 프레임 정렬이 **Orin Nano CPU에서 ~108ms/프레임 = 루프의 80%**(YOLO TRT는 24ms뿐). 박스 중심 몇 점 depth만 필요한데 과함 → `DepthCal`+`deproject_box`로 **검출별 color→depth 픽셀 투영(`rs2_project_color_pixel_to_depth_pixel`)+depth→color 외부변환(`rs2_transform_point_to_point`)**, 정렬 제거(2.9ms). **7→30fps(카메라 상한), frame_age 165→60ms.** 좌표는 align 방식과 같은 프레임 대조 시 **평균 11mm·95%ile 25mm 일치**(센서 노이즈 이내) — extrinsic 변환 빼면 depth 프레임 기준이라 ~수cm 어긋나니 변환 필수. 교훈: Jetson에서 sparse depth 조회는 절대 full-frame align 쓰지 말 것. 진단법: capture/align/yolo/depth 단계별 time.time() 누적 프로파일러(컨테이너에서 1회 실행).
- 수신 창이 hidpi에서 손톱만 함 = recv_yolo3d.py가 640x480 1:1 표시 → `cv2.WINDOW_NORMAL`+`resizeWindow`(`--scale` 기본 1.8x)로 키움.
- **2026-06-14 기본 해상도 848x480(16:9, commit 734e53c)**: D435i 16:9 모드(848x480=깊이 네이티브) 중 선택, 30fps 무손실(정렬 제거 덕). 양쪽 `--width/--height` 일치 필요. 4:3 되돌리려면 640x480. TRT 엔진은 해상도별(480x864) 1회 재빌드.
- **2026-06-14 레이턴시 최적화 + 수신 2경로 분할(commit ad14264)**: 측정(`--tx-stamp` 큰글씨 + `recv_yolo3d --clock`로 노트북시각 오버레이 → 같은 프레임 tx vs laptop 차이; cv2 GUI 창이 XWayland Qt폰트 깨져 안 뜰 땐 프레임에 시각 그려 PNG 저장해 판독). 기기 시계 오프셋 ~1.4ms(ControlMaster ssh 라운드트립으로 측정, 일반 ssh는 핸드셰이크 250ms+라 부정확). **레버2개: ① 영상을 YOLO(24ms) 전에 먼저 송신(좌표는 뒤에 별도채널) ② SRT latency 120→낮춤.** 결과: SRT=120·추론후송신 송신→표시 254ms(g2g~320) → SRT=15·영상우선 송신→디코드 ~115ms. 글래스투글래스 ≈ 카메라40+송신디코드115+표시40 ≈ 데이터155/화면190. **표시경로 마지막 40ms(cv2/컴포지터)는 네이티브 gst가 더 빠름.** SRT latency는 송·수신 max 협상이라 양쪽 같이 맞춰야 함, 기본 60.
- **수신 2경로**: ① `scripts/recv_stream.sh [PORT] [HOST] [LATENCY]` 저지연 네이티브 gst(오버레이X·원격주행) ② `scripts/recv_yolo3d.py` 좌표오버레이 cv2(표시지연↑·정밀접근). 둘 다 SRT caller·IPv4강제.
- **2026-06-14 기본 YOLO 모델 v8n→YOLO26n(팀 결정, commit cf4575c)**: ultralytics 8.4.52가 YOLO26 지원, `yolo26n.pt`(5.5MB) GitHub assets v8.4.0에서 자동 다운로드. **NMS-free(end-to-end) 모델이 `results[0].boxes` API로 투명 통합** — TRT 익스포트(480x864)·30fps·좌표·오버레이 v8n과 동일하게 정상. 커스텀 학습 모델은 `--model` 로 지정. 캐시: `/workspace/yolo26n_480x864_fp16.engine`.
- HIL 워크플로: 레포 동기화는 git pull 대신 **scp로 vision 파일만 직접 복사**(Jetson 로컬 미커밋 변경 보존 — `~/hil_backup_YYYYMMDD/`에 백업 후). 레포는 `../:/workspace:rw` bind-mount라 host 파일 수정 즉시 컨테이너 반영. 송신기 `docker exec -d ... nohup ... --bench-frames N`, 노트북 수신기 `--headless`로 fps/coord_age 폴링. 본 세션 laptop 실제 레포 경로 `/home/light/ZETIN/robotics/power-train-sw`(`/home/light/Defence_Robot`는 도구 매핑용 논리경로, Bash엔 없음).

## motor-gui-bl70200-config-contamination

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/motor-gui-bl70200-config-contamination.md`

---
name: motor-gui-bl70200-config-contamination
description: motor_gui --track usb 의 게인/리밋 튜너블 기본값이 X2212 기준 → BL70200 ODrive 에 적용/저장 시 NVM 오염(current_lim 100A·pos_gain 8·vel_int 0·ifbw 50·vel_gain 0.015). 증상=캘리 OK·state8인데 vel_int 0 라 부하 시 안 돎(Iq 안 감김). 점검=BL70200 셋업 §2 vs NVM 전수 대조
metadata:
  node_type: memory
  type: project
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

**2026-06-25 실HW 디버깅 중 발견.** motor_gui 웹 GUI 로 BL70200 ODrive(node11/axis1) 를 `--track usb` 로 테스트하면서 게인/리밋 튜너블을 만지면(또는 그 직후 `save_configuration()` — 예: torque_constant 저장), **motor_gui DEFAULT_TUNABLES(X2212-13 기준값)가 BL70200 ODrive 에 적용되고 NVM 까지 저장돼 오염**된다.

**오염된 값 (정상=BL70200 셋업 페이지 §2):**
- `motor.config.current_lim` **100 A** → 9 A (⚠️ 가장 위험 — 고전류 명령 시 모터/보드 손상 가능)
- `controller.config.pos_gain` 8.0 → 2.0
- `controller.config.input_filter_bandwidth` 50.0 → 2.0
- `controller.config.vel_gain` 0.015 → 0.06
- `controller.config.vel_integrator_gain` **0.0** → 0.2

**증상 (헷갈림 주의):** 캘리는 정상(is_calibrated·enc_ready True), CLOSED_LOOP state=8, err 0x0 인데 **속도 명령에 모터가 안 돈다**. 이유: vel_integrator_gain=0 + vel_gain 0.015 라 P항(0.015×err)만 있고 적분기가 안 감겨 **정지마찰/코깅을 못 이김**(vel_setpoint 은 정상인데 Iq 가 안 치솟음 ~0.1A). vel_int 0.2 로 고치면 적분기가 감겨 break-free → 정상 회전. (무부하 자유축이면 낮은 게인에도 돌아서 안 들킬 수 있음 — 부하 걸리면 드러남.)

**점검·복구:** ODrive NVM 을 BL70200 셋업 §2 최적값과 **전수 대조**(motor/encoder/controller/board/CAN). 불일치는 setattr 로 복구 후 `save_configuration()`(리부팅 → 캘리 소실 → 재캘리 필요). 핵심 정상값: pp10·cpr60·HALL·bw30·calib_scan_omega6.0·**pos2.0/vel0.06/vel_int0.2**·ifbw2.0·vel_limit50·current_lim9·torque_constant0.353·UV40/OV56/brake2.0·node11/500k.

**Why:** motor_gui DEFAULT_TUNABLES(`backend/transport/base.py`)는 X2212-13+TLE5012B 스윕 최적값(pos8.0/vel0.015/vel_int0/ifbw50)이다. BL70200(HALL)과 전혀 다름. CLAUDE.md "Never mix tracks on the same ODrive" 경고의 실사례.

**How to apply:** ① BL70200 을 motor_gui `--track usb` 로 만진 뒤엔 반드시 §2 대조로 게인/current_lim 복구. ② motor_gui 로 BL70200 굴릴 땐 게인 튜너블 적용·NVM 저장 주의(X2212 기준값 박힘). ③ 향후 motor_gui 에 모터 프로파일 분리(BL70200/X2212 별 DEFAULT_TUNABLES) 검토. 관련 [[bl70200-odrive-jetson-bringup]] [[motor-gui-build]].

## motor-gui-build

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/motor-gui-build.md`

---
name: motor-gui-build
description: "motor_gui (웹 기반 모터 진단 GUI) 빌드 상태 — 코드(Task 1-10) 완료·25 테스트 통과·push 됨, Task 11 실하드웨어 HIL 만 남음"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

`motor_gui/` — Jetson 에서 도는 웹 기반 ODrive/AK 모터 벤치 진단 GUI (FastAPI+WS 백엔드 + 바닐라 JS/uPlot 프론트, 100Hz 텔레메트리 plot + 제어). spec `docs/specs/2026-05-20-motor-gui-design.md`, plan `docs/plans/2026-05-20-motor-gui-plan.md`.

**상태 (2026-05-20):**
- Task 1–10 (코드 전체: transport base/fake/usb_odrive/can_bus, worker, commands, recorder, server, frontend, Dockerfile 의존성, README) **완료 + push (commit ~592b2a8)**. fake 트랙으로 25 테스트 통과. 최종 리뷰 "Ready to merge".
- **Task 11 (실하드웨어 HIL) 만 남음** — Jetson 온라인 + 하드웨어 연결 필요:
  1. Jetson `git pull` + jetson 이미지 재빌드 (`docker compose -f docker/docker-compose.jetson.yml up -d --build`, fastapi/uvicorn 추가됨)
  2. Jetson 컨테이너 fake E2E pytest
  3. HIL: ODrive USB → ODrive CAN → AK CAN (한 번에 하나씩, 사용자가 연결). `--track usb|can`.

**HIL 시 우선 확인 (리뷰 지적):**
- USB: connect/sample 실측, set_mode→closed_loop jump 방지, 게인변경 plot 반영, save_nvm.
- CAN: heartbeat/encoder cyclic 디코드 (encoder 는 RTR 안 함 → ODrive `encoder_rate_ms` cyclic 설정 의존), RTR(iq/temp/busVI).
- **CAN ODrive+AK 동시**: `_decode_odrive` 드레인 루프와 `AK.poll()` 이 같은 socket 에서 각각 `bus.recv()` → 서로 상대 프레임을 버려 텔레메트리 갱신율 저하 가능. 동시 연결 가능 시점에 단일 통합 recv 루프로 리팩터 검토. (현재 사용자 하드웨어는 한 번에 하나만 연결 가능 → Fake 가 동시 케이스 커버)

**Deferred (merge-blocking 아님):** server.py `@app.on_event` → lifespan 마이그레이션, estop ack target 표기(첫 device 만 — 실제 estop 은 전 device 적용), TRAP_TRAJ input mode·reboot 명령 미노출.

테스트·실행 환경: [[docker-dev-env-for-tests]] (x86 dev 컨테이너). 조향 hw 로직은 `motor_control/steering/ak_control.py` 의 `AK40` 클래스 (can_bus 가 `AK40 as AK` 재사용).

**⚠️ 2026-07-03 갱신:** '한 번에 하나만 연결 가능' 가정은 낡음 — 이제 **10모터 전부 단일 can0 동시**(구동 CAN 통일). AK-CAN(`--track ak`)·ODrive-CAN(`--track odrive_can`) 트랙은 HIL 후 main 병합 완료([[ak-can-gui]] [[odrive-can-gui]]) → GUI HIL(Task 11)은 사실상 진행됨(코너모듈 HIL에도 `--track usb/ak`로 활용). 젯슨 배포=`git pull`(레포 origin/main 싱크).

## notion-sw-template-required

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/notion-sw-template-required.md`

---
name: notion-sw-template-required
description: 노션 SW 페이지는 문서 성격 불문(계획·로드맵 포함) SW 표준 템플릿 구조·톤을 반영해서 쓸 것 — 지적받은 실수
metadata:
  node_type: memory
  type: feedback
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

노션 SW 문서를 만들 때 **문서 성격을 이유로 표준 템플릿을 건너뛰지 말 것.** 계획/로드맵이라도 「SW 문서 표준 템플릿」의 구조·번호·컨벤션을 반영해서 쓴다.

**Why:** 팀이 💻 Software 문서의 구조·톤 통일을 중시함(템플릿 페이지 자체가 "다른 문서와 구조·톤 맞추기 위한 표준"이라 명시). 내가 키네마틱스 페이지는 부분 반영(§1~5로 재번호·③배선/④설치/⑥트러블슈팅 누락), 자율주행 계획 페이지는 "이건 기능문서 아니니 템플릿 안 맞다"며 미반영으로 만들었다가 사용자에게 지적받고 둘 다 재작성함.

**How to apply:** 개요 콜아웃(blue)→목차→①환경/시스템구성 ②핵심개념/파라미터 ③배선(HW時) ④설치·사전준비 ⑤실행/사용법 ⑥트러블슈팅 표 ⑦검증결과 표 ⑧코드·참고. 안 쓰는 섹션은 삭제 후 순차 재번호(예: HW 없으면 ③배선 생략). 컨벤션: 콜아웃 색(파랑=개요/빨강=위험/회색=팁·함정)·⚠️함정 명시·현재값·풀코드는 레포 경로만. **계획/로드맵 문서도** 이 구조·톤에 맞춤 — 배선/설치/실행/트러블슈팅이 안 맞으면 그 자리에 '로드맵·블로커'를 넣되 환경·핵심개념·코드참고는 유지. 템플릿 스펙 원문은 프로젝트 `.claude/CLAUDE.md`의 "SW Notion 문서 표준" 참고. [[notion-team-hub]]

## notion-team-hub

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/notion-team-hub.md`

---
name: notion-team-hub
description: "팀 Notion \"극한로봇 파워트레인\" 허브와 코드베이스 대비 핵심 사실·불일치"
metadata:
  node_type: memory
  type: reference
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

팀 설계 허브 = Notion "극한로봇 파워트레인" 페이지(id 31d2d27b08d38030832ac73b42ce0c03, 동아리 ZEro To INfinite). Notion MCP 연결됨(search/fetch 등). 섹션: COMPETITION ROADMAP / MECHANICAL DRIVETRAIN / ELECTRICAL(PCB·BMS) / TECH STACK(SW·Firmware) / BOM / 회의자료. 국방·극한로봇 경연용 6륜 로커보기. 사용자=김광민(SW·Design/control).

**규칙: 기존 페이지 무단 수정 금지(예: VESC outdated 자동수정 하지 말 것). 단 새 문서 페이지 생성은 사용자 요청 시 OK.** 통합 권한 주의: 💻 Software 서브페이지(31d2d27b08d3808c87fed20d052fb9a0)는 create 시 404 → 메인 페이지(31d2d27b08d38030832ac73b42ce0c03) 아래에 생성해야 함. 신규 doc은 기존 SW문서 형식(callout·table_of_contents·table·details 코드·mermaid) 따름. 2026-05-25 생성: 코너모듈 HIL 페이지 36b2d27b08d381818b04c1d194bcade1 (팀 비전공자용 쉬운 수준, 결과위주, 버그기록 제외). **함정: notion-update-page(replace_content)는 new_str의 `\n`/`\"` 이스케이프를 NFM 이스케이프로 오인해 본문을 깨뜨림 → 실제 줄바꿈 사용 + 따옴표 속성(callout icon/color, table attr) 피하고 속성없는 `<table>`·`>` 인용구 사용. create-pages(content)는 `\n` 정상.** 문서 수준은 팀이 SW 비전공자 많으니 기존 SW문서 정도로 쉽게.

2026-05-26: 파라미터 v4 페이지 36b2d27b08d3819b9303d1f8554b0425 생성. **Github 페이지(35c2d27b08d380709e9ee06dc5b66664)=레포 README 복붙본이 stale였음** → README/CLAUDE.md 최신화(corner_module·motor_gui 누락분 추가, parameter_calc v4 15차원/7지형, AK45-36) 후 푸시(커밋 0ecfaa2, 10b2d21) + Notion Github 페이지 동기화. 이제 코드/README/CLAUDE/Notion 일치. (motor_gui·corner_module 둘 다 main에 있었으나 README/CLAUDE 미문서화였던 것 보강.)

2026-05-31: 중간 점검 양식 페이지(3702d27b08d380089d2dcedf635a7a28, "통합 회의 자료" 하위) SW 칸 기입 완료. **SW팀=팀장 김광민 + 팀원 선(US-100 담당) 단 2명.** 양식 규칙: 1)목표·3)리스크는 전팀 공통 섹션이라 SW 줄 앞에 `SW:` 프리픽스, 인원별은 본인 칸만(선 칸은 선 본인 작성), 팀별 표는 SW팀 표만. **검증된 안전 편집법(replace_content 깨짐 회피): update_content(content_updates 배열 old_str/new_str) 표적 치환 — 멀티라인 new_str에 실제 줄바꿈 + 리스트 탭 그대로 사용, `<mention-date .../>`·`<table ...>` 속성은 fetch본을 그대로 복사하면 따옴표 라운드트립 정상. old_str은 유니크하게(반복 플레이스홀더는 `#### 이름`·유니크 셀 포함해 매칭). 매치 실패=무변경(안전)이라 일괄 시도 후 재fetch 검증.** 원본 보존: 편집 전 로컬 백업 docs/2026-05-31-midcheck-notion-backup.md + Notion 페이지 히스토리.

2026-07-02: WP3 차체 통합 제어(ChassisManager) SW 페이지 생성 = **3912d27b08d381e79716e04398e34bd2**. **함정 확인: motor_control 서브페이지(0f80d999e59a4ebf897069d42ada2329, 코너모듈 페이지가 사는 곳)도 통합 권한 밖 404** → Software 서브페이지뿐 아니라 이것도 create 불가 → 결국 **메인 허브(31d2d27b…) 루트에 생성**해야 함(그래서 WP3 페이지는 motor_control 밑이 아니라 허브 루트에 형제로 붙음 — 원하면 UI에서 드래그 이동). create-pages(content NFM)로 callout/table/mermaid 정상 렌더, `\n` 정상. README 매핑표에 링크 추가·커밋 efc0b59 push. **표준 템플릿 = 「📄 SW 문서 표준 템플릿(복제해서 사용)」 id 3892d27b08d38111af86ff7eae30bdf3** — 신규 SW 문서는 반드시 이걸 먼저 fetch/복제해 §1환경→§2핵심개념→§3배선(HW시)→§4설치(ssh직후 cd~/power-train-sw→can_setup→docker up→exec)→§5실행(위치+✅기대출력)→§6트러블슈팅표→§7검증→§8코드 골격 맞출 것(안 쓰는 섹션 삭제, 풀 파이썬 금지=레포 경로만). 이번에 처음엔 템플릿 안 읽고 임의 구조로 써서 사용자 지적받고 §1~§7 골격으로 재작성함. **replace_content 재검증(2026-07-02): 인용 callout color/table header-row 속성 그대로 써도 안 깨짐**(create-pages식 content+실제 줄바꿈, 즉시 re-fetch 정상) → 예전 "replace_content 깨짐" 주의는 버전 개선된 듯(단 편집 후 항상 re-fetch 검증). [[corner-module-ackermann]]

코드베이스↔Notion 대조 핵심:
- 조향모터 = **AK45-36**(36:1, peak 24Nm/rated 8Nm, KV80, peak 65A, backlash 12′, back-drive 0.8Nm). 보유 테스트는 AK40-10(10:1). 2026-05-25 reconcile: ak_control.py `MOTOR_PROFILES`/`ACTIVE_MOTOR` 파라미터화(현재 AK40-10), CLAUDE.md 정정. 커밋 e46cdc6 (branch feature/corner-module). [[corner-module-ackermann]]
- 구동 드라이버 = **ODrive v3.6 가 맞음**. Notion 전장/BMS 문서의 VESC 75100 은 outdated(사용자 확인). 코드 변경 불필요.
- 구동모터 BL70200S: 인휠 3상BLDC, 바퀴~200mm(R 0.1m), 4.5kg/개×6=27kg, 48V, 정격~6.67A/320W·stall~12A/개, 내장 HALL×3(pp5 cpr30).
- 통신: 단일 CAN 버스 통합(Jetson↔구동ODrive↔조향 AK45-36 ×4), SN65HVD230 트랜시버, 데이지체인 120Ω 종단(끝=60Ω), 4WS 애커만. ODrive header↔AK A1257 커넥터 직결 불가→커스텀 케이블 필요 (**→ 2026-07 커스텀 케이블로 구동도 CAN 통일 완료: 단일 can0 500k에 10모터(AK×4 + ODrive×6)**).

**질량 정정:** ~86kg은 2026-05의 구 설계추정치다. parameter_calc v4 계산 질량은 **50kg 확정**이며 86kg 재최적화를 하지 않는다. 실차 질량·최대허용질량은 별도 기계사양이다. [[param-calc-v4-newtrack]]

## odrive-can-gui

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/odrive-can-gui.md`

---
name: odrive-can-gui
description: "motor_gui ODrive-CAN 트랙(--track odrive_can) — 컴포저블 OdriveCanDevice, 1Mbps 종단저항/TX 견고성, 소프트 영점, HIL 학습"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

motor_gui 3단계 ODrive-CAN GUI 완료 (2026-05-23, main 병합 `1192b25`).
spec/plan: `docs/specs/2026-05-23-odrive-can-gui-design.md`, `docs/plans/2026-05-23-odrive-can-gui-plan.md`.

**구조**: `transport/odrive_can_device.py` = `OdriveCanDevice(CanDevice)` (모놀리식 `can_bus.py`
ODrive CANSimple 로직 추출). `--track odrive_can` → `CanTransport([OdriveCanDevice()], track="can")`.
[[ak-can-gui]] 의 컴포저블 구조 그대로. 4단계(ODrive+AK 동시)는 `CanTransport([Odrive, Ak])` 한 줄.
node1=axis1, 3모드(position/position_traj/velocity), torque 모드 제외(CAN 으로
enable_current_mode_vel_limit 설정 불가 → runaway 위험).

**1Mbps 가 실전 필수 (AK+최종 10모터)이고 HIL 핵심 교훈:**
- **CAN 종단저항 120Ω ×2 필수.** 미흡 시 1M 비트에러 → tx-error 누적(rx=0) → ERROR-PASSIVE
  → bus-off, 텔레메트리 프리즈/명령 유실/ENOBUFS 잼. 진단: `ip -details link show can0` 의
  `berr-counter tx` 만 오름. 종단 정상화 후 tx-error **0**. (250kbps 는 비트타임 4× 여유라
  마진 배선에서도 동작 — 검증 폴백. baud 불일치여도 250k끼리/1M끼리면 OK.)
- **호스트 TX 최소화.** 100Hz×4 RTR(=400/s)이 1M 마진버스 bus-off 유발 → `request()` 를
  `_POLL_HZ`(15Hz)로 throttle(=54/s, 워커는 100Hz 유지 명령반응성), `_request()` 가
  CanError/ENOBUFS 흡수(텔레메트리 보호), `scripts/can_setup.sh` 에 **restart-ms 100**
  (bus-off 자동복구) + **txqueuelen 1000** 추가. wedged TX 큐는 restart-ms 로도 안 풀려
  `ip link set can0 down/up` 필요.
- **CAN `Set_Linear_Count`(0x019)는 절대엔코더(TLE5012B) zero 불가** (raw RTR 로도 확인).
  네이티브 영점 폐기 → **소프트 오프셋**(`_pos_offset`): set_origin 시 raw 를 offset 으로,
  sample 의 pos 에서 차감, set_input pos 엔 가산(`_send_input_pos` 헬퍼로 중앙화). 모놀리식 방식.
- **알려진 한계**: position_traj + 극저 vel_limit(1.0) + 높은 pos_gain(8.0) → 위치보정이
  하드캡(vl×1.3) 넘겨 overspeed(axis_err **0x200**) 트립. TRAP 정밀저속은 **vel_limit ≥ 3** 권장.

**ODrive 설정은 USB 필요** (CANSimple config-write 불가). 증명된 스크립트
`motor_control/drive/x2212_test/odrive_can_setup.py`(공장초기화+캘리+pre_calibrated,
기본 baud **250000**, node1, pp=7 cpr=16384) / `odrive_can_drive.py`(CAN 구동 검증).
1M 전환: USB odrivetool `odrv0.can.set_baud_rate(1000000)` + `odrv0.save_configuration()` →
Jetson can0 도 `bitrate 1000000` 재설정. (`can.config.baud_rate=` 속성쓰기는 미지원, 메서드 사용.)

기능: setpoint 오버레이(pos_cmd 로컬추적), Kt 편집 튜너블(torque_est=iq×Kt), 하드웨어 재연결,
TRAP 헤드룸. 프론트 무변경(capability-driven). HIL/배포는 [[jetson-deploy-can]] 참고.
관련 [[motor-gui-build]] [[docker-dev-env-for-tests]].

## param-calc-v4-newtrack

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/param-calc-v4-newtrack.md`

---
name: param-calc-v4-newtrack
description: "v4 옵티마이저 권위본 = parameter_calc/python_gpu_triangle (2026-06-04 평탄화로 new_parameter_calc 중첩 제거), 면-기준 물리 수정·envelope 캐시·test_v4 재작성 완료"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

**2026-06-04 구조 평탄화 완료**: 한때 `parameter_calc/new_parameter_calc/parameter_calc/`에
3중 중첩돼 있던 v4 완전본을 `parameter_calc/` 직하로 승격하고 `new_parameter_calc/`(레포 전체 stale 사본)는 제거.
이제 **v4 준정적 GPU 옵티마이저 권위본 = `parameter_calc/python_gpu_triangle/`**(functions/ + validate/cross_validate/analyze/plot 도구 포함),
최종 결과 = `parameter_calc/python/zetin_optimal_params_v4.pkl`, 초기 v4(f 0.2624) = `parameter_calc/archive/`.
(과거 맥락: 5/18 서버 스냅샷으로 들어왔고 5/25 "기존 calc와 별개 트랙" 지시로 독립 진화했었음 — 지금은 단일 트랙으로 통합.)

**원래(메인) vs new 차이**: 메인은 v4 "Phase 0"(`functions/` 없이 `../python_gpu` v3함수 import,
20요소 p_arr=brk_v무시, p0 30kg/D6374 5:1/tau4.95/TAU_REF1.85/5키W/4지형). new는
"Phase 1+2+2b+3+"(로컬 functions fork, brk_v 운동학 배선, BL70200 50kg, 사다리꼴속도+모터토크속도곡선,
슬립, RMS/배터리/stuck, 적응샘플링, patch_width, 7지형 + validate_mujoco/cross_validate/analyze/design_review/plot 도구).

**이번에 적용한 수정(검증됨, CPU dev 컨테이너 `powertrain_dev`에서 jax[cpu]+scipy 임시설치로 실행)**:
- **면-기준(측면-절반) 물리 일관화** `functions/calc_dynamics_jax.py`: `W_side=0.5W`, `mass_side=0.5mass` 도입.
  법선력·수직관성·stuck demand를 절반으로. F_drv는 `0.5W`사용+`/2`제거로 **수치 불변**, 토크/에너지/전류/stuck도 불변.
  **slip만 ~2× 보수적으로 정정**(종전 N에 전체W 써서 2× 낙관이던 버그). → 기존 pkl 무효, 재실행 필요.
- **envelope/지형 캐시** `ZETIN_JointOptSearch_v4_gpu.py`: 지형·envelope가 상수(R_w/patch_width/obs_h)에만
  의존 → `TERRAIN_CACHE` 1회 계산, objective 매호출 재계산 제거(큰 가속, 결과 동일).
- **test_v4.py 전면 재작성**: 옛 버전은 v3함수 import+14차원+30kg상수+assert 0개라 무효였음.
  이제 로컬 v4함수·15차원·실제 assert 48개. 핵심 가드 = 평지 `ΣN ≈ 0.5·mass·g`(=245.2N, 면-기준 검증·무게회귀 검출).
- 도구 버그: validate_mujoco(MU_TERRAIN incline 2종 추가, R_w/tau_clip을 p_opt에서 주입, pkl 존재확인),
  cross_validate(N_PTS 160→100 일치, 조용한 except에 traceback), analyze(MonteCarlo None-역참조 수정, N_PTS env),
  design_review(서브프로세스 반환코드 집계·경고), plot_diagnostics(xs≥xe 가드), plot_geometry(title=pkl version).

**남은 알려진 한계(미수정, 문서화만)**: validate_mujoco의 측면-절반 질량 부기(6휠 빼고 3휠 배치 → MuJoCo 총질량≠면-기준)와
incline 초기 자세는 secondary 검증기라 미손질(mujoco 미설치). 준정적 모델 한계는 v5(시간영역 ODE) 설계로 이관
(`docs/specs/2026-05-18-v5-architecture-design.md`).

**2026-05-25 풀 재실행 완료**(HPC SLURM 654551, A10 GPU, 11.7h/701.7분, DE 955스텝·430,200평가):
`parameter_calc/python/zetin_optimal_params_v4.pkl` = **f_opt 0.2004**,
결과에 **brk_v=355.8mm(피벗→축 수직)** 브라켓 항 포함(= main 레포 CPU `python/functions/wpos.py`엔 없음, GPU triangle 트랙에만 배선됨 — 전에 "브라켓=0" 답한 것의 정정값). 콘솔로그 HPC
`/home1/zetin348/Defence_Robot/logs/run_v4_rev_654551.out`(180MB, .err 0바이트=클린런), 결과정리 `docs/reports/2026-06-01-param-v4-result-log.md`.
(이전 스모크가 pkl 덮어썼던 문제는 이 풀런으로 해소.) HPC 접속: `ssh gate1_Internal`(zetin348, Oracle→RPi 게이트웨이 경유 ssh config).

2026-06-01: v4 수정본 **알맹이만**(`new_parameter_calc/parameter_calc/` 77파일 = v4 코드 + 결과 pkl + 4지형 mp4 + 진단 figs + v3 참조) GitHub **main 머지 완료** — 브랜치 param-calc-v4-revised(f25a66d) 푸시 후 origin/main 위로 리베이스해 푸시(**main 커밋 3758856**, 선 US-100 Task5~6 `3519bd5` 위). feature 브랜치는 머지 후 잔존(로컬·원격). 중복 최상위 사본(motor_control/docker/scripts/README/.claude)은 그때 제외·미추적 유지했다가 **2026-06-04 평탄화에서 완전 제거**(zetin_v4_viewer_opt.html만 parameter_calc/로 살림). origin=github.com/lightminn/power-train-sw.git.
관련 [[docker-dev-env-for-tests]].

## patent-whitepaper

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/patent-whitepaper.md`

---
name: patent-whitepaper
description: 파워트레인 SW 전체를 특허 출원하려는 사용자 의도 + 기술차별성 백서(작성됨)와 채택된 프레이밍 전략
metadata:
  node_type: memory
  type: project
  originSessionId: 4e7045b0-96e3-4724-8716-c83cfe8a50d8
---

사용자는 자신이 만든 파워트레인 관련 SW **전체를 종합해 특허 출원**할 계획이다(2026-07-01 착수). 이를 위해 `docs/patent/2026-07-01-파워트레인-SW-기술차별성-백서.md` 를 작성했다.

산출물(전부 `docs/patent/`, `.git/info/exclude`로 git 제외): 전체본 `2026-07-01-파워트레인-SW-기술차별성-백서.{md,pdf}`(pandoc→Typst, `main.typ` 스타일·`arch.dot`/`arch.png` 다이어그램·`shape_param.png` 형상도면, 재생성 `build_pdf.sh`) + **3p 비주얼 요약본** `2026-07-01-파워트레인-SW-요약본.{typ,pdf}`(hand-written Typst, 그림·표 위주; "교수님" 라벨 뺌). 스타일=섹션 밴드+서브섹션 액센트바+남색헤더 표+zebra, 표 셀 justify OFF(단어간격 방지), 줄간격 넉넉.

채택된 결정(사용자 승인):
- **용도** = 기술 차별성 백서(핵심기술 요약·종래기술 대비표·차별화 포인트·정량 검증). 변리사 인계 겸용.
- **특허 전략** = 통합 시스템 1건(umbrella) + 요소 후보 랭킹.
- **형식** = Markdown 먼저(확정 후 Typst PDF 변환 가능).
- **프레이밍 톤** = 균형: 강점 부각 + 개별 메커니즘이 prior art면 솔직히 경계, 특허성 상/중 명시, §8 별도 한계 공개.

핵심 전략 결론(5개 서브에이전트 코드분석 공통): **개별 메커니즘 다수는 종래기술 → 신규성은 시스템 통합·측정된 비자명 조합에 있다. 청구는 조합/시스템 중심으로.** 요소 강도: E1 이종 단일-CAN 8모터 ≈ E2 안전내장 코너모듈 ≈ E3 정렬없는 3D역투영+분리채널 > E4 위상×치수 동시최적화(50kg/quasi-static 캐비엇) > E5 publish-only 안전판정(통합 안전으로서 강함). `0xFF` UART 우회는 영업비밀 권고.

**E4 형상 최적화 = MuJoCo 별도 `sim/` 트랙(2026-07-01 통합 완료):** 옛 parameter_calc JAX 준정적 v4(50kg·f_opt 0.2004)는 **대체됨**. 재최적화 실체 = 팀 제공 `docs/patent/sim-public-staging.zip`(MuJoCo 동역학, 12파라미터=자유10+종속2 평지동시접지로 유도, 8지형 DEM, COT 목적함수 0.40·COT+0.25·시간+0.25·미완주+0.10·자세, 2단계 prescreen48→DE popsize24×maxiter40, terrain-random·hold-out, 실측 질량예산 **≈62.7kg**, 구동계=데이터시트 입력 BL70200 39/22Nm·AK45 24/8Nm·30A) + 별도 백서 `docs/patent/ZETIN_형상최적화_실용성_특허성_백서.md`. 핵심 신규성=**3구조관계**(①평지 동시접지 종속유도[모터오프셋 상쇄]·②차동 1자유도 폐루프·③조향모터 몸통중앙 접합 91.3mm) + **「구동계=교체가능 입력」** 범용성 → E4는 유일하게 물리장치라 **장치+방법 독립항** 가능(상). 형상 도면 `shape_param.png`(=Shape_Param_view.png) 삽입. 통합 백서 §1·§2.1·§3·§4.4(전면)·§5·§6·§7·§8·§9 전부 이 내용으로 갱신·PDF 재생성 완료. (86kg=옛 설계추정 vs 62.7kg=sim 부품실측 예산; 형상 상대우열은 총질량에 둔감.)

**E6 기구(CAD) 통합(2026-07-01):** 팀 제공 `docs/patent/rover_cad_spec.tex`(특허 명세서 초안, 2 실시예 — ①인휠 모터 브라켓: 브라켓↔액추에이터 사이 베어링=축/모멘트 하중·커플링=토크만 → 조향 액추에이터 하중 격리, 일체형 U-채널 절삭+커버+하단 필렛으로 FEA 최대응력 ≈21%↓ ②로커보기 차동 바: 기어 없이 로드엔드 베어링만으로 좌우 로커 각도 평균화, 백래시 0·이물질 내성·차체 상부 배치)를 메인 백서 §4.6 새 요소 **E6(물리 장치 특허, 상)** 으로 통합. E6 차동 바 = E4 §4.4 '차동 1자유도 폐루프'의 실물 구현. 원리 개념도 `cad.dot`/`cad.png`(그림 3) + **실물 CAD 도면 2매 삽입**(그림 4 브라켓·그림 5 차동바) — 팀이 컴파일본 `docs/patent/rover_cad_spec (1).pdf`(6p) 제공, p5·6 도면을 `pdftoppm -r300` + `magick chop/trim`으로 `bracket_drawing.png`/`differential_drawing.png` 추출(정적 파일, build 재생성 안 됨). §1·범위·§2.2·§4.4·§5·§6·§7·§8·§9 갱신. 요약본도 6요소로 갱신. **요소 = E1~E5(SW) + E6(기구), E4·E6=물리 장치라 단독 청구 강함.** 청구범위는 변리사 영역.

관련: [[corner-module-ackermann]] [[can-bus-multidevice-topology]] [[us100-safety-module]] [[param-calc-v4-newtrack]] [[jetson-realsense-d435i]]

## robot-arm-team-resources

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/robot-arm-team-resources.md`

---
name: robot-arm-team-resources
description: "로봇팔 팀(꼬물이/뚱이) 자료 위치 + 2026-07-02 재분석 — 단일 젯슨·ROS2 확정, 커스텀 msg 5종(PR"
metadata:
  node_type: memory
  type: reference
  originSessionId: f8c0afd4-b8a3-489d-9b90-4889d0b4350b
---

극한/국방 대회 **로봇팔 파트는 별도 팀**(팀명 "뚱이의 세계정복", 로봇팔 태명 꼬물이→뚱이; 파워트레인=부릉이). 자료(읽기 참조, 수정 금지): GitHub `ksp118/extreme-robot`, Notion 허브 `3312d27b08d380868cc5c6bd37dca69a`.

**2026-07-02 재분석 (스코프 변경 — 이제 연동이 우리 액션아이템):**
- **단일 젯슨 확정**(그들 6/25 결정): 두 팀 SW가 젯슨 1대 + **ROS2 Humble 토픽**으로 통신. 인식(YOLO)은 로봇팔 팀 전담(우리 `yolo_depth_3d.py` 로직을 그들이 이식해감, markerless seg 전환 — 대회규정 마커금지로 ArUco 폐기). 우리 `powertrain_jetson` 컨테이너는 통합 전 개발환경으로 정리됨.
- **커스텀 메시지 5종 `robot_arm_msgs`** (계약): `/detected_objects`(팔→우리, 30fps 전체 인식), `/arm_status`(팔→우리, DONE=재출발 트리거), `/arrival_status`(우리→팔, mission_id+status), `/chassis_mode`(우리→팔, DRIVING/CORNERING/ROUGH_TERRAIN/MISSION_STOP/FOLLOW_LEAD — 팔 자세락), `/pick_target`(팔 내부, latched). 핸드셰이크: MISSION_STOP→ArrivalStatus→팔 작업→DONE→재출발.
- **우리 몫 명시**(그들 통합계획 §6): Nav2 미사용·레인 추종, 4WS 키네마틱스(CornerModule 위), odometry(휠+IMU 단거리), 레인은 raw 센서 직접 처리, 정지선/신호등/마커는 100% `/detected_objects` 구독(단일 인식 소스 원칙). 우리 착수계획: `docs/plans/2026-07-02-autonomous-driving-kickoff.md`.
- ✅ **PR #11 main 머지 완료(2026-07-03, main HEAD `0ec4a0a`)** → `robot_arm_msgs`(5종)·`robot_arm_perception`(perception_node)·`arm_fsm`(12상태)·MoveIt-Dynamixel 브릿지·설계문서가 **main에 반영**(이전엔 열린 PR에만 있었음). 팔은 6/24 6축→**3축** 대격변(그리퍼 재검토 중).
- ✅ **D435i 독점 충돌 — 2026-07-07 해소**: 원래 그들 perception_node가 pyrealsense2로 카메라 직접 오픈(단일 프로세스만) → 우리 레인용 raw 영상 접근 불가였음. **재발행/통일 협상 대신 '센서 분리'로 종결** — D435i=로봇팔 전용(통째로), **L515가 우리 레인·depth·IMU 카메라**(우리 realsense-ros 드라이버 `l515_camera`). IMU도 **L515 내장(BMI085)** → D435i 내장 IMU 의존 없음. 상세=착수계획 센서배치 배너.
- 미결(그들이 "파워트레인 합의 대기"로 보류): status enum(대문자 스네이크 잠정), mission_id 관리 주체(우리 미션 시퀀서가 맞음), MISSION_STOP→ArrivalStatus 순서, ROS_DOMAIN_ID, 라이다(/scan, 후방 마운트) 역할 분담.
- **젯슨 7/2~3 역사 기록:** 당시 두 레포 main 싱크와 ROS 환경 존재 확인. 최신 7/10 실측은 상단 CURRENT STATE OVERRIDE와 `project-state-2026-07-10.md`를 따른다; WP4·WP5는 완료됐다.
- 그들 요구사항 문서 `docs/requirements/`는 gitignore(레포에 없음) — 스펙은 노션+직접 소통. 그들 CLAUDE.md도 stale(6축 서술 vs 실제 3축).
- 일정: **7/19 설계문서 확정 → 7/31 국방 서류 → 9/13 국방 본선 → 10/2 극한 본선.** 원격 점수 33~40% → 텔레옵 1급 유지.

분석 시점 2026-07-02. 상태 유동적(3축 리팩터 등) — 메시지 5종 계약에만 의존, 내부 구현 결합 금지. [[fsm-competition-track]] [[corner-module-ackermann]] [[jetson-realsense-d435i]]

## us100-safety-module

Preserved copy: `/home/light/.codex/claude-migration/memory/home-light-ZETIN-robotics-power-train-sw/us100-safety-module.md`

---
name: us100-safety-module
description: "US-100 충돌방지 안전 모듈(#3) 범위·설계와 인계 팀원이 초보라는 제약"
metadata:
  node_type: memory
  type: project
  originSessionId: 4ba9f147-fdf4-408e-8ea0-198769499294
---

motor_control/safety_us100/ (신규) = US-100 센서 1개로 앞쪽 장애물 거리를 재 safe/warn/stop 3단계 판정만 발행(publish-only, 모터 직접 안 멈춤). 소비측(텔레옵/코너모듈)이 판정 보고 조치. tick()/verdict() 폴링 모델(코너모듈과 동일). fail-safe=못 읽으면 stop, 연속 fail_stop_count(3)회 도달 시 stop. 히스테리시스로 채터 방지. Verdict={level, distance_mm}만. 기본 임계 warn 400mm/stop 200mm. us100_robust.py의 0xFF prefix 읽기 패턴 재사용. corner_module/motor_gui를 import 안 함(독립).

**이번 범위: 충돌방지만. cliff(추락방지)·다센서·모터직접정지는 제외(나중 과제).** US-100 자체 용도는 충돌방지+추락방지지만 이 구현은 충돌방지 단일.

**핵심 제약: 이 모듈은 python·하드웨어 아주 초보인 팀원에게 인계** → 스펙·계획 문서를 전문용어 풀어 쓰고 비유·쉬운 말로 작성했음(docs/specs·docs/plans 2026-05-25-us100-*). 이 트랙 관련 문서/코드 주석은 초보자 기준으로 계속 쉽게.

**2026-05-25 결정: 구현은 팀원이 담당 → 우리(이 세션/Claude)는 손 뗌, 코드 구현하지 않음.** 스펙·계획만 작성해 main에 머지 완료(초보자용 쉬운 말). 설계 docs/specs/2026-05-25-us100-safety-module-design.md, 계획 docs/plans/2026-05-25-us100-safety-module-plan.md. 관련: [[corner-module-ackermann]]

<!-- END CLAUDE_TO_CODEX_MEMORY -->

<!-- BEGIN CLAUDE_TO_CODEX_HISTORY -->
# Claude History Handoff

When starting a new Codex session in this project, read `.codex/claude-history/HISTORY.md` once before using prior project context. If the current task depends on historical decisions, commands, user preferences, bugs, or unfinished work, open the linked session Markdown files from that history index.
<!-- END CLAUDE_TO_CODEX_HISTORY -->

<!-- BEGIN CLAUDE_AI_EXPORT_AF79641E -->
# Claude.ai Export Handoff

At the start of a new Codex session in this project, read `.codex/claude-history/claude-ai-export-af79641e/HISTORY.md` once. Use its category indexes to locate the migrated Claude.ai conversations, and open full session files when relevant.
<!-- END CLAUDE_AI_EXPORT_AF79641E -->
