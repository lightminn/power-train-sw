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
| WP5.3 (관측성: journal/데몬/CAN health/depth 품질/팔 결과 adapter) | ✅ Task 1~5 배포 | Task 6는 WP6-B/C 뒤, 7~8은 sim 뒤 |
| **WheelStopPredicate 실측 자격화** | ✅ 오늘 HIL | `wheel_stop.yaml qualified: true`, 임계 0.10 rev/s |
| WP6-A (wheel+IMU 상태 추정 코어) | ✅ SW 완료 | `dfdfb32`. 실측 5 m ±5%·90°는 **차체 조립 후** |
| WP6-S P0 1부 (scenario 계약·analytic fixture·recorded replay) | ✅ | `9a5f37f`. production 추정기 합성 5 m 오차 0%, 피벗 yaw 0.0008% |
| **WP6-S P0 2부 (MuJoCo fast 브리지)** | 🔜 **다음 개발 항목** | 1부 산출물이 입력 계약 |
| WP6-B (NumPy terrain, Task 4 소비) | 대기 | P0 뒤 |
| L515 경량 파이프라인 | ✅ | 29.91 fps RGB SRT, raw depth 10 Hz |
| 원격운용 (teleop 유/무선 + operator_console) | ✅ | 콘솔은 팀원 PR #2 병합·정합화 |
| WP8 미션 시퀀서·풀 핸드셰이크 | 미착수 | `MISSION_STOP`·언락 순서 크로스팀 계약 포함 |

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
```

기준선(07-16): 호스트 332+92+65(sim 42+state_estimation 23) / dev 컨테이너 ~700 /
ros 컨테이너 ~310 / 젯슨 autonomy 이미지 27. **테스트 실행과 commit/push는 반드시 `&&`
체인**(0a89098에서 비체인 스크립트가 1 failed를 그대로 커밋한 사고 있음 — fe67096로 수습).

**젯슨 배포 절차**: `git pull --ff-only` →
`docker compose -f docker/docker-compose.jetson.yml up -d --force-recreate powertrain_ros powertrain_control`
(엔트리포인트 colcon 재빌드 ~5 s) → 90 s 대기 → 헬스 확인. 서비스명 생략 `up -d` 금지
(`/run/powertrain` fail-closed와 결합해 미프로비저닝 보드에서 실패).

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

1. **WP6-S P0 2부 — MuJoCo fast 브리지**: 절차 생성 고가 트랙·다리(bridge)·hidden seed.
   1부의 scenario/기록 계약을 소비. 스펙 초안 참고:
   세션 스크래치 `wp6s_p0a_spec.md` 패턴(정본은 마스터 계획 §WP6-S).
2. **D 런북 — WP5.2 원격 E2E 실기 스모크**: authority_enabled 경로 launch 플래그,
   wheel-stop 자격화 완료로 핸드오버 시험 가능해짐. FULL HIL 벤치 항목.
3. WP6-B(NumPy terrain, Task 4 산출물 소비) → WP5.3 Task 6(콘솔 CAN 표시 일원화 포함).
4. 기술 백로그: ARMED 유휴 ~530 ms 주기 스톨(원인 미상), health matrix 유휴 플래핑.
5. WP8·`MISSION_STOP`·언락 순서·풀 핸드셰이크 1사이클(크로스팀).

## 7. 크로스팀 상태 (로봇팔, `extreme-robot`)

- 그들 main에 PR #17 병합: **ipc:host 실증**(0→37건) + `/arm_status` 10 Hz heartbeat.
  ⚠️ 그들 `ros2_humble` 컨테이너 **재기동해야 ipc:host 실효** — 아직 안 됨.
- 잔여 합의 3건(LOWER_RELEASE·접힘 근접·controller_fault)은 팔 젯슨 워킹트리 미커밋 —
  WP5.2 Task 7 합동 HIL의 전제. + 접힘 캘리브레이션, DualSense 키매핑 v1a 확정 대기.
- 팔 젯슨 체크아웃(`~/extreme-robot`)은 **read-only** — 우리가 수정하지 않는다.

## 8. 미커밋·로컬 전용 항목 (분실 주의)

- `docs/defence_docs/초안.docx`(팀 원본)·`초안_v2.docx`(07-16 갱신본 — wheel-stop 자격화,
  안전 장기시험, WP6-A/S 반영) — **의도적 미커밋**(원본이 사용자 영역). 커밋 여부는 사용자 판단.
- 팀원 노트북에만 있는 미푸시 4종: `scripts/extract_geometry_from_cad_urdf.py`,
  `parameter_calc/python_gpu_triangle/export_chassis_geometry.py`,
  `docs/specs/2026-07-13-min-rev-speed-range.md` 등 — `kinematics.py` docstring이 첫
  파일을 provenance로 인용하는 재현성 갭. push 요청 필요(노션 배너 처리됨).

## 9. 반복 함정 모음 (누적)

1. dustynv/l4t 베이스 pip = 죽은 미러 → 파생 이미지 `--index-url https://pypi.org/simple` 필수.
2. rclpy 로거에 %-스타일 포지셔널 인자 = TypeError (3회 발견 — f-string으로).
3. SimpleNamespace 레거시 픽스처 계약 — chassis_node 신규 속성 접근은 getattr 가드
   (`test_legacy_simplenamespace_tick_fixture_stays_usable_without_arm_fields`).
4. zsh: 매치 없는 glob·`=word` 확장이 명령 전체를 죽임 — 스크립트는 bash로.
5. 바퀴 지령 <0.3 rev/s = HALL 코깅존, 실물 정지 + 그럴듯한 텔레메트리.
6. 캘리는 RAM-only — 전원 사이클마다 `can_calibrate_all.py`.
7. 노션 SW 문서는 표준 템플릿 + 「주제 — 부제」 제목 컨벤션 + 복붙 완주 가능해야 함;
   제목 변경 시 README 매핑표 동기화.
