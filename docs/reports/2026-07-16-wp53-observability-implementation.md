# WP5.3 관측성 기반 구현·통합 보고 (2026-07-15 ~ 07-16)

정본 계획: `docs/plans/2026-07-13-observability-data-quality-remote-assist-plan.md`.
이 보고는 해당 계획의 소프트웨어 가용분(Task 1~5) 구현과 같은 기간의 팀원 기여
통합(PR #1·#2), 발견 결함, 배포 상태를 기록한다. 구현은 Codex 위임(스펙 → TDD 구현 →
검토자 3환경 검증) 파이프라인으로 진행했다.

## 1. 결과 요약

| 커밋 | 내용 | 검증 |
|---|---|---|
| `09b2fec`·`6cad9bb` | 팀원 PDIST80B 3종 스냅샷 + 정합화(rclpy 로거 수정·엔트리포인트·코덱 테스트 9) | dev 550 |
| `a16a5fe` | **Task 1** journal/health 순수 코어 (JSONL 세그먼트·회전·torn-tail 복구·시퀀스 단독 부여·비차단 drop counter) | 컨테이너 import smoke + 24 tests |
| `4c0885a` | **Task 2** observability 데몬 (`@powertrain-observability-events/status`, SO_PASSCRED+SCM_CREDENTIALS, flock 싱글턴, `/var/lib/powertrain/runs`) + TUI 독립 조회 + compose 서비스 | 호스트 실소켓 307 |
| `fd6aa09` | 팀원 **PR #1** — MOTION_HOLD 중 명령 폐기 + 해제 후 fresh command 요구(command_recovery) | 병합 후 chassis 253 |
| `12fe45d` | **Task 3** CAN 10노드 health matrix(수신 경로 카운터만·신규 I/O 없음) + wheel consistency(WARN/speed-cap 제안만) + CAN_HEALTH 이벤트 + TUI 매트릭스 | 호스트 310 · dev 541 · ros 297 |
| `a1977d0` | **PR #1 후속 결함 수정** — chassis_node mission hold가 interlock 직접 호출로 폐기 시맨틱 우회 → hold 이전 명령 재생(실측 0.8 m/s→1.27 rev/s). 매니저 래퍼 경유 + 아키텍처 계약 테스트 | ros 298 |
| `ce6368d` | **Task 5** 팔 결과 journal adapter (FAILED/GRIP_LOST/posture→ARM_RESULT, unknown→CONTRACT_VIOLATION, TUI raw status·mission ID·hold reason) | 호스트 332 · dev 607 · ros 304 |
| `8dd2b54`~`9aec6eb` | **Task 4** robust depth 품질(전체 ROI·명시적 reject)·sensor/TF qualification·commissioning CLI(YAML SHA-256 동결, repo YAML fail-closed)·`powertrain_autonomy` 이미지/서비스(profile 게이트·무발행) | 호스트 92 · dev 634 · **젯슨 이미지 내 27** |
| `484530a`~`ef68367` | 팀원 **PR #2** read-only operator console 병합 + 정합화(§3) | dev 642 · ros 303 |
| `1f21752` | PR #2 레이아웃 컨벤션 재배치(§3) | ros 303 · 검사기 PASS |

전부 main 푸시 + 젯슨 배포 완료. 젯슨 상태: `powertrain_ros`·`powertrain_control`·
`powertrain_observability`(신규)·`canwatchdog` 스택 healthy, `powertrain-sw:autonomy`
이미지 빌드·인이미지 검증 통과. observability 데몬 라이브 status 조회 확인
(`status: OK`, run_id 발급, drop 0).

## 2. 검증 방법 기준선 (재사용할 것)

- **3환경 패턴**: ① 호스트 파이썬(실 abstract socket·SO_PASSCRED — Codex 샌드박스는
  EPERM이라 반드시 검토자가 재실행) ② dev 컨테이너(python-can 포함 전체 회귀)
  ③ ros 컨테이너(colcon `/tmp` install-space 빌드 후 전체 스위트 — G1이 엔트리포인트 검증).
- Codex가 실행 못 하는 rclpy 통합시험은 반드시 ros 컨테이너에서 돌린다. Task 3에서
  미가드 신규 호출부가 레거시 SimpleNamespace 픽스처 테스트 5건을 깨뜨린 것을 이
  단계에서 잡았다(가드 계약: `test_legacy_simplenamespace_tick_fixture_stays_usable_without_arm_fields`).

## 3. 팀원 기여 통합

- **PR #1 (MOTION_HOLD 강화)**: 방향 승인. 단 mission hold(chassis_node)가 래퍼를
  우회해 **pre-hold 명령 재생**이 실측 재현됨 → `cm.set_motion_hold` 경유로 수정하고
  `cm._interlock.set_motion_hold` 금지를 계약 테스트로 고정(`a1977d0`).
- **PR #2 (operator console)**: read-only 경계 준수(카메라/CAN/모터 소유 없음, US-100
  UART 거부). 정합화 4건(`ef68367`): CAN 검사기 allowlist(RX 전용 + no-send 계약 테스트),
  setup.py 중복 엔트리 제거, gi-free `pipelines.py` 분리, rclpy 로거 결함 1건.
  레이아웃 재배치(`1f21752`): systemd/udev/tmpfiles → `scripts/systemd/`(기존
  gateway-tmpfiles.conf 포함 — docker/는 컨테이너 정의 전용), `docs/handoffs/` →
  `docs/reports/`, `chassis_telemetry` 콘솔스크립트 등록 + 유닛 ExecStart를
  install-space `ros2 run`으로.
- 젯슨의 동일 작업 구초안은 `~/operator-console-draft-backup-20260715` 백업 후 정리.

## 4. 발견 함정 (재발 방지 반영됨)

1. **rclpy 로거 %-스타일 포지셔널 인자 = TypeError** — 팀원 코드 2곳(pdist monitor,
   telemetry sender). f-string으로 수정 + 회귀 가드 테스트.
2. **dustynv/l4t 베이스의 pip 인덱스가 죽은 미러**(jetson.webredirect.org) — 파생 이미지
   pip install은 `--index-url https://pypi.org/simple` 필수. PyYAML·pytest는 베이스에 없음.
3. **compose 서비스명 생략 `up -d`** — `/run/powertrain` `create_host_path:false`와 결합해
   미프로비저닝 보드에서 fail-closed 실패(노션 문서들에 반영됨).

## 5. 크로스팀 상태

- 로봇팔 main에 PR #17 병합: **ipc:host 실증 적용**(0→37건, 양방향 51/43건) +
  `/arm_status` 10Hz heartbeat(`_do_idle pass` 수정). ⚠️ 그들 `ros2_humble` 컨테이너
  재기동 필요. 잔여 합의 3건(LOWER_RELEASE·접힘 근접·controller_fault)은 팔 젯슨
  워킹트리 미커밋 — Task 7 컷오버 전제.
- 팀원 Phase A 실기(노션 통신GUI §11): CAN **ERROR-ACTIVE·오류 0 회복 관측**
  (07-14 ERROR-PASSIVE는 일회성 잔재 가능성 높음 — 벤치 최종 확인만 남음),
  `/wheel_states` 50 Hz 안정, `/arm_status` publisher 부재로 arm HIL 보류(우회 안 함).

## 6. 남은 게이트

- **WP5.3 Task 6**: WP6-B/C 뒤. + PR #2 콘솔 CAN 표시의 observability 일원화(계획에 등재).
- **WP5.3 Task 7~8**: P0/P1 simulation·replay 뒤.
- **다음 개발**(계획 §9 순서 10~11): WP6-S P0(fixture/replay/MuJoCo fast) 및 **WP6-A
  wheel+IMU 상태 추정** → WP6-B(NumPy terrain, Task 4 산출물 소비).
- **HIL 벤치 목록**: WP5.2 실기 스모크(원격 E2E·WheelStopPredicate 실측·arm_absent_field),
  CAN ERROR-PASSIVE 최종 확인, L515 커미셔닝 브래킷(기구팀 인계), 문서 복붙 리허설 실기분.
