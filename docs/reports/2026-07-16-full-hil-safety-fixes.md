# 2026-07-16 첫 FULL HIL 세션 — wheel-stop 자격화 + 안전 결함 2건 근본수정

운용 방식: **FULL HIL 모드** — 사용자는 물리 조작(전원·바퀴 리프트·육안 확인)만 하고,
모든 명령(SSH·컨테이너·launch·서비스 호출·bag 기록)은 에이전트가 직접 수행했다.
벤치 조건: 실차체 미조립(모터 10개를 벤치에 배열), 로봇팔 미장착 → 전 시나리오 wheels-up,
`arm_gate_mode:=arm_absent_field`. 지상 주행류(오도메트리 5 m/90°, `stop_mm` 커미셔닝)는
차체 조립 후로 이월.

벤치 런치 정본(안전 게이트 전부 유지):

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=200.0 arm_gate_mode:=arm_absent_field
```

서비스는 `~/arm` `~/disarm` `~/estop` `~/reset_estop`(`~/reset` 아님) + `~/arm_lock_override`.
⚠️ ros2 CLI 데몬이 컨테이너 재기동 직후 불안정(`rcl node's context is invalid`) —
서비스 호출은 rclpy 직접 클라이언트 스크립트(docker exec heredoc)로 몰았다.

## 1. 결과 요약

| 항목 | 결과 | 커밋 |
|---|---|---|
| A. CAN ERROR-PASSIVE 재확인 | **종결** — 유휴·캘리 6/6·폐루프·주행 전 구간 오류 0. 07-14 관측은 일회성 확정 | (기록만) |
| B. wheel-stop 임계 자격화 | **`qualified: true` 동결** — 0.10 rev/s·dwell 300 ms, 실측 5695 표본, 육안 확인 | `0a89098` `fe67096` `dc7ebc8` |
| C. 거짓 safety-stale 래치 (블로킹 서비스) | **근본수정 + 회귀 테스트** | `a191116` `149302e` |
| D. US-100 발행-UART 결합 | **근본수정** (리더 스레드 분리) | `09cb606` |
| 최종 재검증 | 15분 ARMED 소크 무래치 + arm/disarm 3사이클 무래치 | — |

## 2. B — wheel-stop 자격화 (WP5.2 `WheelStopPredicate` 실측치)

- 절차: 전진 0.6 m/s ×5 정지 사이클(run1) + 후진 0.6 m/s ×3 + 피벗 ±0.8 rad/s ×2/방향(run4),
  각 사이클 `/wheel_states` bag 기록. **바퀴 회전 5회 버스트는 사용자 육안 확인 완료**
  (HIL 통과 조건 — 텔레메트리만으로 판정 금지 원칙).
- 정지 노이즈 |v| 전바퀴 최대 **0.047 rev/s**(p99 ≤0.021) → 임계 **0.10 rev/s** = 최악
  노이즈의 2.1배, 최저 구동 플로어 1.0 rev/s의 1/10. 정지 감쇠(>0.5→<0.1 rev/s)는
  wheels-up 관성 포함 1.6~2.9 s(0.5 s 명령 워치독 포함).
- 산출: `ros2/src/powertrain_ros/config/wheel_stop.yaml`(provenance 주석 포함),
  원천 bag `docs/hil_data/2026-07-16-wheelstop/`, 자격화 상태를 고정하는 테스트
  `test_wheel_stop.py::test_shipped_yaml_is_hil_qualified_with_sane_thresholds`.
- ⚠️ 지상 커미셔닝 때 정지 과도(transient)만 재확인(부하 영향); 정지 노이즈는 유사 예상.

## 3. C — 블로킹 서비스發 거짓 `safety_topic_stale` 래치

**증상**: arm 직후 또는 유휴 중 간헐적으로 latched ESTOP. US-100 거리 정상(2399 mm),
클럭 점프 0, `/wheel_states` 49.7 Hz 정상, 외부 verdict 5 Hz 무결점이라 수 시간 유령.

**근본 원인**: `~/arm`(~0.8 s: 코너 6개 폐루프 진입+피드백 대기)과 `~/disarm`(~1.3 s) 등
블로킹 Trigger 서비스가 단일 스레드 executor를 점유 → 그동안 `/safety_verdict` 콜백이
큐잉 → 서비스 종료 후 첫 50 Hz tick이 `age > 750 ms`(실측 783 ms/1264 ms)로 오판 →
거짓 latch. 젯슨 워크트리 디버그 계측으로 arm 직후 수신 갭 800 ms를 직접 관측해 확정.

**수정**: `chassis_node._refresh_safety_baseline()` 헬퍼 — 블로킹 서비스 4종
(`_srv_arm`/`_srv_disarm`/`_srv_estop`/`_srv_reset_estop`) 완료 직후 `_last_safety_ms`를
재기준화(단 최초 verdict 미수신 `None`이면 그대로 — startup 게이트 유지). arm만 고친
`a191116` 이후 15분 소크 **종료 시점 disarm에서 같은 클래스 재발**(age 1264 ms)을 잡아
`149302e`에서 4종 전부로 일반화. 회귀 테스트:
`test_chassis_arm_gate.py::test_srv_{arm,disarm}_refreshes_safety_freshness_baseline`.

**진단 공백도 함께 해소**: stale 래치 전이에 ERROR 로그(`_safety_stale_active` 에지),
verdict 수신 갭 >500 ms에 WARN 신설 — 이번 유령 진단이 어려웠던 이유(래치가 무로그)의
재발 방지. 07-15 배포한 관측성 데몬이 이 진단에서 첫 실전 원샷 기여.

## 4. D — US-100 발행이 블로킹 UART에 결합

**증상**: 유휴 12분 중 2회, `/safety_verdict` 발행이 최대 1.17 s 정지(5 Hz 기대).
센서 딸꾹질 1회(Jetson THS1 TX 떨림 계열)가 곧바로 안전 하트비트 정지로 전파.

**수정**(`09cb606`): us100_safety_node에 노드 소유 리더 스레드 신설 — 스레드가
`monitor.tick()`(블로킹 UART)을 돌고, ROS 타이머는 록 보호된 최신 스냅샷만 sample_hz로
발행. 리더 스레드 사망 → fail-safe verdict 발행. `us100.py`에 `write_timeout=0.1` 추가,
`SerialTimeoutException`은 기존 serial_error 경로로 합류. 종료 시 join-then-close.

## 5. 최종 재검증 (수정 후)

- **15분 ARMED 소크**: stale 래치 0, 전 구간 verdict 5 Hz 유지.
- **arm↔disarm 3사이클**: 전이 시퀀스 `IDLE→ARMED→IDLE→ARMED→IDLE→ARMED→IDLE`,
  래치 grep 0 (수정 전엔 매 disarm마다 래치).

## 6. 신규 백로그 (이번 세션에서 관측, 미해결)

1. ARMED 유휴 중 **~530 ms 주기성(~2분 간격) executor 스톨** 3회 관측 — 750 ms 임계
   미만이라 무해했지만 원인 미상. 소크 로그 재발 시 우선 조사.
2. WP5.3 Task 3 health matrix가 **유휴 시 거짓 stale 플래핑** — 드라이버가 구동 중일 때만
   소켓을 drain하는 구조 탓. 주행 중은 정상. Task 6(콘솔 CAN 표시 일원화) 때 함께 정리.

## 7. 정리 상태

벤치 원상복구: launch 프로세스 종료, `hil_bags/` 제거(컨테이너 root 소유라
`docker exec ... rm` 필요), 젯슨 워크트리 clean @ `149302e`, 표준 컨테이너 스택
(powertrain_ros/control/observability/jetson + canwatchdog) healthy.
