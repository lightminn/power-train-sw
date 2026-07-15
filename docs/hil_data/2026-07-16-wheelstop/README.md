# 2026-07-16 wheels-up wheel-stop HIL bags

`wheel_stop.yaml` 자격화(commit 0a89098)의 원천 데이터. ros2 bag (sqlite3).

- `wheelstop_run1/` — 전진 0.6 m/s × 5 정지 사이클 (44 s, /wheel_states·/odom·/imu/filtered·/safety_verdict·/chassis_state)
- `wheelstop_run4/` — 후진 0.6 m/s × 3 + 피벗 ±0.8 rad/s × 각 2 (70 s, /wheel_states·/chassis_state·/safety_verdict)

조건: Jetson 실기, 바퀴 6개 리프트(무부하), 로봇팔 미장착(arm_absent_field),
stop_mm 200, 캘리 6/6 직후. 분석 수치는 wheel_stop.yaml 주석과
docs/reports/2026-07-16-wp53-observability-implementation.md 참조.
run4는 arm-blocking 래치 수정(a191116) 이후 기록 — 전 구간 ARMED 유지.
