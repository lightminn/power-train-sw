# 2026-07-17 벤치 HIL — D 런북 원격 E2E + 무동작 검증 (실결함 4건 발견·수정)

운용: **FULL HIL 모드** — 사용자는 전원·DualSense 조작·육안 확인만, 모든 명령은
에이전트가 SSH로 직접. 벤치: 실차체 미조립(모터 10개 벤치 배열), 로봇팔 미장착
(`arm_gate_mode:=arm_absent_field`), wheels-free. **트랙 정책(사용자 확정)**: 실전/모의
트랙은 앞으로도 준비하지 않음 — 트랙 의존 검증(TRACKING/완주)은 시뮬레이터
(`powertrain_sim` 폐루프)가 영구 정본, 실기는 트랙 불필요 항목만.

벤치 런치 정본(이번부터 authority 플래그 사용):

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=200.0 \
  arm_gate_mode:=arm_absent_field authority_enabled:=true
```

## 1. 결과 요약

| 항목 | 결과 |
|---|---|
| ① 유휴 플래핑 소멸(`3c1e098` 실증) | **PASS** — IDLE 10분, AK4+ODrive6 전 노드 fresh 안정, 전이 1회(최초 수신)뿐. 수정 전엔 유휴 상시 플래핑 |
| ② ~530 ms ARMED 유휴 스톨 재현 | **미재현** — 격리(도메인42 FAKE) 30분 90,000 tick 갭 0 + 라이브(실 CAN·US-100·도메인0·동거 컨테이너) ARMED 15분 갭 0. 07-16 관측은 세션 특이 요인으로 강등(감시만) |
| ③ D 런북 원격 E2E | **완주** — 하단 상세. 실결함 4건 발견 |
| ④(a) terrain fail-closed 스모크 | **PASS** — 실물 L515 depth(7.4 Hz)→estimator가 벤치 장면을 `no_connected_support`로 정직 기각 + 팔 부재 `BLOCKED\|gate_missing` 이중 fail-closed, `/autonomy/cmd_vel` 0 발행. ④(b) TRACKING은 시뮬 정본으로 대체(트랙 정책) |
| ⑥ 실물 depth 자산 1호 | 벤치 장면 45 s bag(79 MB, depth+IMU) — 젯슨 `~/wp53_soak/l515_bench_scene`. repo 회귀 통합은 bag→TerrainFrame 재생 어댑터 후속 |
| ⑦ 뷰어 실영상 | 보류 — 게이트웨이 SRT 미기동(팀원 aligned-depth WIP 상태 보존 우선) |
| ⑧ fault matrix 실 kill | 부분 대체 — 원격 클라이언트 사망→서버 생존→재접속 수락을 실기 확인(RST 수정 실증). 나머지 채널 kill은 다음 벤치 |

## 2. ③ D 런북 상세 — 실증된 시퀀스

- 6축 풀캘리 6/6 무오류 → ARMED/RUN.
- **프로토콜 v2 E2E**: DualSense→노트북 클라이언트(v2, R1=ASSIST_BYPASS)→게이트웨이
  수락(위반 0)→`/teleop/cmd_vel` 30.2 Hz(최대 갭 48 ms).
- **TELEOP 권한 인계**: 중립 확인 게이트 통과 → `TELEOP|teleop`. L1(데드맨)+RT로
  **바퀴 6개 실회전 육안 확인**.
- **주행 중 AUTONOMY 핸드오버**(피크 2.51 rev/s에서 발사):
  `+0.03 s zero commanded → wheel stop pending → +0.78 s wheel stop confirmed
  (qualified 0.10 rev/s·dwell 300 ms) → +0.81 s AUTONOMY 중립 확인 — 권한 인계`.
  운전자 입력(RT 유지) 무시하고 시스템 강제 0 → 실물 정지 육안 확인 후 전환.
- **R1→`/teleop/assist_bypass`**: hold 중 True 100%(82/82, ~20 Hz).
- **○ E-stop 엣지**: 게이트웨이 MOTION_HOLD → `clear_hold`→중립 재확인→DRIVE 복귀.
- **부수 재검증**: 벤치 활동 중 US-100 latched E-stop 실발동(200 mm 이내 접근) →
  `reset_estop→IDLE(명시)→별도 arm` 의미론 실기 확인. 신규 `authority_enabled`
  launch 플래그(`6f64b98`) 실사용.

## 3. 실결함 4건 (전부 이 경로의 첫 실물 시험에서)

| # | 결함 | 수정 |
|---|---|---|
| 1 | **스틱 데드존 부재** — 실측 휴지 드리프트(left_x −0.0118, right_y +0.0431)가 게이트웨이 "정확히 0.0" 중립 게이트를 영원히 통과 못 함 → DISCONNECTED 고착 | `2c30abc` — 클라이언트 0.08 데드존(게이트웨이 계약은 엄격 유지) |
| 2 | **클라이언트 RST 1회가 TCP 서버 영구 사살** — recv의 ConnectionResetError가 accept 루프까지 전파(실증: `ConnectionResetError(104)`) → 노드 재시작 전까지 원격 불능 | `2c30abc` — 연결별 OSError 격리 + RST 회귀 테스트. 실기 재확인: 클라이언트 사망 후 재접속 수락 |
| 3 | **TCP_NODELAY 부재(전 원격 엔드포인트)** — Nagle+지연 ACK가 30 Hz 소형 프레임을 Wi-Fi에서 ~11.5 Hz 버스트로 뭉쳐 게이트웨이 0.2 s·authority 0.3 s 신선도를 관통 → 주행 중 MOTION_HOLD | `6bb4029` — 클라이언트·서버·레거시 클라이언트 NODELAY. 실측 30.2 Hz·최대 갭 48 ms로 회복 |
| 4 | **autonomy 노드 executor 기아** — terrain 처리(젯슨 CPU ~100 ms+/frame)가 20 Hz 명령 tick과 단일 스레드 executor 공유 → depth 버스트 시 발행 공백 >0.3 s → AUTONOMY가 stale hold(fail-safe 방향은 정상 작동) | 수정 진행 중 — latest-only 슬롯+워커 스레드(계획 §WP6-B YOLO 패턴). 재발 방지 테스트 포함 예정 |

교훈: 합성 프레임 단위시험은 축값 0.0·무결 TCP·즉시 처리 가정을 공유한다 — 실물
컨트롤러/Wi-Fi/젯슨 CPU가 그 가정 셋을 전부 깼다. 이 경로의 결함 4건 전부 소프트웨어
수정으로 종결 가능했고 안전 방향(fail-closed)은 한 번도 뚫리지 않았다.

## 4. 정리 상태

disarm→IDLE 후 HIL 프로세스 전부 종료(잔여 0 — 좀비 없음), 표준 컨테이너 5개 healthy,
팀원 WIP(l515 gateway·urdf·untracked 스크립트) 무접촉 보존. 아침에 발견한 팀원의
전일 wp5 스택(can0 락 점유 좀비)은 SIGINT 정상 종료로 정리했음.
