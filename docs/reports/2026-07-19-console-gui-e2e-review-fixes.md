# 운용 콘솔 GUI 전 유즈케이스 E2E 리뷰·검증·수정 (2026-07-19)

사용자 지시: "우리 gui 모든 유즈케이스에 대해서 실제 e2e 코드리뷰 및 검증 하고
버그고쳐라." 대상 = `operator_console/` GTK 콘솔과 그 젯슨측 상대 전부
(SRT :5000/:5002, UDP :5003/:5004/:5005/:5007, ops TCP :9001, 원클릭 브링업).

## 1. 방법

- 3방향 적대 리뷰(관측 채널 와이어 계약 / ops 명령 경로 / 영상·수명주기) +
  리뷰어 직접 확증(핵심 결함은 실재현: 독성 패킷 스레드 사망, 유휴 10초
  절단 t=10.0s 실측, CAN 문자열 대조, estop 커밋 이력 추적).
- 실 E2E: 노트북 Xvfb 실콘솔 ↔ 젯슨 실스택(브링업 `jetson_gui_up.sh` 종료 0).
  수정 전 기준선 → 수정 → 동일 경로 재검증. ops 명령은 안전 액션만 실행
  (estop 멱등·reset·US-100 토글·not_idle 거부 — 모터 회전 액션 제외, FULL HIL).
- 구현은 Codex 2갈래 위임(콘솔측 F1–F10 / 브로커·스크립트측 F11–F18).
  브로커 갈래는 RED→GREEN 완주, 콘솔 갈래는 F7–F10 미착수 상태로 중도
  종료되어 **리뷰어가 직접 마무리**(F7–F10 + 리뷰 A#2/A#6, TDD 동일).

## 2. 수정된 결함 (20건)

### 비상정지·명령 경로 (안전)
| # | 심각도 | 결함 | 수정 |
|---|---|---|---|
| F1 | critical | **비상정지 즉시발동 회귀** — 승인 스펙 KGUI D2(무확인 즉시)를 a5bfc79가 구현했으나 3e4050c(R06)가 스펙을 모른 채 2단 확인+state/revision 전제로 되돌리고 테스트로 고정 | `_on_immediate_clicked` 직접 submit 복원, 고정 테스트 반대 방향 교체 |
| F2 | high | **ops 링크 사망 미감지** — 하부 클라이언트가 소켓 사망을 삼켜 상태 냉동·명령 무통보 증발(2초 후 조용히 드롭) | ConsoleOpsClient에 push 하트비트 기반 2.0s 생존 게이트(`PUSH_LIVENESS_TIMEOUT_S`) — 사망 시 inflight OUTCOME_UNKNOWN 보고+상태 UNKNOWN+재연결 |
| F12 | high | **유휴 콘솔 10초마다 강제 절단**(브로커 CLIENT_IDLE_TIMEOUT_S, 실측 t=10.0s) → ~10초마다 ~1초 비상 경로 사각 | 유휴 절단은 핸드셰이크 미완 연결만 + TCP keepalive(5/2/3)로 죽은 피어 정리 |
| F11 | critical | **`mission_clear_grip_lost` 서비스 타입 불일치**(계약 Trigger vs chassis SetBool) — 버튼이 영원히 실패+채널 10초 웻지 | 계약 `service_setbool` 등록, 콘솔 `needs_bool=True`(data=운영자 인가) |
| F16 | 게이트 | 위 유형 재발 방지 | 계약↔chassis_node 서비스 타입 정합 소스스캔 테스트 신설(음성 대조 RED 확인) |
| F13 | medium | 부재 서비스 1회 호출이 mutation 채널 10초 잠금 | never-ready는 3.0s abandon(`SERVICE_UNAVAILABLE_ABANDON_S`), was-ready만 10s |
| F14 | medium | estop이 단일 mutation 슬롯을 덮어써 busy 불변식 상실 + estop 재전송이 서비스콜 중복 발사 | pending 주문 dict 추적 + estop 멱등 캐시 조회(재전송=PENDING 재응답) |
| F15 | low | 브로커 재시작 시 revision 0 재시작 → 구 스냅샷 앨리어싱 | revision 무작위 시드 |
| F7 | low | 로봇팔 잠금 해제를 **걸 수만 있고 풀 수 없음** | `PanelAction.bool_value` + "로봇팔 잠금 해제 취소" 행(data=False), flow에 행 객체 전달 |

### 관측 채널
| # | 심각도 | 결함 | 수정 |
|---|---|---|---|
| F3 | critical | **팔 텔레메트리 수신 스레드 독살**(OverflowError/RecursionError가 except 밖 — 실재현: 독성 1패킷에 영구 사망) | per-packet `except Exception` 경계 + invalid 카운터(타 수신기와 동일 패턴) |
| A#2 | high | **합법 JointState(velocity 생략·NaN)가 :5007 전체 침묵**(모터 온도까지 소실) | 미러가 joints만 강등(None), 모터는 계속 송신 + NaN은 joints 단계에서 검증 |
| F4 | high | **CAN 미수신이 배너에 LIVE**(송신 "UNAVAILABLE · …" vs 수신 정확일치 "unavailable") | 대소문자 무관 prefix 판정(odom/drive/can 공통) |
| F5 | medium | 팔 링크 사망 시 요약 냉동 표시 + 상단 배너에 팔 부재 | `arm_panel_summary(수신 age)` + 배너에 `팔 {신선도}` 추가 |
| F6 | medium | 퇴화 bbox(폭/높이 0) 1개가 metadata 프레임 전체 기각 | 해당 detection만 skip |
| A#6 | medium | metadata sequence=header stamp — stamp 0 스택이면 오버레이 1프레임 후 동결 | 브리지 자체 단조 카운터를 `capture_sequence`로 전달 |

### 영상·브링업
| # | 심각도 | 결함 | 수정 |
|---|---|---|---|
| F9 | medium | srtsrc auto-reconnect가 리스너 사망을 ERROR 없이 삼켜 **웻지 스트림 영구 STALE** + 재시작 시 신선도 상태 잔존 | 5초 stale 워치독 강제 재시작(프레임 본 스트림 한정) + 재시작 시 상태 리셋 |
| F17 | medium | 브링업 마지막 안내가 도달 불가 IP(`hostname -I` 첫 토큰 = 유선 192.168.50.98) | `ip route get OPERATOR_HOST`의 src 우선 (실측: .106 정상 출력) |
| F18 | medium | `--operator-host`가 env 라인 부재 시 기록·재시작 생략하며 ✅ 보고 | 라인 추가/파일 생성 + 유닛 재시작 |
| F10 | 계측 | 영상 패널 상태 외부 관측 불가(E2E 단언 불가) | smoke probe에 `video_l515`/`video_d435` 추가 |

리뷰어 추가 수정: Codex가 estop 분기의 토큰 재검증(token/role mismatch)을
제거한 것 복원(비상 경로가 생략하는 전제는 rate/sequence/busy 뿐).

## 3. 검증 (수정 후, 전부 green)

- 호스트: operator_console 165 · 통합(autonomy/console/sim/tests/scripts) 482 ·
  브로커 순수+계약+정합+스크립트 87 · **runtime_smoke 단독 PASS**(31 ticks)
- dev 컨테이너: **1380 passed + 3 skip** / ros x86 컨테이너(colcon 격리): **659 passed**
- **실 E2E(젯슨 실스택 ↔ 실콘솔)**:
  - 유휴 25s 무절단(수정 전 t=10.0s 절단) + revision 무작위 시드 확인
  - 부재 서비스 3.1s 거부(수정 전 10s), estop 멱등 FINAL_SUCCESS 0.1s,
    reset fail-closed 거부(active us100), US-100 토글 왕복 push 반영,
    drive 토글 not_idle 거부
  - 콘솔 probe: telemetry/chassis/arm **LIVE** + **video_l515/video_d435 LIVE**
    (실 SRT 17.4/24.4 fps), traceback 0, 안전배너 `비상정지(ESTOP) ·
    liveness_timeout`(벤치 진실). metadata STALE = 팔 인식이
    `/detected_objects`를 낼 때만 송신되는 데이터 주도 채널로 정상
  - 브링업 재실행 종료 0 + 안내 `--host 192.168.8.106` 교정 확인

## 4. 젯슨 배포 상태 (주의)

젯슨 체크아웃은 팀원 브랜치(a177fe2)라 소스 트리는 무접촉. 수정 5파일
(ops_broker_core/node·ops_contract·arm_console_mirror·arm_console_bridge_node)은
**install 아티팩트에만 임시 반영**(`ros2/install/.../powertrain_ros/`, 백업
`/tmp/gui_fix/backup/`) + 컨테이너/브리지 재기동. ⚠️ **다음 colcon 재빌드가
팀원 브랜치 소스로 되돌린다** — main 머지/브랜치 동기화 시 정식 반영 필요.
`jetson_gui_up.sh`는 젯슨에서 untracked라 수정본 상주.

## 5. 이월(백로그, 수정 안 함)

- 콘솔이 `status_query`를 안 써 PENDING 후 브로커 재시작 시 결과 미조회
  (링크 사망 자체는 F2가 OUTCOME_UNKNOWN으로 커버)
- SourceSequenceGate 동일포트 고속 재시작 잠금(저확률), metadata 0.25s STALE
  기준 vs 이벤트 주도 송신의 표시 플랩(사양 재검토 대상)
- EventLog 100줄 홍수 시 안전 이력 밀림, 종료 시 최악 ~6s 조인,
  runtime_smoke가 클린 셧다운 경로 미검증(TERM 미전달)·stderr 미드레인,
  ops 클릭 상호작용(확인 스트립·홀드 타이밍)의 자동 게이트 부재(유닛만)
