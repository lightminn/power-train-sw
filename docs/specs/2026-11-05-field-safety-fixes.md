# 현장 통합 브랜치 안전 결함 수정 설계 (2026-11-05)

`deploy/jetson-field-20261028`(cd776a2)를 main에 병합하기 전 검토(Codex 지적, 코드로 직접 확인)에서
나온 다섯 결함을 우리 쪽에서 고친다(2026-11-05 사용자 결정). 작업 브랜치 `fix/field-safety-20261105`
(= `docs/final-audit-20261105` 8b023ba 위). 모든 수정은 실패하는 시험을 먼저 쓴다.

## 공통: 연결별 상태 회신 `G` 줄과 schema 3 (F1·F2가 함께 쓴다)

- 통합 입력 노드(`teleop_command_node`)의 연결 스레드(`_serve_client`)는 기존 `S <state> <v> <ω>` 줄을 보낼
  때마다 이어서 `G <input_epoch> <estop_ack_id>` 줄을 보낸다. 기존 `S` 파서
  (`scripts/recv_remote_operation.py` `parse_status_line`, 정확히 4토큰)는 `G` 줄을 무시한다.
  - `input_epoch`: 게이트웨이의 현재 입력 세대(정수, 노드 프로세스 수명 동안 단조 증가). ROS tick이
    `_status_lock` 아래에 써 두고 연결 스레드가 읽는다.
  - `estop_ack_id`: **이 연결 스레드가 이 연결에서 디코드한** E-stop 레코드의 `estop_id` 중 마지막 값(없으면 0).
    전역 캐시가 아니라 연결 스레드의 지역 변수라서 새 연결은 0에서 시작한다(이전 연결 확인 유출 없음).
- 레코드 schema 3 = schema 2 + `input_epoch`(정수 ≥ 0) + `estop_id`(정수, 0 = 없음, 아니면 1..2^63-1).
  디코더는 2와 3을 모두 받는다(2는 `input_epoch=None`, `estop_id=0`). schema 3에서 `estop_edge`가 참이면
  `estop_id ≠ 0`이어야 하고, 거짓이면 0이어야 한다(위반은 기존처럼 계약 위반).
- 노트북은 연결마다 schema 2로 시작하고, 그 연결에서 `G` 줄을 받은 뒤부터 schema 3을 보낸다 —
  젯슨이 옛 코드여도 새 노트북이 계약 위반을 일으키지 않는다.

## F1. 송신 정체 중 패드 E-stop 엣지 유실 (P1, b6ff0ea)

- 현상: `motor_control/laptop/controller_process.py` `_send()`는 `_outbox`가 남아 있으면 현재 샘플을
  인코딩하지 않는다. `ClientInput.estop_edge`는 O 버튼을 누른 한 샘플에만 참이므로
  (`remote_operation_client.py:192`), 정체 중에 누르면 어떤 레코드에도 실리지 않는다.
  `_awaiting_fresh_sample`로 건너뛰는 샘플, 소켓이 없을 때의 샘플도 같다.
- 수정(이벤트 ID + 연결별 수신 확인):
  - `tick()`에서 패드 샘플의 `estop_edge`가 참이면 새 E-stop 이벤트를 만든다:
    `_estop_pending_id = secrets.randbits(63) or 1`(누를 때마다 새 ID — 이전 이벤트는 새 이벤트에 흡수).
  - 새 레코드를 인코딩할 때 `_estop_pending_id`가 있고 **이 연결에서 그 ID를 아직 싣지 않았으면**
    `estop_edge=True, estop_id=_estop_pending_id`로 싣는다. 같은 이벤트는 연결당 한 번만 싣는다.
    schema 2로 보내는 동안(G 수신 전)은 `estop_edge=True`만 싣고 "ID 없이 실었음"으로 따로 표시한다.
    그 연결에서 `G`를 받아 schema 3으로 바뀌면, ID 없이만 실린 미확인 이벤트를 ID와 함께 한 번 더 싣는다
    (이미 HOLD라 다시 걸려도 같은 정지다) — 그래야 정상 통신에서 확인이 끝난다.
  - 이 연결의 `G` 줄 `estop_ack_id == _estop_pending_id`이면 이벤트를 지운다(정확 일치 — 다른 이벤트나
    이전 연결의 확인이 새 이벤트를 지우지 못한다).
  - 연결이 끊기면(`_close_socket`) 미확인 이벤트는 남기고 "이 연결에서 실었음" 표시만 지운다 → 다음 연결의
    첫 레코드에 다시 실린다(연결 직후 게이트웨이는 중립 확인보다 E-stop을 먼저 본다).
  - schema 2 서버(옛 젯슨)는 `G`를 보내지 않으므로 확인이 오지 않는다 — 그 경우 이벤트는 연결당 한 번 실리고,
    연결이 끊기면 다음 연결에 다시 실린다(옛 서버에서도 정체 중 유실은 막힌다).
  - 남는 위험: 서버가 처리했는데 확인만 유실되고 운용자가 HOLD를 해제한 뒤 노트북이 재연결하면 같은 E-stop이
    한 번 더 걸린다. 안전 방향의 불편이라 수용한다.
- 범위: 통합 운용 경로(`controller_process`). 옛 단독 송신기 `RemoteOperationTransmitter.send()`
  (`scripts/recv_remote_operation.py`)는 재연결 콜백이 샘플을 바꿔 E-stop을 지우므로, 바뀐 샘플에
  원래 샘플의 `estop_edge`를 OR로 유지하는 최소 수정만 한다(schema 2 유지).

## F2. 입력 만료 정지 뒤 지연 도착한 옛 명령으로 재출발 (P1, latch_stale_input=false)

- 현상: 게이트웨이 신선도(`remote_input_gateway._frame_is_fresh`)는 레코드 **수신 완료 시각**만 본다.
  고정을 끈 현장 설정에서는 입력 만료로 0을 낸 뒤, 정체 전에 만들어진 전진 레코드가 늦게 완성되면
  "방금 받은 입력"으로 보고 다시 운전한다(Codex 재현: 1.05 m/s).
- 기각한 안
  - 클라이언트 시각 + 최소 offset: 최소 지연의 상한이 없어 처음부터 늦은 스트림에서 옛 명령을 통과시킨다.
    절전(`CLOCK_MONOTONIC` 정지)·드리프트(30분×20 ppm = 36 ms)도 처리 못 한다.
  - 만료 시 연결 끊기: 입력 채널이 끊기면 통합 세션이 주행 권한을 회수한다(`powertrain_runtime/session.py`
    `_proxy` → `_revoke`). 10-30에 고친 "끊김마다 주행이 풀림"이 되살아난다.
  - 중립 재무장만: 같은 연결 버퍼에 쌓인 옛 중립 → 옛 전진 순서면 통과한다.
  - 세대 번호만(유효기한 없음): 세대를 받은 직후 만든 레코드가 다시 1 s 넘게 지연되면 통과한다(Codex 재현 1.35 s).
- **채택: 서버 발급 입력 세대 + 서버 시각 유효기한(복구 단계).**
  - 게이트웨이는 세대마다 발급 시각(서버 monotonic)을 기록한다(최근 몇 개만).
  - **만료 감지**: (a) tick이 신선 → 만료 전이를 볼 때, (b) `submit`에서 같은 연결의 직전 제출 프레임과 새 프레임의
    수신 간격이 `input_timeout_s`를 넘을 때(tick이 만료를 못 보고 프레임이 교체되는 경우 — 노드 큐가 tick 사이
    프레임을 덮어써도 측정 간격은 실제 간격 이상이다). 두 감지 모두 같은 정책을 따른다: 고정 모드는
    MOTION_HOLD, 비고정 모드는 복구 단계 진입.
  - **복구 단계 진입**: 비고정 모드의 만료 감지, 그리고 `clear_hold()`(같은 연결 버퍼의 옛 중립→전진 방지, 두 모드
    모두). 진입 시 세대를 올리고 `_required_epoch`로 둔다.
  - 복구 단계 동안: 운전 출력·중립 확인·모드 전환에 쓰는 프레임은 `input_epoch ≥ _required_epoch`이고
    `now − issued(input_epoch) ≤ input_timeout_s`여야 한다. 아니면 0. 복구 단계에서 현재 세대가
    `input_timeout_s / 2`보다 오래되면 세대를 다시 올려(요구는 그대로 최소값 유지) 노트북이 늘 유효한 세대를
    받게 한다. 이렇게 통과한 프레임은 생성 시각이 서버 기준으로 `input_timeout_s` 안이다(왕복 증명 + 서버 시계).
  - 유효 프레임을 하나 쓰면 복구 단계를 끝내고, 이후는 기존 수신 시각 신선도로 돈다(연속 스트림).
  - E-stop 엣지는 세대와 무관하게 먼저 처리한다(옛 레코드의 E-stop도 래치).
  - 새 TCP 연결(`begin_connection`)은 복구 단계를 끝낸다: 새 연결에는 옛 레코드가 없고, 기존대로 중립을 요구한다.
  - schema 2 레코드는 복구 단계 동안 쓰지 않는다 — 옛 클라이언트는 만료·HOLD 해제 뒤 재연결까지 0(안전 쪽 저하).
  - 고정 모드에서 만료는 (a)·(b) 어느 쪽으로 감지돼도 MOTION_HOLD다(복구 단계는 그 뒤 `clear_hold()`에서만).
  - 남는 한계(이번 범위 밖, 기록): 끊김 없이 연속으로 도착하지만 전체가 일정하게 늦은 스트림(예: 5 GHz 400 ms)은
    수신 시각 신선도로는 못 잡는다. 현장 대책은 2.4 GHz 로봇 AP다.
- 시각(`client_monotonic_ns`)은 안전 판정에 쓰지 않는다.

## F3. 영상이 1초 넘게 멎으면 재연결 워치독에 못 감 (P2, 00c8768)

- 현상: `operator_console/app.py` `_refresh_video_health()`가 나이 검사(1 s 지연·5 s 재연결) 전에
  `_fresh_frame_run < 3`이면 반환한다. 1 s 지연 분기에서 `_fresh_frame_run = 0`으로 만들기 때문에
  다음 호출부터 영원히 조기 반환해 5 s 재연결(`_schedule_reconnect`)을 부르지 못한다.
  또 재시작 뒤 첫 프레임이 영영 오지 않으면(`_last_frame_monotonic is None`, ERROR/EOS 없음) 다시 재시작하지 않는다.
- 수정:
  - 나이 검사를 먼저 한다. 1 s 초과면 지연·재연결 처리(카운터 0), 아니면 `< 3`일 때만 대기 표시.
    `health_state()`도 1 s 초과 STALE을 먼저 본다(3프레임 전 LIVE 금지는 유지).
  - 파이프라인이 PLAYING(`_pipeline_live`)인데 시작·재시작(`_sample_start`) 뒤 `VIDEO_STALE_RESTART_S` 동안
    프레임이 하나도 없으면 같은 재연결을 예약한다.
  - 워치독이 예약한 재연결은 실행 전에 프레임이 다시 LIVE가 되면 취소하고, 콜백 진입 시에도 회복됐으면
    재시작하지 않는다(불필요한 영상 단절 방지).
- 시험: (a) `test_integrated_app.front_frame`과 `test_terrain_gtk`의 가짜 LIVE 영상에 연속 프레임 수 반영,
  (b) 멎은 영상 5 s 재연결, (c) 첫 프레임 없는 재시작 5 s 재연결, (d) 예약 뒤 회복 시 취소.

## F4. 통합 compose의 `TELEOP_*`를 호스트에서 못 바꿈 (P2)

- 현상: `docker/docker-compose.integrated.yml` `powertrain_control.command`의 `$${TELEOP_*:-…}`는 컨테이너
  셸 확장인데 `environment:`에 해당 변수가 없어 항상 기본값이다.
- 수정: `environment:`에 `TELEOP_INPUT_TIMEOUT_S=${TELEOP_INPUT_TIMEOUT_S:-0.5}`,
  `TELEOP_LATCH_STALE_INPUT=${TELEOP_LATCH_STALE_INPUT:-false}`,
  `TELEOP_PARTIAL_CLOSE_VIOLATION=${TELEOP_PARTIAL_CLOSE_VIOLATION:-false}`를 넣는다. 기본값은 그대로다.
- 시험: compose YAML 시험(`scripts/tests/test_integrated_launchers.py` 방식) + 로컬
  `docker compose config`로 호스트 값이 컨테이너 환경에 들어가는지 확인.

## F5. 비전 스모크 음성 대조 거짓 PASS (P2, d6eba94)

- 현상: `vision_map_view`는 탭 숨김을 idle 콜백(`_queue_visibility` → `_sync_visibility` → `_unmapped`)으로
  미룬다. `vision_smoke`는 탭 전환 직후 카운터를 잡고 `hidden-render` 결함(`worker.configure(active=True)`)을
  넣는데, 그 뒤 실행되는 정상 idle 콜백이 `active=False`로 결함을 지운다. 정상 시험의 카운터도 숨김
  완료 전에 잡힌다. 또 d6eba94 이후 Top·Oblique 지도는 같은 `Gtk.Stack`이라 하나만 mapped인데
  `test_runtime_smoke`는 복귀 상태에 `slots_mapped == [True]*4`를 기대한다.
- 수정(스모크 하네스·시험): 탭을 바꾼 뒤 지도 뷰의 대기 중 가시성 idle과 타이머가 해제될 때까지 펌프한 다음
  카운터를 잡고 결함을 넣는다. 음성 대조는 주입한 `active=True`가 dwell 끝까지 유지됐는지도 확인한다.
  복귀 기대값은 카메라·자세 슬롯 mapped + 지도 둘 중 정확히 하나 mapped로 고친다. 제품 동작은 바꾸지 않는다.
- 시험: `test_runtime_smoke.py`의 hidden-render 음성 대조가 매번 FAIL, 양성이 PASS(로컬 3회 반복).

## 완료 기준

- 각 결함마다 RED 확인 → 수정 → GREEN. 영향 스위트(ros2 gateway·remote_input 시험은 rclpy 없이 도는
  순수 부분, controller_process, operator_console, scripts/tests) 통과.
- ROS 시험은 젯슨 또는 ROS 컨테이너에서(젯슨 오프라인이면 로컬 `powertrain-sw:ros` 이미지가 있으면 그것으로).
- 콘솔 `python -m operator_console.runtime_smoke` PASS.
- Codex 코드 리뷰 수렴 후 커밋. 젯슨 재배포는 사용자 확인 뒤.

## 자체 검토

- ① 직접 확인: 다섯 결함의 코드 경로를 소스에서 읽어 확인(위 각 절의 파일·행).
- ② Codex 적대 검토(1차): F2 최소 offset 안(P1, 최소 지연 상한 없음 — 1.05 m/s 반례), F1 outbox 적재 시 플래그 해제(P1,
  `_close_socket`이 미전송분 폐기), F2 0.5 s 미만 지연 통과·절전·드리프트·검증 위치(P2), F3 첫 프레임 없는 재시작(P2),
  F3 다른 fixture(P3). → F2를 서버 세대 방식으로 바꾸고 F1을 수신 확인 방식으로 바꿨다. F3에 첫 프레임 제한·예약 취소 추가.
- ③ 독립 Codex 검토(결론 비공유): F1·F2 같은 P1 2건, 옛 송신기 재연결 E-stop 유실(P2), 드리프트(P2),
  F5 Top/Oblique 동시 mapped 기대(P2), 예약 재연결 미취소(P3). → F1 범위에 옛 송신기 최소 수정, F5 기대값 수정, F3 취소 추가.
- 연결 끊기 안은 세션 회수(`session.py` `_revoke`)를 직접 확인해 기각했다.
- ② 재검토(2차): 세대 뒤 재지연(P1, 1.35 s 반례), 연결당 1회 E-stop이 새 E-stop을 막음(P1), 상태 캐시로 이전 연결
  확인 유출(P1), clear_hold 뒤 옛 중립→전진(P2), 재시작 콜백 직전 회복(P3). → 세대 발급 시각 유효기한·복구 단계,
  E-stop 이벤트 ID 정확 일치 확인, 연결 스레드 지역 확인값, clear_hold·수신 간격도 복구 단계 진입, 콜백 재확인.
- ② 재검토(3차): (b) 간격 만료가 고정 모드에서도 자동 복구가 됨(P1), 협상 전 schema 2로 보낸 E-stop이 영원히
  미확인(P2). → 감지 위치와 무관하게 모드별 정책, 협상 뒤 ID 포함 재전송.
- ② 재검토(4차): "새 P1/P2 없음"(설계 수렴).
- 구현 리뷰 1차: 첫 프레임 제한이 PLAYING 전 프리롤 정체를 놓침(P2), 접속 대상 변경이 워치독 생략 조건에 걸림(P2),
  고정 모드 HOLD 해제 직후 처리된 공백으로 재고정(P2), 스모크가 순간 재출발을 놓침(P3) → 모두 반영·회귀시험 추가.
  2차: "새 P1/P2 없음". 결과는 `docs/reports/2026-11-05-field-safety-fixes.md`.
