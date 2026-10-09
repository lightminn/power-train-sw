# CAN 자동 재개 호스트 독립 리뷰 — 2026-09-10

검토 대상: `/home/light/ZETIN/robotics/power-train-sw`의 진행 중 변경. 실제 생산 소스는 읽기 전용으로 검토했으며, 이 보고서와 `/tmp` 재현 스크립트/로그만 작성했다. Jetson, 물리 CAN, USB, 모터, 커밋/푸시는 사용하지 않았다. 소스/버스 더블 검증이며 실제 ROS 설치 경로 또는 하드웨어 검증을 뜻하지 않는다.

**판정: 독립 재현 P1 세 건을 수정하기 전에는 호스트 안전 경계 통과로 판정할 수 없다.** 세 건 모두 root와 구현 담당 direction_review에게 전달했다. root는 아래 1·2번에 동의하고 수정 지시했다. 병행 작업으로 행 번호는 이동할 수 있으므로 함께 적은 함수명을 기준으로 찾는다.

## P1-1 — 재무장 후 stale 조향에서 새로운 입력이 일부 축의 비영속도 전송을 허용

- `motor_control/chassis/chassis_manager.py:605-618`, `set()`: WAIT_INPUT에서 새 수신 시각만으로 RESUMED/count 증가 및 CAN hold 해제를 먼저 수행한다.
- 같은 파일 `:923-927`, `_can_preflight()`: 조향 오류와 전류는 검사하지만 stale은 여기서 차단하지 않는다.
- 같은 파일 `:979-982`, `_tick_can_recovery()`: WAIT_INPUT에서는 drive 6축 freshness/state8/error0이면 조향 stale을 확인하지 않고 반환한다. 조향 stale 검사는 이 분기 아래 `:992-994`에 있다.
- 같은 파일 `:1105-1128`, `tick()`: stale 코너가 자기 FAULT로 반환한 뒤에도 나머지 코너 tick을 실행하고 마지막에 global ESTOP을 전파한다.
- `motor_control/corner_module/corner_module.py:193-197`: stale 조향 코너는 로컬 estop 후 정상 return한다.

재현: eligible 복구 → WAIT_INPUT → 앞왼쪽 조향 stale → arm 완료 시각과 같은 receipt로 tick → 여전히 ARMED/WAIT_INPUT. 다음 실제 새 입력을 주면 count1/RESUMED가 먼저 되고, node12~16 다섯 축에 각각 raw 3.073676109313965 motor rev/s가 전송된 다음 전역 ESTOP이 된다. 최종 속도만 검사하면 이 순간 전송을 놓친다.

권고: `set()`은 새 입력 후보만 저장하고, tick의 동일한 전체 건강 검사를 통과한 뒤 hold 해제와 명령 적용을 함께 결정한다. WAIT_INPUT의 조향 stale도 명시 처리하고, 어떤 코너의 사전 안전 실패도 다른 축의 비영속도 전송 전에 전체 차단한다.

근거: `host-independent-probes.py`, `host-independent-probes.log`의 `steering_stale_after_rearm` 결과.

## P1-2 — 별도 소실 축의 재부팅을 정상 peer로 분류해 자동 재무장

- `motor_control/chassis/can_recovery.py:54-86`, `stopped_evidence()`: stale 표본은 건너뛰며, 정상 error0 축을 허용하고 전체 중 하나 이상의 eligible만 있으면 안정 시간을 축적한다.
- 같은 파일 `:67-74`: 이전에 eligible로 기록된 축의 eligibility 취소는 추적하지만, 이전에 소실된 축 자체의 집합은 기록하지 않는다.
- `motor_control/chassis/chassis_manager.py:1064-1072`, `tick()`: 시작 tick의 소실/eligible 축 정보를 episode에 넘기지 않는다.

재현: node13과15가 동시에 소실 → host WAIT_STOP →13은 명시 eligible recovery,15는 MCU reboot 상황인 error0/flags0/gen0/IDLE로 복귀 → 안정 확인 뒤15도 state8로 재무장되고 새 입력에서 비영속도가 나온다. 현재 단일 축 reboot 음성 테스트는 다른 eligible 축이 없으므로 이 혼합 원인을 검출하지 못한다.

권고: episode에서 소실/stale 관측된 모든 drive는 자기 A3 eligible 증거를 요구한다. 계속 정상 fresh를 유지한 error0 peer만 일반 peer 재무장 대상이어야 한다. 시작 tick에서 이미 관측한 eligible도 즉시 기록해 다음 tick 전 reboot/clear로 사라지는 근거를 놓치지 않는다. 이는 root가 확인한 수정 방향이다.

근거: `host-independent-probes.py`, `host-independent-probes.log`의 `second_missing_axis_reboots_error_free` 결과. 이 더블은 노드별 인터페이스이므로 한 node의 reboot 상태를 주입했다. 실제 듀얼축 보드 reboot 인수는 양축을 함께 검사해야 한다.

## P1-3 — 같은 gateway tick 안의 빠른 재접속이 disconnect 취소를 숨김

- `ros2/src/powertrain_ros/powertrain_ros/teleop_command_node.py:429-453`, `_drain_events()`: disconnect, connect, 새 motion frame을 같은 호출에서 순서대로 모두 처리할 수 있다.
- 같은 파일 `:594-623`, `_tick()`: drain 이후 최종 gateway 상태만 발행한다. gateway timer는 `:162`에서30Hz다.
- `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:1485-1496`, `_on_gateway_state()`: DRIVE+fresh 아닌 상태를 수신해야 cancel한다.
- 같은 파일 `:1626-1636`, `_tick_authority()`: 명령의 실제 TCP 시각은 보존하지만 connection/session identity는 없으므로 이전 복구 episode와 새 연결의 입력을 구별하지 못한다.

재현: 복구 WAIT_INPUT 중 TCP worker가 tick 사이에 disconnect(old session), connect(new session), new-session neutral을 큐에 둔다. 실제 `_drain_events()`를 실행하면3개를 처리하고 gateway는 DRIVE+fresh만 발행한다. 실제 chassis callbacks를 실행하면 reconnect neutral로 RESUMED/count1, 이어 새 trigger 입력으로6축 비영속도 전송까지 이루어진다. 명시 manual-start는 없었다. disconnect가 영구 cancel해야 한다는 계약을 위반한다.

권고: 연결/session generation을 명령 provenance에 결합하고, 진행 중 episode의 연결 변경을 취소한다. 다른 대안도 disconnect edge가 gateway 상태 또는 DDS KEEP_LAST(1)에서 유실되지 않음을 증명해야 한다. 상태를 한 번 더 publish하는 것만으로는 latest-state 큐의 edge 보존을 보장하지 못한다.

근거: `host-reconnect-probe.py`, `host-reconnect-probe.log`. 생산 `_drain_events`, `_on_gateway_state`, `_on_manual_drive_command`, `_tick_authority`를 AST에서 추출하여 실행했고, 정책은 다시 구현하지 않았다. rclpy import나 네트워크는 사용하지 않았다.

## 확인한 정상 경계 및 범위

- 실제 TCP `recv()` 직후 host monotonic 시각이 decoder frame에 들어간다(`teleop_command_node.py:313-330`). Gateway output→ManualDriveCommand→authority Command→ChassisManager로 payload와 함께 이동하며, 반복 발행은 원래 TCP 시각을 유지한다.
- DDS receipt는 executor backlog 확인용 별도 시각이다. gateway 상태의 별도 input stamp가 명령 stamp를 대신하지 않는다. legacy Twist에는 source stamp가 없어 ROS 경계에서 자동 재개 active가 되지 않는다.
- pending 중 explicit disarm/E-stop, manual inactive/stale, 다른 motion hold, component/steering mode 변화는 cancel 경로가 있다. 취소 후 뒤늦은 입력만으로 다시 arm하지 않는다.
- helper는6축 identity의 실제 CAN driver 구성을 caller에서 검사하고, error는 정확0x4000+지원marker+eligible+IDLE만 허용한다. generation 변화는200ms 안정 타이머를 재시작하되 최초5s episode 기한을 늘리지 않는다. 원래 수신 시각이 arm 완료보다 새로워야 재개하는 경계가 있다.
- root가 확정한 의미: **deadman release는 정상0명령이며 explicit arm/disarm과 별개다. 따라서 deadman release 자체는 자동 재개 intent 취소 대상이 아니다.** 새0입력으로 재개한 경우 실제 출력도0이어야 하고, 과거 비영명령을 재사용하면 안 된다.

## 진행 중 프로토콜 변경 — 중복 blocker에서 제외

초기 검토 시 A3 guarded CLEAR(0x18) 이후 일반 ARM8 사이에서 외부 manual clear가 eligibility를 취소+error0으로 만들면 CAS 거부 뒤 일반ARM이 성공하는 반례가 구현 담당의 RED 테스트로 발견됐다. Root/firmware/host 담당은 단일 guarded RESUME(0x07, state8+marker+generation)를 합의 중이다. 이 영역은 이번 호스트 리뷰에서 통과로 판정하지 않으며, 알려진 진행 항목이라 위 독립 세 건과 중복 집계하지 않는다. 최신 최종 protocol·firmware 구현은 별도 검토/시험이 필요하다.

## 실행 증거

아래 표적 명령을 직접 실행했다. 당시 결과 **39 passed / 1 failed**. 실패는 위 알려진 CAS integration RED(`test_external_manual_clear_between_host_guard_and_cas_cannot_become_auto_arm`)이며 숨기거나 제외하지 않았다. 병행 수정 중인 스냅샷의 결과다.

```bash
cd /home/light/ZETIN/robotics/power-train-sw
PYTHONPYCACHEPREFIX=/tmp/mks-can-auto-resume-20260910/review-pycache \
PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
/home/light/anaconda3/bin/python -m pytest \
  motor_control/chassis/tests/test_can_auto_recovery.py \
  motor_control/chassis/tests/test_authority_receipt.py \
  motor_control/corner_module/tests/test_can_recovery_metadata.py \
  ros2/src/powertrain_ros/test/test_can_recovery_receipt.py -q
```

독립 반례 재실행(모든 버스는 in-memory fixture):

```bash
cd /home/light/ZETIN/robotics/power-train-sw
PYTHONPYCACHEPREFIX=/tmp/mks-can-auto-resume-20260910/review-pycache \
PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
/home/light/anaconda3/bin/python /tmp/mks-can-auto-resume-20260910/host-independent-probes.py
PYTHONPYCACHEPREFIX=/tmp/mks-can-auto-resume-20260910/review-pycache \
PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
/home/light/anaconda3/bin/python /tmp/mks-can-auto-resume-20260910/host-reconnect-probe.py
```

두 스크립트는 관측값을 기록하는 반례 probe다. 최종 안전 회귀 게이트는 실제0x0D 전송 이력을 검사해야 하며, 최종 모드/속도만 확인해서는 P1-1을 잡지 못한다. 수정 후 별도 재검증 전까지 이 보고서의 발견 상태를 해결됨으로 읽지 않는다.
