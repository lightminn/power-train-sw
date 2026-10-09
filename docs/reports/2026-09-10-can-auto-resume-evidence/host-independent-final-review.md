# CAN 자동 재개 호스트 최종 독립 재검증 — 2026-09-10

**판정: 이전 독립 리뷰의 P1 세 건이 현재 소스/호스트 더블 실행 경계에서 모두 종결됐다. 최종 guarded RESUME 프로토콜 경계에서도 추가 blocker를 발견하지 못했다.** 검토 범위는 앞서 발견한 결함과 그 수정 delta에 한정했다. ROS 설치 E2E·Jetson·보드·물리 모터는 root의 별도 시험 대상이다.

생산 소스는 읽기 전용. `/tmp/mks-can-auto-resume-20260910`의 보고서, 비교 hash, 재현 probe만 작성했다. 원래 반례 스크립트와 실패 기록은 보존했다. 아래 파일:행은 `/home/light/ZETIN/robotics/power-train-sw` 기준이며 최종 파일 SHA-256은 `host-independent-final-fingerprints.json`에 기록했다.

| 기존 발견 | 수정 근거 | 독립 재실행 결과 |
|---|---|---|
| P1-1 조향 stale에서 부분 구동 명령 | `motor_control/chassis/chassis_manager.py:607-619`가 입력을 stage만 한다. `:993-1007`에서 전체 steering/drive 검사 후 hold를 해제한다. `:1101-1103`도 분배 전에 fault/stale을 전체 차단한다. | WAIT_INPUT에서 stale을 먼저 tick한 경우와, stale+새 명령이 같은 tick인 경우를 각각 실행. 두 경우 모두 ESTOP/count0이며6축의 이후 실제0x0D 전송 이력에 비영속도는 **0건**. |
| P1-2 소실/재부팅 축을 정상 peer로 오인 | `motor_control/chassis/can_recovery.py:64-100`은 stale 관측 노드를 required_nodes에 넣고 각자 eligible를 요구한다. `:102-110`과 manager `:970-978`가 최초 관측 eligible/generation도 stop side effect 전에 저장한다. | 두 보드 상황으로 강화:13/14와15/16 소실 뒤13/14만 eligible,15/16은 reboot error0/flags0/gen0. 계속 새 입력을 주어도6축IDLE/속도0 유지, 최초5s 기한 후 CANCELLED/count0. required_nodes는[13,14,15,16]. |
| P1-3 빠른 재접속에서 disconnect edge 유실 | TCP accept가 만든 UUID가 gateway DriveOutput→ManualDriveCommand→authority→CM에 값과 함께 전달된다. `chassis_node.py:1481-1497`은 상태 epoch 변경을 즉시 취소하고 `:1629-1647`은 명령/상태 epoch 일치를 요구한다. manager `:888-909`는 진행 중 epoch 변경을 영구 취소한다. | 실제 gateway `_drain_events`가 disconnect/connect/neutral3개를 같은 호출에서 처리하여 최종 DRIVE만 발행하도록 이전 반례를 재실행. 클라이언트 session ID까지 동일하게 유지했지만 server UUID 변경으로 CANCELLED/IDLE/count0. 이어진 trigger 입력에도6축속도0. |

## 최종 프로토콜 경계

- `motor_control/corner_module/drive_odrive_can.py:218-221`은 A3/DLC8/known flags만 지원으로 인정한다. A2와 unmarked0x4000은 자동 재개 후보가 아니다.
- 같은 파일 `:341-362`에서 guarded generation을 선택한 뒤, affected axis에는 정확히 한 번 `opcode7 + struct.pack('<IBBBB',8,0xA3,generation,0,0)`만 보낸다. 일반 ARM이 뒤따르는 분기가 없다. 자동 경로는 opcode24를 전혀 보내지 않으며, fresh error0 peer는 clear 없이 ordinary ARM만 받는다.
- 같은 파일 `:369-379`는 request 이후 새로운 heartbeat와 source timestamp, fresh encoder, error0/state8 확인을 유지한다.
- 표적 테스트 `test_automatic_rearm_never_uses_unconditional_clear_and_does_not_clear_error0_peers`가 실제 송신 이력을 검사한다. external manual clear 및 새 실제 axis fault를 guarded command 직전에 주입하는 테스트가 이제 모두 통과한다.
- 펌웨어 실제 함수의 원자성·거부 경계는 별도 `patch3-independent-review.md`의86/86 source scenarios로 확인했다. 호스트 더블의 성공을 실제 펌웨어 검증으로 대신하지 않았다.

## 직접 실행한 증거

1. 표적 pytest **45 passed** (`host-independent-final-tests.log`). 이전39pass/1fail CAS RED를 포함해 최종 계약으로 재실행했다.
2. 별도 독립 probe **4케이스 PASS**: 조향 stale2개, 보드별 혼합복구1개, 빠른 재접속1개. `host-final-p1-recheck.py`, `host-final-p1-recheck.log`. 실제 원본 gateway/ROS callback 본문을 추출해 실행하는 `host-reconnect-final-probe.py`도 남겼다.
3. 구현 담당이 보고한 전체838pass는 이 리뷰에서 중복 실행하거나 독립 결과로 집계하지 않았다.

```bash
cd /home/light/ZETIN/robotics/power-train-sw
PYTHONPYCACHEPREFIX=/tmp/mks-can-auto-resume-20260910/review-pycache \
PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
/home/light/anaconda3/bin/python -m pytest motor_control/chassis/tests/test_can_auto_recovery.py \
  motor_control/chassis/tests/test_authority_receipt.py \
  motor_control/corner_module/tests/test_can_recovery_metadata.py \
  ros2/src/powertrain_ros/test/test_can_recovery_receipt.py -q
PYTHONPYCACHEPREFIX=/tmp/mks-can-auto-resume-20260910/review-pycache \
PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
/home/light/anaconda3/bin/python /tmp/mks-can-auto-resume-20260910/host-final-p1-recheck.py
```

확정 범위는 유지된다. deadman release는 정상 새0명령이고 explicit disarm과 구분하며, 그 입력으로 재개해도 출력은0이어야 한다. Legacy Twist는 일반 수동 운용을 유지하지만 source receipt/connection provenance가 없어 자동 재개 대상이 아니다. 최초5s 기한, 새 post-arm receipt, 다른 hold/입력 소실/disarm/E-stop 취소 경계가 표적 스위트에서 유지된다.
