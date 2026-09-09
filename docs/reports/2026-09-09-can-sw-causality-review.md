# CAN 소실의 SW 유발 가능성 복기 — 2026-09-09

**SW 유발 가능성을 배제할 근거가 없다. 제가 추가한 유휴 조회가 우선 비교 대상이다.**
사용자는 이번 관측 사이 하드웨어를 전혀 변경하지 않았다고 확인했다. 수정본 기동 전
10/10이었고 기동 후 13·14가 사라졌다는 기록과 함께 취급한다. “실물 CAN 안정성 미통과”는
검증 대상의 구분이며, 하드웨어 고장으로 원인을 판정했다는 뜻이 아니다.

## 1. 실제 실행 순서

아래 시각은 모두 **같은 Jetson 시계**다. Jetson 벽시계는 당시 2026-08-18로 어긋나 있어
로컬 날짜와 직접 대조하지 않는다. 소실 순간의 연속 raw TX/RX 캡처는 없었다.

| Jetson 시각(UTC) | 직접 확인한 사실 |
|---|---|
| 06:51:51부터 6초 | 모든 10개 모터 각각 300프레임. Jetson 누적 CAN TX 0→0. |
| 06:56:57 | 수정된 설치본 `chassis_node`가 실제 can0에서 50 Hz 루프 시작. |
| 06:57:52 | 실제 콘솔 세션의 IDLE 요청. 콘솔 Start·실물 비영점 명령 미호출. |
| 06:59:04부터 6초 | 13·14만 무응답. Jetson TX 17,093→17,861, 즉 128프레임/s. |
| 07:01:48 이후 | session/control/chassis/watchdog 정지. TX 증가 0이나 13·14 무응답 지속. |
| 07:03:07 | 수동 CAN down/up 1회 후에도 8/10. 공유 reset generation은 이때 1. |
| 07:14:24부터 6초 | writer 정지 유지. TX 증가 0, 8/10 지속. |

기동 스크립트는 이 실행에서 CAN readiness만 확인했다. 소실 전 watchdog 자동 reset 로그는
없고, 수동 reset 후 영구 generation이 1이다. 따라서 이번 재발을 반복 down/up으로 설명할
증거는 없다. 일반 기동에 NVM 쓰기·node ID 변경·baud 변경·캘리·ODrive 재부팅 경로는 없고,
이번 실행에서 그런 정비 명령을 호출하지 않았다. ODrive USB도 연결돼 있지 않았다.

근거: [배포 전](2026-09-09-can-remediation-evidence/physical-can-before.json),
[기동 후](2026-09-09-can-remediation-evidence/physical-can-after.json),
[writer 정지](2026-09-09-can-remediation-evidence/physical-can-writers-stopped.json),
[수동 reset](2026-09-09-can-remediation-evidence/physical-can-after-manual-reset.json),
[최종 수신](2026-09-09-can-remediation-evidence/physical-can-final.json),
[기동·종료 로그](2026-09-09-can-remediation-evidence/retrospective-jetson-runtime.log).

## 2. 제가 변경한 실제 송신 패턴

실물에 추가 송신하지 않고 각 버전의 실제 `ChassisManager`·`CornerModule`·CAN encoder를
10초, 50 Hz, 미무장 → US-100 ESTOP 조건으로 실행했다. 드라이버의 송신 경계에 기록용
sink를 주입했다. CAN 소켓이나 펌웨어는 실행하지 않는다. 모터 응답/버스 타이밍 시험은 아니다.

| 소스 | 10초 RTR 조회 | 초당 조회 |
|---|---:|---:|
| 통합 운용 변경 전 Git `c3fd078` | 0 | **0** |
| 이번 수정 착수 시 Jetson 통합 배포본 스냅샷 | 6,000 | **600** |
| 이번 보완본 Git `b9292af` | 1,260 | **126** |

세 버전 모두 조회 외에는 동일한 정지용 16프레임(ODrive 속도 0·IDLE 각 6개, AK RPM 0
4개)을 보냈다. 이 재실행에서는 비영점 지령·state8·node 변경·재부팅 명령이 없었다.
이 결과를 소실 당시 실제 송신 전체를 캡처한 것처럼 취급하지 않는다.

추가한 경로는 `CornerModule._service_receive()` → `DriveOdriveCan.poll_feedback()`다.
기존에는 IDLE/FAULT에서 수신 버퍼만 처리했지만, 통합 운용에서는 정지 증거를 최신으로
얻기 위해 `0x09` encoder·`0x14` Iq RTR 조회를 계속 보낸다. 이번 보완은 encoder 최대
20 Hz/축, Iq 최대 5 Hz/축으로 제한했다. 50 Hz tick에 주기가 양자화되어 이 재실행의
실제 합계는 126회/s였다. 실제 Jetson의 한 구간 TX 128프레임/s와 비슷하지만 같은
측정은 아니다. **조회 제한 후에도 보드 소실이 재발했으므로 제한 변경은 원인 해결 증거가 아니다.**

[재실행 스크립트](2026-09-09-can-remediation-evidence/replay_idle_transmissions.py),
[이전 Git 결과](2026-09-09-can-remediation-evidence/idle-tx-pre-integrated.json),
[착수 시 배포본 결과](2026-09-09-can-remediation-evidence/idle-tx-before-remediation.json),
[보완본 결과](2026-09-09-can-remediation-evidence/idle-tx-after-remediation.json).

## 3. 무엇을 결론 낼 수 있는가

- **확인:** 제가 정지 상태의 CAN 송신 패턴을 변경했다. 이번 실제 재발은 수정본 기동 후
  관찰됐다. 새 조회 및 시작 시 정지 명령이 원인 후보에서 빠지면 안 된다.
- **미확정:** 조회가 직접 고장을 만들었는지, 특정 보드 펌웨어/통신 처리의 잠재 결함을
  드러냈는지, 다른 원인이 동시에 발생했는지는 아직 분리되지 않았다. 초당 프레임 수만으로
  버스 포화 또는 보드 FIFO 고갈을 확정하지 않는다.
- **정정:** “프로세스를 껐는데도 안 살아나니 SW 탓이 아니다”는 추론은 성립하지 않는다.
  송신이 보드의 지속 오류 상태를 유발했다면 송신자 종료나 Jetson CAN reset 뒤에도 그
  상태가 남을 수 있다. Jetson 오류 카운터 0도 ODrive 내부 오류 0을 증명하지 않는다.
- **조건부 근거:** fw-v0.5.6의 CAN 서버는 HAL 오류 NONE과 TIMEOUT 외의 오류에 복구 처리가
  없는 코드가 있다. 두 축이 공유하는 통신 서버 정지는 관측과 양립한다. 다만 실제 firmware,
  HAL 상태 및 오류 도달 조건은 읽지 못했으므로 실제 원인으로 확정하지 않는다.
  [고정 버전 공식 소스](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/communication/can/odrive_can.cpp#L47-L66).
- **검증의 부족:** 소실 전후 연속 TX/RX 및 보드 상태를 확보하지 않았다. 단위시험·ROS/vcan
  전체 루프는 펌웨어의 지속 오류를 재현하지 않으므로 이번 하드웨어 연결 안정성을 보증하지 못한다.

## 4. 다음 원인 분리 순서

현재 motor writer는 정지해 두었다. 아직 응답하는 다른 보드에 의심 트래픽을 추가하지 않는다.
무응답 보드의 전원을 유지한 채 USB로 실제 firmware·serial·CAN 구성·axis 오류와 노출되는
CAN 상태를 먼저 읽어 보존한다. USB API에 HAL 오류가 노출되지 않으면 확인 불가로 남긴다.

상태 확보 후 보드를 정상화하고, 같은 배선·전원에서 **수신만 → 기존 정지 경로 → encoder
저속 단독 조회 → Iq 단독 조회 → 통합 유휴 조회**를 하나씩 비교한다. 각 단계 시작 전에
10/10이 살아 있는지 확인하고, 처음부터 raw CAN·오류 프레임·링크 카운터·송신 지령을
연속 기록한다. 소실되면 즉시 다음 단계를 중단하고 보드 상태를 보존한다. 단순히 현재
멈춘 보드에 이전 코드를 올리는 것은 유효한 A/B 비교가 아니다. 이 실제 비교는 **아직 수행하지 않았다.**
