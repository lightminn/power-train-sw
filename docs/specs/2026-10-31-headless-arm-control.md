# GUI 없는 Python 로봇팔 제어

## 실행 구조

`operator_console.arm_terminal`은 Linux 터미널과 일반 Python 코드에서 사용한다.
클라이언트에는 GTK·ROS·Dynamixel SDK가 필요 없다. **기존 팔 브리지·인증 ops 브로커·
arm_console_bridge 미러는 실행 중이어야 한다.** USB는 기존 브리지만 소유한다.
기존 `keyboard_teleop`/`teleop_core`/`position_node`를 같은 버스에서 추가 실행하지 않는다.

이 프로그램은 PR #14의 관절 변환·제한·프로파일·자세·볼락·슬롯 파일을 사용한다.
독립 USB 드라이버, 자동 캘리브레이션, NVM 쓰기나 서보 reboot 명령을 추가하지 않았다.
모터를 직접 재부팅/교정하는 정비 작업은 기존 정비 도구의 범위다.

## 준비와 실행

Jetson의 기존 팔 세션 준비/브리지·브로커·미러 기동은
`2026-10-17-local-arm-console.md` §2.6을 따른다. GUI 기동 단계만 아래 클라이언트로 바꾼다.
브로커와 미러의 실제 주소/포트를 확인하고, 기존 세션 JSON과 토큰을 보존한다.
같은 UDP 포트를 쓰는 GUI는 먼저 종료한다. 터미널 클라이언트는 중복 수신 포트를 거부한다.

준비된 로컬 ROS 세션과 같은 호스트의 실제 터미널에서:

```bash
cd ~/power-train-sw
python3 -m operator_console.arm_terminal \
  --host 127.0.0.1 --port 29001 \
  --token-file ~/pr12-arm-session/ops_console.token \
  --telemetry-port 15707 --bench --ignore-lock
```

기대 결과는 상태/각도/볼락/자세 목록과 단축키가 표시되는 터미널 화면이다.
연결만으로 MANUAL 요청·토크 활성화·모션을 보내지 않는다. `a`로 MANUAL을 요청하고,
실제 승인/상태를 확인한 뒤 명령한다. 첫 1~4축 활성화가 필요하면 받친 자세에서
`:release`를 사용한다. `t`는 기존 GUI처럼 5축 현재 위치 복귀이며 전 축 활성화가 아니다.

`--bench`는 기존 1~5축 벤치 기능을 선택한다. `--ignore-lock`은 클라이언트 조그 게이트만
명시적으로 선택하며, 서버의 기존 `local_wrist_lock_gate` 설정을 바꾸지 않는다.
기본 실행에는 두 옵션이 없다. 서버의 `ARM_LOCAL_ALL_AXES=1` 및 세션 파라미터도 필요하다.

노트북에서 원격 실행할 때는 기존 브로커용 SSH 터널과 역할 토큰을 사용하고,
미러의 UDP 목적지를 이 노트북/수신 포트로 설정한다. GUI를 여는 helper는 실행하지 않는다.

## 키와 명령

1~5는 팔 축, 6은 도구다. ↑/W 및 ↓/S로 누름 조그, [/ ]로 속도 0~10,
H로 home, L로 관절 제한, Y로 전류 트립을 선택한다. 터미널 키/숫자패드의 실제 매핑은
터미널 설정을 따른다. `:` 뒤 명령은 Enter로 확정, Esc로 취소한다. 한글 자세 이름도 받는다.

- `save 이름`, `move 이름`, `delete 이름`, `home`
- `freedrive`, `release`: 1~4축 자유 지정/현재 위치 토크 복귀. **토크 OFF 전 팔을 받친다.**
- `lock`, `unlock`, `lock 1`, `unlock -1`, `lock-enable`, `lock-disable`, `lock-stop`
- `zero locked`, `zero unlocked`: 기존 볼락 2점 기록. 실제 장착 상태를 확인해 기록한다.
- `tool-enable`, `tool-disable`, `target single|left|right|both`
- `tool open|close|stop|left|right [대상]`, `select-tool single|dual|cleaner`
- `capture 1|2|3 single|dual|cleaner seat|above`
- `exchange single|dual|cleaner [dry-run]`, `exchange-stop`
- `manual`, `authority-release`, `limits on|off`, `trip on|off`, `stop`, `resume`

단축키 Space는 일반 입력 화면에서, Ctrl+X는 명령 입력 중에도 정지한다.
정지는 **1~4축 예약 이동 홀드 + 5축/볼락 토크 OFF**와 가능한 도구 소프트 정지다.
전체 팔 토크를 차단하는 하드웨어 E-stop으로 표시하지 않는다. q/Ctrl-C/SIGTERM/SIGHUP도
정지를 시도하고 최대 1.5초 ACK를 기다린 뒤 터미널을 복원한다. ACK 미확인은 stderr에 남긴다.
도구 소프트 정지는 토크를 유지할 수 있으므로 도구 해제는 `tool-disable`로 명시한다.

터미널에는 키 해제 이벤트가 없다. 마지막 반복 입력 뒤 **0.15초**가 지나면 조그를 놓은 것으로
간주한다. 최초 자동반복 지연 0.3~0.5초에서는 한 번 끊겼다 재시작할 수 있다.
유효 입력 중 모터 명령 갱신은 **0.2초**, 요청 TTL은 콘솔과 같은 **1.0초**다(2026-11-04 완화, 이전 0.6초).
권한·링크·도구가 바뀌면 진행 중 조그를 끝내고 입력 해제 후 새 누름을 요구한다.
SIGKILL/호스트 전원 상실에서는 종료 핸들러가 실행되지 않으며 브리지 조그 watchdog에 의존한다.
엔드포인트/자세/슬롯 이동은 누름 조그와 별도의 기존 브리지 수명주기를 따른다.

## Python API

다음 코드는 준비된 같은 호스트의 팔 세션에서 실행한다. 실제 볼락을 이동시키는 예제다.
요청 수락/최종 ACK는 브리지 전달 결과다. 물리 도달은 수신 상태로 별도 확인한다.

```python
from operator_console.arm_terminal import ArmTerminalClient

with ArmTerminalClient("127.0.0.1", 29001,
                       "/home/zetin/pr12-arm-session/ops_console.token") as arm:
    arm.wait_until(lambda state: "control_mode" in state.capabilities)
    if not arm.command("manual"):
        raise RuntimeError(arm.last_outcome)
    arm.wait_until(lambda state: state.authority.granted_mode == "MANUAL")
    arm.wait_until(lambda state: "coupling_lock" in state.capabilities)
    if not arm.command("lock-enable"):
        raise RuntimeError(arm.last_outcome)
    arm.wait_until(lambda state: state.coupling_lock.get("torque") == 1)
    if not arm.command("unlock"):
        raise RuntimeError(arm.last_outcome)
    arm.wait_until(lambda state: state.coupling_lock.get("state") == "UNLOCKED")
```

직접 제어 루프에서는 `tick()`을 규칙적으로 호출하고, 누름 동안 `jog(축, ±1)`을 반복 호출한다.
한 번의 `jog()` 호출은 연속 이동을 약속하지 않으며 0.15초 뒤 해제된다.
`wait_until`도 내부에서 tick을 실행한다. 컨텍스트 매니저 또는 `close()`로 정리한다.
동시에 다른 thread에서 같은 클라이언트를 조작하지 않는다.

## 검증 한계

볼락 LOCKED/UNLOCKED는 위치 판정이며 체결 센서 확인이 아니다.
현재 `/joint_states`의 기존 signed tick 결함과 URDF 보정 상태를 변경하지 않았다.
관절 표시가 비정상일 때는 실측값으로 해석하지 않는다. 실제 조그/자세 이동은 기존 브리지의
signed register 읽기와 기존 설정을 사용한다. 속도·영점·제한·교체 teach는 기존 실기 정본을 따른다.
