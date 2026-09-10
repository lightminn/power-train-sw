#!/usr/bin/env python3
"""다이나믹셀 시리얼 버스의 배타 잠금 — 두 번째 런타임이 **기동을 거부**하게 한다.

## 왜 필요한가 (2026-08-12 실기 사고)

`position_node` 와 `moveit_dynamixel_bridge` 는 둘 다 `/dev/ttyUSB0` 를 잡는다.
DynamixelSDK 의 `PortHandler.openPort()` 는 잠금을 걸지 않으므로 두 프로세스가
**둘 다 성공적으로** 포트를 연다. 그런데 결과가 "둘 다 안 됨"이 아니라 **축 하나만
조용히 빠지는** 형태로 나타나서 진단이 매우 어렵다.

실제 사례: `teleop.launch.py` 를 정리 없이 52초 간격으로 두 번 띄웠다. 첫 노드는
ID 13 포함 5축을 정상 등록했는데, 두 번째 노드가 포트를 여는 순간 패킷이 섞여
첫 노드는 `Port is in use!` 로 죽고, 두 번째 노드는 초기화 도중 **하필 ID 13 차례에서만**
응답이 깨져 `operating mode 변경 실패` → 그 축만 미등록. 이후 목표가 올 때마다
`Unknown Dynamixel ID: 13`. Dynamixel Wizard 로는 ID 13 이 멀쩡히 잡혀서 서보 고장으로
오진했고, 커넥터를 뒤지느라 시간을 버렸다.

미등록 ID 는 reboot·torque 요청까지 `등록 안 된 ID` 로 무시된다 — 등록은 기동 시 1회뿐이라
**재기동 말고는 복구 경로가 없다.** 그래서 "먼저 뜬 쪽이 이기고 나중 쪽은 즉시 죽는다"로
못 박는다. 조용한 부분 실패보다 시끄러운 기동 실패가 낫다.

## 어떻게

장치 파일 자체에 `flock(LOCK_EX | LOCK_NB)` 을 건다. 프로세스마다 fd 는 달라도 flock 은
같은 inode 에 대해 프로세스 간에 동작하고, 프로세스가 죽으면 커널이 자동으로 푼다
(락 파일 방식과 달리 stale lock 이 남지 않는다 — `ros2 launch` 가 자식 노드를 흘리는
이 저장소의 고질적인 상황에서 중요하다).

⚠️ **한계**: 이 저장소의 노드끼리만 유효하다. Dynamixel Wizard 같은 외부 프로그램은
flock 을 걸지 않으므로 이 잠금으로 막히지 않는다 — Wizard 를 열어둔 채 노드를 띄우면
여전히 패킷이 섞인다.
"""

import fcntl
import os


class BusInUseError(RuntimeError):
    """다른 프로세스가 이미 이 버스를 잡고 있다."""


def _holders(port):
    """`port` 를 열고 있는 프로세스들의 `(pid, cmdline)` — best effort.

    `/proc` 를 훑는 것뿐이라 실패해도 잠금 자체에는 영향이 없다. 사용자에게
    "무엇을 먼저 내려야 하는지"를 알려주는 용도다.
    """
    found = []
    try:
        target = os.path.realpath(port)
        for pid in os.listdir('/proc'):
            if not pid.isdigit() or int(pid) == os.getpid():
                continue
            fd_dir = f'/proc/{pid}/fd'
            try:
                for fd in os.listdir(fd_dir):
                    if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                        continue
                    with open(f'/proc/{pid}/cmdline', 'rb') as f:
                        cmd = f.read().replace(b'\0', b' ').decode(
                            'utf-8', 'replace').strip()
                    found.append((pid, cmd[:120]))
                    break
            except OSError:
                continue                      # 그 사이 죽었거나 권한 없음
    except OSError:
        pass
    return found


def acquire(port, logger=None):
    """`port` 에 배타 잠금을 걸고 fd 를 돌려준다. 실패하면 `BusInUseError`.

    ⚠️ **반환한 fd 를 반드시 살려둘 것.** 가비지 컬렉션으로 닫히면 잠금도 같이 풀린다
    (노드 인스턴스 속성으로 들고 있으면 된다). 프로세스가 끝나면 커널이 알아서 푼다.
    """
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    except OSError as exc:
        raise BusInUseError(
            f'{port} 를 열 수 없습니다: {exc} — 어댑터가 호스트에 꽂혀 있는지, '
            f'컨테이너가 privileged 인지 확인하세요') from exc

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        os.close(fd)
        holders = _holders(port)
        detail = (
            '\n  잡고 있는 프로세스: '
            + '\n    '.join(f'PID {pid}: {cmd}' for pid, cmd in holders)
        ) if holders else ''
        raise BusInUseError(
            f'{port} 를 다른 프로세스가 이미 사용 중입니다 ({exc}). '
            '같은 버스를 두 런타임이 나눠 쓰면 패킷이 섞여 **축 하나만 조용히 빠지는** '
            '형태로 망가지므로 기동하지 않습니다. 먼저 그 프로세스를 내리세요 '
            '— 예: pkill -f "[t]eleop.launch.py"; pkill -f "[p]osition_node"'
            + detail) from exc

    if logger is not None:
        logger.info(f'{port} 버스 배타 잠금 획득 (flock)')
    return fd
