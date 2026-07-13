"""CAN 버스 **단독 소유권** 락 — 두 프로세스가 같은 모터를 동시에 조종하는 걸 막는다.

    with can_bus_lock("can0"):
        corners = build_real_corners("can0")
        ...

────────────────────────────────────────────────────────────────────────
왜 필요한가
────────────────────────────────────────────────────────────────────────
`chassis_node`(ROS)와 `teleop_server`(직접 제어)가 **둘 다 can0 을 열 수 있다.** 동시에
띄우면 같은 모터에 **상반된 명령**이 50 Hz 로 번갈아 간다 — 로봇이 진동하거나, 꺼진 줄
알았던 쪽이 계속 조종한다. socketcan 은 여러 소켓의 동시 송신을 **막지 않는다**.

지금까지는 **"동시에 띄우지 말 것"이라는 운영 관례**로만 막고 있었다. 사람이 실수하면
그대로 사고다. 강제한다.

────────────────────────────────────────────────────────────────────────
왜 파일 락이 아니라 **추상 소켓**인가
────────────────────────────────────────────────────────────────────────
두 프로세스가 **서로 다른 컨테이너**에 있다(`powertrain_ros` / `powertrain_jetson`).
파일 락은 각자 다른 마운트를 보므로 서로 안 보인다.

둘 다 `network_mode: host` 라 **네트워크 네임스페이스를 공유**한다. 리눅스의 **추상
유닉스 소켓**(이름이 `\\0` 로 시작)은 파일시스템이 아니라 **네트워크 네임스페이스**에
속하므로 컨테이너를 넘어 서로 보인다. L515 Gateway 가 이미 같은 기법으로 카메라
단독 점유를 강제한다(`@powertrain-l515-gateway`).

**프로세스가 죽으면 커널이 소켓을 닫아 락이 자동 해제된다** — 좀비 락 파일이 안 남는다.
(과거에 좀비 teleop 프로세스가 계속 v=0 을 명령해 새 테스트와 싸운 사고가 있었다.)
"""
import contextlib
import os
import socket

_PREFIX = "powertrain-canbus-"


class CanBusBusy(RuntimeError):
    """이미 다른 프로세스가 이 CAN 버스를 잡고 있다."""


def _address(channel: str) -> str:
    return "\0" + _PREFIX + channel


@contextlib.contextmanager
def can_bus_lock(channel: str = "can0", owner: str = None):
    """CAN 버스 단독 소유권. 이미 잡혀 있으면 `CanBusBusy`.

    owner : 로그·에러에 남길 이름 (예: "chassis_node", "teleop_server")
    """
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.bind(_address(channel))
    except OSError as exc:
        sock.close()
        raise CanBusBusy(
            f"{channel} 을 이미 다른 프로세스가 쓰고 있다 "
            f"(chassis_node 와 teleop_server 를 동시에 띄우면 같은 모터에 상반된 명령이 간다). "
            f"실행 중인 것을 먼저 끄고 다시 시도할 것. "
            f"확인: docker exec powertrain_jetson pgrep -fa 'teleop|chassis'"
        ) from exc

    sock.listen(1)                      # 바인드만으로 충분하지만 명시적으로
    who = owner or f"pid={os.getpid()}"
    try:
        yield sock                      # 락은 소켓이 살아 있는 동안 유지된다
    finally:
        sock.close()                    # 죽으면 커널이 알아서 닫는다


def is_locked(channel: str = "can0") -> bool:
    """지금 누가 잡고 있나? (테스트·진단용. 잡아보고 바로 놓는다 — 경쟁 조건 주의)"""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.bind(_address(channel))
    except OSError:
        return True
    finally:
        s.close()
    return False
