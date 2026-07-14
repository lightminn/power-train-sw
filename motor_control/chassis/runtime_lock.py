"""실물 SocketCAN 버스의 프로세스 간 단독 소유권을 강제한다.

왜 필요한가
------------
``chassis_node``와 직접 제어 도구(legacy teleop, motor GUI, 교정 도구)가 같은
``can0``을 동시에 열면 같은 모터에 상반된 명령을 번갈아 보낼 수 있다. SocketCAN은
여러 송신 소켓을 허용하므로 "동시에 띄우지 말 것"이라는 운영 관례만으로는 사고를
막지 못한다. 과거에는 종료되지 않은 teleop이 계속 0 속도를 보내 새 모터 시험과
싸운 좀비 프로세스 사고도 있었다.

``RealCanSession``은 모든 실물 CAN entry point가 공유하는 공개 수명주기 API다. 호스트의
``/run/powertrain``을 컨테이너들이 같은 경로로 bind mount하고, 그 안의 영구 lock 파일에
nonblocking ``flock``을 건다. 프로세스가 죽으면 커널이 fd를 닫아 lock은 자동 해제된다.
파일 자체는 stale 여부와 무관하게 절대 삭제하지 않는다.

런타임 디렉터리 생성 권한은 ``scripts/install_powertrain_runtime_dir.sh``에만 있다. 이
모듈은 디렉터리가 없을 때 임의로 만들지 않고 명확하게 실패한다. Fake, vcan, MuJoCo
경로는 이 API를 호출하지 않아야 한다.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import fcntl
import os
from pathlib import Path
import sys
from typing import Optional


_DEFAULT_CHANNEL = "can0"
_DEFAULT_PATH = "/run/powertrain/can0.lock"


class CanOwnershipError(RuntimeError):
    """실물 CAN owner lock을 획득하지 못했을 때 발생한다."""


@dataclass(frozen=True)
class CanOwnerSnapshot:
    """lock 획득 순간의 변경 불가능한 owner 관측 정보."""

    acquired_at: datetime
    pid: int
    process_name: str
    lock_path: str


class CanOwnerLock:
    """``flock`` fd를 소유하는 내부 primitive.

    Entry point는 이 클래스를 직접 쓰지 말고 :class:`RealCanSession`을 사용한다.
    """

    def __init__(self, path: str = _DEFAULT_PATH):
        self.path = path
        self.fd: Optional[int] = None

    def acquire(self) -> None:
        if self.fd is not None:
            raise CanOwnershipError(f"CAN owner lock already acquired: {self.path}")

        try:
            fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o640)
        except OSError as exc:
            raise CanOwnershipError(
                f"CAN owner lock file open failed: {self.path}: {exc}"
            ) from exc

        self.fd = fd
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            try:
                os.close(fd)
            finally:
                self.fd = None
            raise CanOwnershipError(
                f"CAN owner lock acquisition failed: {self.path}: {exc}"
            ) from exc

    def close(self) -> None:
        if self.fd is not None:
            fd = self.fd
            self.fd = None
            os.close(fd)


class RealCanSession:
    """실물 CAN entry point용 단독-owner context manager.

    Session에 진입한 뒤에만 bus 또는 real corner를 만들고, 모든 bus/corner를 닫은 뒤
    context를 빠져나가야 한다. ``owner_snapshot``은 성공한 획득 시점에 한 번 생성되며
    session 종료 후에도 마지막 owner 관측값으로 남는다.
    """

    def __init__(
        self,
        channel: str = _DEFAULT_CHANNEL,
        owner: Optional[str] = None,
        path: str = _DEFAULT_PATH,
    ):
        self.channel = str(channel)
        self.process_name = owner or Path(sys.argv[0]).name or "python"
        if path == _DEFAULT_PATH and channel != _DEFAULT_CHANNEL:
            path = f"/run/powertrain/{channel}.lock"
        self.path = os.fspath(path)
        self.owner_snapshot: Optional[CanOwnerSnapshot] = None
        self._lock = CanOwnerLock(self.path)

    def __enter__(self):
        try:
            self._lock.acquire()
        except CanOwnershipError as exc:
            cause = exc.__cause__
            if isinstance(cause, FileNotFoundError):
                directory = os.path.dirname(self.path) or "."
                raise CanOwnershipError(
                    f"CAN runtime lock directory does not exist: {directory}. "
                    "디렉터리를 임의 생성하지 말고 Jetson host에서 "
                    "sudo bash scripts/install_powertrain_runtime_dir.sh 를 먼저 실행할 것."
                ) from exc
            if isinstance(cause, OSError) and cause.errno in (
                errno.EACCES,
                errno.EAGAIN,
            ):
                raise CanOwnershipError(self._busy_message()) from exc
            raise CanOwnershipError(
                f"{self.channel} owner lock 획득 실패 ({self.path}): {exc}"
            ) from exc

        self.owner_snapshot = CanOwnerSnapshot(
            acquired_at=datetime.now(timezone.utc),
            pid=os.getpid(),
            process_name=self.process_name,
            lock_path=self.path,
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False

    def close(self) -> None:
        """lock fd를 닫는다. lock 파일은 삭제하지 않는다."""

        self._lock.close()

    def _busy_message(self) -> str:
        return (
            f"{self.channel} 을 이미 다른 프로세스가 소유하고 있다 "
            f"(lock: {self.path}). chassis_node, teleop, motor_gui 또는 CAN 교정/시험 "
            "프로세스를 동시에 띄우면 같은 모터에 상반된 명령이 간다. 실행 중인 owner를 "
            "먼저 확인·종료하고 다시 시도할 것. Jetson host 확인: "
            f"sudo fuser -v {self.path} ; 컨테이너 확인: "
            "docker exec powertrain_jetson pgrep -fa "
            "'teleop|chassis|motor_gui|preflight|calibrate|status_ak|can_|odrive|ak_control' ; "
            "docker exec powertrain_ros pgrep -fa "
            "'teleop|chassis|motor_gui|preflight|calibrate|status_ak|can_|odrive|ak_control'"
        )
