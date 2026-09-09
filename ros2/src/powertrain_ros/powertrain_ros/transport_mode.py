"""구동 트랜스포트 모드 파일.

트랜스포트(CAN↔USB)는 런타임 전환이 불가능하다 — 드라이버를 다시 만들어야 하고,
corners 는 `ChassisManager` 생성자에서 고정된다. USB 축은 전원 인가 뒤 캘리 준비
플래그와 오류를 확인한다. 영속화가 자격화된 준비 축은 바로 쓸 수 있고,
미준비·미자격 축만 출력축을 자유롭게 한 뒤 축당 ~55 s 풀캘리한다. 이 캘리
도우미는 현재 상태만 갱신하며 NVM에는 저장하지 않는다. 그래서 콘솔은 "다음
기동에 무엇을 쓸지"만 이 파일에 남기고, launch 가 기동 시 읽는다.

위치는 `/etc/powertrain/ops_*.token` 과 같은 규약이다.

⚠️ **인식할 수 없는 값은 `can` 으로 떨어진다.** 오타 하나로 조향이 없는 스택이
뜨는 쪽이 더 위험하기 때문이다.

ROS-free 순수 모듈이라 호스트 pytest 로 검증한다 (`ops_contract.py` 와 같은 이유).
"""
from pathlib import Path

DEFAULT_PATH = "/etc/powertrain/drive_transport"
VALID = ("can", "usb")
FALLBACK = "can"

#: 트랜스포트별 조향모드 기본값. USB 스택에는 조향 액추에이터가 없으므로
#: (조향 AK45-36 은 CAN 전용) 스키드만 가능하다.
_DEFAULT_STEERING = {"can": "ackermann", "usb": "skid"}


def read(path=DEFAULT_PATH) -> str:
    """모드 파일을 읽어 ``"can"`` 또는 ``"usb"`` 를 반환한다.

    읽기 실패(파일 없음·디렉토리·권한·인코딩)는 예외를 내지 않고 ``"can"`` 으로
    떨어진다. 모드 파일 하나 때문에 제어 스택 기동이 막히면 안 된다.
    """
    try:
        raw = Path(path).read_text(encoding="utf-8")
    except (OSError, UnicodeError, ValueError):
        return FALLBACK
    value = raw.strip().lower()
    return value if value in VALID else FALLBACK


def default_steering_mode(drive_transport) -> str:
    """트랜스포트에 짝이 되는 조향모드 기본값.

    launch 인자 두 개가 서로 어긋나지 않도록 여기 한 곳에서만 정한다 —
    ``drive_transport=usb`` 에 ``steering_mode=ackermann`` 이 붙으면
    `chassis_node` 가 기동 시 `ValueError` 로 거부한다.
    """
    return _DEFAULT_STEERING.get(str(drive_transport), "ackermann")
