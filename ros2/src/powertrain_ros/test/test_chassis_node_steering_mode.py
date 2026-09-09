"""chassis_node 의 조향모드 서비스 — 핸들러 단위 검증.

⚠️ rclpy 필요 — 젯슨/ROS 환경에서만 수집된다.
노드 전체 기동은 P5 스모크가 담당한다. 여기서는 서비스 핸들러가
ChassisManager 계약을 올바로 옮기는지만 본다.
"""
from powertrain_ros import chassis_node as node_mod


class _FakeManager:
    def __init__(self, available=True, accept=True, reason="pending_skid"):
        self.steering_mode = "ackermann"
        self.steering_available = available
        self.requested = []
        self._accept = accept
        self._reason = reason

    def request_steering_mode(self, mode):
        self.requested.append(mode)
        return self._accept, self._reason


class _Response:
    success = None
    message = None


class _Request:
    def __init__(self, data):
        self.data = data


def _node_with(manager):
    node = object.__new__(node_mod.ChassisNode)
    node.cm = manager
    return node


def test_steer_mode_service_maps_true_to_skid():
    manager = _FakeManager()
    node = _node_with(manager)

    response = node._srv_steer_mode_skid(_Request(True), _Response())

    assert manager.requested == ["skid"]
    assert response.success is True


def test_steer_mode_service_maps_false_to_ackermann():
    manager = _FakeManager(reason="pending_ackermann")
    node = _node_with(manager)

    node._srv_steer_mode_skid(_Request(False), _Response())

    assert manager.requested == ["ackermann"]


def test_steer_mode_service_reports_refusal():
    manager = _FakeManager(available=False, accept=False,
                           reason="steering_unavailable")
    node = _node_with(manager)

    response = node._srv_steer_mode_skid(_Request(False), _Response())

    assert response.success is False
    assert "steering_unavailable" in response.message


def test_steer_mode_service_survives_a_missing_manager():
    node = object.__new__(node_mod.ChassisNode)
    node.cm = None

    response = node._srv_steer_mode_skid(_Request(True), _Response())

    assert response.success is False
