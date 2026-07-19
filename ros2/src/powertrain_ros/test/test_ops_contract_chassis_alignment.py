"""Keep broker action kinds aligned with chassis_node service types."""
from pathlib import Path
import re

from powertrain_ros import ops_contract as oc


PACKAGE = Path(__file__).resolve().parents[1]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
COMPONENTS = ("drive", "steer", "us100", "robot_arm")
SERVICE_PATTERN = re.compile(
    r"""create_service\(\s*(Trigger|SetBool)\s*,\s*["']~/([^"']+)["']""",
    re.DOTALL,
)


def _chassis_services():
    source = CHASSIS_NODE.read_text(encoding="utf-8")
    services = {}
    for service_type, name in SERVICE_PATTERN.findall(source):
        if name == "component_enable_%s":
            for component in COMPONENTS:
                services["component_enable_%s" % component] = service_type
        else:
            services[name] = service_type
    return services


def test_ops_contract_chassis_service_types_align():
    services = _chassis_services()
    compatible_kinds = {
        "Trigger": {"service", "composite"},
        "SetBool": {"service_setbool"},
    }
    checked = []

    for action, spec in oc.ACTIONS.items():
        for target in spec.target:
            prefix = "/chassis_node/"
            if not target.startswith(prefix):
                continue
            service_name = target[len(prefix):]
            service_type = services.get(service_name)
            if service_type is None:
                continue
            checked.append(action)
            assert spec.kind in compatible_kinds[service_type], (
                "%s targets chassis %s service %s but declares kind %s"
                % (action, service_type, target, spec.kind)
            )

    assert "mission_clear_grip_lost" in checked
    assert set(COMPONENTS).issubset(
        {
            action.removesuffix("_enable")
            for action in checked
            if action.endswith("_enable")
        }
    )
