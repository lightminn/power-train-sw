"""Robot-arm ops command contract and its pure verification skeleton.

What this package is
--------------------
The design and the safety logic for sending robot-arm commands through the
console's existing **token-gated ops channel** — and nothing that actually
sends one.  It contains no socket, no ROS import, no motor motion and no
calibration arithmetic.  A test asserts all of that rather than trusting it.

The layers, innermost first:

``contract``   action names, parameter schemas, and the still-unresolved
               obligations on the arm side.  Nothing here guesses a ROS topic.
``policy``     the pure decision: may this command exist in this context?
               Capability, grant, freshness, tool identity, calibration.
``jog``        press-and-hold lifetime: deadman TTL, latest-value coalescing,
               and stop intents that survive a dead link.
``session``    the orchestrator — the only builder of requests.
``transport``  the single door out, currently shut: the arm actions are not
               registered in ``ops_contract.ACTIONS``, so nothing can be sent.
``adapter``    binds a session to ``arm_ui.ArmUiCallbacks``.

Why nothing can be sent yet
---------------------------
``ops_contract.ACTIONS`` is a closed dict and the broker rejects anything
outside it.  This task deliberately does not register the ``robot_arm_*``
actions, so :class:`~transport.ContractGatedTransport` refuses them locally and
:class:`~transport.NullArmTransport` — the default — refuses everything.  The
result is that the whole path can be built, wired to the UI and tested, while
remaining incapable of commanding a real arm.

Usage at the (future) integration point::

    from operator_console.arm_ops import (
        ArmCommandSession, ArmOpsCallbackAdapter, ContractGatedTransport,
        registered_actions_from,
    )

    transport = ContractGatedTransport(
        submit_fn=ops_client.submit,
        registered_actions=registered_actions_from(ops_contract),
    )
    session = ArmCommandSession(clock=time.monotonic, transport=transport)
    adapter = ArmOpsCallbackAdapter(session, outcome_sink=show_ack)

    manual_tab = ArmManualTab(adapter.callbacks())
    ...
    adapter.update_state(state, state_revision=ops_revision)   # 상태 갱신마다
    adapter.pump()                                             # 타이머마다
"""
from __future__ import annotations

from .contract import (
    ARM_ACTIONS,
    ARM_PARAMS_VERSION,
    BACKEND_REQUIREMENTS,
    PLANNED_ACTIONS,
    ArmOpsAck,
    ArmOpsRequest,
    ArmParamsError,
    STOP_REASONS,
    unregistered_actions,
    unresolved_requirements,
    validate_params,
)
from .jog import JogChannel, JogRegistry, PendingStop
from .policy import ArmCommandContext, Decision, authorize
from .session import ArmCommandSession, CommandOutcome
from .transport import (
    ArmActionNotRegistered,
    ArmTransportError,
    ContractGatedTransport,
    NullArmTransport,
    registered_actions_from,
)

# ``adapter`` is the only module here that touches ``arm_ui``, and importing
# ``arm_ui`` runs its package __init__, which pulls in the GTK tab modules.
# Exposing the adapter lazily keeps ``import operator_console.arm_ops`` free of
# GTK entirely, so the contract, policy, jog and session layers stay importable
# and testable with no display — asserted by
# ``test_the_pure_core_imports_without_gtk``.
_LAZY = {
    "ArmOpsCallbackAdapter": "adapter",
    "context_from_state": "adapter",
}


def __getattr__(name: str):
    module_name = _LAZY.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module

    return getattr(import_module(f".{module_name}", __name__), name)


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_LAZY))


__all__ = [
    "ARM_ACTIONS",
    "ARM_PARAMS_VERSION",
    "BACKEND_REQUIREMENTS",
    "PLANNED_ACTIONS",
    "STOP_REASONS",
    "ArmActionNotRegistered",
    "ArmCommandContext",
    "ArmCommandSession",
    "ArmOpsAck",
    "ArmOpsCallbackAdapter",
    "ArmOpsRequest",
    "ArmParamsError",
    "ArmTransportError",
    "CommandOutcome",
    "ContractGatedTransport",
    "Decision",
    "JogChannel",
    "JogRegistry",
    "NullArmTransport",
    "PendingStop",
    "authorize",
    "context_from_state",
    "registered_actions_from",
    "unregistered_actions",
    "unresolved_requirements",
    "validate_params",
]
