"""The one door arm commands may leave by — and the lock that is still shut.

The console's charter is observation plus a token-gated ops channel.  Nothing
in :mod:`operator_console.arm_ops` opens a socket; a transport here is a thin
wrapper around an already-authenticated submit callable, whose only real job is
to refuse actions the ops contract has not been taught yet.

Why that refusal matters
------------------------
``ops_contract.ACTIONS`` is a closed dict and the broker rejects anything
outside it.  None of the ``robot_arm_*`` actions are in it today.  If this
layer simply handed them to the ops client, every arm command would travel the
network and come back rejected — indistinguishable, from the operator's seat,
from a command the arm refused for a *safety* reason.  Refusing locally, with a
reason that says "not registered", keeps "we have not wired this yet" and "the
arm said no" as two different sentences.

The production default is :class:`NullArmTransport`: it accepts nothing.  A
console that has not been given a real transport cannot emit an arm command,
which is the correct behaviour for the current state of this work.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol, runtime_checkable

from . import contract as K


class ArmTransportError(RuntimeError):
    """The command could not be handed over.  Never a silent drop."""


class ArmActionNotRegistered(ArmTransportError):
    """The ops contract has no entry for this action, so it cannot be sent."""


@runtime_checkable
class ArmOpsTransport(Protocol):
    """What the session needs from whatever actually carries a command."""

    def submit(self, request: K.ArmOpsRequest) -> str:
        """Hand over one request and return its ops ``request_id``.

        Raises :class:`ArmTransportError` when the request cannot be handed
        over.  Returning normally means *submitted*, never *executed* — the
        outcome arrives later as an :class:`~contract.ArmOpsAck`.
        """


class NullArmTransport:
    """Refuses everything.  The production default until the arm path exists."""

    reason = (
        "로봇팔 ops 전송 경로가 아직 연결되지 않았습니다 "
        "(robot_arm_* 액션 미등록)."
    )

    def submit(self, request: K.ArmOpsRequest) -> str:
        raise ArmActionNotRegistered(f"{request.action}: {self.reason}")


@dataclass
class ContractGatedTransport:
    """Passes a request to ``submit`` only if the ops contract knows the action.

    ``registered_actions`` is injected rather than imported: ``operator_console``
    must not take a dependency on the ROS package to answer a question about
    what it is allowed to send.  Pass ``ops_contract.ACTIONS`` (or its keys) at
    the integration point where that module is legitimately in scope; the empty
    default means "nothing is registered", which is both true today and the
    safe thing to assume.
    """

    submit_fn: Callable[[str, Mapping[str, Any], int | None], str]
    registered_actions: frozenset[str] = frozenset()
    # Every refusal is recorded so the reason can be surfaced instead of the
    # command simply not happening.
    refusals: list[tuple[str, str]] = field(default_factory=list)

    def known(self, action: str) -> bool:
        return action in self.registered_actions

    def missing_actions(self) -> tuple[str, ...]:
        return K.unregistered_actions(self.registered_actions)

    def submit(self, request: K.ArmOpsRequest) -> str:
        if not self.known(request.action):
            reason = (
                f"{request.action} 은(는) ops 계약에 등록되지 않았습니다 — "
                "백엔드 어댑터 연결 전에는 전송하지 않습니다."
            )
            self.refusals.append((request.action, reason))
            raise ArmActionNotRegistered(reason)
        return self.submit_fn(
            request.action, dict(request.params), request.expected_state_revision,
        )


def registered_actions_from(ops_contract_module: Any) -> frozenset[str]:
    """Read the registered action names off an ops contract module.

    Keeps the ``ACTIONS`` attribute lookup in one place so the integration
    point is a single call rather than an import spread through the console.
    """
    actions = getattr(ops_contract_module, "ACTIONS", None)
    if actions is None:
        raise ArmTransportError("ops contract module exposes no ACTIONS")
    return frozenset(actions)


@dataclass
class RecordingArmTransport:
    """**Test double only.**  Records requests instead of sending them.

    Deliberately not importable from the package root and deliberately named
    for what it is.  Production code that wants "a transport that does not send"
    must use :class:`NullArmTransport`, which *refuses*; this one accepts, and
    an accepting fake in a production path would look exactly like a working
    command channel while doing nothing.
    """

    accepted: list[K.ArmOpsRequest] = field(default_factory=list)
    next_request_id: int = 0
    fail_with: Exception | None = None

    def submit(self, request: K.ArmOpsRequest) -> str:
        if self.fail_with is not None:
            raise self.fail_with
        self.accepted.append(request)
        self.next_request_id += 1
        return f"test-{self.next_request_id}"

    @property
    def actions(self) -> tuple[str, ...]:
        return tuple(request.action for request in self.accepted)
