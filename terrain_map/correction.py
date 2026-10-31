"""Pose correction hook (spec §4.7). A ships IdentityCorrector; B plugs a class in here.

The runtime keeps an SE(2) correction ``delta``: the seed for a frame is
``delta ∘ odom`` and a corrected pose ``P`` updates ``delta ← P ∘ odom⁻¹``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .geometry import IDENTITY, Pose2D, compose, inverse


@dataclass(frozen=True)
class Correction:
    pose: Pose2D
    applied: bool
    score: float | None
    reason: str             # contract.CORRECTOR_REASONS


class PoseCorrector(Protocol):
    name: str

    def correct(self, scan, seed: Pose2D, class_map) -> Correction: ...


class IdentityCorrector:
    """A: the dead-reckoned pose is used as is."""
    name = "identity"

    def correct(self, scan, seed: Pose2D, class_map) -> Correction:
        return Correction(seed, False, None, "identity")


def make_corrector(name: str) -> PoseCorrector:
    if name == "identity":
        return IdentityCorrector()
    raise ValueError(f"corrector {name!r} is not implemented (B starts after the field test, spec §4.7)")


def apply_delta(delta: Pose2D, odom: Pose2D) -> Pose2D:
    return compose(delta, odom)


def update_delta(corrected: Pose2D, odom: Pose2D) -> Pose2D:
    return compose(corrected, inverse(odom))


__all__ = ["Correction", "PoseCorrector", "IdentityCorrector", "make_corrector",
           "apply_delta", "update_delta", "IDENTITY"]
