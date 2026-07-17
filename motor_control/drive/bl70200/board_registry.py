"""Load the BL70200 board-serial to CAN-node registry.

The registry is a JSON object whose values are ``[axis0_node, axis1_node]``.
Keeping this parser independent from ODrive and CAN libraries makes registry
validation safe to run on development hosts.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping


def load(path: str | Path) -> dict[str, tuple[int, int]]:
    """Load and validate a serial-to-node-pair JSON registry.

    Missing files, invalid JSON, invalid entries, and a node assigned more than
    once all use the public ``ValueError`` contract.
    """

    registry_path = Path(path)
    try:
        raw = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid board registry {registry_path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ValueError("invalid board registry: expected a JSON object")

    registry: dict[str, tuple[int, int]] = {}
    used_nodes: dict[int, str] = {}
    for serial, nodes in raw.items():
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError("invalid board registry: serial must be a non-empty string")
        if not isinstance(nodes, (list, tuple)) or len(nodes) != 2:
            raise ValueError(
                f"invalid board registry entry for {serial!r}: expected two nodes"
            )
        if any(isinstance(node, bool) or not isinstance(node, int) for node in nodes):
            raise ValueError(
                f"invalid board registry entry for {serial!r}: nodes must be integers"
            )
        if any(node < 0 or node > 63 for node in nodes):
            raise ValueError(
                f"invalid board registry entry for {serial!r}: node outside 0..63"
            )

        pair = (nodes[0], nodes[1])
        for node in pair:
            if node in used_nodes:
                raise ValueError(
                    f"duplicate node {node} in board registry: "
                    f"{used_nodes[node]!r} and {serial!r}"
                )
            used_nodes[node] = serial
        registry[serial] = pair

    return registry


def resolve_node(
    registry: Mapping[str, tuple[int, int]], serial: str, axis: int | str
) -> int:
    """Resolve one board axis to its CAN node or raise ``ValueError``."""

    if serial not in registry:
        raise ValueError(f"unknown board serial: {serial!r}")
    if axis in (0, "0"):
        axis_index = 0
    elif axis in (1, "1"):
        axis_index = 1
    else:
        raise ValueError(f"invalid axis {axis!r}: expected 0 or 1")

    try:
        return registry[serial][axis_index]
    except (IndexError, TypeError) as exc:
        raise ValueError(f"invalid board registry entry for {serial!r}") from exc
