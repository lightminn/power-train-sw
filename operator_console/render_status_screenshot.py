"""Render the real robot-status GTK page with contract-valid fixture packets."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import tempfile

import gi

gi.require_version("Gdk", "3.0")
gi.require_version("GdkPixbuf", "2.0")
gi.require_version("Gtk", "3.0")
gi.require_version("Gst", "1.0")
from gi.repository import Gdk, GdkPixbuf, GLib, Gst, Gtk  # noqa: E402

from .app import OperatorConsole
from .runtime_smoke import (
    _arm_payload,
    _chassis_payload,
    _metadata_payload,
    _telemetry_payload,
)


def _free_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--page", choices=("mission", "systems"), default="systems")
    parser.add_argument(
        "--status-panel",
        choices=("drive", "power", "safety", "network", "ai", "arm"),
        default="drive",
    )
    parser.add_argument("--scroll-y", type=float, default=0.0)
    args = parser.parse_args()
    Gst.init(None)
    ports = {name: _free_port() for name in ("power", "chassis", "metadata", "arm")}
    with tempfile.NamedTemporaryFile(mode="w", delete=False) as token:
        token.write("screenshot-only-token")
        token_path = token.name
    window = OperatorConsole(
        "127.0.0.1", _free_port(), _free_port(), ports["metadata"], 60,
        ports["power"], ports["chassis"], ports["arm"],
        ops_host="127.0.0.1", ops_port=_free_port(),
        ops_token_file=token_path,
    )
    window.set_decorated(False)
    window.resize(args.width, args.height)
    window.move(0, 0)
    window.show_all()
    window._stack.set_visible_child_name(args.page)
    if args.page == "systems":
        window._robot_status._card_buttons[args.status_panel].clicked()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0

    def publish() -> bool:
        nonlocal sequence
        sequence += 1
        for name, builder in (
            ("power", _telemetry_payload),
            ("chassis", _chassis_payload),
            ("metadata", _metadata_payload),
            ("arm", _arm_payload),
        ):
            sender.sendto(
                json.dumps(builder(sequence)).encode("utf-8"),
                ("127.0.0.1", ports[name]),
            )
        return True

    def capture() -> bool:
        gdk_window = window.get_window()
        source_width = gdk_window.get_width()
        source_height = gdk_window.get_height()
        pixbuf = Gdk.pixbuf_get_from_window(
            gdk_window, 0, 0, source_width, source_height,
        )
        if pixbuf is None:
            raise RuntimeError("GTK window capture returned no pixels")
        if pixbuf.get_width() != args.width or pixbuf.get_height() != args.height:
            pixbuf = pixbuf.scale_simple(
                args.width, args.height,
                GdkPixbuf.InterpType.BILINEAR,
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        pixbuf.savev(str(args.output), "png", [], [])
        window.destroy()
        return False

    publish()
    GLib.timeout_add(200, publish)
    if args.page == "systems" and args.scroll_y > 0:
        GLib.timeout_add(
            1500,
            lambda: (
                window._systems_scroll.get_vadjustment().set_value(args.scroll_y),
                False,
            )[1],
        )
    GLib.timeout_add(2200 if args.scroll_y > 0 else 1800, capture)
    Gtk.main()
    sender.close()
    Path(token_path).unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
