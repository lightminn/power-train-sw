"""D435 metadata labels must render Korean class names (xvfb-run + system python).

cairo's toy text API (``show_text``) has no font fallback, so Hangul came out as
missing-glyph boxes; labels go through PangoCairo like the terrain badge.
"""
import json
import os
import time

os.environ.setdefault("GDK_BACKEND", "x11")

import cairo  # noqa: E402
import gi  # noqa: E402
import pytest  # noqa: E402

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.demo_source import build_payloads  # noqa: E402
from operator_console.metadata import displayable_detections, parse_metadata  # noqa: E402


class RecordingContext(cairo.Context):
    def __init__(self, surface):
        super().__init__()
        self.toy_text = []

    def show_text(self, text):
        self.toy_text.append(text)
        super().show_text(text)


class FakeReceiver:
    def __init__(self, frame):
        self._frame = frame

    def latest(self):
        return self._frame


@pytest.fixture
def korean_frame():
    if not Gtk.init_check()[0]:
        pytest.skip("display unavailable")
    raw = json.dumps(build_payloads(0, 0.0)["metadata"]).encode()
    frame = parse_metadata(raw, received_monotonic_s=time.monotonic())
    assert any("구조" in d.class_name for d in displayable_detections(frame))
    return frame


def test_korean_labels_do_not_use_the_cairo_toy_text_api(korean_frame):
    from operator_console.app import MetadataCanvas
    canvas = MetadataCanvas(FakeReceiver(korean_frame))
    canvas.set_video_size(korean_frame.width, korean_frame.height)
    context = RecordingContext(cairo.ImageSurface(cairo.FORMAT_ARGB32, 640, 360))
    canvas._on_draw(canvas, context)
    assert context.toy_text == []


def test_korean_label_layout_has_no_missing_glyphs(korean_frame):
    from operator_console.app import MetadataCanvas
    canvas = MetadataCanvas(FakeReceiver(korean_frame))
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 400, 60)
    layout = canvas._label_layout(cairo.Context(surface), "구조 대상 물체  92%  1.23 m")
    assert layout.get_unknown_glyphs_count() == 0
