"""Cache still draws Hangul correctly, survives resizes and remains bounded."""
import cairo
import gi
import pytest

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from operator_console.text_layout import TextLayouts


def test_hangul_layouts_reuse_and_evict_with_style_and_scale_invalidation():
    if not Gtk.init_check()[0]:
        pytest.skip('needs GTK display')
    widget = Gtk.DrawingArea()
    cache = TextLayouts(widget, limit=4)
    cr = cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32, 640, 360))
    first = cache.get(cr, '바닥 · 장애물 미수신', 'Sans 11', 300)
    assert first.get_unknown_glyphs_count() == 0
    assert cache.get(cr, '바닥 · 장애물 미수신', 'Sans 11', 300) is first
    assert cache.get(cr, '바닥 · 장애물 미수신', 'Sans 11', 200) is not first
    cr.scale(2, 2)
    assert cache.get(cr, '바닥 · 장애물 미수신', 'Sans 11', 300) is not first
    for i in range(40):
        cache.get(cr, f'거리 {i} m', 'Sans 11')
    assert len(cache.entries) == 4
    widget.emit('style-updated')
    assert len(cache.entries) == 0
    cache.get(cr, '복구', 'Sans 11')
    widget.notify('scale-factor')
    assert len(cache.entries) == 0
    cache.get(cr, '다시 수신', 'Sans 11')
    widget.destroy()
    assert len(cache.entries) == 0
