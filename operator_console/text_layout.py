"""Bounded, per-widget Pango cache; use only on the GTK drawing thread."""
from collections import OrderedDict

import gi

gi.require_version('PangoCairo', '1.0')
gi.require_version('Pango', '1.0')
from gi.repository import Pango, PangoCairo


class TextLayouts:
    def __init__(self, widget, limit=128):
        self.limit = limit
        self.entries = OrderedDict()
        widget.connect('style-updated', self.clear)
        widget.connect('notify::scale-factor', self.clear)
        widget.connect('destroy', self.clear)

    def clear(self, *_args):
        self.entries.clear()

    def get(self, context, text, font, width=None):
        matrix = context.get_matrix()
        key = (text, font, width, matrix.xx, matrix.xy, matrix.yx, matrix.yy,
               context.get_target().get_device_scale())
        layout = self.entries.pop(key, None)
        if layout is None:
            layout = PangoCairo.create_layout(context)
            layout.set_font_description(Pango.FontDescription(font))
            if width is not None:
                layout.set_width(int(max(40., width)*Pango.SCALE))
                layout.set_ellipsize(Pango.EllipsizeMode.END)
            layout.set_text(text, -1)
        self.entries[key] = layout
        while len(self.entries) > self.limit:
            self.entries.popitem(last=False)
        return layout
