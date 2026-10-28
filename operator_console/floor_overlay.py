"""Small optional floor layer on the existing, delay-aligned TerrainCanvas."""
from .terrain import DEPTH_STALE_S, terrain_view
from terrain_overlay.floor import WIDTH, HEIGHT


def display_floor(latest, display, last_rx, now, delay, video_size, video_live, *, view=None):
    if view is None:
        view = terrain_view(latest,display,last_rx,now,delay,video_size,video_live=video_live)
    if (view.probe_state!='LIVE' or display is None or display.status!='OK'
            or not video_live or video_size!=display.frame_size or display.depth_age_ms is None):
        return None
    aged = max(0.,now-delay-display.received_monotonic_s)
    if display.depth_age_ms/1000+aged>DEPTH_STALE_S:
        return None
    return display.floor_mask


class FloorLayer:
    """One 9 KiB alpha surface; decode only when the fixed-size mask changes."""
    def __init__(self):
        self.bits = self.surface = self.data = None
        self.prepared = 0

    def clear(self):
        if self.surface is not None:
            self.surface.finish()
        self.bits = self.surface = self.data = None

    def draw(self, cr, mask, scale, offset_x, offset_y):
        if mask is None:
            self.clear()
            return
        if mask.bits!=self.bits:
            import cairo
            import numpy as np
            self.clear()
            self.bits = mask.bits
            self.data = bytearray((np.unpackbits(np.frombuffer(mask.bits,np.uint8))*255).tobytes())
            self.surface = cairo.ImageSurface.create_for_data(self.data,cairo.FORMAT_A8,WIDTH,HEIGHT,WIDTH)
            self.prepared += 1
        cr.save()
        cr.translate(offset_x,offset_y)
        cr.scale(scale*10,scale*10)
        cr.set_source_rgba(.15,.85,.35,.20)
        import cairo
        pattern=cairo.SurfacePattern(self.surface)
        pattern.set_filter(cairo.FILTER_NEAREST)
        cr.mask(pattern)
        cr.restore()
