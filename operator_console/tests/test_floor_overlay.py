from dataclasses import replace
import time

import cairo
import numpy as np

from operator_console.floor_overlay import display_floor, FloorLayer
from terrain_overlay.contract import encode_packet, parse_packet
from terrain_overlay.floor import encode_floor, NBYTES
from terrain_overlay.tests.test_contract import ok_payload


def test_floor_shares_exact_delay_frame_and_all_video_link_depth_guards():
    now=100.
    legacy=parse_packet(encode_packet(ok_payload()),received_monotonic_s=now)
    current=parse_packet(encode_packet(ok_payload(floor_mask=encode_floor(bytes([255])*NBYTES))),received_monotonic_s=now)
    args=(current,current,now,now,.1,(1280,720),True)
    assert display_floor(*args)==current.floor_mask
    assert display_floor(current,legacy,now,now,.1,(1280,720),True) is None
    assert display_floor(current,current,now,now+.51,0,(1280,720),True) is None
    assert display_floor(current,current,now,now,0,(640,480),True) is None
    assert display_floor(current,current,now,now,0,(1280,720),False) is None
    assert display_floor(current,replace(current,depth_age_ms=700),now,now,0,(1280,720),True) is None


def test_floor_canvas_is_bounded_reuses_mask_and_leaves_unobserved_pixels_clear():
    bits=bytearray(NBYTES);bits[0]=128
    mask=__import__('terrain_overlay.floor',fromlist=['FloorMask']).FloorMask(bytes(bits))
    surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,1280,720);cr=cairo.Context(surface)
    layer=FloorLayer();layer.draw(cr,mask,1,0,0);layer.draw(cr,mask,1,0,0)
    assert layer.prepared==1 and len(layer.data)==128*72
    data=np.frombuffer(surface.get_data(),np.uint8).reshape(720,1280,4)
    assert data[2,2,1]>0 and data[30,30,3]==0
    layer.draw(cr,None,1,0,0)
    assert layer.surface is None and layer.data is None
