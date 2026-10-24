import base64
from dataclasses import replace
import json
import zlib

import numpy as np
import pytest

from terrain_overlay.floor import encode_floor, parse_floor, NBYTES
from terrain_overlay.contract import encode_packet, parse_packet
from terrain_overlay.contours import fit_packet_size
from terrain_overlay.tests.test_contract import ok_payload, error_payload
from terrain_overlay.tests.test_runtime import ready
from terrain_overlay.tests.scenes import default_scene
from terrain_overlay.runtime import RuntimeConfig


def test_real_detector_floor_reaches_same_packet_and_legacy_remains_valid():
    rt=ready();rt.on_depth(default_scene(),111,10);rt._clock.t=10.03
    raw=rt.tick();frame=parse_packet(raw)
    assert len(raw)<=1200 and frame.floor_mask is not None
    cells=np.unpackbits(np.frombuffer(frame.floor_mask.bits,np.uint8)).reshape(72,128)
    assert cells.any() and not cells.all()
    assert parse_packet(encode_packet(ok_payload())).floor_mask is None
    legacy=ready(RuntimeConfig(publish_floor_mask=False))
    legacy.on_depth(default_scene(),111,10);legacy._clock.t=10.03
    assert parse_packet(legacy.tick()).floor_mask is None


@pytest.mark.parametrize('change', [
    {'width':True}, {'height':73}, {'encoding':'other'}, {'data':'not base64!'},
    {'data':base64.b64encode(zlib.compress(b'x'*(NBYTES+100000))).decode()},
    {'data':base64.b64encode(zlib.compress(b'x'*NBYTES)+b'extra').decode()},
])
def test_bad_or_unbounded_floor_is_rejected(change):
    floor=encode_floor(bytes(NBYTES));floor.update(change)
    with pytest.raises(ValueError):parse_floor(floor)


def test_floor_never_displaces_obstacle_data_or_appears_on_error():
    data=ok_payload();data['floor_mask']=encode_floor(bytes(NBYTES))
    fitted=fit_packet_size(data,limit=len(encode_packet(ok_payload())))
    assert fitted['obstacles']==data['obstacles'] and 'floor_mask' not in fitted
    with pytest.raises(ValueError):
        parse_packet(encode_packet(error_payload(floor_mask=data['floor_mask'])))
