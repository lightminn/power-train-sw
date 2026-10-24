"""Optional v1 floor observation grid; fixed bounds, no change to sensor ownership.

One bit represents a 10x10 colour-pixel bin containing at least two projected
samples, all classified as floor. Empty/mixed bins stay transparent. This is a
coarse operator aid, never a traversability or safety mask.
"""
import base64
import binascii
from dataclasses import dataclass
import zlib

WIDTH, HEIGHT = 128, 72
NBYTES = WIDTH*HEIGHT//8
ENCODING = 'zlib-base64-msb-bitset'


@dataclass(frozen=True)
class FloorMask:
    bits: bytes


def parse_floor(value):
    if value is None:
        return None
    if not isinstance(value, dict) or value.get('encoding') != ENCODING:
        raise ValueError('floor_mask encoding invalid')
    if type(value.get('width')) is not int or type(value.get('height')) is not int or (value['width'],value['height']) != (WIDTH,HEIGHT):
        raise ValueError('floor_mask dimensions invalid')
    data = value.get('data')
    if not isinstance(data,str) or len(data)>1536:
        raise ValueError('floor_mask data invalid')
    try:
        packed = base64.b64decode(data,validate=True)
        decoder = zlib.decompressobj()
        bits = decoder.decompress(packed,NBYTES+1)
        if len(bits)!=NBYTES or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
            raise ValueError('floor_mask size invalid')
    except (binascii.Error,zlib.error) as exc:
        raise ValueError('floor_mask data invalid') from exc
    return FloorMask(bits)


def encode_floor(bits):
    if len(bits)!=NBYTES:
        raise ValueError('floor_mask size invalid')
    return dict(encoding=ENCODING,width=WIDTH,height=HEIGHT,
                data=base64.b64encode(zlib.compress(bits)).decode('ascii'))


def project_floor(result, projector):
    import numpy as np
    if result.points is None or result.valid is None or result.floor_mask is None:
        return None
    valid = result.valid
    pixels, kept = projector.project(result.points[valid])
    index = pixels[:,1]//10*WIDTH+pixels[:,0]//10
    total = np.bincount(index,minlength=WIDTH*HEIGHT)
    floor = np.bincount(index,weights=result.floor_mask[valid][kept],minlength=WIDTH*HEIGHT)
    observed = (total>=2)&(floor==total)
    return encode_floor(np.packbits(observed).tobytes())
