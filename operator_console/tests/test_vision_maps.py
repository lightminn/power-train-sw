"""Display criteria, bounded reuse of map code, and worker lifecycle contracts."""
from dataclasses import replace
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from operator_console.terrain_map import DisplayConfig, LatestMapReceiver
from operator_console.runtime_smoke import _terrain_map_packets
from operator_console.vision_config import DEFAULT_CRITERIA, resolve_criteria
from operator_console.vision_render import MapRenderWorker, MapSettings, VisionRenderer, live_map


def wait_until(predicate, timeout=3):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.01)
    return predicate()


@pytest.fixture
def receiver():
    r = LatestMapReceiver(0)
    now = time.monotonic()
    for p in _terrain_map_packets(1):
        r.ingest(p,now)
    yield r
    r.close()


def test_criteria_precedence_relative_config_and_v02_fallback(tmp_path):
    file = tmp_path/'criteria.json'
    file.write_bytes(DEFAULT_CRITERIA.read_bytes())
    invalid_env = {'L515_CRITERIA': '/absent', 'L515_CRITERIA_PROFILE': 'absent'}
    criteria = resolve_criteria(config={'l515_criteria':'criteria.json','l515_criteria_profile':'A_no_stop'},
        config_dir=tmp_path, environ=invalid_env)
    assert criteria.path == str(file)
    assert criteria.version == 'v0.2'
    assert len(criteria.sha256) == 64
    assert 'trench_depth_m' not in criteria.rules
    assert resolve_criteria(DEFAULT_CRITERIA, 'B_stop20', config={'l515_criteria':'/absent'},
        environ=invalid_env).profile == 'B_stop20'
    assert resolve_criteria(environ={'L515_CRITERIA':str(file)}).path == str(file)
    # Replacing the external file changes the rule/hash without changing program code.
    data=json.loads(file.read_text())
    data['criteria_version']='final-fixture'
    file.write_text(json.dumps(data))
    updated=resolve_criteria(file,environ={})
    assert updated.sha256 != criteria.sha256 and updated.version=='final-fixture'


@pytest.mark.parametrize('value', [True, '', 4])
def test_invalid_criteria_path_is_an_explicit_error(value):
    with pytest.raises(ValueError):
        resolve_criteria(value)


def test_bad_rules_and_unknown_profile_are_rejected(tmp_path):
    with pytest.raises(ValueError,match='프로필'):
        resolve_criteria(DEFAULT_CRITERIA,'missing')
    for rule in ({'kind':'higher_is_worse','caution_from':.2,'block_from':.1},
                 {'kind':'higher_is_worse','caution_from':float('nan'),'block_from':.1},
                 {'kind':'higher_is_worse','caution_from':True,'block_from':.1}):
        file=tmp_path/'bad.json'
        file.write_text(json.dumps({'rules':{'step_up_m':rule}}))
        with pytest.raises(ValueError):
            resolve_criteria(file)


def test_null_optional_json_criteria_still_uses_environment_profile():
    selected = resolve_criteria(config={'l515_criteria':None, 'l515_criteria_profile':None},
        environ={'L515_CRITERIA':str(DEFAULT_CRITERIA), 'L515_CRITERIA_PROFILE':'B_stop20'})
    assert selected.profile=='B_stop20'


def test_real_render_reuses_data_and_never_invents_missing_pose(receiver):
    now=time.monotonic()
    snap=receiver.snapshot(now)
    render=VisionRenderer(DisplayConfig(),resolve_criteria())
    images=render.render(snap,MapSettings(),now)
    assert images.top and images.oblique
    pixels=np.frombuffer(images.top.bgra,np.uint8).reshape(images.top.height,images.top.width,4)
    assert np.unique(pixels.reshape(-1,4),axis=0).shape[0]>3
    assert ((pixels[...,2]>200)&(pixels[...,1]>180)&(pixels[...,0]<100)).any()  # live rover arrow
    missing=replace(snap,status=replace(snap.status,pose=None,status='NO_POSE'))
    assert render.render(missing,MapSettings(),now).oblique is None
    stale=render.render(snap,MapSettings(),now+.6)
    assert stale.top is not None and stale.oblique is None
    pixels=np.frombuffer(stale.top.bgra,np.uint8).reshape(stale.top.height,stale.top.width,4)
    assert not ((pixels[...,2]>200)&(pixels[...,1]>180)&(pixels[...,0]<100)).any()


def test_distant_tiles_are_clipped_before_dense_oblique_grid(receiver,monkeypatch):
    now=time.monotonic(); snap=receiver.snapshot(now)
    far=replace(snap.tiles[0],tx=10_000_000,ty=-10_000_000)
    sizes=[]; original=live_map.tmap_local_grid
    def bounded(snap,*args,**kwargs):
        sizes.append(len(snap.tiles))
        return original(snap,*args,**kwargs)
    monkeypatch.setattr(live_map,'tmap_local_grid',bounded)
    result=VisionRenderer(DisplayConfig(),resolve_criteria()).render(
        replace(snap,tiles=(*snap.tiles,far)),MapSettings(),now)
    assert result.oblique is not None and sizes==[1]


def test_policy_hazards_are_not_vetoed_by_sender_classes_or_called_pass_after_filter(receiver,monkeypatch):
    now=time.monotonic(); snap=receiver.snapshot(now)
    tile=snap.tiles[0]
    heights=np.zeros((16,16),np.int16)
    heights[8,10]=20
    # Sender FREE does not imply pass under an independently selected display policy.
    tile=replace(tile,heights=heights.ravel(),codes=np.zeros_like(tile.codes))
    from terrain_map import contract as C
    tile=replace(tile,codes=np.full_like(tile.codes,C.FREE))
    captured=[]; original=live_map.cell_colors
    def capture(z,known,levels,*args):
        captured.append((z.copy(),known.copy(),levels.copy()))
        return original(z,known,levels,*args)
    monkeypatch.setattr(live_map,'cell_colors',capture)
    render=VisionRenderer(DisplayConfig(),resolve_criteria())
    render.render(replace(snap,tiles=(tile,)),MapSettings(),now)
    z,known,levels=captured[-1]
    hazard=z>.1
    assert hazard.any() and not known[hazard].any() and (levels[hazard]==0).all()
    heights[8,11]=20
    render.render(replace(snap,tiles=(replace(tile,heights=heights.ravel()),)),MapSettings(),now)
    z,known,levels=captured[-1]
    assert ((levels>=2)&known).any()


def test_worker_off_main_at_two_hz_sleeps_hidden_and_restarts(receiver,monkeypatch):
    threads=[]; times=[]
    original=VisionRenderer.render
    def capture(self,*args):
        threads.append(threading.get_ident()); times.append(time.monotonic())
        return original(self,*args)
    monkeypatch.setattr(VisionRenderer,'render',capture)
    worker=MapRenderWorker(receiver,DisplayConfig(),resolve_criteria())
    try:
        worker.configure(active=True)
        assert wait_until(lambda:worker.render_count>=3)
        assert set(threads)=={worker._thread.ident}
        assert worker._thread.ident!=threading.get_ident()
        assert all(.45<=b-a<1 for a,b in zip(times,times[1:]))
        worker.configure(active=False)
        count=worker.render_count
        time.sleep(.65)
        assert worker.render_count==count and worker.latest()[1] is None
        worker.configure(active=True)
        assert wait_until(lambda:worker.render_count>count)
    finally:
        worker.close()
    assert wait_until(lambda:not worker._thread.is_alive())


def test_hidden_inflight_result_cannot_be_published_after_remap(receiver,monkeypatch):
    started=threading.Event(); release=threading.Event()
    original=VisionRenderer.render
    def slow(self,*args):
        started.set(); release.wait(2)
        return original(self,*args)
    monkeypatch.setattr(VisionRenderer,'render',slow)
    worker=MapRenderWorker(receiver,DisplayConfig(),resolve_criteria())
    try:
        worker.configure(active=True)
        assert started.wait(2)
        worker.configure(active=False)
        worker.configure(replace(MapSettings(),scale=.25),active=True)
        release.set()
        # Old job's generation cannot become the new frame.
        assert wait_until(lambda:worker.render_count>=1)
        assert '눈금 0.1 m' in worker.latest()[1].top_note
        assert worker.render_count==1
    finally:
        release.set(); worker.close()


def test_status_updates_between_rasters_without_rendering_more_or_polling_hidden(receiver):
    worker=MapRenderWorker(receiver,DisplayConfig(),resolve_criteria())
    try:
        worker.configure(active=True)
        assert wait_until(lambda:worker.render_count>=1)
        count=worker.render_count
        now=time.monotonic()
        for raw in _terrain_map_packets(2):
            receiver.ingest(raw,now)
        assert wait_until(lambda:worker.latest_status() is not None and
                          worker.latest_status().sequence==2,timeout=.3)
        assert worker.render_count==count  # status must not wait for the next 2 Hz image
        worker.configure(active=False)
        assert worker.latest_status() is None
        for raw in _terrain_map_packets(3):
            receiver.ingest(raw,time.monotonic())
        time.sleep(.2)
        assert worker.latest_status() is None and worker.render_count==count
        worker.configure(active=True)
        assert wait_until(lambda:worker.latest_status() is not None and worker.latest_status().sequence==3)
    finally:
        worker.close()
