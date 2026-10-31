"""Remove leaky behavior from the real queue; overload gate must reject it."""
import pytest

@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    for item in items:
        if item.name == 'test_decoded_frame_overload_discards_old_frames_and_bounds_memory':
            namespace = item.obj.__globals__
            original = namespace['pipeline_description']
            namespace['pipeline_description'] = lambda *a, **kw: original(*a, **kw).replace(
                'leaky=downstream', 'leaky=no')
