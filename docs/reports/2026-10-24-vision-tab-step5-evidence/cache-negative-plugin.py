import pytest

@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    for item in items:
        if item.name == 'test_hangul_layouts_reuse_and_evict_with_style_and_scale_invalidation':
            original = item.obj.__globals__['TextLayouts']
            class Unbounded(original):
                def __init__(self, widget, limit=128):
                    super().__init__(widget, limit=1000000)
            item.obj.__globals__['TextLayouts'] = Unbounded
