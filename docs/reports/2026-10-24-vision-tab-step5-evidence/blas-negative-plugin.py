"""Restore inherited 8 threads immediately before the child imports NumPy."""
import os
import subprocess
import pytest

@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    for item in items:
        if item.name == 'test_console_caps_inherited_blas_before_numpy_import':
            original = subprocess.run
            def broken_run(args, **kwargs):
                args = list(args)
                args[2] = args[2].replace('import operator_console;',
                    'import operator_console; import os; os.environ["OPENBLAS_NUM_THREADS"]="8";')
                return original(args, **kwargs)
            item.obj.__globals__['subprocess'] = type('BrokenSubprocess', (), {'run': staticmethod(broken_run)})
