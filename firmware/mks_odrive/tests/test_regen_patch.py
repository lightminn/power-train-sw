#!/usr/bin/env python3
"""regen_patch must reproduce a source tree exactly through prepare_source."""
import os
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import prepare_source  # noqa: E402
import regen_patch  # noqa: E402

ARCHIVE = Path(os.environ.get('MKS_VENDOR_ZIP', ''))


def tree(root):
    return {str(p.relative_to(root)): p.read_bytes() for p in sorted(root.rglob('*'))
            if p.is_file() and p.name != 'mks-source-manifest.json'}


@unittest.skipUnless(ARCHIVE.is_file(), 'set MKS_VENDOR_ZIP to the pinned vendor ZIP')
class RegenPatchTest(unittest.TestCase):
    def test_regenerated_patch_reproduces_work_tree(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            prepare_source.prepare(ARCHIVE, temp / 'original', None)
            prepare_source.prepare(ARCHIVE, temp / 'work', HERE.parent / 'reliability.patch')
            out = temp / 'regen.patch'
            regen_patch.regenerate(temp / 'original', temp / 'work', out)
            prepare_source.prepare(ARCHIVE, temp / 'check', out)
            self.assertEqual(tree(temp / 'work'), tree(temp / 'check'))

    def test_file_list_matches_committed_patch(self):
        text = (HERE.parent / 'reliability.patch').read_text()
        named = [line[6:] for line in text.splitlines() if line.startswith('+++ b/')]
        self.assertEqual(named, regen_patch.PATCH_FILES)


if __name__ == '__main__':
    unittest.main(verbosity=2)
