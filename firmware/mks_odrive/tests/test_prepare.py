#!/usr/bin/env python3
"""Offline checks; full replay needs --zip or MKS_VENDOR_ZIP."""
import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

PYTHON = sys.executable
ARCHIVE = Path(os.environ['MKS_VENDOR_ZIP']) if os.environ.get('MKS_VENDOR_ZIP') else None

spec = importlib.util.spec_from_file_location('prepare_source', Path(__file__).parents[1] / 'prepare_source.py')
prepare_source = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_source)

class PreparationTests(unittest.TestCase):
    def test_mismatched_archive_rejected_before_creating_destination(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            archive = root / 'bad.zip'
            archive.write_bytes(b'not the pinned archive')
            with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
                prepare_source.prepare(archive, root / 'output', None)
            self.assertFalse((root / 'output').exists())

    def test_existing_destination_is_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d)
            sentinel = dest / 'user-work.txt'
            sentinel.write_text('preserve')
            # Isolate destination preservation from the preceding archive hash
            # check; archive authenticity is exercised by the other tests.
            with patch.object(prepare_source, 'sha256', return_value=prepare_source.ZIP_SHA256):
                with self.assertRaises(FileExistsError):
                    prepare_source.prepare(dest / 'unused.zip', dest, None)
            self.assertEqual(sentinel.read_text(), 'preserve')

    def test_offline_patch_replays_and_preserves_config_layout(self):
        if ARCHIVE is None:
            self.skipTest('Full offline replay requires --zip or MKS_VENDOR_ZIP; not qualified without the pinned archive')
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / 'source'
            patch = Path(__file__).parents[1] / 'reliability.patch'
            manifest = prepare_source.prepare(ARCHIVE, dest, patch)
            self.assertEqual(manifest['vendor_archive_sha256'], prepare_source.ZIP_SHA256)
            self.assertEqual(manifest['reliability_patch'], 1)
            self.assertEqual(manifest['installed_board_binary_equivalence'], 'NOT PROVEN')
            self.assertIn('c_getter: get_hal_error()', (dest / 'Firmware/odrive-interface.yaml').read_text())
            changed = {p for p, h in manifest['prepared_file_sha256'].items() if manifest['original_file_sha256'].get(p) != h}
            expected = {
                'Firmware/communication/interface_can.cpp', 'Firmware/communication/interface_can.hpp',
                'Firmware/communication/can_simple.cpp', 'Firmware/MotorControl/axis.hpp',
                'Firmware/MotorControl/axis.cpp', 'Firmware/MotorControl/low_level.cpp',
                'Firmware/odrive-interface.yaml', 'Firmware/Tupfile.lua',
                'tools/odrive/mks_reliability_version.py',
            }
            self.assertEqual(changed, expected)
            # Compare the actual serialized config struct bodies against pinned input.
            def config_body(text):
                start = text.index('struct Config_t {')
                depth, end = 1, text.index('{', start) + 1
                while depth:
                    depth += (text[end] == '{') - (text[end] == '}')
                    end += 1
                return text[start:end]
            with zipfile.ZipFile(ARCHIVE) as archive:
                for name in ['Firmware/MotorControl/axis.hpp', 'Firmware/communication/interface_can.hpp']:
                    old = archive.read('ODrive-fw-v0.5.1/' + name).decode()
                    self.assertEqual(config_body(old), config_body((dest / name).read_text()))
            # Actual protocol tests against a fresh, offline patch application.
            subprocess.run([PYTHON, str(Path(__file__).with_name('run_source_tests.py')), str(dest)], check=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--zip', type=Path, default=ARCHIVE)
    args = parser.parse_args()
    ARCHIVE = args.zip
    unittest.main(argv=[sys.argv[0]], verbosity=2)
