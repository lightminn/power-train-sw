#!/usr/bin/env python3
"""Safety-contract tests for the reproducible firmware build entrypoint."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).parents[1] / 'build_firmware.py'
spec = importlib.util.spec_from_file_location('build_firmware', MODULE_PATH)
build_firmware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build_firmware)


class BuildRequestTests(unittest.TestCase):
    def test_mismatched_archive_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            archive = root / 'vendor.zip'
            output = root / 'build-output'
            archive.write_bytes(b'not the pinned archive')

            with self.assertRaisesRegex(ValueError, 'Archive SHA256 mismatch'):
                build_firmware.validate_request(archive, output)

            self.assertFalse(output.exists())

    def test_existing_output_is_rejected_and_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            archive = root / 'vendor.zip'
            archive.write_bytes(b'test fixture')
            output = root / 'build-output'
            output.mkdir()
            sentinel = output / 'user-work.txt'
            sentinel.write_text('preserve')

            with mock.patch.object(
                    build_firmware, 'sha256_file',
                    return_value=build_firmware.VENDOR_ZIP_SHA256):
                with self.assertRaisesRegex(FileExistsError, 'existing output'):
                    build_firmware.validate_request(archive, output)

            self.assertEqual(sentinel.read_text(), 'preserve')

    def test_wrong_local_image_is_rejected(self):
        completed = mock.Mock(stdout='sha256:wrong\n')
        with mock.patch.object(build_firmware.subprocess, 'run', return_value=completed):
            with self.assertRaisesRegex(RuntimeError, 'image ID mismatch'):
                build_firmware.verify_image_id()


if __name__ == '__main__':
    unittest.main(verbosity=2)
