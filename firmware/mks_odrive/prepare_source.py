#!/usr/bin/env python3
"""Prepare the pinned MKS archive offline, optionally applying the reviewed patch."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
import subprocess
import zipfile

ZIP_SHA256 = '1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777'
ARCHIVE_ROOT = 'ODrive-fw-v0.5.1'
SOURCE_URL = 'https://github.com/makerbase-motor/MKS-ODrive/tree/e15782976ae93d42b1f0648ceec96503141a343b/Firmware/ODrive_V3.6'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(archive, destination, patch):
    actual = sha256(archive)
    if actual != ZIP_SHA256:
        raise ValueError(f'Archive SHA256 mismatch: expected {ZIP_SHA256}, got {actual}')
    if destination.exists():
        raise FileExistsError(f'Refusing existing destination: {destination}')
    # Validate the entire archive before creating any output; never extract links.
    with zipfile.ZipFile(archive) as bundle:
        for info in bundle.infolist():
            p = PurePosixPath(info.filename)
            mode = info.external_attr >> 16
            if p.is_absolute() or '..' in p.parts or not p.parts or p.parts[0] != ARCHIVE_ROOT or stat.S_ISLNK(mode):
                raise ValueError(f'Unsafe archive member: {info.filename}')
        destination.mkdir(parents=True, exist_ok=False)
        for info in bundle.infolist():
            parts = PurePosixPath(info.filename).parts[1:]
            if not parts:
                continue
            path = destination.joinpath(*parts)
            if info.is_dir():
                path.mkdir(parents=True, exist_ok=True)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(bundle.read(info))
    original = {str(p.relative_to(destination)): sha256(p) for p in sorted(destination.rglob('*')) if p.is_file()}
    if patch:
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1', '-i', str(patch.resolve())], cwd=destination, check=True)
    resulting = {str(p.relative_to(destination)): sha256(p) for p in sorted(destination.rglob('*')) if p.is_file()}
    manifest = {
        'vendor_archive_sha256': actual, 'vendor_source_url': SOURCE_URL,
        'archive_root': ARCHIVE_ROOT, 'patch_sha256': sha256(patch) if patch else None,
        'reliability_patch': 1 if patch else 0,
        'installed_board_binary_equivalence': 'NOT PROVEN',
        'original_file_sha256': original, 'prepared_file_sha256': resulting,
    }
    (destination / 'mks-source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--zip', type=Path, required=True)
    ap.add_argument('--dest', type=Path, required=True, help='New source root, must not exist')
    ap.add_argument('--unpatched', action='store_true', help='Prepare original source for RED tests')
    args = ap.parse_args()
    patch = None if args.unpatched else Path(__file__).with_name('reliability.patch')
    prepare(args.zip, args.dest, patch)
    print(args.dest.resolve())

if __name__ == '__main__':
    main()
