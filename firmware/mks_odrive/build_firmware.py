#!/usr/bin/env python3
"""Build the pinned MKS v3.6-56V firmware in a recorded Docker environment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess


VENDOR_ZIP_SHA256 = '1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777'
TRUSTED_IMAGE_ID = 'sha256:977d830e78e15bf17d84d16f17b1c7eb5ad4a6eb4a79cc561cd5221a88e13258'
DEFAULT_IMAGE = TRUSTED_IMAGE_ID
BASE_IMAGE = 'ubuntu:22.04@sha256:2edbbc5dc405e9612ba3584ce95480277e3eb374407b5505fe26f17df77c7dbc'
BOARD_VERSION = 'v3.6-56V'

REFERENCE_ARTIFACTS = {
    'ODriveFirmware.elf': {
        'sha256': 'fd75b613d8fa589c4cf365d390ea885300c61754fa66d3b552a8943123ae22d2',
        'size_bytes': 1102364,
    },
    'ODriveFirmware.bin': {
        'sha256': '7567809465b9646d39b1f9595f8a8e037ed758344f90269ba49038f8c6bfc710',
        'size_bytes': 247880,
    },
}
REFERENCE_SECTIONS = {'text': 246244, 'data': 1580, 'bss': 136064, 'dec': 383888, 'hex': '5db90'}

CONTAINER_COMMANDS = [
    'python3 /tool/prepare_source.py --zip /input/vendor.zip --dest /work/source',
    'cp /work/source/Firmware/tup.config.default /work/source/Firmware/tup.config',
    "printf '\\nCONFIG_BOARD_VERSION=v3.6-56V\\n' >> /work/source/Firmware/tup.config",
    'tup init',
    'tup generate /work/build-generated.sh',
    'bash -e /work/build-generated.sh',
]

CONTAINER_SCRIPT = r'''set -eu
echo '=== PACKAGE_VERSIONS ==='
dpkg-query -W build-essential gcc-arm-none-eabi git libstdc++-arm-none-eabi-newlib patch python3 python3-jinja2 python3-jsonschema python3-yaml tup
echo '=== END_PACKAGE_VERSIONS ==='
echo '=== TOOL_VERSIONS ==='
arm-none-eabi-gcc --version | sed -n '1p'
tup --version | sed -n '1p'
python3 --version
patch --version | sed -n '1p'
echo '=== END_TOOL_VERSIONS ==='
python3 /tool/prepare_source.py --zip /input/vendor.zip --dest /work/source
cd /work/source/Firmware
cp tup.config.default tup.config
grep -Fx 'CONFIG_USB_PROTOCOL=native' tup.config
grep -Fx 'CONFIG_UART_PROTOCOL=ascii' tup.config
grep -Fx 'CONFIG_DEBUG=false' tup.config
printf '\nCONFIG_BOARD_VERSION=v3.6-56V\n' >> tup.config
tup init
tup generate /work/build-generated.sh
bash -e /work/build-generated.sh
test -s build/ODriveFirmware.elf
test -s build/ODriveFirmware.bin
arm-none-eabi-size build/ODriveFirmware.elf | tee /out/firmware-size.txt
install -m 0644 build/ODriveFirmware.elf /out/ODriveFirmware.elf
install -m 0644 build/ODriveFirmware.bin /out/ODriveFirmware.bin
install -m 0644 /work/source/mks-source-manifest.json /out/mks-source-manifest.json
'''


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def validate_request(archive, output):
    archive = archive.expanduser()
    output = output.expanduser()
    if not archive.is_file():
        raise FileNotFoundError(f'Archive is not a file: {archive}')
    actual = sha256_file(archive)
    if actual != VENDOR_ZIP_SHA256:
        raise ValueError(f'Archive SHA256 mismatch: expected {VENDOR_ZIP_SHA256}, got {actual}')
    if output.exists():
        raise FileExistsError(f'Refusing existing output: {output}')
    return actual


def inspect_image_id(image):
    completed = subprocess.run(
        ['docker', 'image', 'inspect', '--format', '{{.Id}}', image],
        check=True, capture_output=True, text=True,
    )
    return completed.stdout.strip()


def verify_image_id(image=DEFAULT_IMAGE):
    actual = inspect_image_id(image)
    if image == DEFAULT_IMAGE and actual != TRUSTED_IMAGE_ID:
        raise RuntimeError(f'Trusted image ID mismatch: expected {TRUSTED_IMAGE_ID}, got {actual}')
    return actual


def extract_section(log, name):
    start = f'=== {name} ===\n'
    end = f'=== END_{name} ==='
    if start not in log or end not in log:
        raise RuntimeError(f'Missing {name} section in build log')
    return log.split(start, 1)[1].split(end, 1)[0].strip().splitlines()


def read_locked_packages(tool_dir):
    entries = {}
    for line in (tool_dir / 'build-packages.lock').read_text().splitlines():
        package, version = line.split('=', 1)
        entries[package] = version
    return entries


def parse_package_versions(lines):
    entries = {}
    for line in lines:
        package, version = line.split('\t', 1)
        entries[package] = version
    return entries


def read_sections(path):
    lines = path.read_text().strip().splitlines()
    if len(lines) != 2:
        raise RuntimeError(f'Unexpected arm-none-eabi-size output: {path}')
    values = lines[1].split()
    if len(values) < 6:
        raise RuntimeError(f'Unexpected arm-none-eabi-size row: {lines[1]}')
    return {
        'text': int(values[0]), 'data': int(values[1]), 'bss': int(values[2]),
        'dec': int(values[3]), 'hex': values[4],
    }


def build(archive, output, image):
    archive = archive.expanduser()
    output = output.expanduser()
    archive_hash = validate_request(archive, output)
    actual_image_id = verify_image_id(image)
    tool_dir = Path(__file__).resolve().parent
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)

    docker_command = [
        'docker', 'run', '--rm', '--network', 'none',
        '--user', f'{os.getuid()}:{os.getgid()}',
        '--tmpfs', '/work:rw,exec,mode=1777',
        '--mount', f'type=bind,src={archive.resolve()},dst=/input/vendor.zip,readonly',
        '--mount', f'type=bind,src={tool_dir},dst=/tool,readonly',
        '--mount', f'type=bind,src={output.resolve()},dst=/out',
        '--entrypoint', '/bin/bash', image, '-lc', CONTAINER_SCRIPT,
    ]
    completed = subprocess.run(
        docker_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    log_path = output / 'build.log'
    log_header = (
        f'image_reference: {image}\n'
        f'image_id: {actual_image_id}\n'
        f'board_version: {BOARD_VERSION}\n'
        f'docker_command: {shlex.join(docker_command[:-1])} <embedded-build-script>\n'
        f'container_commands: {json.dumps(CONTAINER_COMMANDS)}\n'
    )
    log_path.write_text(log_header + completed.stdout)
    if completed.returncode:
        raise RuntimeError(f'Firmware build failed with exit {completed.returncode}; see {log_path}')

    actual_packages = parse_package_versions(extract_section(completed.stdout, 'PACKAGE_VERSIONS'))
    locked_packages = read_locked_packages(tool_dir)
    if actual_packages != locked_packages:
        raise RuntimeError(
            f'Build image package versions do not match build-packages.lock: '
            f'expected {locked_packages}, got {actual_packages}'
        )
    tool_versions = extract_section(completed.stdout, 'TOOL_VERSIONS')
    sections = read_sections(output / 'firmware-size.txt')
    artifacts = {}
    for name, reference in REFERENCE_ARTIFACTS.items():
        path = output / name
        actual = {'sha256': sha256_file(path), 'size_bytes': path.stat().st_size}
        actual['matches_reference_sha256'] = actual['sha256'] == reference['sha256']
        actual['matches_reference_size'] = actual['size_bytes'] == reference['size_bytes']
        artifacts[name] = actual

    source_manifest = output / 'mks-source-manifest.json'
    manifest = {
        'schema': 1,
        'target': {
            'board_version': BOARD_VERSION,
            'usb_protocol': 'native',
            'uart_protocol': 'ascii',
            'debug': False,
        },
        'source': {
            'vendor_archive_sha256': archive_hash,
            'reliability_patch_sha256': sha256_file(tool_dir / 'reliability.patch'),
            'preparation_manifest': source_manifest.name,
            'preparation_manifest_sha256': sha256_file(source_manifest),
        },
        'environment': {
            'image_reference': image,
            'image_id': actual_image_id,
            'trusted_reference_image_id': TRUSTED_IMAGE_ID,
            'matches_trusted_reference_image': actual_image_id == TRUSTED_IMAGE_ID,
            'dockerfile_declared_base_image': BASE_IMAGE,
            'package_versions': actual_packages,
            'tool_versions': tool_versions,
            'dockerfile_rebuild_bit_identity': 'NOT PROVEN',
        },
        'commands': CONTAINER_COMMANDS,
        'artifacts': artifacts,
        'elf_sections': sections,
        'reference_comparison': {
            'reference_artifacts': REFERENCE_ARTIFACTS,
            'reference_elf_sections': REFERENCE_SECTIONS,
            'matches_reference_elf_sections': sections == REFERENCE_SECTIONS,
            'scope': 'Same pinned source, patch, board config, and trusted image; comparison is not a cross-environment reproducibility guarantee.',
        },
        'flash_performed': False,
        'installed_board_binary_equivalence': 'NOT PROVEN',
    }
    manifest_path = output / 'mks-build-manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    with log_path.open('a') as stream:
        stream.write('\n=== ARTIFACT_SUMMARY ===\n')
        for name, values in artifacts.items():
            stream.write(f"{name}\t{values['size_bytes']}\t{values['sha256']}\n")
        stream.write(f'elf_sections\t{json.dumps(sections, sort_keys=True)}\n')
        stream.write(f'manifest\t{manifest_path.name}\n')
    (output / 'firmware-size.txt').unlink()
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip', type=Path, required=True, help='Pinned vendor ZIP')
    parser.add_argument('--output', type=Path, required=True, help='New output directory; must not exist')
    parser.add_argument(
        '--image', default=DEFAULT_IMAGE,
        help='Docker image reference; custom images must match build-packages.lock',
    )
    args = parser.parse_args()
    manifest = build(args.zip, args.output, args.image)
    for name, values in manifest['artifacts'].items():
        print(f"{name}: {values['size_bytes']} bytes sha256={values['sha256']}")
    print(args.output.resolve())


if __name__ == '__main__':
    main()
