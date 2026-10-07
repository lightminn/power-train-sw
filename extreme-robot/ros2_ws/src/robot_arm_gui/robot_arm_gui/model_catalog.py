#!/usr/bin/env python3
"""YOLO 가중치 목록 — 프리셋 ∪ 디렉터리 스캔. ROS 비의존(→ pytest 가능).

## 왜 스캔까지 하나

`model_presets.MODEL_PRESETS` 만 노출하면, 실습 중에 새로 받은 `best.pt` 를 쓰려고
매번 파이썬 소스를 고치고 `colcon build` 를 해야 한다. 그게 바로 이번에 없애려는
병목이다. 그래서 **파일을 디렉터리에 떨구기만 하면 목록에 뜨도록** 스캔을 더한다.

프리셋에 없는 파일은 `classes`/`pick_classes` 를 모르므로 화면에서 입력받는다
(비우면 필터 없음 = 전체 통과).

## 왜 절대경로로 바꾸나

프리셋의 `model_path` 는 **CWD 상대**다(`src/robot_arm_perception/models/best.pt`).
즉 `perception_node` 를 워크스페이스 루트가 아닌 곳에서 띄우면 모델을 못 찾고 죽는다 —
`camera_calib.launch.py` 가 `cwd=` 를 계산해 넘기고 `run_vision_test.sh` 가 `cd` 를
하는 이유다. GUI 는 워크스페이스 루트 기준으로 **절대경로화해서** 넘기므로, 그
의존이 통째로 사라진다.
"""

import os

#: 가중치 확장자. `.engine` 은 입력 크기에 묶여 있어 목록에 넣지 않는다
#: (교체는 항상 `.pt` 로 하고, 필요하면 노드가 엔진을 다시 굽는다).
WEIGHT_SUFFIXES = ('.pt',)

#: 워크스페이스 루트 판정 근거 — 이 디렉터리가 있으면 루트로 본다.
_ROOT_MARKER = os.path.join('src', 'robot_arm_perception')

#: models_dir 기본 위치(워크스페이스 루트 기준 상대).
DEFAULT_MODELS_SUBDIR = os.path.join('src', 'robot_arm_perception', 'models')


def workspace_root(share_dir=None, cwd=None):
    """설치 경로에서 워크스페이스 루트를 역산한다.

    `camera_calib.launch.py:_workspace_root()` 와 같은 규칙이다 —
    `<ws>/install/<pkg>/share/<pkg>` 에서 네 단계 올라간다.
    """
    if share_dir:
        candidate = os.path.abspath(os.path.join(share_dir, *(['..'] * 4)))
        if os.path.isdir(os.path.join(candidate, _ROOT_MARKER)):
            return candidate
    here = os.path.abspath(cwd or os.getcwd())
    while True:
        if os.path.isdir(os.path.join(here, _ROOT_MARKER)):
            return here
        parent = os.path.dirname(here)
        if parent == here:
            return os.path.abspath(cwd or os.getcwd())
        here = parent


def resolve_models_dir(configured, root):
    if configured:
        return os.path.abspath(configured)
    return os.path.join(root, DEFAULT_MODELS_SUBDIR)


def _describe(path):
    try:
        st = os.stat(path)
    except OSError:
        return {'exists': False, 'size': None, 'mtime': None}
    return {'exists': True, 'size': int(st.st_size), 'mtime': float(st.st_mtime)}


def build_catalog(presets, models_dir, root):
    """`[{key,label,path,exists,task,classes,pick_classes,source,size,mtime}]`.

    프리셋이 먼저 오고, 프리셋이 가리키지 않는 스캔 결과가 뒤따른다.
    같은 파일을 둘 다 가리키면 프리셋 쪽만 남긴다(메타데이터가 더 많다).
    """
    entries = []
    claimed = set()

    for key in sorted(presets):
        preset = presets[key]
        path = preset.get('model_path', '')
        if path and not os.path.isabs(path):
            path = os.path.join(root, path)
        path = os.path.abspath(path) if path else ''
        claimed.add(path)
        entry = {
            'key': key,
            'label': f'{key} ({os.path.basename(path) or "경로 없음"})',
            'path': path,
            'task': preset.get('task', 'detect'),
            'classes': preset.get('classes', ''),
            'pick_classes': preset.get('pick_classes', ''),
            'source': 'preset',
        }
        entry.update(_describe(path))
        entries.append(entry)

    for path in _scan(models_dir):
        if path in claimed:
            continue
        name = os.path.basename(path)
        entry = {
            # 프리셋 키와 절대 겹치지 않게 접두어를 붙인다.
            'key': f'file:{name}',
            'label': name,
            'path': path,
            # 스캔으로는 seg/detect 를 알 수 없다. 화면이 고르게 하고, 기본은
            # 안전한 쪽(detect)으로 둔다 — seg 모델을 detect 로 열면 마스크가
            # 없을 뿐이지만, 반대로 열면 ultralytics 가 예외를 낸다.
            'task': 'detect',
            'classes': '',
            'pick_classes': '',
            'source': 'scan',
        }
        entry.update(_describe(path))
        entries.append(entry)

    return entries


def _scan(models_dir):
    try:
        names = sorted(os.listdir(models_dir))
    except OSError:
        return []
    return [os.path.join(models_dir, n) for n in names
            if n.endswith(WEIGHT_SUFFIXES) and
            os.path.isfile(os.path.join(models_dir, n))]


def find(catalog, key):
    for entry in catalog:
        if entry['key'] == key:
            return entry
    return None
