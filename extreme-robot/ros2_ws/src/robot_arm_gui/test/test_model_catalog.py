"""모델 카탈로그 — 프리셋 ∪ 스캔, 그리고 상대경로의 절대경로화."""

import os

from robot_arm_gui.model_catalog import (
    build_catalog, find, resolve_models_dir, workspace_root,
)

PRESETS = {
    'box': {
        'model_path': 'src/robot_arm_perception/models/best.pt',
        'classes': 'box-segmentation',
        'pick_classes': 'box-segmentation',
        'task': 'segment',
    },
    'traffic_light': {
        'model_path': 'src/robot_arm_perception/models/traffic_light_best.pt',
        'classes': 'green light,red light',
        'pick_classes': '',
        'task': 'detect',
    },
}


def make_ws(tmp_path):
    """`src/robot_arm_perception/models` 를 갖춘 가짜 워크스페이스."""
    models = tmp_path / 'src' / 'robot_arm_perception' / 'models'
    models.mkdir(parents=True)
    return str(tmp_path), str(models)


def test_preset_relative_path_becomes_absolute(tmp_path):
    """프리셋 경로는 CWD 상대라 노드를 어디서 띄웠는지에 의존한다 — 그걸 없앤다."""
    root, models = make_ws(tmp_path)
    (tmp_path / 'src/robot_arm_perception/models/best.pt').write_bytes(b'x' * 10)

    catalog = build_catalog(PRESETS, models, root)
    box = find(catalog, 'box')
    assert os.path.isabs(box['path'])
    assert box['path'] == os.path.join(models, 'best.pt')
    assert box['exists'] is True
    assert box['size'] == 10


def test_missing_preset_file_is_listed_but_marked():
    """파일이 없는 프리셋도 목록에는 남긴다 — 화면이 '없음'을 보여줘야 한다."""
    catalog = build_catalog(PRESETS, '/nonexistent', '/ws')
    tl = find(catalog, 'traffic_light')
    assert tl is not None
    assert tl['exists'] is False
    assert tl['size'] is None


def test_scanned_file_appears_without_a_preset(tmp_path):
    """실습 중 파일만 떨궈도 목록에 떠야 한다 — colcon build 없이."""
    root, models = make_ws(tmp_path)
    (tmp_path / 'src/robot_arm_perception/models/새모델.pt').write_bytes(b'y')

    catalog = build_catalog(PRESETS, models, root)
    entry = find(catalog, 'file:새모델.pt')
    assert entry is not None
    assert entry['source'] == 'scan'
    assert entry['exists'] is True
    # 스캔으로는 seg/detect 를 알 수 없다 — 안전한 쪽으로 기본값을 준다.
    assert entry['task'] == 'detect'
    assert entry['classes'] == ''


def test_scan_does_not_duplicate_a_preset_file(tmp_path):
    root, models = make_ws(tmp_path)
    (tmp_path / 'src/robot_arm_perception/models/best.pt').write_bytes(b'z')

    catalog = build_catalog(PRESETS, models, root)
    paths = [e['path'] for e in catalog]
    assert len(paths) == len(set(paths))
    assert find(catalog, 'file:best.pt') is None


def test_only_pt_files_are_scanned(tmp_path):
    """`.engine` 은 입력 크기에 묶여 있어 교체 후보로 내놓지 않는다."""
    root, models = make_ws(tmp_path)
    for name in ('a.pt', 'b.engine', 'c.onnx', 'notes.txt'):
        (tmp_path / 'src/robot_arm_perception/models' / name).write_bytes(b'q')

    catalog = build_catalog(PRESETS, models, root)
    scanned = sorted(e['label'] for e in catalog if e['source'] == 'scan')
    assert scanned == ['a.pt']


def test_scan_of_a_missing_directory_is_not_an_error():
    catalog = build_catalog({}, '/definitely/not/here', '/ws')
    assert catalog == []


# ------------------------------------------------------------ 경로 해석
def test_workspace_root_from_install_share(tmp_path):
    """`<ws>/install/<pkg>/share/<pkg>` 에서 네 단계 올라간다."""
    root, _ = make_ws(tmp_path)
    share = tmp_path / 'install' / 'robot_arm_gui' / 'share' / 'robot_arm_gui'
    share.mkdir(parents=True)
    assert workspace_root(str(share)) == root


def test_workspace_root_walks_up_from_cwd(tmp_path):
    """설치 경로가 예상과 다르면 CWD 에서 위로 올라가며 찾는다."""
    root, models = make_ws(tmp_path)
    assert workspace_root('/somewhere/else', cwd=models) == root


def test_workspace_root_falls_back_to_cwd(tmp_path):
    lonely = tmp_path / 'lonely'
    lonely.mkdir()
    assert workspace_root('', cwd=str(lonely)) == str(lonely)


def test_resolve_models_dir_prefers_the_configured_value(tmp_path):
    root, models = make_ws(tmp_path)
    assert resolve_models_dir('', root) == models
    assert resolve_models_dir('/custom/weights', root) == '/custom/weights'
