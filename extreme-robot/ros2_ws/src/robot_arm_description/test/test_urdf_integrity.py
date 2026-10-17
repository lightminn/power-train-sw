"""고정 로봇 모델의 중복·누락·비연결 병합 손상을 막는다."""

from collections import Counter, defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET


URDF_DIR = Path(__file__).parents[1] / 'urdf'


def _assert_tree(path):
    root = ET.parse(path).getroot()
    links = [item.get('name') for item in root.findall('link')]
    joints = root.findall('joint')
    joint_names = [item.get('name') for item in joints]

    assert len(links) == len(set(links)), 'duplicate link name'
    assert len(joint_names) == len(set(joint_names)), 'duplicate joint name'

    link_set = set(links)
    children = []
    graph = defaultdict(list)
    for joint in joints:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        assert parent in link_set, f'missing parent link: {parent}'
        assert child in link_set, f'missing child link: {child}'
        children.append(child)
        graph[parent].append(child)

    duplicate_children = {
        name for name, count in Counter(children).items() if count > 1}
    assert not duplicate_children, f'multiple parents: {duplicate_children}'
    roots = link_set - set(children)
    assert len(roots) == 1, f'expected one root, got {roots}'
    assert len(joints) == len(links) - 1

    visited = set()
    stack = list(roots)
    while stack:
        link = stack.pop()
        assert link not in visited, f'cycle at {link}'
        visited.add(link)
        stack.extend(graph[link])
    assert visited == link_set, f'disconnected links: {link_set - visited}'
    return root


def test_dual_motor_model_is_a_complete_tree_with_ball_lock():
    root = _assert_tree(URDF_DIR / 'robot_arm.urdf')
    joint_names = [joint.get('name') for joint in root.findall('joint')]
    for index in range(1, 6):
        assert joint_names.count(f'arm_joint_{index}') == 1
    assert root.find("link[@name='toolchanger_lock_rotor']") is not None
    assert root.find("joint[@name='gripper_left_pinion_joint']") is not None


def test_single_motor_model_is_a_complete_tree():
    root = _assert_tree(URDF_DIR / 'robot_arm.single_motor_gripper.urdf')
    assert root.find("joint[@name='gripper_drive_joint']") is not None


def test_ball_lock_has_separate_wrist_and_lock_joints():
    root = _assert_tree(URDF_DIR / 'robot_arm.urdf')
    wrist = root.find("joint[@name='arm_joint_5']")
    lock = root.find("joint[@name='toolchanger_lock_wheel']")
    assert wrist.find('child').get('link') == 'arm_link5'
    assert lock.find('parent').get('link') == 'arm_link5'
    assert lock.find('child').get('link') == 'toolchanger_lock_rotor'
    assert lock.get('type') == 'continuous'
    assert lock.find('limit').get('lower') is None
    assert lock.find('limit').get('upper') is None
    for index in range(1, 9):
        ball = root.find(f"joint[@name='lock_ball_{index}']")
        assert ball.get('type') == 'prismatic'
        assert float(ball.find('limit').get('upper')) == 0.00259
        assert ball.find('mimic') is None


def test_ball_lock_meshes_and_moveit_links_exist():
    root = _assert_tree(URDF_DIR / 'robot_arm.urdf')
    package = URDF_DIR.parent
    for mesh in root.findall('.//mesh'):
        relative = mesh.get('filename').removeprefix(
            'package://robot_arm_description/')
        assert (package / relative).is_file(), relative
    srdf = ET.parse(package.parent / 'robot_arm_moveit_config' /
                    'config' / 'robot_arm.srdf').getroot()
    links = {link.get('name') for link in root.findall('link')}
    for pair in srdf.findall('disable_collisions'):
        assert pair.get('link1') in links
        assert pair.get('link2') in links
        assert pair.get('reason') == 'Adjacent'
