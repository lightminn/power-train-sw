from pathlib import Path

from robot_arm_perception.perception_quality import (
    is_usable_bbox,
    robust_depth_m,
)


def test_rejects_detection_clipped_on_three_frame_borders():
    assert not is_usable_bbox(
        (1, 0, 321, 476), frame_width=848, frame_height=480,
    )


def test_accepts_complete_detection_inside_frame():
    assert is_usable_bbox(
        (120, 80, 420, 360), frame_width=848, frame_height=480,
    )


def test_robust_depth_rejects_background_outliers():
    assert robust_depth_m(
        [320] * 80 + [3000] * 4,
        depth_scale=0.001,
    ) == 0.32


def test_robust_depth_requires_enough_valid_pixels():
    assert robust_depth_m(
        [0, 0, 320, 321],
        depth_scale=0.001,
    ) is None


def test_runtime_uses_deletterboxed_original_frame_mask_polygons():
    source = (
        Path(__file__).resolve().parents[1]
        / "robot_arm_perception"
        / "perception_node.py"
    ).read_text(encoding="utf-8")

    assert "r0.masks.xy" in source
    assert "r0.masks.data.cpu().numpy()" not in source
    assert "cv2.fillPoly" in source
