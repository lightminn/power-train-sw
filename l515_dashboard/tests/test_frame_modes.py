import cv2
import numpy as np
import pytest

from l515_dashboard.frame_modes import (
    FrameMode,
    LatestVideoFrames,
    render_frame,
)


WIDTH = 640
HEIGHT = 480
MAX_DEPTH_MM = 5000


@pytest.fixture
def color():
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:, :, 1] = 73
    return frame


@pytest.fixture
def depth():
    return np.linspace(
        0, MAX_DEPTH_MM, WIDTH, dtype=np.uint16
    )[None, :].repeat(HEIGHT, axis=0)


@pytest.mark.parametrize(
    ("mode", "expected_shape"),
    [
        (FrameMode.COLOR, (HEIGHT, WIDTH, 3)),
        (FrameMode.DEPTH, (HEIGHT, WIDTH, 3)),
        (FrameMode.SIDE_BY_SIDE, (HEIGHT, WIDTH * 2, 3)),
    ],
)
def test_render_modes_return_contiguous_uint8_bgr(
    mode, expected_shape, color, depth
):
    rendered = render_frame(mode, color, depth, WIDTH, HEIGHT)

    assert rendered is not None
    assert rendered.shape == expected_shape
    assert rendered.dtype == np.uint8
    assert rendered.flags.c_contiguous


def test_color_mode_passes_through_the_color_frame(color, depth):
    rendered = render_frame(FrameMode.COLOR, color, depth, WIDTH, HEIGHT)

    np.testing.assert_array_equal(rendered, color)


def test_depth_mode_uses_fixed_zero_aware_turbo_mapping(color):
    depth = np.array([[0, 1, 2500, 5000, 6000]], dtype=np.uint16)
    normalized = np.array([[0, 0, 128, 255, 255]], dtype=np.uint8)
    expected = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    expected[depth == 0] = 0

    rendered = render_frame(FrameMode.DEPTH, color, depth, 5, 1)

    np.testing.assert_array_equal(rendered, expected)
    assert np.array_equal(rendered[0, 0], np.zeros(3, dtype=np.uint8))


@pytest.mark.parametrize(
    ("mode", "color_present", "depth_present"),
    [
        (FrameMode.COLOR, False, True),
        (FrameMode.DEPTH, True, False),
        (FrameMode.SIDE_BY_SIDE, False, True),
        (FrameMode.SIDE_BY_SIDE, True, False),
    ],
)
def test_missing_selected_input_returns_none(
    mode, color_present, depth_present, color, depth
):
    rendered = render_frame(
        mode,
        color if color_present else None,
        depth if depth_present else None,
        WIDTH,
        HEIGHT,
    )

    assert rendered is None


def test_latest_slots_overwrite_and_take_consumes_the_newest(color):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    first = color.copy()
    second = color.copy()
    first[:, :, 0] = 1
    second[:, :, 0] = 2

    frames.put_color(first)
    frames.put_color(second)

    np.testing.assert_array_equal(frames.take(FrameMode.COLOR), second)
    assert frames.take(FrameMode.COLOR) is None


def test_incomplete_side_by_side_take_discards_unpaired_frame(color, depth):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_color(color)

    assert frames.take(FrameMode.SIDE_BY_SIDE) is None
    frames.put_depth(depth)
    assert frames.take(FrameMode.SIDE_BY_SIDE) is None


def test_take_discards_unselected_slot_so_mode_change_cannot_replay_it(
    color, depth
):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_color(color)
    frames.put_depth(depth)

    assert frames.take(FrameMode.COLOR) is not None
    assert frames.take(FrameMode.DEPTH) is None


def test_put_copies_input_to_prevent_concurrent_mutation(color):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    expected = color.copy()

    frames.put_color(color)
    color[:] = 255

    np.testing.assert_array_equal(frames.take(FrameMode.COLOR), expected)


def test_side_by_side_places_color_left_and_depth_rendering_right():
    color = np.full((1, 2, 3), 19, dtype=np.uint8)
    depth = np.array([[0, MAX_DEPTH_MM]], dtype=np.uint16)

    rendered = render_frame(FrameMode.SIDE_BY_SIDE, color, depth, 2, 1)

    np.testing.assert_array_equal(rendered[:, :2], color)
    np.testing.assert_array_equal(
        rendered[:, 2:], render_frame(FrameMode.DEPTH, color, depth, 2, 1)
    )


def test_latest_depth_slot_overwrites_with_newest_frame():
    frames = LatestVideoFrames(2, 1)
    frames.put_depth(np.array([[100, 200]], dtype=np.uint16))
    newest = np.array([[300, 400]], dtype=np.uint16)
    frames.put_depth(newest)

    np.testing.assert_array_equal(
        frames.take(FrameMode.DEPTH),
        render_frame(FrameMode.DEPTH, None, newest, 2, 1),
    )
    assert frames.take(FrameMode.DEPTH) is None
