import cv2
import numpy as np
import pytest

from l515_dashboard.frame_modes import (
    FrameMode,
    LatestVideoFrames,
    render_frame,
)


WIDTH = 1280
HEIGHT = 720
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
        (FrameMode.OVERLAY, (HEIGHT, WIDTH, 3)),
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
    assert rendered is color


def test_rejects_wrong_or_noncontiguous_gateway_frames(color, depth):
    with pytest.raises(ValueError, match="color"):
        render_frame(FrameMode.COLOR, color[:, ::2], depth, WIDTH, HEIGHT)
    with pytest.raises(ValueError, match="aligned depth"):
        render_frame(FrameMode.DEPTH, color, depth[:480, :640], WIDTH, HEIGHT)
    with pytest.raises(ValueError, match="contiguous"):
        render_frame(FrameMode.DEPTH, color, depth[:, ::-1], WIDTH, HEIGHT)


def test_depth_mode_uses_fixed_zero_aware_turbo_mapping(color):
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
    depth[0, :5] = [0, 1, 2500, 5000, 6000]
    normalized = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    normalized[0, :5] = [0, 0, 128, 255, 255]
    expected = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    expected[depth == 0] = 0

    rendered = render_frame(FrameMode.DEPTH, color, depth, WIDTH, HEIGHT)

    np.testing.assert_array_equal(rendered, expected)
    assert np.array_equal(rendered[0, 0], np.zeros(3, dtype=np.uint8))


@pytest.mark.parametrize(
    ("mode", "color_present", "depth_present"),
    [
        (FrameMode.COLOR, False, True),
        (FrameMode.DEPTH, True, False),
        (FrameMode.OVERLAY, False, True),
        (FrameMode.OVERLAY, True, False),
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

    frames.put_color(first, timestamp_ns=1)
    frames.put_color(second, timestamp_ns=2)

    np.testing.assert_array_equal(frames.take(FrameMode.COLOR, 2, 250_000_000), second)
    assert frames.take(FrameMode.COLOR, 2, 250_000_000) is None


def test_incomplete_overlay_take_consumes_color_but_keeps_depth(color, depth):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_color(color, timestamp_ns=1)

    assert frames.take(FrameMode.OVERLAY, 1, 250_000_000) is None
    frames.put_depth(depth, timestamp_ns=2)
    assert frames.take(FrameMode.OVERLAY, 2, 250_000_000) is None


def test_take_discards_unselected_slot_so_mode_change_cannot_replay_it(
    color, depth
):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_color(color, timestamp_ns=1)
    frames.put_depth(depth, timestamp_ns=1)

    assert frames.take(FrameMode.COLOR, 1, 250_000_000) is not None
    assert frames.take(FrameMode.DEPTH, 1, 250_000_000) is None


def test_put_copies_input_to_prevent_concurrent_mutation(color):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    expected = color.copy()

    frames.put_color(color, timestamp_ns=1)
    color[:] = 255

    np.testing.assert_array_equal(
        frames.take(FrameMode.COLOR, 1, 250_000_000), expected
    )


def test_put_depth_copies_input_before_reusable_storage(color, depth):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    expected = depth.copy()
    frames.put_depth(depth, timestamp_ns=1)
    depth[:] = 0
    frames.put_color(color, timestamp_ns=1)

    np.testing.assert_array_equal(
        frames.take(FrameMode.DEPTH, 1, 250_000_000),
        render_frame(FrameMode.DEPTH, None, expected, WIDTH, HEIGHT),
    )


def test_overlay_alpha_blends_color_and_depth():
    color = np.full((HEIGHT, WIDTH, 3), 19, dtype=np.uint8)
    depth = np.full((HEIGHT, WIDTH), MAX_DEPTH_MM, dtype=np.uint16)
    depth[:, 0] = 0

    rendered = render_frame(FrameMode.OVERLAY, color, depth, WIDTH, HEIGHT, overlay_alpha=0.25)

    colored = render_frame(FrameMode.DEPTH, color, depth, WIDTH, HEIGHT)
    expected = cv2.addWeighted(color, 0.75, colored, 0.25, 0)
    np.testing.assert_array_equal(rendered, expected)


def test_latest_depth_slot_overwrites_with_newest_frame():
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_depth(np.full((HEIGHT, WIDTH), 100, dtype=np.uint16), timestamp_ns=1)
    newest = np.full((HEIGHT, WIDTH), 400, dtype=np.uint16)
    frames.put_depth(newest, timestamp_ns=2)
    frames.put_color(np.zeros((HEIGHT, WIDTH, 3), np.uint8), timestamp_ns=2)

    np.testing.assert_array_equal(
        frames.take(FrameMode.DEPTH, 2, 250_000_000),
        render_frame(FrameMode.DEPTH, None, newest, WIDTH, HEIGHT),
    )
    assert frames.take(FrameMode.DEPTH, 2, 250_000_000) is None


def test_overlay_reuses_fresh_depth_for_each_new_color(color, depth):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_depth(depth, timestamp_ns=0)

    for timestamp_ns in (0, 33_333_333, 66_666_666):
        frames.put_color(color, timestamp_ns=timestamp_ns)
        assert frames.take(
            FrameMode.OVERLAY, timestamp_ns, 250_000_000
        ) is not None


@pytest.mark.parametrize("mode", [FrameMode.DEPTH, FrameMode.OVERLAY])
def test_stale_depth_never_replays(mode, color, depth):
    frames = LatestVideoFrames(WIDTH, HEIGHT)
    frames.put_depth(depth, timestamp_ns=0)
    frames.put_color(color, timestamp_ns=300_000_000)

    assert frames.take(mode, 300_000_000, 250_000_000) is None
    assert frames.depth_age_ns(300_000_000) == 300_000_000
