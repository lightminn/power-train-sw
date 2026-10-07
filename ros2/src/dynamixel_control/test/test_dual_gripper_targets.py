"""Pure target calculation tests; no ROS node, SDK, or serial hardware."""

import pytest

from dynamixel_control.dual_gripper_targets import dual_relative_targets


def test_current_dual_profile_pair_jog_uses_endpoint_spans():
    profile = {
        'motor_endpoints': {
            3: {'open': -675, 'close': -2211},
            4: {'open': 2559, 'close': 1017},
        },
    }
    targets = dual_relative_targets(
        profile, (3, 4), {3: -688, 4: 2547}, 'OPEN',
        step_ticks=round(0.5 * 4096 / 360.0))
    assert targets == {3: -682, 4: 2553}


@pytest.mark.parametrize('command,expected', [
    ('CLOSE', {3: -1100, 4: 800}),
    ('OPEN', {3: -900, 4: 700}),
])
def test_relative_targets_follow_each_motor_endpoint_direction(command, expected):
    profile = {
        'motor_endpoints': {
            3: {'open': 500, 'close': -2000},
            4: {'open': 0, 'close': 1250},
        },
    }
    assert dual_relative_targets(
        profile, (3, 4), {3: -1000, 4: 750}, command,
        step_ticks=100) == expected
