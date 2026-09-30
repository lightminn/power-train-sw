#!/usr/bin/env python3
"""Hardware-free tests for the one-axis Dynamixel bench transport."""

import json
import unittest

from dynamixel_control import master_slave_slave_server as MODULE


class MappingTests(unittest.TestCase):
    def test_relative_mapping_and_clamp(self):
        self.assertEqual(MODULE.bounded_target(1000, 2000, 2010, 1, 20), 1010)
        self.assertEqual(MODULE.bounded_target(1000, 2000, 2100, 1, 20), 1020)
        self.assertEqual(MODULE.bounded_target(1000, 2000, 1900, 1, 20), 980)

    def test_unrestricted_delta_still_obeys_single_turn_bounds(self):
        self.assertEqual(MODULE.bounded_target(1800, 1500, 0, 1, 4095), 300)
        self.assertEqual(
            MODULE.bounded_target(1800, 1500, 4095, 1, 4095),
            4095,
        )

    def test_motor_specific_position_limits_win(self):
        self.assertEqual(
            MODULE.bounded_target(2000, 2000, 1000, 1, 4095, 1500, 2500),
            1500,
        )
        self.assertEqual(
            MODULE.bounded_target(2000, 2000, 3000, 1, 4095, 1500, 2500),
            2500,
        )

    def test_direction_and_absolute_limits(self):
        self.assertEqual(MODULE.bounded_target(10, 2000, 2100, -1, 20), 0)
        self.assertEqual(MODULE.bounded_target(4090, 2000, 2100, 1, 20), 4095)

    def test_signed_current(self):
        self.assertEqual(MODULE.signed16(100), 100)
        self.assertEqual(MODULE.signed16(65500), -36)


class FrameTests(unittest.TestCase):
    def test_valid_frame(self):
        raw = json.dumps(
            {"version": 1, "session": "abc", "seq": 2, "position": 2048}
        ).encode()
        self.assertEqual(MODULE.parse_frame(raw)["position"], 2048)

    def test_rejects_bool_as_integer(self):
        raw = json.dumps(
            {"version": 1, "session": "abc", "seq": True, "position": 2048}
        ).encode()
        with self.assertRaises(ValueError):
            MODULE.parse_frame(raw)

    def test_rejects_oversize(self):
        with self.assertRaises(ValueError):
            MODULE.parse_frame(b" " * 1025)


class RestoreTests(unittest.TestCase):
    def test_mode_is_restored_before_mode_dependent_ram(self):
        actuator = object.__new__(MODULE.XL430)
        writes = []
        actuator.write1 = lambda address, value, label: writes.append(
            ("write1", address, value)
        )
        actuator.write2 = lambda address, value, label: writes.append(
            ("write2", address, value)
        )
        actuator.write4 = lambda address, value, label: writes.append(
            ("write4", address, value)
        )
        snapshot = MODULE.Snapshot(
            drive_mode=0,
            operating_mode=4,
            velocity_limit=265,
            min_position=0,
            max_position=4095,
            velocity_i_gain=1000,
            velocity_p_gain=100,
            position_d_gain=4000,
            position_i_gain=0,
            position_p_gain=640,
            feedforward_2nd_gain=0,
            feedforward_1st_gain=0,
            goal_pwm=885,
            profile_acceleration=0,
            profile_velocity=0,
            start_position=2048,
        )

        actuator.stop_and_restore(snapshot)

        self.assertEqual(
            writes[0],
            ("write1", MODULE.ADDR_TORQUE_ENABLE, 0),
        )
        self.assertEqual(
            writes[1],
            ("write1", MODULE.ADDR_OPERATING_MODE, 4),
        )
        self.assertGreater(
            next(
                i
                for i, write in enumerate(writes)
                if write[1] == MODULE.ADDR_PROFILE_VELOCITY
            ),
            1,
        )


if __name__ == "__main__":
    unittest.main()
