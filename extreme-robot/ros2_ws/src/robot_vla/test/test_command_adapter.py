import pytest

from robot_vla.command_adapter import parse_text


def test_clean_text():
    assert parse_text('CLEAN panel') == ('CLEAN', 'panel', 'cleaner')


def test_rejects_motor_level_command():
    with pytest.raises(ValueError):
        parse_text('DYNAMIXEL 5 100')
