from dynamixel_control.single_motor_endpoints import jog_limits, load, save


def test_endpoints_persist_independently_then_form_a_range(tmp_path):
    path = tmp_path / 'endpoints.json'
    assert load(path) == {'open_tick': None, 'close_tick': None}
    assert save('open', -900, path) == {
        'open_tick': -900, 'close_tick': None}
    assert save('close', 250, path) == {
        'open_tick': -900, 'close_tick': 250}
    assert load(path) == {'open_tick': -900, 'close_tick': 250}


def test_identical_endpoints_are_rejected_without_overwriting(tmp_path):
    path = tmp_path / 'endpoints.json'
    save('open', 100, path)
    try:
        save('close', 100, path)
    except ValueError:
        pass
    else:
        raise AssertionError('identical endpoints must be rejected')
    assert load(path) == {'open_tick': 100, 'close_tick': None}


def test_incomplete_calibration_ignores_legacy_dual_profile_limits():
    assert jog_limits({'open_tick': -785, 'close_tick': None}) == (
        -4096, 4095, False)
    assert jog_limits({'open_tick': -785, 'close_tick': 320}) == (
        -785, 320, True)
