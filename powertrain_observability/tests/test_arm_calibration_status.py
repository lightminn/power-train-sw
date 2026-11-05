"""The bench range-calibration report survives the observation projection, bounded."""
import pytest

from powertrain_observability.tool_snapshot import normalize_tool

BASE = {'tool_type': 'spur_1motor_gripper', 'actuator_ids': [5]}
REPORT = {'status': 'RUNNING', 'recording': True, 'measured': '', 'verified': '',
          'applied': False, 'saved': False, 'detail': '측정 중'}


def test_report_is_forwarded():
    out = normalize_tool(dict(BASE, arm_calibration=REPORT))
    assert out['arm_calibration'] == REPORT


def test_long_text_is_cut_and_absent_report_is_absent():
    out = normalize_tool(dict(BASE, arm_calibration=dict(REPORT, detail='x' * 500)))
    assert len(out['arm_calibration']['detail']) == 240
    assert 'arm_calibration' not in normalize_tool(dict(BASE))


@pytest.mark.parametrize('bad', [dict(REPORT, status='DONEISH'), dict(REPORT, saved='yes'),
                                 dict(REPORT, detail=3)])
def test_malformed_report_is_rejected(bad):
    with pytest.raises(ValueError):
        normalize_tool(dict(BASE, arm_calibration=bad))
