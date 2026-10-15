import copy
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import mks_nvm_acceptance as nvm  # noqa: E402

EVIDENCE = ROOT / 'docs/reports/2026-09-10-can-auto-resume-evidence/board-336A33523235-patch3/336A33523235-before-configuration.json'


def board(n0, n1):
    cfg = copy.deepcopy(json.loads(EVIDENCE.read_text()))
    cfg['axis0']['config']['can_node_id'], cfg['axis1']['config']['can_node_id'] = n0, n1
    return cfg


class NvmAcceptanceTest(unittest.TestCase):
    def boards(self):
        return {'A': board(11, 12), 'B': board(13, 14), 'C': board(15, 16)}

    def test_real_evidence_shape_passes(self):
        self.assertEqual(nvm.check(self.boards(), {'A', 'B', 'C'}), [])

    def test_missing_or_extra_board_fails(self):
        boards = self.boards()
        del boards['A']
        self.assertIn('missing board A', nvm.check(boards, {'A', 'B', 'C'}))
        self.assertIn('unexpected board B', nvm.check(self.boards(), {'A', 'C'}))

    def test_startup_flag_fails(self):
        boards = self.boards()
        boards['B']['axis1']['config']['startup_closed_loop_control'] = True
        self.assertIn('B axis1 startup_closed_loop_control=True', nvm.check(boards, {'A', 'B', 'C'}))

    def test_cross_board_duplicate_fails(self):
        boards = self.boards()
        boards['C']['axis0']['config']['can_node_id'] = 13
        self.assertTrue(any('duplicate node 13' in v for v in nvm.check(boards, {'A', 'B', 'C'})))

    def test_baud_protocol_range_fail(self):
        boards = self.boards()
        boards['A']['can']['config']['baud_rate'] = 250000
        boards['A']['can']['config']['protocol'] = 1
        boards['B']['axis0']['config']['can_node_id'] = 77
        found = nvm.check(boards, {'A', 'B', 'C'})
        self.assertIn('A can baud_rate=250000', found)
        self.assertIn('A can protocol=1', found)
        self.assertIn('B axis0 node 77 out of STD range', found)

    def test_probe_node_and_silent_axis_fail(self):
        boards = self.boards()
        boards['B']['axis0']['config']['can_node_id'] = 63
        boards['B']['axis0']['config']['can_heartbeat_rate_ms'] = 0
        found = nvm.check(boards, {'A', 'B', 'C'})
        self.assertIn('B axis0 node 63 is reserved for the CAN probe', found)
        self.assertIn('B axis0 can_heartbeat_rate_ms=0', found)


if __name__ == '__main__':
    unittest.main(verbosity=2)
