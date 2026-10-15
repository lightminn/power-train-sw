from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import can_tx_order_probe as probe  # noqa: E402


class AnalyzeTest(unittest.TestCase):
    def test_complete_in_order_is_accepted(self):
        result = probe.analyze([(0, 0.0), (1, 0.001)], [(0, 0.0005), (1, 0.0030)])
        self.assertTrue(result['accepted'])
        self.assertEqual(result['missing'], [])
        self.assertAlmostEqual(result['max_ms'], 2.0, places=6)

    def test_reordered_or_missing_is_rejected(self):
        result = probe.analyze([(0, 0.0), (1, 0.0), (2, 0.0)], [(1, 0.1), (0, 0.2)])
        self.assertFalse(result['accepted'])
        self.assertEqual(result['missing'], [2])

    def test_nothing_observed_is_rejected(self):
        result = probe.analyze([(0, 0.0), (1, 0.0)], [])
        self.assertFalse(result['accepted'])
        self.assertEqual(result['missing'], [0, 1])

    def test_duplicate_echo_is_rejected(self):
        self.assertFalse(probe.analyze([(0, 0.0)], [(0, 0.1), (0, 0.2)])['accepted'])

    def test_inventory_report_is_required(self):
        probe.require_inventory({'pass': True, 'expected': ['A', 'B', 'C'], 'violations': []})
        for bad in ({'pass': False, 'expected': ['A', 'B', 'C'], 'violations': ['x']},
                    {'pass': True, 'expected': ['A'], 'violations': []}):
            with self.assertRaises(SystemExit):
                probe.require_inventory(bad)

    def test_probe_identifier_is_node_63_only(self):
        self.assertEqual(probe.probe_id(7), (0x3F << 5) | 7)
        with self.assertRaises(ValueError):
            probe.probe_id(32)


if __name__ == '__main__':
    unittest.main(verbosity=2)
