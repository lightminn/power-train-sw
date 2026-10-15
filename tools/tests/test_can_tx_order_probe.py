from pathlib import Path
import socket
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import can_tx_order_probe as probe  # noqa: E402

CONFIRM = socket.MSG_CONFIRM


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

    def test_node63_during_run_is_rejected(self):
        result = probe.analyze([(0, 0.0)], [(0, 0.1)], node63_seen=True)
        self.assertFalse(result['accepted'])
        self.assertTrue(result['node63_seen'])

    def test_partial_run_is_rejected(self):
        # Stopped early (receiver deadline): what was sent may be in order, but it is not the run.
        result = probe.analyze([(0, 0.0), (1, 0.0)], [(0, 0.1), (1, 0.2)], requested=3)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['requested'], 3)

    def test_send_failure_is_rejected(self):
        result = probe.analyze([(0, 0.0)], [(0, 0.1)], send_error='ENOBUFS')
        self.assertFalse(result['accepted'])
        self.assertEqual(result['send_error'], 'ENOBUFS')


class FrameTest(unittest.TestCase):
    nonce = 0x1234

    def frame(self, can_id, seq=7, nonce=None):
        return probe.pack_frame(can_id, probe.encode(seq, self.nonce if nonce is None else nonce))

    def test_probe_identifier_is_ignored_heartbeat_on_node_63(self):
        # cmd 0x01 (heartbeat) is a no-op in the patched router; node 63 is reserved and
        # verified absent live before sending.
        self.assertEqual(probe.PROBE_ID, (0x3F << 5) | 0x01)
        self.assertNotIn('--cmd', probe.build_parser().format_help())

    def test_only_this_sockets_confirmed_echo_counts(self):
        self.assertEqual(probe.own_echo_seq(self.frame(probe.PROBE_ID), CONFIRM, self.nonce), 7)
        # Another local socket's identical frame lacks MSG_CONFIRM.
        self.assertIsNone(probe.own_echo_seq(self.frame(probe.PROBE_ID), 0, self.nonce))
        self.assertIsNone(probe.own_echo_seq(self.frame(probe.PROBE_ID, nonce=0x9999), CONFIRM, self.nonce))
        self.assertIsNone(probe.own_echo_seq(self.frame(probe.PROBE_ID + 1), CONFIRM, self.nonce))
        ext = probe.pack_frame(probe.PROBE_ID | socket.CAN_EFF_FLAG, probe.encode(7, self.nonce))
        self.assertIsNone(probe.own_echo_seq(ext, CONFIRM, self.nonce))

    def test_query_counts_only_once_confirmed_on_the_bus(self):
        rtr = probe.pack_frame(probe.QUERY_ID | socket.CAN_RTR_FLAG, b'')
        self.assertTrue(probe.query_confirmed(rtr, CONFIRM))
        self.assertFalse(probe.query_confirmed(rtr, 0))  # someone else's RTR
        self.assertFalse(probe.query_confirmed(self.frame(probe.QUERY_ID), CONFIRM))  # data, not RTR

    def test_listen_window_must_be_meaningful(self):
        for bad in ('0', 'nan', 'inf', '-inf'):
            with self.assertRaises(SystemExit):
                probe.build_parser().parse_args(['--nvm-report', 'r.json', '--listen-s', bad])
        self.assertEqual(probe.build_parser().parse_args(['--nvm-report', 'r.json']).listen_s, 2.0)
        for flag, bad in (('--timeout-s', 'nan'), ('--timeout-s', '0'), ('--gap-ms', 'nan'),
                          ('--gap-ms', '-1'), ('--count', '0')):
            with self.assertRaises(SystemExit):
                probe.build_parser().parse_args(['--nvm-report', 'r.json', flag, bad])

    def test_any_foreign_node_63_frame_is_detected(self):
        self.assertTrue(probe.foreign_node63(self.frame((0x3F << 5) | 0x09), 0))
        self.assertTrue(probe.foreign_node63(self.frame((0x3F << 5) | 0x01), 0))
        self.assertFalse(probe.foreign_node63(self.frame((0x3F << 5) | 0x09), CONFIRM))  # our own RTR echo
        self.assertFalse(probe.foreign_node63(self.frame((13 << 5) | 0x01), 0))
        self.assertFalse(probe.foreign_node63(
            probe.pack_frame(((0x3F << 5) | 0x01) | socket.CAN_EFF_FLAG, bytes(8)), 0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
