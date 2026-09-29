import sys
import threading
import unittest
from pathlib import Path

import numpy as np


TELEOP_DIR = Path(__file__).resolve().parents[1]
if str(TELEOP_DIR) not in sys.path:
    sys.path.insert(0, str(TELEOP_DIR))

from lcm_types.ref_upper_dof_pos_lcmt import ref_upper_dof_pos_lcmt
from xrobot_teleop_to_pose_zmq_server import LowLatencyTeleopPoseZMQServer


class _FakeLCM:
    def __init__(self):
        self.calls = []

    def publish(self, channel, data):
        self.calls.append((channel, data))


class UpperReferenceLCMTest(unittest.TestCase):
    def test_extracts_left_then_right_arm_from_g1_qpos(self):
        qpos = np.arange(36, dtype=np.float32)

        upper = LowLatencyTeleopPoseZMQServer._extract_upper_dof_pos(qpos)

        np.testing.assert_array_equal(upper, np.arange(22, 36, dtype=np.float64))

    def test_lcm_message_round_trip(self):
        upper = np.linspace(-1.2, 1.2, 14, dtype=np.float64)

        msg = LowLatencyTeleopPoseZMQServer._build_upper_reference_message(upper)
        decoded = ref_upper_dof_pos_lcmt.decode(msg.encode())

        np.testing.assert_allclose(decoded.ref_upper_dof_pos, upper)

    def test_rejects_short_qpos(self):
        with self.assertRaises(ValueError):
            LowLatencyTeleopPoseZMQServer._extract_upper_dof_pos(np.zeros(35))

    def test_publishes_initial_zero_hold_reference(self):
        server = LowLatencyTeleopPoseZMQServer.__new__(LowLatencyTeleopPoseZMQServer)
        server.publish_lcm_ref = True
        server.lcm = _FakeLCM()
        server.lcm_channel = "ref_upper_dof_pos_channel"
        server.lcm_publish_count = 0
        server.lcm_publish_error_reported = False

        server._publish_upper_dof_pos(np.zeros(14))

        self.assertEqual(len(server.lcm.calls), 1)
        _, encoded = server.lcm.calls[0]
        decoded = ref_upper_dof_pos_lcmt.decode(encoded)
        np.testing.assert_array_equal(decoded.ref_upper_dof_pos, np.zeros(14))
        self.assertEqual(server.lcm_publish_count, 1)


if __name__ == "__main__":
    unittest.main()
