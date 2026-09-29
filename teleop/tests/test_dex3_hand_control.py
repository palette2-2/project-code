import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


TELEOP_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = TELEOP_DIR.parent
if str(TELEOP_DIR) not in sys.path:
    sys.path.insert(0, str(TELEOP_DIR))

from lcm_types.hand_action_lcmt import hand_action_lcmt
from xrobot_teleop_to_pose_zmq_server import (
    DEFAULT_DEX3_POSE_CONFIG,
    Dex3HandStateMachine,
    LowLatencyTeleopPoseZMQServer,
    load_dex3_pose_config,
)


class _FakeLCM:
    def __init__(self):
        self.calls = []

    def publish(self, channel, data):
        self.calls.append((channel, data))


def _buttons(a=False, x=False):
    return {"right_key_one": a, "left_key_one": x}


class Dex3HandStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.poses = load_dex3_pose_config(DEFAULT_DEX3_POSE_CONFIG)
        self.machine = Dex3HandStateMachine(
            poses=self.poses, transition_s=0.5, stale_timeout_ms=250
        )
        self.t0 = 1_000_000_000

    def test_release_to_arm_and_independent_rising_edge_toggles(self):
        output, status = self.machine.update(_buttons(a=True), self.t0, self.t0)
        self.assertFalse(status["armed"])
        np.testing.assert_array_equal(output, np.zeros(14))

        _, status = self.machine.update(_buttons(), self.t0 + 1, self.t0 + 1)
        self.assertTrue(status["armed"])

        press_ns = self.t0 + 1_000_000
        output, status = self.machine.update(_buttons(a=True), press_ns, press_ns)
        self.assertEqual(status["left"], "open")
        self.assertEqual(status["right"], "closed")
        np.testing.assert_array_equal(output, np.zeros(14))

        output, status = self.machine.update(
            _buttons(a=True), press_ns, press_ns + 250_000_000
        )
        np.testing.assert_allclose(output[:7], np.zeros(7))
        np.testing.assert_allclose(output[7:], self.poses["right_closed"] * 0.5)
        self.assertEqual(status["right"], "closed")

        self.machine.update(_buttons(), press_ns + 300_000_000, press_ns + 300_000_000)
        self.machine.update(_buttons(x=True), press_ns + 310_000_000, press_ns + 310_000_000)
        _, status = self.machine.update(
            _buttons(x=True), press_ns + 810_000_000, press_ns + 810_000_000
        )
        self.assertEqual(status["left"], "closed")
        self.assertEqual(status["right"], "closed")

    def test_stale_input_freezes_transition_and_requires_release_again(self):
        self.machine.update(_buttons(), self.t0, self.t0)
        press_ns = self.t0 + 1_000_000
        self.machine.update(_buttons(a=True), press_ns, press_ns)

        stale_now = press_ns + 300_000_000
        frozen, status = self.machine.update(_buttons(a=True), press_ns, stale_now)
        self.assertFalse(status["fresh"])
        self.assertFalse(status["armed"])
        later, _ = self.machine.update(_buttons(), press_ns, stale_now + 1_000_000_000)
        np.testing.assert_array_equal(later, frozen)

        _, status = self.machine.update(
            _buttons(a=True), stale_now + 1_000_000_001, stale_now + 1_000_000_001
        )
        self.assertFalse(status["armed"])
        _, status = self.machine.update(
            _buttons(), stale_now + 1_000_000_002, stale_now + 1_000_000_002
        )
        self.assertTrue(status["armed"])


class Dex3PoseConfigTest(unittest.TestCase):
    def test_default_config_is_valid(self):
        poses = load_dex3_pose_config(DEFAULT_DEX3_POSE_CONFIG)
        self.assertEqual(set(poses), {"left_open", "left_closed", "right_open", "right_closed"})
        for pose in poses.values():
            self.assertEqual(pose.shape, (7,))
            self.assertTrue(np.all(np.isfinite(pose)))
        np.testing.assert_array_equal(poses["left_open"], np.zeros(7))
        np.testing.assert_array_equal(poses["right_open"], np.zeros(7))
        np.testing.assert_allclose(
            poses["left_closed"],
            [0.0, 1.05, 1.75, -1.57, -1.75, -1.57, -1.75],
        )
        np.testing.assert_allclose(
            poses["right_closed"],
            [0.0, -1.05, -1.75, 1.57, 1.75, 1.57, 1.75],
        )

    def test_rejects_missing_invalid_and_out_of_range_values(self):
        valid = {
            name: value.tolist()
            for name, value in load_dex3_pose_config(DEFAULT_DEX3_POSE_CONFIG).items()
        }
        cases = []
        missing = dict(valid)
        missing.pop("left_open")
        cases.append(missing)
        unexpected = dict(valid)
        unexpected["typo_pose"] = [0.0] * 7
        cases.append(unexpected)
        invalid = dict(valid)
        invalid["left_open"] = [0.0] * 6
        cases.append(invalid)
        non_finite = dict(valid)
        non_finite["left_closed"] = [float("nan")] + [0.0] * 6
        cases.append(non_finite)
        out_of_range = dict(valid)
        out_of_range["right_closed"] = [99.0] + [0.0] * 6
        cases.append(out_of_range)

        for payload in cases:
            with self.subTest(payload=payload):
                with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as config_file:
                    json.dump(payload, config_file)
                    config_file.flush()
                    with self.assertRaises(ValueError):
                        load_dex3_pose_config(config_file.name)


class Dex3HandLCMTest(unittest.TestCase):
    def test_message_is_wire_compatible_with_deploy_type(self):
        deploy_type_path = REPO_DIR / "src/thor_deploy/lcm_types/hand_action_lcmt.py"
        spec = importlib.util.spec_from_file_location("deploy_hand_action_lcmt", deploy_type_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        action = np.linspace(-0.5, 0.5, 14)
        message = LowLatencyTeleopPoseZMQServer._build_hand_action_message(action)
        decoded = module.hand_action_lcmt.decode(message.encode())
        np.testing.assert_allclose(decoded.act, action)

        deploy_message = module.hand_action_lcmt()
        deploy_message.act = action.tolist()
        decoded_by_teleop = hand_action_lcmt.decode(deploy_message.encode())
        np.testing.assert_allclose(decoded_by_teleop.act, action)

    def test_publish_honors_enable_flag_and_reports_status(self):
        server = LowLatencyTeleopPoseZMQServer.__new__(LowLatencyTeleopPoseZMQServer)
        server.publish_lcm_hand = False
        server.lcm = _FakeLCM()
        server.hand_lcm_channel = "hand_action"
        server.hand_lcm_publish_count = 0
        server.hand_lcm_publish_error_reported = False
        server._publish_hand_action(np.zeros(14))
        self.assertEqual(server.lcm.calls, [])

        server.publish_lcm_hand = True
        server._publish_hand_action(np.zeros(14))
        self.assertEqual(server.lcm.calls[0][0], "hand_action")
        self.assertEqual(server.hand_lcm_publish_count, 1)

        payload = server._build_control_payload(
            buttons=_buttons(),
            upper_status={"state": "paused_hold", "pose_fresh": True},
            wall_time_ms=123,
            hand_status={
                "armed": True,
                "fresh": True,
                "left": "open",
                "right": "closed",
            },
            hand_enabled=True,
        )
        self.assertEqual(
            payload["dex3_hand"],
            {
                "enabled": True,
                "armed": True,
                "fresh": True,
                "left": "open",
                "right": "closed",
            },
        )

    def test_message_builder_rejects_bad_shape_and_non_finite_values(self):
        with self.assertRaises(ValueError):
            LowLatencyTeleopPoseZMQServer._build_hand_action_message(np.zeros(13))
        action = np.zeros(14)
        action[3] = np.inf
        with self.assertRaises(ValueError):
            LowLatencyTeleopPoseZMQServer._build_hand_action_message(action)


if __name__ == "__main__":
    unittest.main()
