import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


TELEOP_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = TELEOP_DIR.parent
if str(TELEOP_DIR) not in sys.path:
    sys.path.insert(0, str(TELEOP_DIR))
DEPLOY_DIR = REPO_DIR / "src"
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))

from lcm_types.rc_command_lcmt import rc_command_lcmt as TeleopRCCommand
from xrobot_teleop_to_pose_zmq_server import (
    LowLatencyTeleopPoseZMQServer,
    PicoControllerMapper,
    UpperTeleopStateMachine,
)
from thor_deploy.utils.rc_command_source import resolve_rc_command_source


class _FakeLCM:
    def __init__(self):
        self.calls = []

    def publish(self, channel, data):
        self.calls.append((channel, data))


def _buttons(**updates):
    value = {
        "left_axis": [0.0, 0.0],
        "right_axis": [0.0, 0.0],
        "right_grip_value": 0.0,
        "right_trigger_value": 0.0,
        "right_key_two": False,
    }
    value.update(updates)
    return value


class PicoControllerMapperTest(unittest.TestCase):
    def setUp(self):
        self.mapper = PicoControllerMapper()
        self.t0 = 1_000_000_000

    def _arm(self):
        state = self.mapper.update(_buttons(), self.t0, self.t0)
        self.assertTrue(state["armed"])
        self.assertEqual(state["left_stick"], [0.0, 0.0])

    def test_radial_deadzone_linear_remap_and_invalid_values(self):
        self._arm()
        state = self.mapper.update(
            _buttons(left_axis=[0.1, 0.0], right_axis=[0.55, 0.0]), self.t0, self.t0
        )
        np.testing.assert_allclose(state["left_stick"], [0.0, 0.0])
        np.testing.assert_allclose(state["right_stick"], [0.5, 0.0])

        state = self.mapper.update(
            _buttons(left_axis=[float("nan"), 1.0], right_axis=[2.0, 0.0]), self.t0, self.t0
        )
        np.testing.assert_allclose(state["left_stick"], [0.0, 0.0])
        np.testing.assert_allclose(state["right_stick"], [1.0, 0.0])

    def test_requires_release_to_arm_at_startup_and_after_stale_input(self):
        held = _buttons(right_grip_value=1.0, right_trigger_value=1.0, right_key_two=True)
        state = self.mapper.update(held, self.t0, self.t0)
        self.assertFalse(state["armed"])
        self.assertFalse(state["r1"])
        self.assertFalse(state["r2"])

        state = self.mapper.update(_buttons(), self.t0 + 1, self.t0 + 1)
        self.assertTrue(state["armed"])
        state = self.mapper.update(
            _buttons(right_grip_value=1.0), self.t0 + 2, self.t0 + 2
        )
        self.assertTrue(state["r1"])

        state = self.mapper.update(_buttons(right_grip_value=1.0), self.t0 + 2, self.t0 + 300_000_000)
        self.assertFalse(state["fresh"])
        self.assertFalse(state["armed"])
        self.assertFalse(state["r1"])

    def test_grip_trigger_hysteresis_and_b_rising_edge(self):
        self._arm()
        state = self.mapper.update(
            _buttons(right_grip_value=0.8, right_trigger_value=0.8, right_key_two=True),
            self.t0,
            self.t0,
        )
        self.assertTrue(state["r1"])
        self.assertTrue(state["r2"])
        self.assertTrue(state["toggle_rising"])

        state = self.mapper.update(
            _buttons(right_grip_value=0.5, right_trigger_value=0.5, right_key_two=True),
            self.t0,
            self.t0,
        )
        self.assertTrue(state["r1"])
        self.assertTrue(state["r2"])
        self.assertFalse(state["toggle_rising"])

        state = self.mapper.update(
            _buttons(right_grip_value=0.2, right_trigger_value=0.2), self.t0, self.t0
        )
        self.assertFalse(state["r1"])
        self.assertFalse(state["r2"])

    def test_existing_controller_button_payload_remains_compatible(self):
        server = LowLatencyTeleopPoseZMQServer.__new__(LowLatencyTeleopPoseZMQServer)
        server.last_controller_buttons = LowLatencyTeleopPoseZMQServer._default_controller_buttons()
        snapshot = {
            "controllers": {
                "left": {
                    "primary_button": True,
                    "secondary_button": False,
                    "axis_click": True,
                    "trigger": 0.8,
                    "grip": 0.2,
                    "axis": [0.25, -0.5],
                },
                "right": {
                    "primary_button": False,
                    "secondary_button": True,
                    "axis_click": False,
                    "trigger": 0.1,
                    "grip": 0.9,
                    "axis": [-0.75, 1.0],
                },
            }
        }
        buttons = server._extract_controller_buttons_from_snapshot(snapshot)
        self.assertTrue(buttons["left_key_one"])
        self.assertTrue(buttons["left_axis_click"])
        self.assertTrue(buttons["left_index_trig"])
        self.assertTrue(buttons["right_key_two"])
        self.assertTrue(buttons["right_grip"])
        self.assertEqual(buttons["left_axis"], [0.25, -0.5])


class UpperTeleopStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.machine = UpperTeleopStateMachine(blend_s=0.5, pose_stale_timeout_ms=250)
        self.t0 = 2_000_000_000
        self.pose = np.ones(14)

    def _step(self, offset_ns=0, pose=None, pose_recv_offset_ns=0, **kwargs):
        now = self.t0 + offset_ns
        if pose is None:
            pose = self.pose
        defaults = {
            "controller_fresh": True,
            "controller_armed": True,
            "toggle_rising": False,
        }
        defaults.update(kwargs)
        return self.machine.step(
            now_ns=now,
            live_pose=pose,
            live_pose_recv_ns=self.t0 + pose_recv_offset_ns,
            **defaults,
        )

    def test_default_blend_live_pause_hold_and_resume_with_latest_target(self):
        output, status, _ = self._step()
        self.assertEqual(status["state"], "paused_initial")
        np.testing.assert_array_equal(output, np.zeros(14))

        output, status, _ = self._step(toggle_rising=True)
        self.assertEqual(status["state"], "blending")
        np.testing.assert_array_equal(output, np.zeros(14))

        output, status, _ = self._step(offset_ns=250_000_000, pose_recv_offset_ns=250_000_000)
        np.testing.assert_allclose(output, np.full(14, 0.5))
        output, status, _ = self._step(offset_ns=500_000_000, pose_recv_offset_ns=500_000_000)
        self.assertEqual(status["state"], "live")
        np.testing.assert_allclose(output, self.pose)

        output, status, _ = self._step(
            offset_ns=510_000_000,
            pose_recv_offset_ns=510_000_000,
            toggle_rising=True,
        )
        self.assertEqual(status["state"], "paused_hold")
        held = output.copy()
        output, _, _ = self._step(
            offset_ns=600_000_000,
            pose=np.full(14, 3.0),
            pose_recv_offset_ns=600_000_000,
        )
        np.testing.assert_array_equal(output, held)

        self._step(
            offset_ns=610_000_000,
            pose=np.full(14, 3.0),
            pose_recv_offset_ns=610_000_000,
            toggle_rising=True,
        )
        output, status, _ = self._step(
            offset_ns=860_000_000,
            pose=np.full(14, 5.0),
            pose_recv_offset_ns=860_000_000,
        )
        self.assertEqual(status["state"], "blending")
        np.testing.assert_allclose(output, np.full(14, 3.0))

    def test_stale_pose_or_controller_pauses_and_never_auto_resumes(self):
        self._step(toggle_rising=True)
        self._step(offset_ns=100_000_000, pose_recv_offset_ns=100_000_000)
        held, status, _ = self._step(
            offset_ns=110_000_000,
            pose_recv_offset_ns=110_000_000,
            controller_fresh=False,
        )
        self.assertEqual(status["state"], "paused_hold")
        output, status, _ = self._step(
            offset_ns=120_000_000, pose=np.full(14, 9.0), pose_recv_offset_ns=120_000_000
        )
        self.assertEqual(status["state"], "paused_hold")
        np.testing.assert_array_equal(output, held)

        self.machine = UpperTeleopStateMachine(blend_s=0.5, pose_stale_timeout_ms=250)
        self._step(toggle_rising=True)
        output, status, _ = self._step(offset_ns=300_000_000)
        self.assertEqual(status["state"], "paused_hold")
        self.assertFalse(status["pose_fresh"])

    def test_b_with_stale_pose_does_not_start(self):
        output, status, notice = self.machine.step(
            now_ns=self.t0,
            live_pose=self.pose,
            live_pose_recv_ns=self.t0 - 300_000_000,
            controller_fresh=True,
            controller_armed=True,
            toggle_rising=True,
        )
        self.assertEqual(status["state"], "paused_initial")
        self.assertFalse(status["pose_fresh"])
        self.assertIn("remains paused", notice)
        np.testing.assert_array_equal(output, np.zeros(14))


class RCCommandWireTest(unittest.TestCase):
    def test_teleop_encoding_is_compatible_with_deploy_type(self):
        deploy_type_path = (
            REPO_DIR / "src/thor_deploy/lcm_types/rc_command_lcmt.py"
        )
        spec = importlib.util.spec_from_file_location("deploy_rc_command_lcmt", deploy_type_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        state = {
            "left_stick": [0.25, -0.5],
            "right_stick": [0.75, -1.0],
            "r1": True,
            "r2": False,
        }
        message = LowLatencyTeleopPoseZMQServer._build_rc_command_message(state)
        decoded = module.rc_command_lcmt.decode(message.encode())
        np.testing.assert_allclose(decoded.left_stick, state["left_stick"])
        np.testing.assert_allclose(decoded.right_stick, state["right_stick"])
        self.assertEqual(decoded.right_lower_left_switch, 1)
        self.assertEqual(decoded.right_lower_right_switch, 0)

        deploy_message = module.rc_command_lcmt()
        deploy_message.right_lower_right_switch = 1
        decoded_by_teleop = TeleopRCCommand.decode(deploy_message.encode())
        self.assertEqual(decoded_by_teleop.right_lower_right_switch, 1)

    def test_deploy_source_selects_exactly_one_channel(self):
        self.assertEqual(resolve_rc_command_source("unitree"), ("unitree", "rc_command"))
        self.assertEqual(resolve_rc_command_source(" PICO "), ("pico", "pico_rc_command"))
        with self.assertRaises(ValueError):
            resolve_rc_command_source("both")

    def test_rc_publisher_uses_only_the_configured_pico_channel(self):
        server = LowLatencyTeleopPoseZMQServer.__new__(LowLatencyTeleopPoseZMQServer)
        server.publish_lcm_rc = True
        server.lcm = _FakeLCM()
        server.rc_lcm_channel = "custom_pico_rc"
        server.rc_lcm_publish_count = 0
        server.rc_lcm_publish_error_reported = False
        server._publish_rc_command(
            {"left_stick": [0.0, 0.0], "right_stick": [0.0, 0.0], "r1": False, "r2": False}
        )
        self.assertEqual(server.lcm.calls[0][0], "custom_pico_rc")
        self.assertEqual(server.rc_lcm_publish_count, 1)

    def test_zmq_control_payload_preserves_buttons_and_adds_read_only_status(self):
        buttons = _buttons(right_key_two=True)
        payload = LowLatencyTeleopPoseZMQServer._build_control_payload(
            buttons,
            {"state": "paused_hold", "pose_fresh": False},
            wall_time_ms=123,
        )
        self.assertIs(payload["controller_buttons"], buttons)
        self.assertEqual(
            payload["upper_teleop"],
            {"state": "paused_hold", "pose_fresh": False},
        )


if __name__ == "__main__":
    unittest.main()
