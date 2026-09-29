import math
import unittest

from thor_deploy.utils.rc_command_mapping import (
    BaseHeightVelocityController,
    BodyYawRateWithHeadingHoldController,
    WaistYawVelocityController,
    apply_planar_deadzone,
    calibrate_stick_axis,
    continuous_deadzone,
    map_planar_velocity_stick,
)


class DeadzoneTests(unittest.TestCase):
    def test_continuous_scalar_deadzone(self):
        self.assertEqual(continuous_deadzone(0.1), 0.0)
        self.assertEqual(continuous_deadzone(-0.1), 0.0)
        self.assertAlmostEqual(continuous_deadzone(0.55), 0.5)
        self.assertAlmostEqual(continuous_deadzone(-0.55), -0.5)
        self.assertEqual(continuous_deadzone(1.5), 1.0)
        self.assertEqual(continuous_deadzone(float("nan")), 0.0)

    def test_planar_deadzone_is_applied_before_scaling(self):
        self.assertEqual(apply_planar_deadzone(0.09, -0.09), (0.0, 0.0))
        self.assertEqual(apply_planar_deadzone(0.15, 0.0), (0.15, 0.0))
        self.assertAlmostEqual(0.5 * apply_planar_deadzone(0.15, 0.0)[0], 0.075)

    def test_asymmetric_axis_calibration_reaches_both_endpoints(self):
        self.assertEqual(calibrate_stick_axis(-0.8, -0.8, 0.1, 0.9), -1.0)
        self.assertEqual(calibrate_stick_axis(0.9, -0.8, 0.1, 0.9), 1.0)
        self.assertEqual(calibrate_stick_axis(0.1, -0.8, 0.1, 0.9), 0.0)

    def test_stick_left_maps_to_positive_body_y(self):
        cmd_x, cmd_y = map_planar_velocity_stick(0.0, -1.0)
        self.assertEqual(cmd_x, 0.0)
        self.assertEqual(cmd_y, 0.5)

    def test_default_velocity_mapping_preserves_existing_limits(self):
        self.assertEqual(map_planar_velocity_stick(1.0, 0.0), (1.0, 0.0))
        self.assertEqual(map_planar_velocity_stick(-1.0, 0.0), (-1.0, 0.0))
        self.assertEqual(map_planar_velocity_stick(0.0, 1.0), (0.0, -0.5))


class WaistYawVelocityControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = WaistYawVelocityController()

    def test_full_stick_integrates_at_point_three_radians_per_second(self):
        target = self.controller.update(1.0, current_position=0.2, enabled=True)
        self.assertAlmostEqual(target, 0.206)
        target = self.controller.update(1.0, current_position=0.2, enabled=True)
        self.assertAlmostEqual(target, 0.212)
        target = self.controller.update(-1.0, current_position=0.2, enabled=True)
        self.assertAlmostEqual(target, 0.206)

    def test_release_latches_actual_position_and_then_holds_it(self):
        self.controller.update(1.0, current_position=0.2, enabled=True)
        target = self.controller.update(0.0, current_position=0.203, enabled=True)
        self.assertAlmostEqual(target, 0.203)
        target = self.controller.update(0.0, current_position=0.1, enabled=True)
        self.assertAlmostEqual(target, 0.203)

    def test_neutral_latches_position_once_when_control_is_enabled(self):
        self.assertAlmostEqual(
            self.controller.update(0.0, current_position=0.4, enabled=True), 0.4
        )
        self.assertAlmostEqual(
            self.controller.update(0.0, current_position=0.1, enabled=True), 0.4
        )

    def test_walking_to_standing_transition_latches_actual_position(self):
        self.assertAlmostEqual(
            self.controller.update(0.0, current_position=0.3, enabled=False), 0.0
        )
        self.assertAlmostEqual(
            self.controller.update(0.0, current_position=0.25, enabled=True), 0.25
        )
        self.assertAlmostEqual(
            self.controller.update(0.0, current_position=0.1, enabled=True), 0.25
        )

    def test_target_is_bounded_and_mode_reset_reinitializes_from_actual(self):
        controller = WaistYawVelocityController(control_dt=1.0, max_rate=1.0)
        self.assertEqual(controller.update(1.0, 0.9, enabled=True), 1.0)
        self.assertEqual(controller.update(0.0, 0.95, enabled=False), 0.0)
        self.assertAlmostEqual(controller.update(1.0, -0.4, enabled=True), 0.6)


class BaseHeightVelocityControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = BaseHeightVelocityController()

    def test_full_stick_integrates_at_point_one_five_meters_per_second(self):
        self.assertAlmostEqual(self.controller.update(1.0), 0.753)
        self.assertAlmostEqual(self.controller.update(1.0), 0.756)
        self.assertAlmostEqual(self.controller.update(-1.0), 0.753)

    def test_continuous_deadzone_and_neutral_hold(self):
        self.assertAlmostEqual(self.controller.update(0.1), 0.75)
        self.assertAlmostEqual(self.controller.update(0.55), 0.7515)
        self.assertAlmostEqual(self.controller.update(0.0), 0.7515)
        self.assertAlmostEqual(self.controller.update(0.0), 0.7515)

    def test_height_is_bounded(self):
        controller = BaseHeightVelocityController(control_dt=1.0, max_rate=1.0)
        self.assertEqual(controller.update(1.0), 0.8)
        self.assertEqual(controller.update(-1.0), 0.5)

    def test_reset_returns_to_nominal_height(self):
        self.controller.update(-1.0)
        self.controller.reset()
        self.assertEqual(self.controller.target, 0.75)


class BodyYawRateWithHeadingHoldControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = BodyYawRateWithHeadingHoldController()

    def test_standing_tracks_current_heading_and_outputs_zero(self):
        self.assertEqual(self.controller.update(1.0, 0.4, enabled=False), 0.0)
        self.assertEqual(self.controller.update(1.0, -0.2, enabled=False), 0.0)
        self.assertAlmostEqual(self.controller.target, -0.2)

    def test_active_stick_is_sent_directly_and_bypasses_p_control(self):
        yaw_rate = self.controller.update(1.0, current_yaw=0.2, enabled=True)
        self.assertAlmostEqual(self.controller.target, 0.2)
        self.assertAlmostEqual(yaw_rate, 1.0)

        yaw_rate = self.controller.update(0.55, current_yaw=0.4, enabled=True)
        self.assertAlmostEqual(self.controller.target, 0.4)
        self.assertAlmostEqual(yaw_rate, 0.5)

    def test_release_holds_last_active_heading_and_rejects_disturbance(self):
        self.controller.update(1.0, current_yaw=0.2, enabled=True)
        self.assertAlmostEqual(
            self.controller.update(0.0, current_yaw=0.21, enabled=True), -0.03
        )
        self.assertAlmostEqual(self.controller.target, 0.2)
        self.assertAlmostEqual(
            self.controller.update(0.0, current_yaw=0.1, enabled=True), 0.3
        )

    def test_heading_hold_uses_shortest_path_across_pi(self):
        self.controller.update(1.0, current_yaw=math.pi - 0.01, enabled=True)
        yaw_rate = self.controller.update(
            0.0, current_yaw=-math.pi + 0.01, enabled=True
        )
        self.assertAlmostEqual(yaw_rate, -0.06)

    def test_mode_transition_does_not_reuse_an_old_target(self):
        self.controller.update(1.0, current_yaw=0.0, enabled=True)
        self.controller.update(0.0, current_yaw=0.01, enabled=False)
        yaw_rate = self.controller.update(0.0, current_yaw=-0.7, enabled=True)
        self.assertAlmostEqual(yaw_rate, 0.0)
        self.assertAlmostEqual(self.controller.target, -0.7)


if __name__ == "__main__":
    unittest.main()
