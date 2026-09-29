import unittest

import numpy as np

from thor_deploy.lcm_types.motor_safety_state_lcmt import (
    motor_safety_state_lcmt,
)
from thor_deploy.utils.motor_safety_monitor import MotorSafetyMonitor


class MotorSafetyMonitorTests(unittest.TestCase):
    def test_lcm_telemetry_round_trip_keeps_all_motor_fields(self):
        message = motor_safety_state_lcmt()
        message.tau_est = [float(index) for index in range(29)]
        message.temperature = list(range(58))
        message.motor_state[7] = 42
        message.timestamp_us = 123456789
        decoded = motor_safety_state_lcmt.decode(message.encode())
        self.assertEqual(len(decoded.tau_est), 29)
        self.assertEqual(len(decoded.temperature), 58)
        self.assertEqual(decoded.motor_state[7], 42)
        self.assertEqual(decoded.timestamp_us, 123456789)

    def test_persistent_saturation_requires_configured_duration(self):
        monitor = MotorSafetyMonitor(
            effort_limits=np.ones(29) * 10.0,
            saturation_ratio=0.9,
            persistent_seconds=0.2,
            control_dt=0.1,
        )
        tau = np.zeros(29)
        tau[4] = 9.5
        first = monitor.update(tau, np.zeros(58), np.zeros(29))
        second = monitor.update(tau, np.zeros(58), np.zeros(29))
        self.assertFalse(first.persistent_saturation)
        self.assertTrue(second.persistent_saturation)
        self.assertEqual(second.saturated_joint_count, 1)

    def test_persistence_uses_telemetry_timestamps(self):
        monitor = MotorSafetyMonitor(
            effort_limits=np.ones(29) * 10.0,
            persistent_seconds=0.2,
            control_dt=0.02,
        )
        tau = np.ones(29) * 9.5
        snapshot = None
        for index in range(37):
            snapshot = monitor.update(
                tau,
                np.zeros(58),
                np.zeros(29),
                timestamp_us=index * 5000,
            )
        self.assertFalse(snapshot.persistent_saturation)
        for index in range(37, 41):
            snapshot = monitor.update(
                tau,
                np.zeros(58),
                np.zeros(29),
                timestamp_us=index * 5000,
            )
        self.assertTrue(snapshot.persistent_saturation)

    def test_temperature_rise_and_faults_are_reported(self):
        monitor = MotorSafetyMonitor(effort_limits=np.ones(29) * 10.0)
        monitor.update(np.zeros(29), np.ones(58) * 30, np.zeros(29))
        state = np.zeros(29)
        state[5] = 3
        temperature = np.ones((29, 2)) * 30
        temperature[0] = 50
        temperature[5] = 46
        snapshot = monitor.update(
            np.zeros(29), temperature, state
        )
        self.assertEqual(snapshot.max_temperature_rise_c, 20.0)
        self.assertEqual(snapshot.max_ankle_temperature_rise_c, 16.0)
        self.assertEqual(snapshot.faulted_joint_count, 1)


if __name__ == "__main__":
    unittest.main()
