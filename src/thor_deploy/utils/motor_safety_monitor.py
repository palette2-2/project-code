"""Stateful safety diagnostics for G1 joint telemetry."""

from dataclasses import dataclass

import numpy as np


G1_EFFORT_LIMITS = np.asarray(
    [
        88,
        88,
        88,
        139,
        50,
        50,
        88,
        88,
        88,
        139,
        50,
        50,
        88,
        50,
        50,
        25,
        25,
        25,
        25,
        25,
        5,
        5,
        25,
        25,
        25,
        25,
        25,
        5,
        5,
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class MotorSafetySnapshot:
    max_torque_ratio: float
    saturated_joint_count: int
    persistent_saturation: bool
    max_temperature_rise_c: float
    max_ankle_temperature_rise_c: float
    faulted_joint_count: int


class MotorSafetyMonitor:
    def __init__(
        self,
        effort_limits=G1_EFFORT_LIMITS,
        saturation_ratio=0.9,
        persistent_seconds=0.2,
        control_dt=0.02,
        ankle_joint_indices=(4, 5, 10, 11),
    ):
        self.effort_limits = np.asarray(effort_limits, dtype=np.float64)
        if self.effort_limits.shape != (29,) or np.any(self.effort_limits <= 0):
            raise ValueError("effort_limits must contain 29 positive values")
        self.saturation_ratio = float(saturation_ratio)
        self.persistent_seconds = float(persistent_seconds)
        self.control_dt = float(control_dt)
        if self.persistent_seconds <= 0.0 or self.control_dt <= 0.0:
            raise ValueError("persistent_seconds and control_dt must be positive")
        self._saturation_duration = np.zeros(29, dtype=np.float64)
        self._last_timestamp_us = None
        self._temperature_baseline = None
        self.ankle_joint_indices = np.asarray(ankle_joint_indices, dtype=np.int64)
        if (
            self.ankle_joint_indices.ndim != 1
            or self.ankle_joint_indices.size == 0
            or np.any(self.ankle_joint_indices < 0)
            or np.any(self.ankle_joint_indices >= 29)
        ):
            raise ValueError("ankle_joint_indices must select valid G1 joints")

    def update(self, tau_est, temperature, motor_state, timestamp_us=None):
        tau = np.asarray(tau_est, dtype=np.float64)
        temp = np.asarray(temperature, dtype=np.float64).reshape(29, 2)
        state = np.asarray(motor_state, dtype=np.int64)
        if tau.shape != (29,) or state.shape != (29,):
            raise ValueError("motor telemetry must contain 29 joints")
        if self._temperature_baseline is None:
            self._temperature_baseline = temp.copy()

        ratio = np.abs(tau) / self.effort_limits
        saturated = ratio >= self.saturation_ratio
        sample_dt = self.control_dt
        if timestamp_us is not None:
            timestamp_us = int(timestamp_us)
            if self._last_timestamp_us is None:
                sample_dt = 0.0
            elif timestamp_us > self._last_timestamp_us:
                sample_dt = min(
                    (timestamp_us - self._last_timestamp_us) / 1.0e6,
                    self.control_dt,
                )
            self._last_timestamp_us = timestamp_us
        self._saturation_duration = np.where(
            saturated, self._saturation_duration + sample_dt, 0.0
        )
        temperature_rise = temp - self._temperature_baseline
        return MotorSafetySnapshot(
            max_torque_ratio=float(np.max(ratio)),
            saturated_joint_count=int(np.count_nonzero(saturated)),
            persistent_saturation=bool(
                np.any(self._saturation_duration >= self.persistent_seconds)
            ),
            max_temperature_rise_c=float(np.max(temperature_rise)),
            max_ankle_temperature_rise_c=float(
                np.max(temperature_rise[self.ankle_joint_indices])
            ),
            faulted_joint_count=int(np.count_nonzero(state)),
        )
