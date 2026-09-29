"""Stateful joystick mappings used by the hardware deployment controller."""

import math


def _finite_float(value, fallback=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return value if math.isfinite(value) else float(fallback)


def continuous_deadzone(value, deadzone=0.1):
    """Map a scalar stick axis continuously from its deadzone to full scale."""
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")

    value = max(-1.0, min(1.0, _finite_float(value)))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    mapped_magnitude = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(mapped_magnitude, value)


def apply_planar_deadzone(x, y, deadzone=0.1):
    """Preserve the legacy paired planar deadzone before velocity scaling."""
    x = _finite_float(x)
    y = _finite_float(y)
    if abs(x) < deadzone and abs(y) < deadzone:
        return 0.0, 0.0
    return x, y


def calibrate_stick_axis(
    value,
    negative_endpoint=-1.0,
    center=0.0,
    positive_endpoint=1.0,
):
    """Normalize an asymmetric raw stick axis to [-1, 1]."""
    negative_endpoint = _finite_float(negative_endpoint, -1.0)
    center = _finite_float(center, 0.0)
    positive_endpoint = _finite_float(positive_endpoint, 1.0)
    if not negative_endpoint < center < positive_endpoint:
        raise ValueError(
            "stick calibration must satisfy negative_endpoint < center < positive_endpoint"
        )
    value = _finite_float(value, center)
    if value >= center:
        normalized = (value - center) / (positive_endpoint - center)
    else:
        normalized = (value - center) / (center - negative_endpoint)
    return max(-1.0, min(1.0, normalized))


def map_planar_velocity_stick(
    forward_axis,
    horizontal_axis,
    forward_calibration=(-1.0, 0.0, 1.0),
    horizontal_calibration=(-1.0, 0.0, 1.0),
    deadzone=0.1,
    max_forward_speed=1.0,
    max_lateral_speed=0.5,
):
    """Map calibrated RC axes to body-frame x/y velocity commands.

    Unitree horizontal stick convention is inverted so stick-left maps to
    positive body-y (robot left).
    """
    forward = calibrate_stick_axis(forward_axis, *forward_calibration)
    horizontal = calibrate_stick_axis(horizontal_axis, *horizontal_calibration)
    raw_x, raw_y = apply_planar_deadzone(forward, -horizontal, deadzone)
    return max_forward_speed * raw_x, max_lateral_speed * raw_y


def wrap_angle_to_pi(angle):
    """Wrap a scalar angle to [-pi, pi)."""
    return (_finite_float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class WaistYawVelocityController:
    """Integrate a stick rate into a bounded waist-yaw position target."""

    def __init__(
        self,
        control_dt=0.02,
        max_rate=0.3,
        deadzone=0.1,
        lower=-1.0,
        upper=1.0,
    ):
        if control_dt <= 0.0:
            raise ValueError("control_dt must be > 0")
        if max_rate < 0.0:
            raise ValueError("max_rate must be >= 0")
        if lower >= upper:
            raise ValueError("lower must be less than upper")

        self.control_dt = float(control_dt)
        self.max_rate = float(max_rate)
        self.deadzone = float(deadzone)
        self.lower = float(lower)
        self.upper = float(upper)
        self.target = 0.0
        self.enabled = False
        self.was_active = False

    def _bounded_position(self, value):
        value = _finite_float(value, self.target if self.enabled else 0.0)
        return max(self.lower, min(self.upper, value))

    def reset(self, current_position):
        self.target = self._bounded_position(current_position)
        self.enabled = False
        self.was_active = False

    def update(self, stick_rate, current_position, enabled):
        current_position = self._bounded_position(current_position)
        if not enabled:
            self.reset(current_position)
            return 0.0

        if not self.enabled:
            self.target = current_position
            self.enabled = True

        mapped_rate = continuous_deadzone(stick_rate, self.deadzone)
        active = mapped_rate != 0.0
        if active:
            self.target = self._bounded_position(
                self.target + mapped_rate * self.max_rate * self.control_dt
            )
            self.was_active = True
        elif self.was_active:
            # Drop any untracked target error at the instant the stick returns.
            self.target = current_position
            self.was_active = False

        return self.target


class BaseHeightVelocityController:
    """Integrate a stick velocity into a bounded base-height command."""

    def __init__(
        self,
        control_dt=0.02,
        max_rate=0.15,
        deadzone=0.1,
        lower=0.5,
        upper=0.8,
        initial_height=0.75,
    ):
        if control_dt <= 0.0:
            raise ValueError("control_dt must be > 0")
        if max_rate < 0.0:
            raise ValueError("max_rate must be >= 0")
        if lower >= upper:
            raise ValueError("lower must be less than upper")

        self.control_dt = float(control_dt)
        self.max_rate = float(max_rate)
        self.deadzone = float(deadzone)
        self.lower = float(lower)
        self.upper = float(upper)
        self.initial_height = self._bounded_height(initial_height, 0.75)
        self.target = self.initial_height

    def _bounded_height(self, value, fallback):
        value = _finite_float(value, fallback)
        return max(self.lower, min(self.upper, value))

    def reset(self, target_height=None):
        if target_height is None:
            target_height = self.initial_height
        self.target = self._bounded_height(target_height, self.initial_height)

    def update(self, stick_rate):
        mapped_rate = continuous_deadzone(stick_rate, self.deadzone)
        self.target = self._bounded_height(
            self.target + mapped_rate * self.max_rate * self.control_dt,
            self.target,
        )
        return self.target


class BodyYawRateWithHeadingHoldController:
    """Pass through active yaw-rate input and hold heading after release."""

    def __init__(
        self,
        proportional_gain=3.0,
        deadzone=0.1,
        yaw_rate_limit=1.0,
    ):
        if proportional_gain < 0.0:
            raise ValueError("proportional_gain must be >= 0")
        if yaw_rate_limit <= 0.0:
            raise ValueError("yaw_rate_limit must be > 0")

        self.proportional_gain = float(proportional_gain)
        self.deadzone = float(deadzone)
        self.yaw_rate_limit = float(yaw_rate_limit)
        self.target = 0.0
        self.enabled = False

    def reset(self, current_yaw):
        self.target = wrap_angle_to_pi(current_yaw)
        self.enabled = False

    def update(self, stick_input, current_yaw, enabled):
        current_yaw = wrap_angle_to_pi(current_yaw)
        if not enabled:
            # Standing continuously tracks the measured heading and commands zero.
            self.reset(current_yaw)
            return 0.0

        if not self.enabled:
            self.target = current_yaw
            self.enabled = True

        mapped_rate = continuous_deadzone(stick_input, self.deadzone)
        active = mapped_rate != 0.0
        if active:
            # While the operator is turning, pass the requested rate directly
            # to the policy and continuously refresh the future hold heading.
            self.target = current_yaw
            yaw_rate = mapped_rate * self.yaw_rate_limit
            return max(-self.yaw_rate_limit, min(self.yaw_rate_limit, yaw_rate))

        # With the stick released, retain the last heading observed during
        # active turning and use P control only to hold that fixed heading.
        heading_error = wrap_angle_to_pi(self.target - current_yaw)
        yaw_rate = self.proportional_gain * heading_error
        return max(-self.yaw_rate_limit, min(self.yaw_rate_limit, yaw_rate))
