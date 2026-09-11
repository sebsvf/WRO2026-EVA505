"""Timestamped PID with conditional anti-windup and filtered derivative."""

import math
import time


class PID:
    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        output_limits=(None, None),
        integral_limits=(None, None),
        derivative_tau_s=0.08,
    ):
        if not all(math.isfinite(v) for v in (kp, ki, kd, derivative_tau_s)):
            raise ValueError("PID parameters must be finite")
        if derivative_tau_s < 0:
            raise ValueError("Derivative time constant must be nonnegative")
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits
        self.derivative_tau_s = derivative_tau_s
        for lower, upper in (output_limits, integral_limits):
            if any(v is not None and not math.isfinite(v) for v in (lower, upper)):
                raise ValueError("PID limits must be finite or None")
            if lower is not None and upper is not None and lower > upper:
                raise ValueError("PID lower limit exceeds upper")
        self.reset()

    def reset(self):
        self._integral = self._derivative = 0.0
        self._prev_error = self._prev_time = None

    @staticmethod
    def _clamp(value, lower, upper):
        if lower is not None:
            value = max(lower, value)
        if upper is not None:
            value = min(upper, value)
        return value

    def update(self, error, now=None):
        now = time.monotonic() if now is None else float(now)
        if not math.isfinite(error) or not math.isfinite(now):
            raise ValueError("error/now must be finite")
        dt = 0 if self._prev_time is None else now - self._prev_time
        if self._prev_time is not None and dt <= 0:
            raise ValueError("now must increase")
        if dt > 0 and self._prev_error is not None:
            raw = (error - self._prev_error) / dt
            alpha = dt / (self.derivative_tau_s + dt)
            self._derivative += alpha * (raw - self._derivative)
        candidate = (
            self._clamp(self._integral + error * dt, self.integral_min, self.integral_max)
            if self.ki
            else 0
        )
        base = self.kp * error + self.kd * self._derivative
        proposed = base + self.ki * candidate
        saturated = self._clamp(proposed, self.output_min, self.output_max)
        # Integrate if unsaturated, or if integral change brings output out of saturation.
        if proposed == saturated or (proposed - saturated) * self.ki * error <= 0:
            self._integral = candidate
        output = self._clamp(base + self.ki * self._integral, self.output_min, self.output_max)
        self._prev_error, self._prev_time = error, now
        return float(output)
