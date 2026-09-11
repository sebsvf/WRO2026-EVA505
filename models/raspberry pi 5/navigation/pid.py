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
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits

        for lower, upper in (output_limits, integral_limits):
            if lower is not None and upper is not None and lower > upper:
                raise ValueError("El límite mínimo debe ser <= al máximo")

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    @staticmethod
    def _clamp(value, lower, upper):
        if lower is not None:
            value = max(lower, value)
        if upper is not None:
            value = min(upper, value)
        return value

    def update(self, error: float, now: float = None) -> float:
        now = time.monotonic() if now is None else float(now)
        error = float(error)

        if not math.isfinite(now) or not math.isfinite(error):
            raise ValueError("error y now deben ser números finitos")

        dt = 0.0
        if self._prev_time is not None:
            dt = now - self._prev_time
            if dt <= 0.0:
                raise ValueError("now debe aumentar entre llamadas")

        p_term = self.kp * error

        # No acumular integral mientras está desactivada.
        if self.ki != 0.0:
            self._integral = self._clamp(
                self._integral + error * dt,
                self.integral_min,
                self.integral_max,
            )
        else:
            self._integral = 0.0

        i_term = self.ki * self._integral

        # La primera llamada no tiene derivada.
        d_term = 0.0
        if self._prev_error is not None and dt > 0.0:
            d_term = self.kd * (error - self._prev_error) / dt

        output = self._clamp(
            p_term + i_term + d_term,
            self.output_min,
            self.output_max,
        )

        self._prev_error = error
        self._prev_time = now

        return float(output)