
import time


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, output_limits=(None, None),
                 integral_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits

        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def update(self, error: float, now: float = None) -> float:
        now = time.monotonic() if now is None else now

        if self._prev_time is None:
            dt = 0.0
        else:
            dt = max(1e-6, now - self._prev_time)

        # Proportional
        p_term = self.kp * error

        # Integral (with clamping to avoid windup -- relevant if ki != 0;
        # the steering controller currently runs with ki=0 by design,
        # see steering_controller.py for the rationale)
        self._integral += error * dt
        if self.integral_min is not None:
            self._integral = max(self.integral_min, self._integral)
        if self.integral_max is not None:
            self._integral = min(self.integral_max, self._integral)
        i_term = self.ki * self._integral

        # Derivative
        if self._prev_error is None or dt == 0.0:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self._prev_error) / dt

        output = p_term + i_term + d_term

        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        self._prev_error = error
        self._prev_time = now
        return output
