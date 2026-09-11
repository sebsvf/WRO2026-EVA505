import math
import time

from navigation.pid import PID


class SteeringController:
    def __init__(
        self,
        kp: float,
        kd: float,
        curvature_gain: float,
        center_deg: float = 90.0,
        max_angle_deg: float = 30.0,
        max_rate_deg_s: float | None = None,
        direction: int = 1,
    ):
        """
        max_angle_deg:
            Desviación máxima respecto al centro.

        max_rate_deg_s:
            Cambio máximo del ángulo ordenado, en grados/segundo.
            None desactiva este limitador.
        """
        values = (kp, kd, curvature_gain, center_deg, max_angle_deg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Los parámetros deben ser finitos")

        if max_angle_deg < 0.0:
            raise ValueError("max_angle_deg debe ser >= 0")

        if not (0.0 <= center_deg - max_angle_deg <= center_deg + max_angle_deg <= 180.0):
            raise ValueError("El rango angular debe estar entre 0 y 180°")

        if max_rate_deg_s is not None:
            if not math.isfinite(max_rate_deg_s) or max_rate_deg_s <= 0.0:
                raise ValueError("max_rate_deg_s debe ser positivo y finito")

        self.center_deg = center_deg
        self.max_angle_deg = max_angle_deg
        self.curvature_gain = curvature_gain
        self.max_rate_deg_s = max_rate_deg_s
        if direction not in (-1, 1):
            raise ValueError("direction debe ser +1 o -1")
        self.direction = direction

        self._pid = PID(
            kp=kp,
            ki=0.0,
            kd=kd,
            output_limits=(-max_angle_deg, max_angle_deg),
        )

        self.reset()

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def reset(self, current_angle_deg: float | None = None):
        """
        Reinicia el controlador.

        Si conoces el último ángulo ordenado, pásalo para conservar
        la continuidad del limitador. Si se omite, se asume el centro.
        Esta función no envía ninguna orden al servo.
        """
        angle = self.center_deg if current_angle_deg is None else float(current_angle_deg)

        if not math.isfinite(angle):
            raise ValueError("current_angle_deg debe ser finito")

        self._pid.reset()
        self._prev_time = None
        self._last_angle = self._clamp(
            angle,
            self.center_deg - self.max_angle_deg,
            self.center_deg + self.max_angle_deg,
        )

    def compute(
        self,
        lane_error: float,
        curvature: float,
        now: float | None = None,
    ) -> float:
        """
        Entradas normalizadas en [-1, 1].
        Devuelve el ángulo ordenado al servo, en grados.
        """
        now = time.monotonic() if now is None else float(now)

        if not all(math.isfinite(value) for value in (lane_error, curvature, now)):
            raise ValueError("Las entradas deben ser finitas")

        dt = 0.0
        if self._prev_time is not None:
            dt = now - self._prev_time
            if dt <= 0.0:
                raise ValueError("now debe aumentar entre llamadas")

        lane_error = self._clamp(lane_error, -1.0, 1.0)
        curvature = self._clamp(curvature, -1.0, 1.0)

        pd_output = self._pid.update(lane_error, now=now)

        feedforward = self.curvature_gain * curvature * self.max_angle_deg

        offset = self._clamp(
            pd_output + feedforward,
            -self.max_angle_deg,
            self.max_angle_deg,
        )

        target_angle = self.center_deg + self.direction * offset

        if self.max_rate_deg_s is not None:
            # Primera llamada: conserva el ángulo inicial.
            max_delta = self.max_rate_deg_s * dt
            target_angle = self._last_angle + self._clamp(
                target_angle - self._last_angle,
                -max_delta,
                max_delta,
            )

        self._last_angle = float(target_angle)
        self._prev_time = now

        return self._last_angle
