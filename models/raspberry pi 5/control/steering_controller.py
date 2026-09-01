
import numpy as np

from raspberry_pi.control.pid import PID


class SteeringController:
    def __init__(self, kp: float, kd: float, curvature_gain: float,
                 center_deg: float = 90.0, max_angle_deg: float = 30.0):
        self.center_deg = center_deg
        self.max_angle_deg = max_angle_deg
        self.curvature_gain = curvature_gain

        # ki is intentionally 0 -- see module docstring.
        self._pid = PID(kp=kp, ki=0.0, kd=kd,
                         output_limits=(-max_angle_deg, max_angle_deg))

        self._pillar_bias_deg = 0.0  # set by apply_pillar_offset()

    def reset(self):
        self._pid.reset()
        self._pillar_bias_deg = 0.0

    def apply_pillar_offset(self, target_bias_deg: float, ramp_step_deg: float):
        """
        Called every frame by the FSM while a pillar is in the ROI
        (target_bias_deg != 0) or has just cleared it (target_bias_deg
        == 0, ramping back to center). See state_machine/fsm.py
        OBSTACLE_AVOID handling and Sec 4.9 of the architecture doc.
        """
        if self._pillar_bias_deg < target_bias_deg:
            self._pillar_bias_deg = min(target_bias_deg,
                                         self._pillar_bias_deg + ramp_step_deg)
        elif self._pillar_bias_deg > target_bias_deg:
            self._pillar_bias_deg = max(target_bias_deg,
                                         self._pillar_bias_deg - ramp_step_deg)

    def compute(self, lane_error: float, curvature: float, now: float = None) -> float:
        """
        lane_error, curvature: normalized floats in [-1, 1], as produced
        by vision/lane_detection.py.

        Returns the target servo angle in degrees, already clamped to
        the mechanical range and including the current pillar bias.
        """
        pd_output_deg = self._pid.update(lane_error, now=now)
        feedforward_deg = self.curvature_gain * curvature * self.max_angle_deg

        angle_offset = pd_output_deg + feedforward_deg + self._pillar_bias_deg
        angle_offset = float(np.clip(angle_offset, -self.max_angle_deg, self.max_angle_deg))

        return self.center_deg + angle_offset
