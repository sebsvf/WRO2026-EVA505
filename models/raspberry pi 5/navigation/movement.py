"""Temporal navigation guards; no distance or velocity is inferred."""

from dataclasses import dataclass


class SectionCounter:
    """Count one selected floor color with presence/absence hysteresis.

    The first encountered color is latched in auto mode. Each accepted crossing
    represents a provisional corner section; correspondence needs track validation.
    """

    def __init__(self, color="auto", confirm_frames=2, clear_frames=3, min_interval_s=0.8):
        self.color = None if color == "auto" else color
        self.confirm_frames = confirm_frames
        self.clear_frames = clear_frames
        self.min_interval_s = min_interval_s
        self.sections = 0
        self.last_crossing = None
        self._candidate = None
        self._hits = self._clear = 0
        self._latched = False

    def update(self, color, now):
        if color is not None and color not in ("blue", "orange"):
            raise ValueError("Unknown section color")
        relevant = color if self.color is None or color == self.color else None
        if relevant is None:
            self._hits = 0
            self._candidate = None
            self._clear += 1
            if self._clear >= self.clear_frames:
                self._latched = False
            return False
        self._clear = 0
        if relevant == self._candidate:
            self._hits += 1
        else:
            self._candidate, self._hits = relevant, 1
        if self._hits < self.confirm_frames or self._latched:
            return False
        self._latched = True
        if self.last_crossing is not None and now - self.last_crossing < self.min_interval_s:
            return False
        self.color = relevant
        self.last_crossing = now
        self.sections += 1
        return True


@dataclass(frozen=True)
class MotionStep:
    duration_s: float
    power: float
    steering_offset_deg: float


class TimedManeuver:
    """Bounded experimental plan. Duration does not prove movement or distance."""

    def __init__(self, steps):
        self.steps = [MotionStep(**item) for item in steps]
        self.started_at = None

    def start(self, now):
        self.started_at = now

    def sample(self, now):
        if self.started_at is None:
            raise RuntimeError("Maneuver not started")
        elapsed = now - self.started_at
        for step in self.steps:
            if elapsed < step.duration_s:
                return step
            elapsed -= step.duration_s
        return None
