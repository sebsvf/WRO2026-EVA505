

import logging
import time
from dataclasses import dataclass, field

from config.loader import validate_config
from control import commands
from control.steering_controller import SteeringController
from navigation.movement import SectionCounter, TimedManeuver
from vision.common import frame_hsv
from vision.lane_detection import LaneEstimate, estimate_lane
from vision.obstacle_detection import PillarEstimate, estimate_pillar
from vision.parking_detection import ParkingEstimate, estimate_parking_zone
from vision.section_detection import estimate_section_marker

from . import transitions as T
from .states import State

logger = logging.getLogger(__name__)
TERMINAL = (State.FINISHED, State.SAFE_STOP)


@dataclass
class FSMContext:
    lane: LaneEstimate = field(default_factory=LaneEstimate)
    pillar: PillarEstimate = field(default_factory=PillarEstimate)
    parking: ParkingEstimate = field(default_factory=ParkingEstimate)
    min_confidence: float = 0.55
    laps_required: int = 3
    completed_laps: int = 0
    lost_lane_since: float | None = None
    stop_reason: str = ""
    command_power: float = 0.0
    command_angle: float = 90.0


class RobotFSM:
    def __init__(self, camera, serial_link, config, *, clock=time.monotonic):
        validate_config(config)
        self.camera, self.serial_link, self.config = camera, serial_link, config
        self.clock = clock
        self.state = State.INITIALIZATION
        fsm = config["fsm"]
        self.ctx = FSMContext(
            min_confidence=fsm["min_confidence"], laps_required=fsm["laps_required"]
        )
        self.steering = SteeringController(**config["control"]["steering"])
        self.sections = SectionCounter(
            fsm["section_color"],
            fsm["section_confirm_frames"],
            fsm["section_clear_frames"],
            fsm["section_min_interval_s"],
        )
        self.maneuver = TimedManeuver(config["parking"]["reverse_steps"])
        self._entered_at = self.clock()
        self._started_at = None
        self._tick_start = self.clock()
        self._last_pillar_at = None
        self._last_pillar_error = None
        self._parking_hits = 0
        self._hsv = config["hsv_thresholds"]

    def _transition(self, state):
        if self.state != state:
            logger.info("%s -> %s", self.state.name, state.name)
            self.state = state
            self._entered_at = self.clock()
            self._parking_hits = 0

    def _stop(self, reason, finished=False):
        self.ctx.stop_reason = reason
        self.ctx.command_power = 0
        self._transition(State.FINISHED if finished else State.SAFE_STOP)
        try:
            self.serial_link.send(commands.stop())
        except Exception:
            logger.exception("Could not send STOP; firmware lease must expire")

    def shutdown(self):
        if self.state not in TERMINAL:
            self._stop("Application shutdown")
        else:
            try:
                self.serial_link.send(commands.stop())
            except Exception:
                logger.warning("UART already unavailable during shutdown")

    def _emit(self, power, angle):
        if self.clock() - self._tick_start > self.config["fsm"]["max_tick_s"]:
            self._stop("Perception/control deadline exceeded")
            return
        if not self.serial_link.is_alive():
            self._stop("Telemetry stale or firmware fault")
            return
        limit = self.config["control"]["power"]["maximum"]
        power = max(-limit, min(limit, power))
        self.serial_link.send(commands.drive(power, angle))
        self.ctx.command_power, self.ctx.command_angle = power, angle

    def tick(self):
        if self.state in TERMINAL:
            return
        self._tick_start = now = self.clock()
        try:
            if self.state == State.INITIALIZATION:
                if not self.serial_link.handshake_ok():
                    self._stop("ESP32 handshake timeout")
                    return
                self.serial_link.send(commands.arm())
                self._transition(State.READY)
                return
            if not self.serial_link.is_alive():
                self._stop("Telemetry stale or firmware fault")
                return
            status = self.serial_link.telemetry
            if self.state == State.READY:
                self.serial_link.send(commands.ping())
                if not status.started:
                    if not status.armed and now - self._entered_at > 1:
                        self._stop("ESP32 did not arm")
                    return
                self._started_at = now
                self.steering.reset()
                self._transition(State.FOLLOW_TRACK)
            elif not status.armed or not status.started:
                self._stop("Local START button stopped robot, or ESP32 disarmed")
                return
            if now - self._started_at >= self.config["fsm"]["max_run_s"]:
                self._stop("Maximum run duration reached")
                return
            frame = self.camera.capture_frame()
            hsv = frame_hsv(frame)
            if self.state in (State.PARKING_ALIGN, State.PARKING_REVERSE, State.PARKING_CORRECT):
                self._parking_control(frame, hsv, now)
            else:
                self._track_control(frame, hsv, now)
        except Exception as exc:
            logger.exception("Control failure")
            self._stop(f"{type(exc).__name__}: {exc}")

    def _track_control(self, frame, hsv, now):
        cfg, ctx = self.config, self.ctx
        ctx.lane = estimate_lane(frame, self._hsv, cfg["roi"]["lane_roi_y_range"], hsv=hsv)
        if T.lane_confidence_lost(ctx):
            # Zero immediately, permit brief visual recovery without automatic re-arming.
            if ctx.lost_lane_since is None:
                ctx.lost_lane_since = now
            self.steering.reset(ctx.command_angle)
            if now - ctx.lost_lane_since >= cfg["fsm"]["lost_lane_stop_timeout_s"]:
                self._stop("Lane confidence lost")
            else:
                self._emit(0, self.steering.center_deg)
            return
        ctx.lost_lane_since = None
        if self.state in (State.FOLLOW_TRACK, State.OBSTACLE_AVOID):
            marker = estimate_section_marker(frame, self._hsv, hsv=hsv)
            if self.sections.update(marker, now):
                ctx.completed_laps = self.sections.sections // cfg["fsm"]["corner_sections_per_lap"]
                logger.info(
                    "Visual sections=%d laps=%d", self.sections.sections, ctx.completed_laps
                )
            if T.laps_complete(ctx):
                if cfg["fsm"]["challenge"] == "obstacle" and cfg["parking"]["enabled"]:
                    self._transition(State.PARKING_SEARCH)
                else:
                    self._transition(State.FINISH_APPROACH)

        if self.state == State.FINISH_APPROACH:
            if now - self._entered_at >= cfg["fsm"]["finish_approach_s"]:
                self._stop(
                    "Configured visual lap target reached; parking not attempted", finished=True
                )
                return
        if self.state == State.PARKING_SEARCH:
            if now - self._entered_at > cfg["parking"]["search_timeout_s"]:
                self._stop("Parking search timeout")
                return
            ctx.parking = estimate_parking_zone(frame, self._hsv, hsv=hsv)
            self._parking_hits = self._parking_hits + 1 if T.parking_marker_found(ctx) else 0
            if self._parking_hits >= cfg["parking"]["evidence_frames"]:
                self._transition(State.PARKING_ALIGN)
                self._emit(0, self.steering.center_deg)
                return

        error, curvature = ctx.lane.lane_error, ctx.lane.curvature
        avoiding = False
        ctx.pillar = PillarEstimate()
        if cfg["fsm"]["challenge"] == "obstacle":
            candidates = estimate_pillar(
                frame, self._hsv, cfg["roi"]["pillar_roi_y_range"], hsv=hsv
            )
            valid = [
                p
                for p in candidates
                if p.confidence >= cfg["control"]["pillar_offset"]["min_confidence"]
            ]
            if valid:
                ctx.pillar = valid[0]
                x, y, w, h = ctx.pillar.pillar_bbox
                width = frame.shape[1]
                margin = cfg["control"]["pillar_offset"]["clearance_fraction"] * width
                desired_x = x + w + margin if ctx.pillar.pillar_color == "red" else x - margin
                if not 0.03 * width <= desired_x <= 0.97 * width:
                    self._stop("No visible clearance on required side of pillar")
                    return
                desired_error = (desired_x - width / 2) / (width / 2)
                error = (
                    max(error, desired_error)
                    if ctx.pillar.pillar_color == "red"
                    else min(error, desired_error)
                )
                self._last_pillar_at, self._last_pillar_error = now, error
                avoiding = True
            elif (
                self._last_pillar_at is not None
                and now - self._last_pillar_at < cfg["control"]["pillar_offset"]["hold_s"]
            ):
                error = self._last_pillar_error
                avoiding = True
        if self.state in (State.FOLLOW_TRACK, State.OBSTACLE_AVOID):
            self._transition(State.OBSTACLE_AVOID if avoiding else State.FOLLOW_TRACK)

        angle = self.steering.compute(error, curvature, now=now)
        power_cfg = cfg["control"]["power"]
        curve = max(abs(error), abs(curvature))
        power = max(
            power_cfg["minimum"], power_cfg["cruise"] * (1 - power_cfg["curve_reduction"] * curve)
        )
        # Less visual evidence -> less power; duty does not imply a speed.
        power *= min(1.0, ctx.lane.confidence / 0.8)
        if avoiding:
            power = min(power, power_cfg["obstacle"])
        if self.state in (State.PARKING_SEARCH, State.FINISH_APPROACH):
            power = min(power, cfg["parking"]["power"])
        self._emit(power, angle)

    def _parking_control(self, frame, hsv, now):
        cfg, ctx = self.config["parking"], self.ctx
        ctx.parking = estimate_parking_zone(frame, self._hsv, hsv=hsv)
        visible = T.parking_marker_found(ctx)
        if self.state == State.PARKING_REVERSE:
            if now - self._entered_at >= cfg["reverse_timeout_s"]:
                self._stop("Parking maneuver deadline reached")
                return
            step = self.maneuver.sample(now)
            if step is None:
                self._transition(State.PARKING_CORRECT)
                self._emit(0, self.steering.center_deg)
            else:
                angle = (
                    self.steering.center_deg + self.steering.direction * step.steering_offset_deg
                )
                self._emit(step.power, angle)
            return
        if now - self._entered_at >= cfg["align_timeout_s"]:
            self._stop("Parking visual alignment timeout")
            return
        if not visible:
            # Disappearance is never evidence that parking succeeded, nor a reverse trigger.
            self._parking_hits = 0
            self._emit(0, self.steering.center_deg)
            return
        aligned = abs(ctx.parking.parking_offset) <= cfg["align_tolerance"]
        close = ctx.parking.bottom_fraction >= cfg["close_bottom_fraction"]
        self._parking_hits = self._parking_hits + 1 if aligned and close else 0
        if self._parking_hits >= cfg["evidence_frames"]:
            if self.state == State.PARKING_ALIGN:
                self.maneuver.start(now)
                self._transition(State.PARKING_REVERSE)
                self._emit(0, self.steering.center_deg)
            else:
                self._stop(
                    "Parking visual criteria reached (physical position unverified)", finished=True
                )
            return
        if self.state == State.PARKING_CORRECT:
            # Observation only after the bounded plan; no endless forward corrections.
            self._emit(0, self.steering.center_deg)
            return
        angle = self.steering.compute(ctx.parking.parking_offset, 0, now=now)
        self._emit(cfg["power"], angle)
