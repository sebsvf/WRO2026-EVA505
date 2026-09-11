from states import State
import transitions as T
import time
from dataclasses import dataclass, field
import logging

from vision.lane_detection import (
    estimate_lane,
    LaneEstimate
)

from vision.obstacle_detection import (
    estimate_pillar,
    PillarEstimate
)

from vision.parking_detection import (
    estimate_parking_zone,
    ParkingEstimate,
    BlindApproachEstimator
)

from control.steering_controller import SteeringController
from control import commands


logger = logging.getLogger("fsm")


@dataclass
class FSMContext:

    lane: LaneEstimate = field(
        default_factory=lambda: LaneEstimate(0,0,0)
    )

    pillar: PillarEstimate = field(
        default_factory=lambda: PillarEstimate(
            False,None,None,0,0
        )
    )

    parking: ParkingEstimate = field(
        default_factory=lambda: ParkingEstimate(
            False,None,0,0
        )
    )

    encoder_ticks: int = 0

    min_confidence: float = 0.55

    laps_required: int = 3
    completed_laps: int = 0

    corner_sections_this_lap: int = 0
    corner_sections_per_lap: int = 4

    prev_curvature: float = 0

    lost_lane_since: float = None
    lost_lane_stop_timeout_s: float = 1.0



class RobotFSM:


    def __init__(
        self,
        camera,
        serial_link,
        config
    ):

        self.camera = camera
        self.serial_link = serial_link
        self.config = config

        self.state = State.INITIALIZATION


        self.ctx = FSMContext(
            min_confidence=config["fsm"]["min_confidence"],
            laps_required=config["fsm"]["laps_required"],
            corner_sections_per_lap=config["fsm"]["corner_sections_per_lap"],
            lost_lane_stop_timeout_s=config["fsm"]["lost_lane_stop_timeout_s"]
        )


        steer = config["control"]["steering"]

        self.steering = SteeringController(
            kp=steer["kp"],
            kd=steer["kd"],
            curvature_gain=steer["curvature_gain"],
            center_deg=steer["center_deg"],
            max_angle_deg=steer["max_angle_deg"]
        )


        self.pillar_bias_deg = (
            config["control"]
            ["pillar_offset"]
            ["bias_deg"]
        )


        self.pillar_ramp_deg = (
            config["control"]
            ["pillar_offset"]
            ["ramp_step_deg"]
        )


        self.blind_approach = BlindApproachEstimator()


        self._hsv = config["hsv_thresholds"]

        self._lane_roi = (
            config["roi"]
            ["lane_roi_y_range"]
        )

        self._pillar_roi = (
            config["roi"]
            ["pillar_roi_y_range"]
        )



    def tick(self):

        try:

            frame = self.camera.capture_frame()

            self.ctx.encoder_ticks = (
                self.serial_link.last_encoder_ticks
            )

        except Exception:

            logger.exception(
                "Sensor failure"
            )

            self._enter_safe_stop()

            return



        handler = getattr(
            self,
            f"_handle_{self.state.name.lower()}",
            None
        )


        if handler is None:

            logger.error(
                "Missing handler for %s",
                self.state
            )

            self._enter_safe_stop()

            return


        handler(frame)




    # ---------------- STATES ----------------


    def _handle_initialization(self, frame):

        if self.serial_link.handshake_ok():

            self.state = State.CALIBRATION

        else:

            logger.info(
                "Waiting ESP32"
            )



    def _handle_calibration(self, frame):

        from camera.calibration import (
            verify_hsv_thresholds
        )


        result = verify_hsv_thresholds(
            frame,
            self._hsv
        )


        if all(result.values()):

            self.completed_laps_reset()

            self.steering.reset()

            self.state = State.READY



    def _handle_ready(self, frame):

        pass



    def _handle_follow_track(self, frame):

        self.ctx.lane = estimate_lane(
            frame,
            self._hsv,
            self._lane_roi
        )


        self.ctx.pillar = estimate_pillar(
            frame,
            self._hsv,
            self._pillar_roi
        )


        self._update_lap_counter()



        if T.laps_complete(self.ctx):

            self.state = State.PARKING_SEARCH

            return



        if T.lane_confidence_lost(self.ctx):

            self._handle_lost_lane()

            return


        self.ctx.lost_lane_since = None



        if T.should_enter_obstacle_avoid(self.ctx):

            self.state = State.OBSTACLE_AVOID

            return



        self.steering.apply_pillar_offset(
            0,
            self.pillar_ramp_deg
        )


        angle = self.steering.compute(
            self.ctx.lane.lane_error,
            self.ctx.lane.curvature
        )


        self.serial_link.send(
            commands.set_steering(angle)
        )



    def _handle_obstacle_avoid(self, frame):

        self.ctx.lane = estimate_lane(
            frame,
            self._hsv,
            self._lane_roi
        )


        self.ctx.pillar = estimate_pillar(
            frame,
            self._hsv,
            self._pillar_roi
        )


        if T.should_exit_obstacle_avoid(self.ctx):

            self.state = State.FOLLOW_TRACK

            return



        bias = (
            self.pillar_bias_deg
            if self.ctx.pillar.pillar_color=="red"
            else -self.pillar_bias_deg
        )


        self.steering.apply_pillar_offset(
            bias,
            self.pillar_ramp_deg
        )


        angle = self.steering.compute(
            self.ctx.lane.lane_error,
            self.ctx.lane.curvature
        )


        self.serial_link.send(
            commands.set_steering(angle)
        )



    def _handle_parking_search(self, frame):

        self.serial_link.send(
            commands.set_mode_park()
        )


        self.ctx.parking = estimate_parking_zone(
            frame,
            self._hsv
        )


        if T.parking_marker_found(self.ctx):

            self.state = State.PARKING_ALIGN



    def _handle_parking_align(self, frame):

        self.ctx.parking = estimate_parking_zone(
            frame,
            self._hsv
        )


        if not self.ctx.parking.parking_lot_detected:

            return



        if T.parking_marker_lost_from_fov(self.ctx):

            self.blind_approach.start(
                self.ctx.encoder_ticks
            )

            self.state = State.PARKING_REVERSE

            return



        angle = (
            self.steering.center_deg +
            self.ctx.parking.parking_offset *
            self.steering.max_angle_deg
        )


        self.serial_link.send(
            commands.set_steering(angle)
        )

        self.serial_link.send(
            commands.set_speed(50)
        )



    def _handle_parking_reverse(self, frame):

        distance = (
            self.blind_approach
            .distance_traveled_mm(
                self.ctx.encoder_ticks
            )
        )


        self.serial_link.send(
            commands.set_speed(-50)
        )


        if distance >= 300:

            self.serial_link.send(
                commands.stop()
            )

            self.state = State.PARKING_CORRECT



    def _handle_parking_correct(self, frame):

        self.ctx.parking = estimate_parking_zone(
            frame,
            self._hsv
        )


        if (
            not self.ctx.parking.parking_lot_detected
            or abs(self.ctx.parking.parking_offset)<0.1
        ):

            self.serial_link.send(
                commands.stop()
            )

            self.state = State.FINISHED

            return



        angle = (
            self.steering.center_deg +
            self.ctx.parking.parking_offset *
            self.steering.max_angle_deg
        )


        self.serial_link.send(
            commands.set_steering(angle)
        )

        self.serial_link.send(
            commands.set_speed(30)
        )



    def _handle_finished(self, frame):

        self.serial_link.send(
            commands.stop()
        )



    def _handle_safe_stop(self, frame):

        self.serial_link.send(
            commands.stop()
        )



    # ---------------- HELPERS ----------------


    def _handle_lost_lane(self):

        now = time.monotonic()


        if self.ctx.lost_lane_since is None:

            self.ctx.lost_lane_since = now

            self.serial_link.send(
                commands.set_speed(20)
            )

            return



        if (
            now-self.ctx.lost_lane_since
            >= self.ctx.lost_lane_stop_timeout_s
        ):

            self._enter_safe_stop()



    def _enter_safe_stop(self):

        self.state = State.SAFE_STOP

        self.serial_link.send(
            commands.stop()
        )



    def completed_laps_reset(self):

        self.ctx.completed_laps = 0

        self.ctx.corner_sections_this_lap = 0



    def _update_lap_counter(self):

        if T.corner_section_entered(
            self.ctx,
            self.ctx.prev_curvature
        ):

            self.ctx.corner_sections_this_lap += 1


            if (
                self.ctx.corner_sections_this_lap
                >= self.ctx.corner_sections_per_lap
            ):

                self.ctx.completed_laps += 1

                self.ctx.corner_sections_this_lap = 0



        self.ctx.prev_curvature = (
            self.ctx.lane.curvature
        )