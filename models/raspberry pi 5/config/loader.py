"""Reject contradictory/nonfinite configuration before opening actuators."""

import math
from pathlib import Path

import yaml
from raspberry_pi.vision.common import roi_slice
from vision.hsv_config import DEFAULT_CONFIG, as_numpy_bounds


def _number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label}: expected a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label}: expected {low}..{high}")
    return value


def load_config(path=DEFAULT_CONFIG):
    cfg = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("Config must be a mapping")
    try:
        camera, control, fsm, park, uart = (
            cfg[k] for k in ("camera", "control", "fsm", "parking", "serial")
        )
        resolution = camera["resolution"]
        if len(resolution) != 2 or any(type(v) is not int or v < 16 for v in resolution):
            raise ValueError("camera.resolution must contain two integers >= 16")
        _number(camera["target_fps"], "camera.target_fps", 5, 120)
        _number(camera["stale_frame_timeout_s"], "camera timeout", 0.01, 0.25)
        for bounds in cfg["hsv_thresholds"].values():
            intervals = [bounds] if isinstance(bounds, dict) else bounds
            if not isinstance(intervals, (list, tuple)) or not intervals:
                raise ValueError("Each HSV color needs at least one interval")
            for interval in intervals:
                as_numpy_bounds(interval)
        required = {
            "lane_boundary",
            "pillar_red",
            "pillar_red_high",
            "pillar_green",
            "parking_marker",
            "section_blue",
            "section_orange",
        }
        if not required <= cfg["hsv_thresholds"].keys():
            raise ValueError("Missing required HSV color")
        for bounds in cfg["roi"].values():
            roi_slice(resolution[1], bounds)
        exposure, gain = camera.get("exposure_us"), camera.get("analogue_gain")
        if (exposure is None) != (gain is None):
            raise ValueError("exposure_us and analogue_gain must be set together")
        if exposure is not None:
            _number(exposure, "exposure_us", 1, 1_000_000 / camera["target_fps"])
            _number(gain, "analogue_gain", 0.01, 100)
        if camera.get("awb_gains") is not None:
            if len(camera["awb_gains"]) != 2:
                raise ValueError("awb_gains requires red and blue gain")
            for value in camera["awb_gains"]:
                _number(value, "awb gain", 0.01, 100)
        steer, power = control["steering"], control["power"]
        for name in ("kp", "kd", "curvature_gain"):
            _number(steer[name], name, 0, 100)
        center = _number(steer["center_deg"], "center", 60, 120)
        limit = _number(steer["max_angle_deg"], "steering limit", 0.1, 30)
        if center - limit < 60 or center + limit > 120:
            raise ValueError("Pi steering limits exceed ESP32 limits 60..120")
        if steer["direction"] not in (-1, 1):
            raise ValueError("steering.direction must be +1 or -1")
        _number(steer["max_rate_deg_s"], "steering rate", 1, 180)
        maximum = _number(power["maximum"], "maximum duty", 0.01, 0.35)
        for name in ("minimum", "cruise", "obstacle"):
            _number(power[name], f"power.{name}", 0, maximum)
        if power["minimum"] > min(power["cruise"], power["obstacle"]):
            raise ValueError("minimum power exceeds cruise/obstacle")
        _number(power["curve_reduction"], "curve_reduction", 0, 1)
        pillar = control["pillar_offset"]
        _number(pillar["clearance_fraction"], "pillar clearance", 0, 0.4)
        _number(pillar["hold_s"], "pillar hold", 0, 0.5)
        _number(pillar["min_confidence"], "pillar confidence", 0, 1)
        if fsm["challenge"] not in ("open", "obstacle"):
            raise ValueError("challenge must be open or obstacle")
        if fsm["section_color"] not in ("auto", "blue", "orange"):
            raise ValueError("section_color must be auto, blue or orange")
        _number(fsm["min_confidence"], "lane confidence", 0.1, 1)
        _number(fsm["lost_lane_stop_timeout_s"], "lane timeout", 0, 2)
        _number(fsm["max_tick_s"], "tick deadline", 0.01, 0.25)
        _number(fsm["max_run_s"], "run limit", 1, 180)
        _number(fsm["finish_approach_s"], "finish approach", 0, 3)
        _number(fsm["section_min_interval_s"], "section interval", 0.1, 10)
        for name in (
            "laps_required",
            "corner_sections_per_lap",
            "section_confirm_frames",
            "section_clear_frames",
        ):
            if type(fsm[name]) is not int or not 1 <= fsm[name] <= 20:
                raise ValueError(f"{name} must be an integer 1..20")
        if type(park["enabled"]) is not bool:
            raise ValueError("parking.enabled must be boolean")
        for name in ("search_timeout_s", "align_timeout_s", "reverse_timeout_s"):
            _number(park[name], name, 0.1, 30)
        _number(park["power"], "parking power", 0, maximum)
        _number(park["align_tolerance"], "parking tolerance", 0, 0.5)
        _number(park["close_bottom_fraction"], "parking proximity", 0.5, 1)
        if type(park["evidence_frames"]) is not int or park["evidence_frames"] < 2:
            raise ValueError("parking requires >= 2 evidence frames")
        total = 0
        for step in park["reverse_steps"]:
            total += _number(step["duration_s"], "maneuver duration", 0.02, 3)
            _number(step["power"], "maneuver power", -park["power"], park["power"])
            _number(step["steering_offset_deg"], "maneuver steering", -limit, limit)
        if total > park["reverse_timeout_s"]:
            raise ValueError("parking plan exceeds reverse timeout")
        if park["enabled"] and not park["reverse_steps"]:
            raise ValueError("Enabled parking requires a calibrated maneuver plan")
        if uart["port"] is not None and (not isinstance(uart["port"], str) or not uart["port"]):
            raise ValueError("serial.port must be a nonempty path or null")
        if uart["baudrate"] != 115200:
            raise ValueError("UART baudrate must match ESP32: 115200")
        for name in ("read_timeout_s", "write_timeout_s", "watchdog_reply_timeout_s"):
            _number(uart[name], name, 0.001, 0.25)
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Missing or malformed configuration: {exc}") from exc
    return cfg
