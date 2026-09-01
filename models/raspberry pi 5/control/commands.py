

def set_speed(speed_mm_s: float) -> str:
    return f"SET_SPEED:{speed_mm_s:.1f}"


def set_steering(angle_deg: float) -> str:
    return f"SET_STEERING:{angle_deg:.1f}"


def stop() -> str:
    return "STOP"


def set_mode_park() -> str:
    return "MODE:PARK"


def set_mode_drive() -> str:
    return "MODE:DRIVE"


def ping() -> str:
    return "PING"
