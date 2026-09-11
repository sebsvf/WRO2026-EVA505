
import math


def drive(power: float, angle_deg: float) -> str:
    if not all(math.isfinite(v) for v in (power, angle_deg)):
        raise ValueError("Control must be finite")
    if not -1 <= power <= 1 or not 0 <= angle_deg <= 180:
        raise ValueError("Control outside protocol range")
    return f"DRIVE:{power:.3f}:{angle_deg:.1f}"


def stop() -> str:
    return "STOP"


def arm() -> str:
    return "ARM"


def ping() -> str:
    return "PING"
