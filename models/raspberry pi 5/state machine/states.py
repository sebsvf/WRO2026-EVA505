from enum import Enum, auto


class State(Enum):
    INITIALIZATION = auto()
    CALIBRATION = auto()
    READY = auto()
    FOLLOW_TRACK = auto()
    OBSTACLE_AVOID = auto()
    PARKING_SEARCH = auto()
    PARKING_ALIGN = auto()
    PARKING_REVERSE = auto()
    PARKING_CORRECT = auto()
    FINISHED = auto()
    SAFE_STOP = auto()
