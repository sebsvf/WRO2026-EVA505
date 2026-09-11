from enum import Enum, auto


class State(Enum):
    INITIALIZATION = auto()
    READY = auto()
    FOLLOW_TRACK = auto()
    OBSTACLE_AVOID = auto()
    FINISH_APPROACH = auto()
    PARKING_SEARCH = auto()
    PARKING_ALIGN = auto()
    PARKING_REVERSE = auto()
    PARKING_CORRECT = auto()
    FINISHED = auto()
    SAFE_STOP = auto()
