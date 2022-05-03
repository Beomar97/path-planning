from enum import Enum


class Mode(Enum):
    """
    Path Planner Mode.

    Different modes used by the Path Planner.
    """

    EXPLORATION = 0
    OPTIMIZATION = 1
