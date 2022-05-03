from enum import Enum


class Setup(Enum):
    """
    Path Planner Setup.

    Different setups used by the Path Planner.
    """

    ACCELERATION = 0
    SKID_PAD = 1
    TRACK_DRIVE = 2
