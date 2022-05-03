from enum import Enum


class Tag(Enum):
    """
    Cone Tag.

    Different tags (color) a cone can have.
    """

    BLUE = 'blue'
    YELLOW = 'yellow'
    ORANGE = 'orange'
    BIG_ORANGE = 'big_orange'
    UNKNOWN = 'unknown'
    CAR_START = 'car_start'
