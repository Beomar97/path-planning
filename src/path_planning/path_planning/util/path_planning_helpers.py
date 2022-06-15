"""Module containing various helper functions for the Path Planning package."""
import math

from path_planning.model.coordinate import Coordinate


def get_distance(
    point_a: Coordinate,
    point_b: Coordinate
) -> float:
    """
    Get distance between two points.

    :param point_a: Point a.
    :param point_b: Point b.
    :returns: Distance between point a and point b.
    """
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)


def get_midpoint(
    point_a: Coordinate,
    point_b: Coordinate
) -> Coordinate:
    """
    Get the midpoint between two points.

    :param point_a: Point a.
    :param point_b: Point b.
    :returns: Midpoint between point a and point b.

    """
    return [(point_a[0] + point_b[0]) / 2, (point_a[1] + point_b[1]) / 2]
