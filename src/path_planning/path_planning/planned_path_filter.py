"""Planned Path Filter module."""
from typing import List

from fszhaw_msgs.msg import CurrentPosition

from path_planning.model.coordinate import Coordinate
from path_planning.util.path_planning_helpers import get_distance


class PlannedPathFilter:
    """
    Filter for the Planned Path by the Exploration Algorithm.

    Filters the planned path by the Exploration Algorithm,
    so that only new path points are published to the Autopilot.
    There should not be any overlap with past path points.
    """

    # set constants
    MAX_CONES = 40

    # set class variables
    travelled_path = []

    def __init__(
        self,
        starting_position: CurrentPosition
    ):
        """Initialize the Planned Path Filter."""
        self.travelled_path.append([starting_position.vehicle_position_x,
                                   starting_position.vehicle_position_y])

    def filter_path(
        self,
        planned_path: List[Coordinate]
    ) -> List[Coordinate]:
        """
        Filter the the planned path.

        Compares the current planned path received by the Exploration Algorithm with the already travelled or published path.
        It saves all path points which haven't been published yet and returns them.

        :param planned_path: The planned path calculated by the Exploration Algorithm.
        :returns: The filtered path only containing path points not yet published.
        """
        filtered_path = []
        for planned_point in planned_path:
            no_overlap = True
            for travelled_point in self.travelled_path[-PlannedPathFilter.MAX_CONES:]:
                dist_planned_point_to_travelled_point = get_distance(
                    travelled_point, planned_point)
                if dist_planned_point_to_travelled_point < 1:
                    no_overlap = False

            if no_overlap:
                filtered_path.append(planned_point)

        if filtered_path:
            self.travelled_path.extend(filtered_path)

        return filtered_path
