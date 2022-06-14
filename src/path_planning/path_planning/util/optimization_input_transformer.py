import math
from typing import List

from path_planning.model.coordinate import Coordinate
from path_planning.model.refpoint import Refpoint
from scipy.spatial.distance import cdist


class OptimizationInputTransformer:
    """
    Transformer for the input of the Optimization algorithm.

    Needs to transform the given output from the Exploration algorithm to a format the Optimization algorithm can interpret.
    """

    def transform(
        blue_cones: List[Coordinate],
        yellow_cones: List[Coordinate],
        orange_cones: List[Coordinate],
        big_orange_cones: List[Coordinate],
        refline: List[Coordinate]
    ) -> List[Refpoint]:
        """
        Transform the given input from the Exploration algorithm to a format the Optimization algorithm can use.

        :param blue_cones: All blue cones used during exploration.
        :param yellow_cones: All yellow cones used during exploration.
        :param orange_cones: All orange cones used during exploration.
        :param big_orange_cones: All big orange cones used during exploration.
        :param refline: Reference line of the track (e.g. a middle line).
        :returns: reference track including the reference points and its distance to the track limits.

        """
        reftrack = []
        refpoints_to_use = []

        # number of refpoints to use = max len of blue or yellow cones
        if len(blue_cones) > len(yellow_cones):
            for blue_cone in blue_cones:
                nearest_refpoint = refline[cdist(
                    [blue_cone], refline).argmin()]
                refpoints_to_use.append(nearest_refpoint)
        else:
            for yellow_cone in yellow_cones:
                nearest_refpoint = refline[cdist(
                    [yellow_cone], refline).argmin()]
                refpoints_to_use.append(nearest_refpoint)

        for refpoint in refpoints_to_use:
            # Get closest points from refpoint for all blue and all yellow cones (euclidean)
            # https://codereview.stackexchange.com/questions/28207/finding-the-closest-point-to-a-list-of-points
            nearest_blue_cone = blue_cones[cdist(
                [refpoint], blue_cones).argmin()]
            nearest_yellow_cone = yellow_cones[cdist(
                [refpoint], yellow_cones).argmin()]

            # Calculate distance between refpoint and nearest cone
            dist_blue = math.hypot(
                nearest_blue_cone[0] - refpoint[0], nearest_blue_cone[1] - refpoint[1])
            dist_yellow = math.hypot(
                nearest_yellow_cone[0] - refpoint[0], nearest_yellow_cone[1] - refpoint[1])

            if orange_cones:
                nearest_orange_cone = orange_cones[cdist(
                    [refpoint], orange_cones).argmin()]
                dist_orange = math.hypot(
                    nearest_orange_cone[0] - refpoint[0], nearest_orange_cone[1] - refpoint[1])
                if dist_orange < dist_blue:
                    dist_blue = dist_orange
                if dist_orange < dist_yellow:
                    dist_yellow = dist_orange
            if big_orange_cones:
                nearest_big_orange_cone = big_orange_cones[cdist(
                    [refpoint], big_orange_cones).argmin()]
                dist_big_orange = math.hypot(
                    nearest_big_orange_cone[0] - refpoint[0], nearest_big_orange_cone[1] - refpoint[1])
                if dist_big_orange < dist_blue:
                    dist_blue = dist_big_orange
                if dist_big_orange < dist_yellow:
                    dist_yellow = dist_big_orange

            reftrack.append([refpoint[0], refpoint[1], dist_yellow, dist_blue])

        return reftrack
