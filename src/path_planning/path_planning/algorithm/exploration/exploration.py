import itertools
import logging
import math
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from fszhaw_msgs.msg import Cone, CurrentPosition
from path_planning.model.coordinate import Coordinate
from path_planning.track_config import TrackConfig
from scipy import interpolate
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist


class Exploration:
    """
    The Exploration algorithm.

    Class housing all needed functionality for the exploration algorithm.
    """

    # how many points should be used for cubic spline interpolation (smoothing)
    NR_OF_SPLINE_POINTS = 20

    def calculate_path(
        current_position: CurrentPosition,
        next_cone: Cone,
        blue_cones: List[Coordinate],
        yellow_cones: List[Coordinate],
        orange_cones: List[Coordinate],
        big_orange_cones: List[Coordinate],
        track_config, show_plot: bool = False
    ) -> Tuple[float, List[Coordinate]]:
        """
        Calculate the path for the vehicle given the current position, the receiving cone and the past received cones.

        :param current_position: Current position of the vehicle.
        :param next_cone: Next received cone from perception.
        :param blue_cones: Previously received blue cones.
        :param yellow_cones: Previously received yellow cones.
        :param orange_cones: Previously received orange cones.
        :param big_orange_cones: Previously received big orange cones.
        :param track_config: Config of the track housing the different thresholds and more.
        :param show_plot: If the calculated path and more should be plotted (Default value = False).
        :returns: An updated current position (testing only) and the planned path.

        """
        # get thresholds from track config
        POSITION_DISTANCE_THRESHOLD = track_config.POSITION_DISTANCE_THRESHOLD
        CONE_DISTANCE_THRESHOLD = track_config.CONE_DISTANCE_THRESHOLD
        CONES_THRESHOLD = track_config.CONES_THRESHOLD
        EDGE_DISTANCE_THRESHOLD = track_config.EDGE_DISTANCE_THRESHOLD
        UNKNOWN_EDGE_DISTANCE_MINIMUM = track_config.UNKNOWN_EDGE_DISTANCE_MINIMUM
        UNKNOWN_EDGE_DISTANCE_MAXIMUM = track_config.UNKNOWN_EDGE_DISTANCE_MAXIMUM

        # if only coordinates (x, y) are needed
        next_coordinate = [next_cone.location.x, next_cone.location.y]

        # cones, which will be used for this iteration's triangulation
        cones_to_triangulate = []
        triangulated = False

        # midpoints of the planned path
        midpoints = []

        # planned path to return
        planned_path = []

        # number of in total received cones
        length_cones = len(blue_cones) + len(yellow_cones) + \
            len(orange_cones) + len(big_orange_cones)

        # check if enough cones for triangulation are received
        if length_cones < CONES_THRESHOLD:
            logging.info(
                f'Not enough cones! At least {CONES_THRESHOLD} cones are needed...')
        else:

            # calculate distance from current position to the next cone
            distance_to_next_cone = cdist(
                [[current_position.vehicle_position_x, current_position.vehicle_position_y]], [next_coordinate])[0][0]
            logging.debug(f'Distance to Next Cone: {distance_to_next_cone}')

            # check if distance from car to the next cone is too big
            if distance_to_next_cone > POSITION_DISTANCE_THRESHOLD:
                logging.info(
                    'Over Threshold! Distance to Next Cone is too big...')
            else:
                cones = []
                cones.extend(blue_cones)
                cones.extend(yellow_cones)
                cones.extend(orange_cones)
                cones.extend(big_orange_cones)

                for cone in cones:
                    distance_to_cone = cdist([next_coordinate], [cone])[0][0]

                    if distance_to_cone <= CONE_DISTANCE_THRESHOLD:
                        cones_to_triangulate.append(cone)
                        logging.debug(f'Add for Triangulation: {cone}')

                if len(cones_to_triangulate) >= CONES_THRESHOLD:
                    triangulation = Delaunay(cones_to_triangulate)
                    triangulated = True

                    edges = []
                    for simplicy, i in itertools.product(triangulation.simplices, range(3)):
                        j = i + 1  # a simplicy is always made of three points
                        if j == 3:
                            j = 0

                        # don't want to add mirrored edges (e.g. 1<->2 and 2<->1)
                        edge = [cones_to_triangulate[simplicy[i]],
                                cones_to_triangulate[simplicy[j]]]
                        edge_mirrored = [
                            cones_to_triangulate[simplicy[j]], cones_to_triangulate[simplicy[i]]]

                        if edge and edge_mirrored not in edges:
                            edges.append(edge)

                    for edge in edges:
                        # an edge is made up of two points [a[x,y],b[x,y]]
                        point_a = edge[0]
                        point_b = edge[1]

                        point_a_color = determine_color(
                            point_a, next_cone, blue_cones, yellow_cones, orange_cones, big_orange_cones)
                        point_b_color = determine_color(
                            point_b, next_cone, blue_cones, yellow_cones, orange_cones, big_orange_cones)

                        # don't want to use cones of other colors except of blue and yellow for midpoints
                        if point_a_color != Cone.BLUE and point_a_color != Cone.YELLOW:
                            point_a_color = Cone.UNKNOWN
                        if point_b_color != Cone.BLUE and point_b_color != Cone.YELLOW:
                            point_b_color = Cone.UNKNOWN

                        # for (big) orange cones (labeled as unknown cones),
                        # use for midpoints if it fulfills a min and max threshold
                        if (point_a_color == Cone.UNKNOWN and point_b_color == Cone.UNKNOWN):
                            if get_distance(point_a, point_b) >= UNKNOWN_EDGE_DISTANCE_MINIMUM and get_distance(point_a, point_b) < UNKNOWN_EDGE_DISTANCE_MAXIMUM:
                                midpoint = get_midpoint(point_a, point_b)
                                midpoints.append(midpoint)
                                logging.debug(f'Add as Midpoint: {midpoint}')
                        # don't use midpoints between two edges with the same tag (e.g. yellow<->yellow or blue<->blue)
                        # and use midpoint only if distance between edges is not too big
                        else:
                            if point_a_color != Cone.UNKNOWN and point_b_color != Cone.UNKNOWN and point_a_color != point_b_color and get_distance(point_a, point_b) <= EDGE_DISTANCE_THRESHOLD:
                                midpoint = get_midpoint(point_a, point_b)
                                midpoints.append(midpoint)
                                logging.debug(f'Add as Midpoint: {midpoint}')

                        if midpoints:
                            # sort midpoints by distance => furthest midpoint comes last
                            midpoints.sort(key=lambda midpoint: get_distance(
                                [current_position.vehicle_position_x, current_position.vehicle_position_y], midpoint))

                            # smooth path
                            m = len(midpoints)
                            k = 3  # cubic spline
                            # number of elements must be +1 of the spline coefficient
                            possible_to_interpolate = True if m > k else False
                            if possible_to_interpolate:
                                midpoints_x, midpoints_y = zip(*midpoints)
                                tck, *rest = interpolate.splprep(
                                    [midpoints_x, midpoints_y], k=k)
                                u = np.linspace(
                                    0, 1, Exploration.NR_OF_SPLINE_POINTS)
                                xint, yint = interpolate.splev(u, tck)
                                # zip and map back to one list e.g. [x1, x2] [y1, y2] => [[x1, y1], [x2,y2]]
                                planned_path = list(map(list, zip(xint, yint)))
                            else:
                                planned_path = midpoints

                            # set furthest midpoint as new car position (used for testing only)
                            current_position.vehicle_position_x = midpoints[-1][0]
                            current_position.vehicle_position_y = midpoints[-1][1]

        # plotting
        if show_plot:
            plt.ion()

            # plot current cone received
            if next_cone.color == Cone.BLUE:
                plt.plot(next_cone.location.x,
                         next_cone.location.y, 'o', c='blue')
            if next_cone.color == Cone.YELLOW:
                plt.plot(next_cone.location.x,
                         next_cone.location.y, 'o', c='yellow')
            if next_cone.color == Cone.ORANGE_SMALL:
                plt.plot(next_cone.location.x,
                         next_cone.location.y, 'o', c='orange')
            if next_cone.color == Cone.ORANGE_BIG:
                plt.plot(next_cone.location.x,
                         next_cone.location.y, 'o', c='red')

            # plot triangulation and planned path if calculated
            if triangulated:
                cones_to_triangulate_x, cones_to_triangulate_y = zip(
                    *cones_to_triangulate)
                plt.triplot(cones_to_triangulate_x,
                            cones_to_triangulate_y, triangulation.simplices)
                if midpoints:
                    planned_path_x, planned_path_y = zip(*planned_path)
                    plt.plot(planned_path_x, planned_path_y, c='black')

            plt.show()
            plt.pause(0.0001)

        return current_position, planned_path


def get_distance(
    point_a: float,
    point_b: float
) -> float:
    """
    Get distance between two points.

    :param point_a: Point a.
    :param point_b: Point b.
    :returns: Distance between point a and point b.

    """
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)


def get_midpoint(
    point_a: float,
    point_b: float
) -> float:
    """
    Get the midpoint between two points.

    :param point_a: Point a.
    :param point_b: Point b.
    :returns: Midpoint between point a and point b.

    """
    return [(point_a[0] + point_b[0]) / 2, (point_a[1] + point_b[1]) / 2]


def determine_color(
    point: Coordinate,
    next_cone: Cone,
    blue_cones: List[Coordinate],
    yellow_cones: List[Coordinate],
    orange_cones: List[Coordinate],
    big_orange_cones: List[Coordinate]
) -> int:
    """
    Determine the color (tag) of a given point.

    :param point: Point to determine its color.
    :param next_cone: The current received cone.
    :param blue_cones: All previously received blue cones.
    :param yellow_cones: All previously received yellow cones.
    :returns: Color of the cone (blue, yellow, orange or big orange).

    """
    if point in blue_cones:
        point_color = Cone.BLUE
    elif point in yellow_cones:
        point_color = Cone.YELLOW
    elif point in orange_cones:
        point_color = Cone.ORANGE_SMALL
    elif point in big_orange_cones:
        point_color = Cone.ORANGE_BIG
    else:
        point_color = next_cone.color

    return point_color
