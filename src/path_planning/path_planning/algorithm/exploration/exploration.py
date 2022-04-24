import itertools
import logging
import math

import matplotlib.pyplot as plt
from fszhaw_msgs.msg import Cone
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist


class Exploration:
    POSITION_DISTANCE_THRESHOLD = 20
    CONE_DISTANCE_THRESHOLD = 9
    CONES_THRESHOLD = 3
    EDGE_DISTANCE_THRESHOLD = 7

    def calculate_path(current_position, next_cone, blue_cones, yellow_cones, orange_cones, big_orange_cones, show_plot=False):

        # if only coordinates (x, y) are needed
        next_coordinate = [next_cone.location.x, next_cone.location.y]

        # cones, which will be used for this iteration's triangulation
        cones_to_triangulate = []
        triangulated = False

        # midpoints of the planned path
        midpoints = []

        # check if enough cones for triangulation are received
        if len(blue_cones) + len(yellow_cones) < Exploration.CONES_THRESHOLD:
            logging.info(
                f'Not enough cones! At least {Exploration.CONES_THRESHOLD} cones are needed...')
            return [current_position, midpoints]

        # calculate distance from current position to the next cone
        distance_to_next_cone = cdist(
            [current_position], [next_coordinate])[0][0]
        logging.debug(f'Distance to Next Cone: {distance_to_next_cone}')

        # check if distance from car to the next cone is too big
        if distance_to_next_cone > Exploration.POSITION_DISTANCE_THRESHOLD:
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

                if distance_to_cone <= Exploration.CONE_DISTANCE_THRESHOLD:
                    cones_to_triangulate.append(cone)
                    logging.debug(f'Add for Triangulation: {cone}')

            if len(cones_to_triangulate) >= Exploration.CONES_THRESHOLD:
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
                    # An edge is made up of two points [a[x,y],b[x,y]]
                    point_a = edge[0]
                    point_b = edge[1]

                    if point_a in blue_cones:
                        point_a_tag = Cone.BLUE
                    elif point_a in yellow_cones:
                        point_a_tag = Cone.YELLOW
                    elif point_a in orange_cones:
                        point_a_tag = Cone.ORANGE_SMALL
                    elif point_a in big_orange_cones:
                        point_a_tag = Cone.ORANGE_BIG
                    else:
                        point_a_tag = next_cone.color

                    if point_b in blue_cones:
                        point_b_tag = Cone.BLUE
                    elif point_b in yellow_cones:
                        point_b_tag = Cone.YELLOW
                    elif point_b in orange_cones:
                        point_b_tag = Cone.ORANGE_SMALL
                    elif point_b in big_orange_cones:
                        point_b_tag = Cone.ORANGE_BIG
                    else:
                        point_b_tag = next_cone.color

                    # don't use midpoints between two edges with the same tag (e.g. yellow<->yellow or blue<->blue)
                    # and use midpoint only if distance between edges is not too big
                    if point_a_tag != point_b_tag and get_length(point_a, point_b) <= Exploration.EDGE_DISTANCE_THRESHOLD:
                        midpoint = get_midpoint(point_a, point_b)
                        midpoints.append(midpoint)
                        logging.debug(
                            f'Add as Midpoint: {midpoint}')

                    if midpoints:
                        # sort midpoints by distance => furthest midpoint comes last
                        midpoints.sort(key=lambda midpoint: get_length(
                            current_position, midpoint))

                        # TODO smooth path

                        # TODO densify smoothed path with additional points

                        # TEMPORARY: set furthest midpoint as new car position
                        current_position = midpoints[-1]

        # Plotting
        if show_plot:
            plt.ion()

            # Plot current cone received
            if next_cone.color == Cone.BLUE:
                cone_plot = plt.plot(next_cone.location.x,
                                     next_cone.location.y, 'o', c='blue')
            if next_cone.color == Cone.YELLOW:
                cone_plot = plt.plot(next_cone.location.x,
                                     next_cone.location.y, 'o', c='yellow')
            if next_cone.color == Cone.ORANGE_SMALL:
                cone_plot = plt.plot(next_cone.location.x,
                                     next_cone.location.y, 'o', c='orange')
            if next_cone.color == Cone.ORANGE_BIG:
                cone_plot = plt.plot(next_cone.location.x,
                                     next_cone.location.y, 'o', c='red')

            # Plot triangulation and midpoints if happened
            if triangulated:
                cones_to_triangulate_x, cones_to_triangulate_y = zip(
                    *cones_to_triangulate)
                tri_plot = plt.triplot(cones_to_triangulate_x,
                                       cones_to_triangulate_y, triangulation.simplices)
                if midpoints:
                    midpoints_x, midpoints_y = zip(*midpoints)
                    mid_plot = plt.plot(midpoints_x, midpoints_y, 'o', c='red')

            # Plot current position
            pos_plot = plt.plot([current_position[0]], [
                                current_position[1]], 'o', c='black')

            plt.show()
            plt.pause(0.0001)

        return [current_position, midpoints]


def get_length(point_a, point_b):
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)


def get_midpoint(point_a, point_b):
    return [(point_a[0] + point_b[0])/2, (point_a[1] + point_b[1])/2]
