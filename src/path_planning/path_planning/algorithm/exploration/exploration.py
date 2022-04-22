import itertools
import logging
import math

import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist

from path_planning.model.tag import Tag


class Exploration:
    position_distance_threshold = 20
    cone_distance_threshold = 9
    cones_threshold = 3
    edge_distance_threshold = 7

    def calculate_path(current_position, next_cone, blue_cones, yellow_cones, show_plot=False):

        # if only coordinates (x, y) are needed
        next_coordinate = [next_cone.x, next_cone.y]
        # cones, which will be used for this iteration's triangulation
        cones_to_triangulate = []
        triangulated = False

        # calculate distance from current position to the next cone
        distance_to_next_cone = cdist(
            [current_position], [next_coordinate])[0][0]
        logging.info(f'Distance to Next Cone: {distance_to_next_cone}')

        # check if distance from car to the next cone is too big
        if distance_to_next_cone > Exploration.position_distance_threshold:
            logging.info(
                'Over Threshold! Distance to Next Cone is too big...')
        else:
            cones = []
            cones.extend(blue_cones)
            cones.extend(yellow_cones)

            for cone in cones:
                distance_to_cone = cdist([next_coordinate], [cone])[0][0]

                if distance_to_cone <= Exploration.cone_distance_threshold:
                    cones_to_triangulate.append(cone)
                    logging.info(f'Add for Triangulation: {cone}')

            if len(cones_to_triangulate) >= Exploration.cones_threshold:
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

                midpoints = []
                for edge in edges:
                    # An edge is made up of two points [a[x,y],b[x,y]]
                    point_a = edge[0]
                    point_b = edge[1]

                    if point_a in blue_cones:
                        point_a_tag = Tag.BLUE.value
                    elif point_a in yellow_cones:
                        point_a_tag = Tag.YELLOW.value
                    else:
                        point_a_tag = Tag.BLUE.value if next_cone.tag == Tag.BLUE.value else Tag.YELLOW.value

                    if point_b in blue_cones:
                        point_b_tag = Tag.BLUE.value
                    elif point_b in yellow_cones:
                        point_b_tag = Tag.YELLOW.value
                    else:
                        point_b_tag = Tag.BLUE.value if next_cone.tag == Tag.BLUE.value else Tag.YELLOW.value

                    # don't use midpoints between two edges with the same tag (e.g. yellow<->yellow or blue<->blue)
                    # and use midpoint only if distance between edges is not too big
                    if point_a_tag != point_b_tag and getLength(point_a, point_b) <= Exploration.edge_distance_threshold:
                        midpoint = getMidpoint(point_a, point_b)
                        midpoints.append(midpoint)
                        logging.info(
                            f'Add as Midpoint: {midpoint}')

                    if midpoints:
                        # sort midpoints by distance => furthest midpoint comes last
                        midpoints.sort(key=lambda midpoint: getLength(
                            current_position, midpoint))
                        #  set furthest midpoint as new car position
                        current_position = midpoints[-1]

        # Plotting
        if show_plot:
            plt.ion()

            # Plot current cone received
            if next_cone.tag == Tag.BLUE.value:
                cone_plot = plt.plot(next_cone.x, next_cone.y, 'o', c='blue')
            if next_cone.tag == Tag.YELLOW.value:
                cone_plot = plt.plot(next_cone.x, next_cone.y, 'o', c='yellow')
            if next_cone.tag == Tag.ORANGE.value:
                cone_plot = plt.plot(next_cone.x, next_cone.y, 'o', c='orange')
            if next_cone.tag == Tag.BIG_ORANGE.value:
                cone_plot = plt.plot(next_cone.x, next_cone.y, 'o', c='red')

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

        return current_position


def getLength(point_a, point_b):
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)


def getMidpoint(point_a, point_b):
    return [(point_a[0] + point_b[0])/2, (point_a[1] + point_b[1])/2]
