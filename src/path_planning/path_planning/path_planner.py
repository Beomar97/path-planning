import copy
import itertools
import logging
import math

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist

from interfaces.msg import Coordinate
from path_planning.model.tag import Tag


class PathPlanner(Node):

    msg_type = Coordinate
    topic = 'map'
    queue_size = 10

    cones_threshold = 1
    position_distance_threshold = 20
    cone_distance_threshold = 6
    edge_distance_threshold = 8

    def __init__(self):
        super().__init__('path_planner')

        # set logging level and format
        logging.basicConfig(level=logging.INFO,
                            format='%(levelname)s:%(message)s')

        self.current_position = [-5.3, 10.5]  # test value
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        self.big_orange_cones = []

        self.new_blue_cones = []  # blue cones for next Delaunay iteration
        self.new_yellow_cones = []  # yellow cones for next Delaunay iteration

        self.reset_new_cones = False

        self.subscription = self.create_subscription(
            PathPlanner.msg_type,
            PathPlanner.topic,
            self.listener_callback,
            PathPlanner.queue_size)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, next_cone):

        logging.info('-----------------------')
        logging.info(f'Current Position: {self.current_position}')
        logging.info(
            f'Next Cone: Tag={next_cone.tag}, Coordinates=({next_cone.x}, {next_cone.y})')

        # for if only coordinates (x, y) are needed
        next_coordinate = [next_cone.x, next_cone.y]

        if len(self.blue_cones) >= PathPlanner.cones_threshold and len(self.yellow_cones) >= PathPlanner.cones_threshold:

            # if empty, add last blue and/or yellow cone as new cones
            if len(self.new_blue_cones) == 0:
                self.new_blue_cones.append(
                    self.blue_cones[-1])
            if len(self.new_yellow_cones) == 0:
                self.new_yellow_cones.append(
                    self.yellow_cones[-1])

            # calculate distance from current position to the next cone
            distance_to_next_cone = cdist(
                [self.current_position], [next_coordinate])[0][0]
            logging.info(f'Distance to Next Cone: {distance_to_next_cone}')

            # check if distance from car to the next cone is too big
            if distance_to_next_cone > PathPlanner.position_distance_threshold:
                logging.info(
                    'Over Threshold! Distance to Next Cone is too big...')
                # Save cone for next triangulation
                if next_cone.tag == Tag.BLUE.value:
                    self.new_blue_cones.append(next_coordinate)
                else:
                    self.new_yellow_cones.append(next_coordinate)
            else:
                # copy cones of opposite color (blue or yellow), will check against them
                opposite_cones = copy.deepcopy(
                    self.new_yellow_cones) if next_cone.tag == Tag.BLUE.value else copy.deepcopy(self.new_blue_cones)

                # check if distance from next cone to the opposite cones is too big
                while opposite_cones:
                    opposite_cone = opposite_cones.pop()
                    distance_to_opposite = cdist(
                        [next_coordinate], [opposite_cone])[0][0]
                    logging.info(
                        f'Distance to Opposite Cone: {distance_to_opposite}')

                    # use Delaunay Triangulation if distance is not too big
                    if distance_to_opposite <= PathPlanner.cone_distance_threshold:

                        # use next cone and all new blue and yellow cones for triangulation
                        cones_to_triangulate = [next_coordinate]
                        cones_to_triangulate.extend(self.new_blue_cones)
                        cones_to_triangulate.extend(self.new_yellow_cones)
                        logging.info(
                            f'Use Triangulation with: {cones_to_triangulate}')

                        triangulation = Delaunay(cones_to_triangulate)

                        edges = []
                        for simplicy, i in itertools.product(triangulation.simplices, range(3)):
                            j = i + 1  # a simplicy is always made of three points
                            if j == 3:
                                j = 0

                            # don't want to add mirrored edges (e.g. 1<->2 and 2<->1)
                            edge = (
                                cones_to_triangulate[simplicy[i]], cones_to_triangulate[simplicy[j]])
                            edge_mirrored = (
                                cones_to_triangulate[simplicy[j]], cones_to_triangulate[simplicy[i]])
                            if edge and edge_mirrored not in edges:
                                edges.append(edge)

                        midpoints = []
                        for edge in edges:
                            # don't use midpoints between two edges with the same tag (e.g. yellow<->yellow or blue<->blue)
                            if edge[0] in self.blue_cones:
                                edge1 = Tag.BLUE.value
                            elif edge[0] in self.yellow_cones:
                                edge1 = Tag.YELLOW.value
                            else:
                                edge1 = Tag.BLUE.value if next_cone.tag == Tag.BLUE.value else Tag.YELLOW.value

                            if edge[1] in self.blue_cones:
                                edge2 = Tag.BLUE.value
                            elif edge[1] in self.yellow_cones:
                                edge2 = Tag.YELLOW.value
                            else:
                                edge2 = Tag.BLUE.value if next_cone.tag == Tag.BLUE.value else Tag.YELLOW.value

                            # use midpoint only if distance between edges is not too big
                            if edge1 != edge2 and (math.sqrt((edge[0][0] - edge[1][0]) ** 2 + (edge[0][1] - edge[1][1]) ** 2) <= PathPlanner.edge_distance_threshold):
                                midpoint = (
                                    (edge[0][0] + edge[1][0])/2, (edge[0][1] + edge[1][1])/2)
                                midpoints.append(midpoint)
                                logging.info(f'Add as Midpoint: {midpoint}')

                        if midpoints:
                            # sort midpoints by distance => furthest midpoint comes last
                            midpoints.sort(key=lambda midpoint: math.sqrt(
                                (midpoint[0] - self.current_position[0]) ** 2 + (midpoint[1] - self.current_position[1]) ** 2))
                            #  set furthest midpoint as new car position
                            self.current_position = midpoints[-1]

                        # Reset the new blue cones and new yellow cones arrays => don't use for next triangulations
                        self.reset_new_cones = True
                        self.new_blue_cones = []
                        self.new_yellow_cones = []
                        break

                # Save cone for next triangulation, if not used
                if not self.reset_new_cones:
                    if next_cone.tag == Tag.BLUE.value:
                        self.new_blue_cones.append(next_coordinate)
                    else:
                        self.new_yellow_cones.append(next_coordinate)

        # Plotting
        if next_cone.tag == Tag.BLUE.value:
            self.blue_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.YELLOW.value:
            self.yellow_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.ORANGE.value:
            self.orange_cones.append([next_cone.x, next_cone.y])
        else:
            self.big_orange_cones.append([next_cone.x, next_cone.y])

        plt.ion()

        if self.blue_cones:
            blue_cones_x, blue_cones_y = zip(*self.blue_cones)
            plt.plot(blue_cones_x, blue_cones_y, 'o', c='blue')
        if self.yellow_cones:
            yellow_cones_x, yellow_cones_y = zip(*self.yellow_cones)
            plt.plot(yellow_cones_x, yellow_cones_y, 'o', c='yellow')
        if self.orange_cones:
            orange_cones_x, orange_cones_y = zip(*self.orange_cones)
            plt.plot(orange_cones_x, orange_cones_y, 'o', c='orange')
        if self.big_orange_cones:
            big_orange_cones_x, big_orange_cones_y = zip(
                *self.big_orange_cones)
            plt.plot(big_orange_cones_x, big_orange_cones_y, 'o', c='red')

        # Plot triangulation and midpoints if happened
        if self.reset_new_cones:
            cones_to_triangulate_x, cones_to_triangulate_y = zip(
                *cones_to_triangulate)
            plt.triplot(cones_to_triangulate_x,
                        cones_to_triangulate_y, triangulation.simplices)
            midpoints_x, midpoints_y = zip(*midpoints)
            plt.plot(midpoints_x, midpoints_y, 'o', c='red')
            self.reset_new_cones = False

        plt.show()
        plt.pause(0.0001)


def main(args=None):
    rclpy.init(args=args)

    path_planner = PathPlanner()

    rclpy.spin(path_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
