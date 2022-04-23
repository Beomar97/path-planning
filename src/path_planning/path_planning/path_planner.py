import logging

import rclpy
from rclpy.node import Node

from interfaces.msg import Coordinate
from path_planning.algorithm.exploration.exploration import Exploration
from path_planning.model.tag import Tag


class PathPlanner(Node):

    show_plot = True

    msg_type = Coordinate
    topic = 'map'
    queue_size = 10

    cones_threshold = 3
    position_distance_threshold = 20
    cone_distance_threshold = 9
    edge_distance_threshold = 7

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
        self.unknown_cones = []

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

        # add next cone to its corresponding list regarding it's tag
        if next_cone.tag == Tag.BLUE.value:
            self.blue_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.YELLOW.value:
            self.yellow_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.ORANGE.value:
            self.orange_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.BIG_ORANGE.value:
            self.big_orange_cones.append([next_cone.x, next_cone.y])
        else:
            self.unknown_cones.append([next_cone.x, next_cone.y])

        if len(self.blue_cones) + len(self.yellow_cones) >= PathPlanner.cones_threshold:
            new_current_position = Exploration.calculate_path(
                self.current_position, next_cone, self.blue_cones, self.yellow_cones, self.orange_cones, self.big_orange_cones, PathPlanner.show_plot)

            self.current_position = new_current_position


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
