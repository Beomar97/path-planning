import logging

import rclpy
from fszhaw_msgs.msg import Cone, CurrentPosition, PlannedTrajectory
from rclpy.node import Node

from path_planning.algorithm.exploration.exploration import Exploration
from path_planning.model.mode import Mode


class PathPlanner(Node):

    # set constants
    SHOW_PLOT = True
    EXPLORATION_VELOCITY = 5.0

    # set class variables
    mode = Mode.EXPLORATION
    index = 0
    current_position = [-5.3, 10.5]  # test value
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    unknown_cones = []

    def __init__(self):
        super().__init__('path_planner')

        # set logging level and format
        logging.basicConfig(level=logging.INFO,
                            format='%(levelname)s:%(message)s')

        # init cone subscriber
        self.cone_subscription = self.create_subscription(
            Cone, 'cone', self.__cone_listener_callback, 10)
        self.cone_subscription  # prevent unused variable warning

        # init current position subscriber
        self.current_position_subscription = self.create_subscription(
            CurrentPosition, 'current_position', self.__current_position_listener_callback, 10)
        self.current_position_subscription  # prevent unused variable warning

        # init planned path publisher
        self.planned_trajectory_publisher = self.create_publisher(
            PlannedTrajectory, 'planned_trajectory', 10)

    def __cone_listener_callback(self, next_cone):
        logging.info('-----------------------')
        logging.info(f'Current Position: {self.current_position}')
        logging.info(
            f'Next Cone: Tag={next_cone.color}, Coordinates=({next_cone.location.x}, {next_cone.location.y})')

        # add next cone to its corresponding list regarding it's color
        self.__add_to_received_cones(next_cone)

        if self.mode == Mode.EXPLORATION:
            output = Exploration.calculate_path(
                self.current_position, next_cone, self.blue_cones, self.yellow_cones, self.orange_cones, self.big_orange_cones, PathPlanner.SHOW_PLOT)

            # TEMPORARY set new current position
            new_current_position = output[0]
            self.__current_position_listener_callback(new_current_position)

            # publish planned path
            planned_path = output[1]
            self.__publish_planned_path(planned_path)

    def __publish_planned_path(self, planned_path):
        for point in planned_path:
            x = point[0]
            y = point[1]
            v = PathPlanner.EXPLORATION_VELOCITY

            logging.info('-----------------------')
            logging.info(
                f'Publishing Planned Path: i:{self.index} x:{x} y:{y} v:{v}')
            self.planned_trajectory_publisher.publish(PlannedTrajectory(
                index=self.index, target_x=x, target_y=y, target_velocity=v))
            self.index += 1

    def __current_position_listener_callback(self, current_position):
        logging.info('-----------------------')
        logging.info(f'Set new Position: {current_position}')
        self.current_position = current_position

    def __add_to_received_cones(self, next_cone):
        if next_cone.color == Cone.BLUE:
            self.blue_cones.append(
                [next_cone.location.x, next_cone.location.y])
        elif next_cone.color == Cone.YELLOW:
            self.yellow_cones.append(
                [next_cone.location.x, next_cone.location.y])
        elif next_cone.color == Cone.ORANGE_SMALL:
            self.orange_cones.append(
                [next_cone.location.x, next_cone.location.y])
        elif next_cone.color == Cone.ORANGE_BIG:
            self.big_orange_cones.append(
                [next_cone.location.x, next_cone.location.y])
        else:
            self.unknown_cones.append(
                [next_cone.location.x, next_cone.location.y])


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
