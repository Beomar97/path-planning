import logging
from typing import List

import matplotlib.pyplot as plt
import rclpy
from fszhaw_msgs.msg import Cone, CurrentPosition, PlannedTrajectory
from rclpy.node import Node
from scipy.spatial.distance import cdist

from path_planning.algorithm.exploration.exploration import Exploration
from path_planning.algorithm.optimization.main_globaltraj import optimize_path
from path_planning.model.coordinate import Coordinate
from path_planning.model.mode import Mode
from path_planning.model.racetrajectory import RaceTrajectory
from path_planning.track_config import TrackConfig
from path_planning.util.optimization_input_transformer import \
    OptimizationInputTransformer


class PathPlanner(Node):
    """
    The Path Planner.

    The path planner receives the current position from Localisation and the perceived cones from Perception
    and calculates the path the vehicle should take, and optimizes the path once the full track is known.
    """

    # testing config
    PROD = False
    LOG_LEVEL = logging.INFO
    SHOW_PLOT = True
    MOCK_CURRENT_POSITION = True
    TRACK_CONFIG = TrackConfig.Rand

    # set constants
    EXPLORATION_VELOCITY = 5.0
    MAX_CONES = 50

    # set class variables
    mode = Mode.EXPLORATION
    index = 0
    laps = 0

    current_position = TRACK_CONFIG.START_CURRENT_POSITION
    all_cones = []
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    unknown_cones = []

    calculated_path = []
    optimized_path = []

    start_finish_cones = []

    def __init__(self):
        """
        Initialize the path planner.

        Creates the subscribtion for the current position and the receiving cones,
        and creates a publisher for the planned path.
        """
        super().__init__('path_planner')

        # set logging level and format
        logging.basicConfig(level=PathPlanner.LOG_LEVEL,
                            format='%(levelname)s:%(message)s')

        # init cone subscriber
        self.cone_subscription = self.create_subscription(
            Cone,
            'cone',
            self.__cone_listener_callback,
            10)
        self.cone_subscription  # prevent unused variable warning

        # init current position subscriber
        self.current_position_subscription = self.create_subscription(
            CurrentPosition,
            'current_position',
            self.__current_position_listener_callback,
            10)
        self.current_position_subscription  # prevent unused variable warning

        # init planned path publisher
        self.planned_trajectory_publisher = self.create_publisher(
            PlannedTrajectory, 'planned_trajectory', 10)

        logging.info('-----------------------')
        logging.info('Path Planner initialized!')

    def __current_position_listener_callback(self, current_position: CurrentPosition):
        """
        Execute callback function when receiving an update for the current position.

        :param current_position: The updated current position of the vehicle.
        """
        logging.debug('-----------------------')
        logging.debug(
            f'Set new Position: x:{current_position.vehicle_position_x} y:{current_position.vehicle_position_y}')
        self.current_position = current_position

    def __cone_listener_callback(self, next_cone: Cone):
        """
        Execute callback function when receiving the next cone.

        :param next_cone: The received next cone.
        """
        logging.info('-----------------------')
        logging.info(
            f'Current Position: {self.current_position.vehicle_position_x} {self.current_position.vehicle_position_y}')
        logging.info(
            f'Next Cone: Tag={next_cone.color}, \
            Coordinates=({next_cone.location.x}, {next_cone.location.y})')

        # add next cone to its corresponding list regarding it's color
        self.__add_to_received_cones(next_cone)

        # -----------

        if self.index >= 20:
            if next_cone.color == Cone.ORANGE_BIG:
                distance_to_cone = cdist(
                    [[self.current_position.vehicle_position_x,
                        self.current_position.vehicle_position_y]],
                    [[next_cone.location.x, next_cone.location.y]])[0][0]
                if distance_to_cone < 5:
                    self.start_finish_cones.append(
                        [next_cone.location.x, next_cone.location.y])
                if len(self.start_finish_cones) >= 3:
                    d = cdist([[next_cone.location.x, next_cone.location.y]],
                              self.start_finish_cones)[0]
                    for oc in d:
                        if oc < 12:
                            self.laps += 1
                            self.mode = Mode.OPTIMIZATION

            if self.index % 20 == 0:
                self.start_finish_cones.clear()

        # -----------

        if self.mode == Mode.EXPLORATION:
            last_midpoint, planned_path = Exploration.calculate_path(
                self.current_position,
                next_cone,
                self.blue_cones[-self.MAX_CONES:],
                self.yellow_cones[-self.MAX_CONES:],
                self.orange_cones,
                self.big_orange_cones,
                PathPlanner.TRACK_CONFIG,
                PathPlanner.SHOW_PLOT)

            if PathPlanner.MOCK_CURRENT_POSITION:
                # set last midpoint as new current position
                self.__current_position_listener_callback(last_midpoint)

            # publish planned path
            self.calculated_path.extend(planned_path)
            self.__publish_planned_path(planned_path)

            # if len(self.all_cones) >= PathPlanner.TRACK_CONFIG.NR_OF_CONES:
            #    self.mode = Mode.OPTIMIZATION
        else:
            if PathPlanner.SHOW_PLOT:
                plt.ioff()  # deactivate interactive mode (from exploration algorithm)

            reftrack = OptimizationInputTransformer.transform(
                self.blue_cones, self.yellow_cones, self.orange_cones, self.big_orange_cones, self.calculated_path)
            optimized_path = optimize_path(
                reftrack, PathPlanner.SHOW_PLOT)
            self.__publish_optimized_path(optimized_path)

    def __publish_planned_path(self, planned_path: List[Coordinate]):
        """
        Publish the planned path.

        :param planned_path: The planned path to be published.
        """
        for point in planned_path:
            x = point[0]
            y = point[1]
            v = PathPlanner.EXPLORATION_VELOCITY

            logging.info('-----------------------')
            logging.info(
                f'Publishing Planned Path: i:{self.index} x:{x} y:{y} velocity:{v}')
            self.planned_trajectory_publisher.publish(PlannedTrajectory(
                index=self.index, target_x=x, target_y=y, target_velocity=v))
            self.index += 1

    def __publish_optimized_path(self, optimized_path: List[RaceTrajectory]):
        """
        Publish the optimized path.

        :param optimized_path: The optimized path to be published.
        """
        for entry in optimized_path:
            x = entry[1]  # x_m
            y = entry[2]  # y_m
            v = entry[5]  # vx_mps

            logging.info('-----------------------')
            logging.info(
                f'Publishing Optimized Path: i:{self.index} x:{x} y:{y} velocity:{v}')
            self.planned_trajectory_publisher.publish(PlannedTrajectory(
                index=self.index, target_x=x, target_y=y, target_velocity=v))
            self.index += 1

    def __add_to_received_cones(self, next_cone: Cone):
        """
        Add receiving cone to a list corresponding to its color.

        :param next_cone: The received next cone.
        """
        self.all_cones.append([next_cone.location.x, next_cone.location.y])
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
    """
    Run the path planner.

    :param args: Additional arguments (Default value = None).

    """
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
