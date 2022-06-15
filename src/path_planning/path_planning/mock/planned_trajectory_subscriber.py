"""Planned Trajectory Subscriber module."""
import logging

import rclpy
from fszhaw_msgs.msg import PlannedTrajectory
from rclpy.node import Node


class PlannedTrajectorySubscriber(Node):
    """
    Planned Trajectory Subscriber.

    Subscribes on the planned trajectory topic.
    """

    # logging
    LOG_LEVEL = logging.INFO

    def __init__(self):
        """Initialize the subscriber."""
        super().__init__('planned_trajectory_subscriber')

        # set logging level and format
        logging.basicConfig(level=PlannedTrajectorySubscriber.LOG_LEVEL,
                            format='%(levelname)s:%(message)s')

        self.subscription = self.create_subscription(
            PlannedTrajectory,
            'planned_trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, planned_trajectory):
        """
        Execute callback function.

        :param planned_trajectory: The receiving planned trajectory.
        """
        logging.info(
            f'Received: i:{planned_trajectory.index} \
                x:{planned_trajectory.target_x} \
                y:{planned_trajectory.target_y} \
                v:{planned_trajectory.target_velocity}')


def main(args=None):
    """
    Run the subscriber.

    :param args: Additional arguments (Default value = None).
    """
    rclpy.init(args=args)

    planned_trajectory_subscriber = PlannedTrajectorySubscriber()

    rclpy.spin(planned_trajectory_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planned_trajectory_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
