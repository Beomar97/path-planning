import rclpy
from fszhaw_msgs.msg import PlannedTrajectory
from rclpy.node import Node


class PlannedTrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('planned_trajectory_subscriber')
        self.subscription = self.create_subscription(
            PlannedTrajectory,
            'planned_trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, planned_trajectory):
        self.get_logger().info(
            f'Received: i:{planned_trajectory.index} x:{planned_trajectory.target_x} y:{planned_trajectory.target_y} v:{planned_trajectory.target_velocity}')


def main(args=None):
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
