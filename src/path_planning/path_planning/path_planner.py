import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from interfaces.msg import Coordinate
from path_planning.model.tag import Tag
from path_planning.algorithm.densify import Densify
from path_planning.algorithm.interpolate import Interpolate
from path_planning.algorithm.ultimate import Ultimate

class PathPlanner(Node):

    msg_type = Coordinate
    topic = 'map'
    queue_size = 10

    threshold = 2
    #cone_num = 3

    def __init__(self):
        super().__init__('path_planner')

        self.current_pos = [-13.0,10.3]
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        self.big_orange_cones = []

        self.subscription = self.create_subscription(
            PathPlanner.msg_type,
            PathPlanner.topic,
            self.listener_callback,
            PathPlanner.queue_size)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, cone):
        if cone.tag == Tag.BLUE.value:
            self.blue_cones.append([cone.x, cone.y])
        elif cone.tag == Tag.YELLOW.value:
            self.yellow_cones.append([cone.x, cone.y])
        elif cone.tag == Tag.ORANGE.value:
            self.orange_cones.append([cone.x, cone.y])
        else:
            self.big_orange_cones.append([cone.x, cone.y])

        plt.ion()

        if len(self.blue_cones) >= PathPlanner.threshold and len(self.yellow_cones) >= PathPlanner.threshold:
            path_x, path_y = Ultimate.calculate_path(self.current_pos, self.blue_cones, self.yellow_cones)
            plt.plot(path_x, path_y, 'o', c='green')

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
            big_orange_cones_x, big_orange_cones_y = zip(*self.big_orange_cones)
            plt.plot(big_orange_cones_x, big_orange_cones_y, 'o', c='red')

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