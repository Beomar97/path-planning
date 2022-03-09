import rclpy
from rclpy.node import Node
from interfaces.msg import Input

import numpy as np
import matplotlib.pyplot as plt

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')
        self.subscription = self.create_subscription(
            Input, # CHANGE for custom interface
            'input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, input):
        blue_cones = []
        for i in range(len(input.blue_cones_x)):
            blue_cones.append([input.blue_cones_x[i], input.blue_cones_y[i]])

        yellow_cones = []
        for i in range(len(input.yellow_cones_x)):
            yellow_cones.append([input.yellow_cones_x[i], input.yellow_cones_y[i]])

        self.calculatePath(np.array(blue_cones), np.array(yellow_cones))

    def calculatePath(self, blue_cones, yellow_cones):

        # Find the range of x values in the arrays
        blue_cones_min_x, blue_cones_max_x = min(blue_cones[:,0]), max(blue_cones[:,0])
        yellow_cones_min_x, yellow_cones_max_x = min(yellow_cones[:,0]), max(yellow_cones[:,0])

        # Create an evenly spaced array (100) that ranges from the minimum to the maximum => will be used as the new x values
        blue_cones_new_x = np.linspace(blue_cones_min_x, blue_cones_max_x, 100)
        yellow_cones_new_x = np.linspace(yellow_cones_min_x, yellow_cones_max_x, 100)

        # "densifying" both arrays to the same number of points
        blue_cones_new_y = np.interp(blue_cones_new_x, blue_cones[:,0], blue_cones[:,1])
        yellow_cones_new_y = np.interp(yellow_cones_new_x, yellow_cones[:,0], yellow_cones[:,1])

        # find average x and average y value for each of our estimate arrays
        # receive midpoints between our 2 estimate arrays
        path_x = [np.mean([blue_cones_new_x[i], yellow_cones_new_x[i]]) for i in range(100)]
        path_y = [np.mean([blue_cones_new_y[i], yellow_cones_new_y[i]]) for i in range(100)]

        # original points
        cones = np.concatenate((blue_cones, yellow_cones))
        cones_x, cones_y = zip(*cones)

        plt.plot(blue_cones_new_x, blue_cones_new_y,c='blue')
        plt.plot(yellow_cones_new_x, yellow_cones_new_y,c='yellow')
        plt.plot(path_x, path_y, 'o', c='green')
        plt.plot(cones_x, cones_y, 'ko')
        plt.show()


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