import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from interfaces.msg import Coordinate
from path_planning.model.tag import Tag
# from .edge import Edge
from path_planning.path_planning._testing.edge import Edge
from rclpy.node import Node
from sympy import false, true

from .delauney_method import Delauney_Method
from .delauney_method_imp import Delauney_Method_Imp
from .middle_point_method import Middle_Point_Method


class RRTPerception(Node):

    msg_type = Coordinate
    topic = 'map'
    queue_size = 10

    threshold = 3
    cone_num = 3

    def __init__(self):
        super().__init__('rrt_perception')

        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        self.big_orange_cones = []
        self.car_position = None

        self.middle_points = []

        self.x_min = 0
        self.x_max = 0
        self.y_min = 0
        self.y_max = 0

        self.del_method = Delauney_Method()
        self.middle_point_method = Middle_Point_Method()
        self.delauney_method_imp = Delauney_Method_Imp()

        self.subscription = self.create_subscription(
            RRTPerception.msg_type,
            RRTPerception.topic,
            self.listener_callback,
            RRTPerception.queue_size)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, coordinate):
        # if coordinate.tag == Tag.BLUE.value:
        #    self.blue_cones.append([coordinate.x, coordinate.y])
        #    plt.plot(coordinate.x, coordinate.y, 'o', c='blue')
        # elif coordinate.tag == Tag.YELLOW.value:
        #    self.yellow_cones.append([coordinate.x, coordinate.y])
        #    plt.plot(coordinate.x, coordinate.y, 'o', c='yellow')
        # elif coordinate.tag == Tag.ORANGE.value:
        #    self.orange_cones.append([coordinate.x, coordinate.y])
        #    plt.plot(coordinate.x, coordinate.y, 'o', c='orange')
        if coordinate.tag == Tag.CAR_START.value:
            self.car_position = coordinate
            plt.plot(coordinate.x, coordinate.y, 'o', c='black')

        plt.ion()

        if self.x_min > coordinate.x:
            self.x_min = coordinate.x
        if self.x_max < coordinate.x:
            self.x_max = coordinate.x
        if self.y_min > coordinate.y:
            self.y_min = coordinate.y
        if self.y_max < coordinate.y:
            self.y_max = coordinate.y

        plt.axis([self.x_min - 2, self.x_max + 2,
                 self.y_min - 2, self.y_max + 2])

        # self.delauney_method_imp.delauney_method_improved(coordinate,self.blue_cones,self.yellow_cones)
        newMiddlePoints = []
        # if len(self.middle_points) > 0:
        if coordinate.tag != Tag.CAR_START.value and self.car_position != None:
            newMiddlePoints = self.delauney_method_imp.delauney_method_improved(
                coordinate, self.car_position)
        # elif self.car_start != []:
        #    newMiddlePoints = self.delauney_method_imp.delauney_method_improved(coordinate, self.car_start)
        # else:
        #    newMiddlePoints = self.delauney_method_imp.delauney_method_improved(coordinate, None)

        if len(newMiddlePoints) > 0:
            for middle_point in newMiddlePoints:
                self.middle_points.append(middle_point)
            self.car_position = self.middle_points[len(self.middle_points)-1]
            plt.plot(self.car_position.x, self.car_position.y, 'o', c='black')
            print("settet")

        # self.del_method.delaunay_method(coordinate,self.blue_cones,self.yellow_cones)
        # self.middle_point_method.middle_point_method(coordinate,self.blue_cones,self.yellow_cones)

        plt.show()
        plt.pause(0.0001)


def main(args=None):
    rclpy.init(args=args)

    rrt_perception = RRTPerception()

    rclpy.spin(rrt_perception)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rrt_perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()