import csv
import os
import sys

import rclpy
from fszhaw_msgs.msg import Cone
from geometry_msgs.msg import Point
from rclpy.node import Node

from path_planning.model.tag import Tag


class ConePublisher(Node):

    # config
    LAPS = 2  # how many laps to publish

    # set constants
    MSG_TYPE = Cone
    TOPIC = 'cone'
    QUEUE_SIZE = 10
    TIMER_PERIOD = 0.2  # seconds

    # init class variables
    cones = []
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    unknown_cones = []
    current_lap = 1

    # indexes
    i = 0  # global index
    b_i = 0  # blue
    y_i = 0  # yellow
    bo_i = 0  # big orange
    o_i = 0  # orange

    def __init__(self):
        super().__init__('cone_publisher')

        # load track information from csv
        with open(os.getcwd() + '/src/path_planning/resource/maps/' + sys.argv[1]) as csv_file:
            csv_reader = csv.DictReader(csv_file, delimiter=',')

            for row in csv_reader:
                self.cones.append(
                    [row['tag'], float(row['x']), float(row['y'])])
                if row['tag'] == Tag.BLUE.value:
                    self.blue_cones.append(
                        [row['tag'], float(row['x']), float(row['y'])])
                elif row['tag'] == Tag.YELLOW.value:
                    self.yellow_cones.append(
                        [row['tag'], float(row['x']), float(row['y'])])
                elif row['tag'] == Tag.ORANGE.value:
                    self.orange_cones.append(
                        [row['tag'], float(row['x']), float(row['y'])])
                elif row['tag'] == Tag.BIG_ORANGE.value:
                    self.big_orange_cones.append(
                        [row['tag'], float(row['x']), float(row['y'])])
                else:
                    self.unknown_cones.append(
                        [row['tag'], float(row['x']), float(row['y'])])

        self.publisher_ = self.create_publisher(
            ConePublisher.MSG_TYPE,
            ConePublisher.TOPIC,
            ConePublisher.QUEUE_SIZE
        )

        self.timer = self.create_timer(
            ConePublisher.TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        if self.i < len(self.cones):

            if self.i < len(self.big_orange_cones):
                self.__publish_cone(self.big_orange_cones[self.bo_i])
                self.bo_i += 1
            elif self.i % 2 == 0:
                if self.b_i < len(self.blue_cones):
                    self.__publish_cone(self.blue_cones[self.b_i])
                    self.b_i += 1
                else:
                    self.__publish_cone(self.yellow_cones[self.y_i])
                    self.y_i += 1
            else:
                if self.y_i < len(self.yellow_cones):
                    self.__publish_cone(self.yellow_cones[self.y_i])
                    self.y_i += 1
                else:
                    self.__publish_cone(self.blue_cones[self.b_i])
                    self.b_i += 1

            self.i += 1

        else:
            if self.LAPS > self.current_lap:
                self.__reset_indexes()

    def __publish_cone(self, cone):
        color = map_tag_to_color(cone[0])
        x = cone[1]
        y = cone[2]
        z = 0.0

        self.get_logger().info(
            f'Publishing {self.i}: {color}, {x}, {y}')
        self.publisher_.publish(
            Cone(color=color, location=Point(x=x, y=y, z=z)))

    def __reset_indexes(self):
        self.i = 0
        self.b_i = 0
        self.y_i = 0
        self.bo_i = 0
        self.o_i = 0


def map_tag_to_color(tag):
    if tag == Tag.BLUE.value:
        return Cone.BLUE
    elif tag == Tag.YELLOW.value:
        return Cone.YELLOW
    elif tag == Tag.ORANGE.value:
        return Cone.ORANGE_SMALL
    elif tag == Tag.BIG_ORANGE.value:
        return Cone.ORANGE_BIG
    else:
        return Cone.UNKNOWN


def main(args=None):
    rclpy.init(args=args)

    cone_publisher = ConePublisher()

    rclpy.spin(cone_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
