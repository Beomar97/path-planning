import csv
import os
import sys

import rclpy
from fs_msgs.msg import Track
from fszhaw_msgs.msg import Cone
from geometry_msgs.msg import Point
from path_planning.model.tag import Tag
from rclpy.node import Node


class ConePublisher(Node):

    # config
    LAPS = 2  # how many laps to publish

    # set constants
    MSG_TYPE = Cone
    TOPIC = "cone"
    QUEUE_SIZE = 10
    TIMER_PERIOD = 0.2  # seconds

    # init class variables
    is_acceleration = False
    cones = []
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    unknown_cones = []
    current_lap = 1
    track_received = False

    # indexes
    i = 0  # global index
    b_i = 0  # blue
    y_i = 0  # yellow
    bo_i = 0  # big orange
    o_i = 0  # orange

    def __init__(self):
        super().__init__("cone_publisher")

        if len(sys.argv) >= 2:

            self.is_acceleration = sys.argv[1].startswith("acceleration")

            # load track information from csv
            self.get_logger().info(
                f'Loading track from csv file {sys.argv[1]}')
            with open(
                os.getcwd() + "/src/path_planning/resource/maps/" + sys.argv[1]
            ) as csv_file:
                csv_reader = csv.DictReader(csv_file, delimiter=",")

                for row in csv_reader:
                    self.cones.append(
                        [row["tag"], float(row["x"]), float(row["y"])])
                    if row["tag"] == Tag.BLUE.value:
                        self.blue_cones.append(
                            [row["tag"], float(row["x"]), float(row["y"])]
                        )
                    elif row["tag"] == Tag.YELLOW.value:
                        self.yellow_cones.append(
                            [row["tag"], float(row["x"]), float(row["y"])]
                        )
                    elif row["tag"] == Tag.ORANGE.value:
                        self.orange_cones.append(
                            [row["tag"], float(row["x"]), float(row["y"])]
                        )
                    elif row["tag"] == Tag.BIG_ORANGE.value:
                        self.big_orange_cones.append(
                            [row["tag"], float(row["x"]), float(row["y"])]
                        )
                    else:
                        self.unknown_cones.append(
                            [row["tag"], float(row["x"]), float(row["y"])]
                        )
        else:
            self.get_logger().info(
                f'Receiving track from Simulation Tool at topic testing_only/track')
            self.track_subscription = self.create_subscription(
                Track,
                'testing_only/track',
                self.__track_listener_callback,
                10)
            self.track_subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            ConePublisher.MSG_TYPE,
            ConePublisher.TOPIC,
            ConePublisher.QUEUE_SIZE
        )

        self.timer = self.create_timer(
            ConePublisher.TIMER_PERIOD, self.timer_callback)

    def __track_listener_callback(self, msg):
        if self.track_received:
            return
        else:
            for cone in msg.track:
                if cone.color == Cone.BLUE:
                    self.cones.append(
                        [Tag.BLUE.value, cone.location.x, cone.location.y])
                    self.blue_cones.append(
                        [Tag.BLUE.value, cone.location.x, cone.location.y])
                elif cone.color == Cone.YELLOW:
                    self.cones.append(
                        [Tag.YELLOW.value, cone.location.x, cone.location.y])
                    self.yellow_cones.append(
                        [Tag.YELLOW.value, cone.location.x, cone.location.y])
                elif cone.color == Cone.ORANGE_SMALL:
                    self.cones.append(
                        [Tag.ORANGE.value, cone.location.x, cone.location.y])
                    self.orange_cones.append(
                        [Tag.ORANGE.value, cone.location.x, cone.location.y])
                elif cone.color == Cone.ORANGE_BIG:
                    self.cones.append(
                        [Tag.BIG_ORANGE.value, cone.location.x, cone.location.y])
                    self.big_orange_cones.append(
                        [Tag.BIG_ORANGE.value, cone.location.x, cone.location.y])
                else:
                    self.cones.append(
                        [Tag.UNKNOWN.value, cone.location.x, cone.location.y])
                    self.unknown_cones.append(
                        [Tag.UNKNOWN.value, cone.location.x, cone.location.y])
            self.track_received = True

    def timer_callback(self):
        if self.i < len(self.cones):

            if self.is_acceleration:
                self.__handle_acceleration()
            else:
                self.__handle_trackdrive()

            self.i += 1

        else:
            if self.LAPS > self.current_lap:
                self.__reset_indexes()

    def __handle_acceleration(self):
        if self.i < 4:
            self.__publish_cone(self.big_orange_cones[self.bo_i])
            self.bo_i += 1
        elif self.b_i >= len(self.blue_cones) and self.y_i >= len(self.yellow_cones) and self.bo_i < len(self.big_orange_cones):
            self.__publish_cone(self.big_orange_cones[self.bo_i])
            self.bo_i += 1
        elif self.b_i >= len(self.blue_cones) and self.y_i >= len(self.yellow_cones) and self.bo_i >= len(self.big_orange_cones) and self.o_i < len(self.orange_cones):
            self.__publish_cone(self.orange_cones[self.o_i])
            self.o_i += 1
        elif self.i % 2 == 0:
            if self.b_i < len(self.blue_cones):
                self.__publish_cone(self.blue_cones[self.b_i])
                self.b_i += 1
        else:
            if self.y_i < len(self.yellow_cones):
                self.__publish_cone(self.yellow_cones[self.y_i])
                self.y_i += 1

    def __handle_trackdrive(self):
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

    def __publish_cone(self, cone):
        color = map_tag_to_color(cone[0])
        x = cone[1]
        y = cone[2]
        z = 0.0

        self.get_logger().info(f"Publishing {self.i}: {color}, {x}, {y}")
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


if __name__ == "__main__":
    main()
