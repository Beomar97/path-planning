import csv
import os
import sys

import rclpy
from fszhaw_msgs.msg import Cone, Track
from geometry_msgs.msg import Point
from path_planning.model.tag import Tag
from rclpy.node import Node


class ConePublisher(Node):
    """
    Cone Publisher.

    A mock for publishing cones to the Path Planner.
    """

    # default parameters
    TRACK_NAME = 'sim_tool'
    LAPS = 2

    # set constants
    MSG_TYPE = Cone
    TOPIC = 'cone'
    QUEUE_SIZE = 10
    TIMER_PERIOD = 0.1  # seconds

    # init class variables
    is_acceleration = False
    is_skidpad = False
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
        """
        Initialize the cone publisher.

        Loads the track from the supplied csv file or
        creates a subscribtion to receive the track from the simulation tool.
        """
        super().__init__("cone_publisher")
        # track_name ros parameter
        self.declare_parameter('track_name', ConePublisher.TRACK_NAME)
        self.track_name = self.get_parameter(
            'track_name').get_parameter_value().string_value

        # laps ros parameter
        self.declare_parameter('laps', ConePublisher.LAPS)
        self.laps = self.get_parameter(
            'laps').get_parameter_value().integer_value

        if self.track_name == 'sim_tool':

            self.get_logger().info(
                f'Receiving track from Simulation Tool at topic testing_only/track')
            self.track_subscription = self.create_subscription(
                Track,
                'testing_only/track',
                self.__track_listener_callback,
                10)
            self.track_subscription  # prevent unused variable warning

        else:

            self.is_acceleration = self.track_name.startswith("acceleration")
            self.is_skidpad = self.track_name.startswith("skidpad")

            if self.is_skidpad:
                self.__load_map('skidpad_parts/skidpad_start.csv')
                self.__load_map('skidpad_parts/skidpad_right.csv')
                self.__load_map('skidpad_parts/skidpad_left.csv')
                self.__load_map('skidpad_parts/skidpad_end.csv')
            else:
                # load track information from csv
                self.__load_map(self.track_name)

        self.publisher_ = self.create_publisher(
            ConePublisher.MSG_TYPE,
            ConePublisher.TOPIC,
            ConePublisher.QUEUE_SIZE
        )

        self.timer = self.create_timer(
            ConePublisher.TIMER_PERIOD, self.timer_callback)

    def __load_map(self, filename):
        """
        Load the track from the given csv file.

        :param filename: Filename of the csv containing the track.
        """
        self.get_logger().info(
            f'Loading track from csv file {filename}')
        with open(
            os.getcwd() + "/src/path_planning/resource/maps/" + filename
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
        self.get_logger().info(f'Blue Cones: {len(self.blue_cones)}')
        self.get_logger().info(f'Yellow Cones: {len(self.yellow_cones)}')
        self.get_logger().info(f'Orange Cones: {len(self.orange_cones)}')
        self.get_logger().info(
            f'Big Orange Cones: {len(self.big_orange_cones)}')
        self.get_logger().info(f'Unknown Cones: {len(self.unknown_cones)}')

    def __track_listener_callback(self, msg):
        """
        Execute callback function when receiving the track from the simulation tool.

        :param msg: Message with the track.
        """
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
        """Execute timer callback function for publishing cones."""
        if self.i < len(self.cones) - len(self.unknown_cones):

            if self.is_acceleration:
                self.__handle_acceleration()
            elif self.is_skidpad:
                self.__handle_skidpad()
            else:
                self.__handle_trackdrive()

            self.i += 1

        else:
            if self.laps > self.current_lap:
                self.__reset_indexes()

    def __handle_acceleration(self):
        """Publish cones logic for acceleration track."""
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

    def __handle_skidpad(self):
        """Publish cones logic for skidpad track."""
        self.__publish_cone(self.cones[self.i])

    def __handle_trackdrive(self):
        """Publish cones logic for general trackdrive tracks."""
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
        """
        Publish a cone to the Path Planner.

        :param cone: Cone to publish.
        """
        color = map_tag_to_color(cone[0])
        x = cone[1]
        y = cone[2]
        z = 0.0

        self.get_logger().debug(f"Publishing {self.i}: {color}, {x}, {y}")
        self.publisher_.publish(
            Cone(color=color, location=Point(x=x, y=y, z=z)))

    def __reset_indexes(self):
        """Reset indexes."""
        self.i = 0
        self.b_i = 0
        self.y_i = 0
        self.bo_i = 0
        self.o_i = 0

        self.current_lap += 1


def map_tag_to_color(tag):
    """
    Map a tag to it's corresponding cone color.

    :param tag: Tag of the cone or coordinate.
    """
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
    """
    Run the cone publisher.

    :param args: Additional arguments (Default value = None).
    """
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
