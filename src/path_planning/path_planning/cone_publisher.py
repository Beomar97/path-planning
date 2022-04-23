import csv
import os

import rclpy
from fszhaw_msgs.msg import Cone
from geometry_msgs.msg import Point
from rclpy.node import Node

from path_planning.model.tag import Tag


class ConePublisher(Node):

    msg_type = Cone
    topic = 'cone'
    queue_size = 10

    def __init__(self):
        super().__init__('cone_publisher')

        self.cones = []
        self.blue_cones = []
        self.yellow_cones = []

        with open(os.getcwd() + '/src/path_planning/resource/maps/small_track.csv') as csv_file:
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

        self.publisher_ = self.create_publisher(
            ConePublisher.msg_type,
            ConePublisher.topic,
            ConePublisher.queue_size
        )

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.b_i = 0
        self.y_i = 0

    def timer_callback(self):
        if self.i < len(self.blue_cones) + len(self.yellow_cones):

            cone = Cone()
            if self.i % 2 == 0:
                if self.b_i >= len(self.blue_cones):
                    cone = self.publish_yellow()
                else:
                    cone = self.publish_blue()
            else:
                if self.y_i >= len(self.yellow_cones):
                    cone = self.publish_blue()
                else:
                    cone = self.publish_yellow()

            self.publisher_.publish(cone)
            self.get_logger().info(
                f'Publishing {self.i}: {"BLUE" if cone.color == 0 else "YELLOW"}, {cone.location.x}, {cone.location.y}')
            self.i += 1

    def publish_yellow(self):
        tag = Cone.YELLOW
        x = self.yellow_cones[self.y_i][1]
        y = self.yellow_cones[self.y_i][2]
        z = 0.0

        self.y_i += 1
        return Cone(color=tag, location=Point(x=x, y=y, z=z))

    def publish_blue(self):
        tag = Cone.BLUE
        x = self.blue_cones[self.b_i][1]
        y = self.blue_cones[self.b_i][2]
        z = 0.0

        self.b_i += 1
        return Cone(color=tag, location=Point(x=x, y=y, z=z))


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
