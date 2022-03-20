import sys
import csv
import os
import rclpy
from rclpy.node import Node
from interfaces.msg import Coordinate
from path_planning.model.tag import Tag

class CoordinatePublisher(Node):

    msg_type = Coordinate
    topic = 'map'
    queue_size = 10

    def __init__(self):
        super().__init__('coordinate_publisher')

        self.cones = []
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        self.big_orange_cones = []
        self.car_start = []

        #with open(os.getcwd() + '/src/path_planning/resource/maps/small_track.csv') as csv_file:
        with open(os.getcwd() + '/src/path_planning/resource/maps/' + sys.argv[1]) as csv_file:
            csv_reader = csv.DictReader(csv_file, delimiter=',')

            for row in csv_reader:
                if row['tag'] == Tag.BLUE.value or row['tag'] == Tag.YELLOW.value or row['tag'] == Tag.ORANGE.value or row['tag'] == Tag.BIG_ORANGE.value:
                    self.cones.append([row['tag'], float(row['x']), float(row['y'])])
                    
                    if row['tag'] == Tag.BLUE.value:
                        self.blue_cones.append([row['tag'], float(row['x']), float(row['y'])])
                    elif row['tag'] == Tag.YELLOW.value:
                        self.yellow_cones.append([row['tag'], float(row['x']), float(row['y'])])
                    elif row['tag'] == Tag.ORANGE.value:
                        self.orange_cones.append([row['tag'], float(row['x']), float(row['y'])])
                    elif row['tag'] == Tag.BIG_ORANGE.value:
                        self.big_orange_cones.append([row['tag'], float(row['x']), float(row['y'])])

                elif row['tag'] == Tag.CAR_START.value:
                    self.car_start = [row['tag'], float(row['x']), float(row['y'])]

        self.publisher_ = self.create_publisher(
            CoordinatePublisher.msg_type, 
            CoordinatePublisher.topic, 
            CoordinatePublisher.queue_size
        )

        
        # publish car start
        coor_car_start = self.publish_car_start()
        self.publisher_.publish(coor_car_start)
        self.get_logger().info(f'Publishing: {coor_car_start.tag}, {coor_car_start.x}, {coor_car_start.y}')

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.b_i = 0
        self.y_i = 0
        self.o_i = 0
        self.bo_i = 0

    def timer_callback(self):
        if self.i < len(self.blue_cones) + len(self.yellow_cones):

            coordinate = Coordinate()
            if self.i % 2 == 0:
                if self.b_i >= len(self.blue_cones):
                    coordinate = self.publish_yellow()
                else:
                    coordinate = self.publish_blue()
            else:
                if self.y_i >= len(self.yellow_cones):
                    coordinate = self.publish_blue()
                else:
                    coordinate = self.publish_yellow()

            #cone.tag = self.cones[self.i][0]
            #cone.x = self.cones[self.i][1]
            #cone.y = self.cones[self.i][2]

            self.publisher_.publish(coordinate)
            self.get_logger().info(f'Publishing {self.i}: {coordinate.tag}, {coordinate.x}, {coordinate.y}')
            self.i += 1

    def publish_yellow(self):
        coordinate = Coordinate()
        coordinate.tag = self.yellow_cones[self.y_i][0]
        coordinate.x = self.yellow_cones[self.y_i][1]
        coordinate.y = self.yellow_cones[self.y_i][2]
        self.y_i += 1
        return coordinate

    def publish_blue(self):
        coordinate = Coordinate()
        coordinate.tag = self.blue_cones[self.b_i][0]
        coordinate.x = self.blue_cones[self.b_i][1]
        coordinate.y = self.blue_cones[self.b_i][2]
        self.b_i += 1
        return coordinate

    def publish_orange(self):
        coordinate = Coordinate()
        coordinate.tag = self.big_orange_cones[self.o_i][0]
        coordinate.x = self.big_orange_cones[self.o_i][1]
        coordinate.y = self.big_orange_cones[self.o_i][2]
        self.bo_i += 1
        return coordinate

    def publish_big_orange(self):
        coordinate = Coordinate()
        coordinate.tag = self.big_orange_cones[self.bo_i][0]
        coordinate.x = self.big_orange_cones[self.bo_i][1]
        coordinate.y = self.big_orange_cones[self.bo_i][2]
        self.bo_i += 1
        return coordinate

    def publish_car_start(self):
        coordinate = Coordinate()
        coordinate.tag = self.car_start[0]
        coordinate.x = self.car_start[1]
        coordinate.y = self.car_start[2]
        return coordinate


def main(args=None):
    rclpy.init(args=args)

    coordinate_publisher = CoordinatePublisher()

    rclpy.spin(coordinate_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    coordinate_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()