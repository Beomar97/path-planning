import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from interfaces.msg import Coordinate
from path_planning.model.tag import Tag
import copy
from scipy.spatial.distance import cdist
from scipy.spatial import Delaunay

class PathPlanner(Node):

    msg_type = Coordinate
    topic = 'map'
    queue_size = 10

    cone_threshold = 1
    pos_threshold = 8
    distance_threshold = 5

    def __init__(self):
        super().__init__('path_planner')

        self.current_pos = [-5.3,10.5]
        self.all_cones = []
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        self.big_orange_cones = []

        self.new_blue_cones = []
        self.new_yellow_cones = []

        self.reset = False

        self.subscription = self.create_subscription(
            PathPlanner.msg_type,
            PathPlanner.topic,
            self.listener_callback,
            PathPlanner.queue_size)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, next_cone):

        if len(self.blue_cones) >= PathPlanner.cone_threshold and len(self.yellow_cones) >= PathPlanner.cone_threshold:
            print('-----------------------')
            reset = False

            # if empty, add last blue and/or yellow cone as new cones
            if len(self.new_blue_cones) == 0:
                self.new_blue_cones.append(self.blue_cones[len(self.blue_cones)-1])
            if len(self.new_yellow_cones) == 0:
                self.new_yellow_cones.append(self.yellow_cones[len(self.yellow_cones)-1])
            
            print('Current Pos:', self.current_pos)
            print('New Cone:', [next_cone.x, next_cone.y])
            print('New Blue Cones:', self.new_blue_cones)
            print('New Yellow Cones:', self.new_yellow_cones)

            new_cone = [next_cone.x, next_cone.y]
            distance_to_new_cone = cdist([self.current_pos], [new_cone])[0][0]
        
            print('Distance from pos to new cone:', distance_to_new_cone)
            if distance_to_new_cone > PathPlanner.pos_threshold:
                print('Over Treshold!')
                if next_cone.tag == Tag.BLUE.value:
                    self.new_blue_cones.append(new_cone)
                else:
                    self.new_yellow_cones.append(new_cone)
            else:
                opposite_cones = copy.deepcopy(self.new_yellow_cones) if next_cone.tag == Tag.BLUE.value else copy.deepcopy(self.new_blue_cones)
                print('Opposite Cones:', opposite_cones)

                i = 0
                while opposite_cones:
                    opposite_cone = opposite_cones.pop()
                    distance_to_opposite = cdist([new_cone], [opposite_cone])[0][0]
                    print('Distance to Opposite ', i, distance_to_opposite)

                    if distance_to_opposite <= PathPlanner.distance_threshold:
                        print('Delaunay with:', new_cone, self.new_blue_cones, self.new_yellow_cones)
                        cones_to_triangulate = []
                        cones_to_triangulate.append(new_cone)
                        cones_to_triangulate.extend(self.new_blue_cones)
                        cones_to_triangulate.extend(self.new_yellow_cones)
                        triangulation = Delaunay(cones_to_triangulate)

                        self.reset = True
                        self.new_blue_cones = []
                        self.new_yellow_cones = []
                        break

                if not self.reset:    
                    print('add to new ones')
                    if next_cone.tag == Tag.BLUE.value:
                        self.new_blue_cones.append(new_cone)
                    else:
                        self.new_yellow_cones.append(new_cone)

        print('add general')
        if next_cone.tag == Tag.BLUE.value:
            self.blue_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.YELLOW.value:
            self.yellow_cones.append([next_cone.x, next_cone.y])
        elif next_cone.tag == Tag.ORANGE.value:
            self.orange_cones.append([next_cone.x, next_cone.y])
        else:
            self.big_orange_cones.append([next_cone.x, next_cone.y])

        self.all_cones.append([next_cone.x, next_cone.y])

        plt.ion()

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

        if self.reset:
            x, y = zip(*self.all_cones)
            plt.triplot(x, y, triangulation.simplices)
            self.reset = False

        plt.show()
        plt.pause(0.0001)

        """
        plt.ion()

        if len(self.blue_cones) >= PathPlanner.threshold and len(self.yellow_cones) >= PathPlanner.threshold:
            path_x, path_y = Ultimate.calculate_path(self.current_pos, self.blue_cones, self.yellow_cones)
            #plt.plot(path_x, path_y, 'o', c='green')

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
        """


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