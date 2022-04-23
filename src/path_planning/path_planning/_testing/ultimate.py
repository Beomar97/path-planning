import copy

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist

from path_planning.model.tag import Tag


class Ultimate:

    def calculate_path(current_pos, new_cone, new_blue_cones, new_yellow_cones):
        print('-----------------------')

        print('Current Pos:', current_pos)
        print('New Cone:', [new_cone.x, new_cone.y])
        print('Blue Cones:', new_blue_cones)
        print('Yellow Cones:', new_yellow_cones)

        distance_treshold = 7

        new_coordinate = [new_cone.x, new_cone.y]
        distance_to_new_cone = cdist([current_pos], [new_coordinate])[0][0]
        
        if distance_to_new_cone > distance_treshold:
            print('Over Treshold!')
            return 0,0
        else:
            print('Distance from pos to new cone:', distance_to_new_cone)

            opposite_cones = copy.deepcopy(new_yellow_cones) if new_cone.tag == Tag.BLUE.value else copy.deepcopy(new_blue_cones)
            print('Opposite Cones:', opposite_cones)

            i = 0
            while i < len(opposite_cones):
                opposite_cone = opposite_cones.pop()
                distance_to_opposite = cdist([new_coordinate], [opposite_cone])
                print('Distance to Opposite ', i, distance_to_opposite)

                if distance_to_opposite <= distance_treshold:
                    print('Delaunay with:', new_coordinate, opposite_cone, )

            print('-----------------------')
            return 5,5

            '''
            furthest_blue_cone = blue_cones[cdist([current_pos], blue_cones).argmax()]
            furthest_yellow_cone = yellow_cones[cdist([current_pos], yellow_cones).argmax()]

                cones_to_use = []
                if furthest_blue_cone > furthest_yellow_cone:
                    # use yellow
                    starting_point = furthest_yellow_cone
                    cones_to_use.append(starting_point)
                    nearest_yellow_cone = cdist([starting_point], yellow_cones)
                    test = np.argmin(np.where(nearest_yellow_cone > 0.001))
                    cones_to_use.append(yellow_cones[test])
                    distance = cdist([starting_point], blue_cones)
                    idx = np.where(distance < 7)
                    print(idx)
                    print(starting_point)
                    for i in range(len(idx[1])):
                        print(blue_cones[idx[1][i]])
                        cones_to_use.append(blue_cones[idx[1][i]])


                else:
                    # use blue
                    starting_point = furthest_blue_cone
                    cones_to_use.append(starting_point)
                    nearest_blue_cone = cdist([starting_point], blue_cones)
                    test = np.argmin(np.where(nearest_blue_cone > 0.001))
                    cones_to_use.append(blue_cones[test])
                    distance = cdist([starting_point], yellow_cones)
                    idx = np.where(distance < 7)
                    print(idx)
                    print(starting_point)
                    for i in range(len(idx[1])):
                        print(yellow_cones[idx[1][i]])
                        cones_to_use.append(yellow_cones[idx[1][i]])

                print(cones_to_use)
                # Delaunay Triangulation
                triangulation = Delaunay(np.array(cones_to_use))

            x, y = zip(*cones_to_use)
            plt.triplot(x, y, triangulation.simplices)
            plt.plot(x, y, 'o')
            plt.show()
            '''
