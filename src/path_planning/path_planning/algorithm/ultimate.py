import numpy as np
from scipy.spatial.distance import cdist
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt

class Ultimate:

    def calculate_path(current_pos, blue_cones, yellow_cones):

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

        path_x = 0
        path_y = 0

        return path_x, path_y