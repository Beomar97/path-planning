import numpy as np

class Ultimate:

    def calculate_path(current_pos, blue_cones, yellow_cones):

        furthest_blue_cone = blue_cones[cdist([refpoint], blue_cones).argmax()]
        furthest_yellow_cone = yellow_cones[cdist([refpoint], yellow_cones).argmax()]

        if furthest_blue_cone > furthest_yellow_cone:
            # use yellow
            starting_point = furthest_yellow_cone
            distance = cdist([starting_point], blue_cones)
            idx = np.where(distance < 7)
            print(idx)

        else:
            # use blue
            starting_point = furthest_blue_cone
            distance = cdist([starting_point], yellow_cones)
            idx = np.where(distance < 7)
            print(idx)

        path_x = 0
        path_y = 0

        return path_x, path_y