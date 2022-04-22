
# Source code: https://stackoverflow.com/questions/49037902/how-to-interpolate-a-line-between-two-other-lines-in-python

import numpy as np

class Densify:

    def calculate_path(blue_cones, yellow_cones):

        blue_cones_np = np.array(blue_cones)
        yellow_cones_np = np.array(yellow_cones)

        # Find the range of x values in the arrays
        blue_cones_min_x, blue_cones_max_x = min(blue_cones_np[:,0]), max(blue_cones_np[:,0])
        yellow_cones_min_x, yellow_cones_max_x = min(yellow_cones_np[:,0]), max(yellow_cones_np[:,0])

        # Create an evenly spaced array (100) that ranges from the minimum to the maximum => will be used as the new x values
        blue_cones_new_x = np.linspace(blue_cones_min_x, blue_cones_max_x, 100)
        yellow_cones_new_x = np.linspace(yellow_cones_min_x, yellow_cones_max_x, 100)

        # "densifying" both arrays to the same number of points
        blue_cones_new_y = np.interp(blue_cones_new_x, blue_cones_np[:,0], blue_cones_np[:,1])
        yellow_cones_new_y = np.interp(yellow_cones_new_x, yellow_cones_np[:,0], yellow_cones_np[:,1])

        # find average x and average y value for each of our estimate arrays
        # receive midpoints between our 2 estimate arrays
        path_x = [np.mean([blue_cones_new_x[i], yellow_cones_new_x[i]]) for i in range(100)]
        path_y = [np.mean([blue_cones_new_y[i], yellow_cones_new_y[i]]) for i in range(100)]

        return path_x, path_y
