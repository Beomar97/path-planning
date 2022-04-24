
# Source code: https://stackoverflow.com/questions/49037902/how-to-interpolate-a-line-between-two-other-lines-in-python

import numpy as np


class Interpolate:

    def calculate_path(blue_cones, yellow_cones, poly_deg=3, n_points=100):

        a1 = np.array(blue_cones)
        a2 = np.array(yellow_cones)

        min_a1_x, max_a1_x = min(a1[:,0]), max(a1[:,0])
        new_a1_x = np.linspace(min_a1_x, max_a1_x, n_points)
        a1_coefs = np.polyfit(a1[:,0],a1[:,1], poly_deg)
        new_a1_y = np.polyval(a1_coefs, new_a1_x)

        min_a2_x, max_a2_x = min(a2[:,0]), max(a2[:,0])
        new_a2_x = np.linspace(min_a2_x, max_a2_x, n_points)
        a2_coefs = np.polyfit(a2[:,0],a2[:,1], poly_deg)
        new_a2_y = np.polyval(a2_coefs, new_a2_x)

        midx = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(n_points)]
        midy = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(n_points)]

        return midx, midy