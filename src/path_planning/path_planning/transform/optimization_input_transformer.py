import math

import numpy as np
from scipy.spatial.distance import cdist


class OptimizationInputTransformer:

    def transform(blue_cones, yellow_cones, refline):

        # input
        # blue_cones: [x, y]
        # yellow cones: [x, y]
        # refline: [x, y]

        # output
        # reftrack: x_m, y_m, w_tr_right_m, w_tr_left_m
        
        reftrack = []

        for refpoint in refline:
            # Get closest points from refpoint for all blue and all yellow cones (euclidean)
            # https://codereview.stackexchange.com/questions/28207/finding-the-closest-point-to-a-list-of-points
            nearest_blue_cone = blue_cones[cdist([refpoint], blue_cones).argmin()]
            nearest_yellow_cone = yellow_cones[cdist([refpoint], yellow_cones).argmin()]

            # Calculate distance between refpoint and nearest cone
            dist_blue = math.hypot(nearest_blue_cone[0]-refpoint[0], nearest_blue_cone[1]-refpoint[1])
            dist_yellow = math.hypot(nearest_yellow_cone[0]-refpoint[0], nearest_yellow_cone[1]-refpoint[1])
            
            reftrack.append([refpoint[0], refpoint[1], dist_yellow, dist_blue])

        return reftrack
        