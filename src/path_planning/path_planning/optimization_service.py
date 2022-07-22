"""Optimization Service module."""
import logging

import rclpy
from interfaces.srv import OptimizePath
from rclpy.node import Node

from path_planning.algorithm.optimization.main_globaltraj import optimize_path
from path_planning.optimization_input_transformer import \
    OptimizationInputTransformer


class OptimizationService(Node):
    """
    The Optimization Service.

    The service receives the received cones and the reference line (midpoints) from the client (path planner)
    and calculates the optimized path with it.
    """

    # default parameters
    OPTIMIZATION_TYPE = "mincurv"
    MINIMUM_TRACK_WIDTH = None
    SHOW_PLOT_OPTIMIZATION = False

    # logging
    LOG_LEVEL = logging.INFO

    def __init__(self):
        """
        Initialize the optimization service.

        Creates the ROS service.
        """
        super().__init__('optimization_service')

        # set logging level and format
        logging.basicConfig(level=OptimizationService.LOG_LEVEL,
                            format='%(levelname)s:%(message)s')

        # initialize ROS parameters
        self.__initialize_parameters()

        self.srv = self.create_service(OptimizePath,
                                       'optimize_path',
                                       self.__optimize_path_callback)

        logging.info('-----------------------')
        logging.info('Optimization Service initialized!')

    def __initialize_parameters(self):
        """
        Initialize the ROS parameters.

        Initializes the ROS parameters with the passed values
        or with the default values.
        """
        # optimization type ros parameter
        self.declare_parameter('optimization_type',
                               OptimizationService.OPTIMIZATION_TYPE)
        self.optimization_type = self.get_parameter(
            'optimization_type').get_parameter_value().string_value

        # minimum track width ros parameter
        self.declare_parameter('minimum_track_width',
                               OptimizationService.MINIMUM_TRACK_WIDTH)
        self.minimum_track_width = self.get_parameter(
            'minimum_track_width').get_parameter_value().double_value

        # show plot optimization ros parameter
        self.declare_parameter('show_plot_optimization',
                               OptimizationService.SHOW_PLOT_OPTIMIZATION)
        self.show_plot_optimization = self.get_parameter(
            'show_plot_optimization').get_parameter_value().bool_value

    def __optimize_path_callback(self, request: OptimizePath, response: OptimizePath):
        """
        Execute callback function when receiving the planned path by the Exploration Algorithm.

        :param request: The request containing the received cones and the reference line.
        :param response: The response model.
        :returns: The response containing the optimized path.
        """
        logging.info(
            f'Incoming request:\n\
                Blue Cones {len(request.blue_cones_x)},\n\
                Yellow Cones {len(request.yellow_cones_x)},\n\
                Orange Cones {len(request.orange_cones_x)},\n\
                Big Orange Cones {len(request.big_orange_cones_x)},\n\
                Reference Points {len(request.refline_x)},\n\
                Number of Laps: {request.num_of_laps}')

        # zip and map cones back to one list e.g. [x1, x2] [y1, y2] => [[x1, y1], [x2,y2]]
        blue_cones = list(
            map(list, zip(request.blue_cones_x,
                          request.blue_cones_y)))
        yellow_cones = list(
            map(list, zip(request.yellow_cones_x,
                          request.yellow_cones_y)))
        orange_cones = list(
            map(list, zip(request.orange_cones_x,
                          request.orange_cones_y)))
        big_orange_cones = list(
            map(list, zip(request.big_orange_cones_x,
                          request.big_orange_cones_y)))
        refline = list(
            map(list, zip(request.refline_x,
                          request.refline_y)))

        # transform input to correct format => [x, y, w_tr_right, w_tr_left]
        reftrack = OptimizationInputTransformer.transform(blue_cones,
                                                          yellow_cones,
                                                          orange_cones,
                                                          big_orange_cones,
                                                          refline)

        # optimize path
        optimized_path = optimize_path(optimization_type=self.optimization_type,
                                       reference_track=reftrack,
                                       minimum_track_width=self.minimum_track_width,
                                       num_of_laps=request.num_of_laps,
                                       show_plot=self.show_plot_optimization)

        # zip optimized path to response model
        response.optimized_path_s_m, response.optimized_path_x_m, response.optimized_path_y_m, response.optimized_path_psi_rad, response.optimized_path_kappa_radpm, response.optimized_path_vx_mps, response.optimized_path_ax_mps2 = zip(
            *optimized_path)

        logging.info(f'Returning response: {len(optimized_path)} Points')

        return response


def main(args=None):
    """
    Run the optimization service.

    :param args: Additional arguments (Default value = None).
    """
    rclpy.init(args=args)

    optimization_service = OptimizationService()

    rclpy.spin(optimization_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
