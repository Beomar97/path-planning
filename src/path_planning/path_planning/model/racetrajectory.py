

class RaceTrajectory:
    """
    The Race Trajectory.

    The optimized race trajectory for a given track.
    """

    def __init__(self, s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2):
        """
        Create a point of the race trajectory.

        :param s_m: float32, meter. Curvi-linear distance along the raceline.
        :param x_m: float32, meter. X-coordinate of raceline point.
        :param y_m: float32, meter. Y-coordinate of raceline point.
        :param psi_rad: float32, rad. Heading of raceline in current point from -pi to +pi rad. Zero is north (along y-axis).
        :param kappa_radpm: float32, rad/meter. Curvature of raceline in current point.
        :param vx_mps: float32, meter/second. Target velocity in current point.
        :param ax_mps2: float32, meter/second². Target acceleration in current point. We assume this acceleration to be constant from current point until next point.

        """
        self.s_m = s_m
        self.x_m = x_m
        self.y_m = y_m
        self.psi_rad = psi_rad
        self.kappa_radpm = kappa_radpm
        self.vx_mps = vx_mps
        self.ax_mps2 = ax_mps2
