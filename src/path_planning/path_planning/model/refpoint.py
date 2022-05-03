

class Refpoint:
    """
    A Reference Point.

    A collection of reference points (reference track) is needed as input for the optimization algorithm.
    """

    def __init__(self, x: float, y: float, w_tr_right: float, w_tr_left):
        """
        Create a reference point.

        :param x: x-value.
        :param y: y-value.
        :param w_tr_right: track width right (distance to yellow cone).
        :param w_tr_left: track width left (distance to blue cone).
        """
        self.x = x
        self.y = y
        self.w_tr_right = w_tr_right
        self.w_tr_left = w_tr_left
