"""Start Finish Detector module."""
import copy
import logging

from fszhaw_msgs.msg import Cone, CurrentPosition
from scipy.spatial.distance import cdist

from path_planning.track_config import TrackConfig


class StartFinishDetector:
    """
    Detector for detecting the Start / Finish of the track.

    Detects the start / finish by comparing the initial starting position with the current position 
    or by detecting enough big orange cones.
    """

    # set constants
    STARTING_POSITION_THRESHOLD = 3
    START_FINISH_CONES_NEEDED = 2
    START_FINISH_CONES_RANGE = 20

    # logging
    LOG_LEVEL = logging.INFO

    # set class variables
    starting_position = None
    track_config = None
    start_finish_cones = []

    def __init__(
        self,
        starting_position: CurrentPosition,
        track_config: TrackConfig
    ):
        """Initialize the Start Finish Detector."""
        # set logging level and format
        logging.basicConfig(level=StartFinishDetector.LOG_LEVEL,
                            format='%(levelname)s:%(message)s')

        self.starting_position = copy.deepcopy(starting_position)
        self.track_config = track_config

    def detect_by_starting_position(
        self,
        current_position: CurrentPosition,
        new_starting_position: CurrentPosition = None
    ) -> bool:
        """
        Handle the start finish line detection by checking the current with the start position.

        The current position of the car must be near enough the initial starting position.

        :param current_position: The current position of the car.
        :param new_starting_position: The new starting position to use.
        :returns: if the vehicle is at the start/finish.
        """
        if new_starting_position is not None:
            self.starting_position = new_starting_position

        distance_to_starting_position = cdist([[current_position.vehicle_position_x, current_position.vehicle_position_y]],
                                              [[self.starting_position.vehicle_position_x, self.starting_position.vehicle_position_y]])[0][0]

        if distance_to_starting_position <= StartFinishDetector.STARTING_POSITION_THRESHOLD:
            logging.info(
                f'Start / Finish detected by Starting Position!\n\
                Starting Position: {self.starting_position}\n\
                Current Position: {current_position}\n\
                Distance: {distance_to_starting_position}')

            return True
        else:
            return False

    def detect_by_cones(
        self,
        index: int,
        current_position: CurrentPosition,
        next_cone: Cone
    ) -> bool:
        """
        Handle the start finish line detection by detecting big orange cones.

        The received cone must be near enough the vehicle's current position, 
        if so, the cone will be saved as a valid 'start finish' cone.
        If enough 'start finish' cones have been received in the last couple cones (e.g. 20),
        check if at least a number of cones are in the threshold.

        :param current_position: The current position of the car.
        :param next_cone: The just received cone.
        :returns: if the vehicle is at the start/finish.
        """
        if index % StartFinishDetector.START_FINISH_CONES_RANGE == 0:
            # enough cones need to be detected in the given range (last 20 cones)
            self.reset_cones()

        distance_to_cone = cdist([[current_position.vehicle_position_x, current_position.vehicle_position_y]],
                                 [[next_cone.location.x, next_cone.location.y]])[0][0]

        # check distance current position <-> receiving cone
        if distance_to_cone < self.track_config.POSITION_DISTANCE_THRESHOLD:
            # save as a valid start finish cone
            self.start_finish_cones.append(
                [next_cone.location.x, next_cone.location.y])

            # check if enough valid start finish cones have been received
            if len(self.start_finish_cones) >= self.track_config.CONES_THRESHOLD:
                distances_to_start_finish_cones = cdist(
                    [[next_cone.location.x, next_cone.location.y]], self.start_finish_cones)[0]

                # if at least e.g. 2 satisfy the condition => sucessfully detected start finish line
                if sum(1 for distance in distances_to_start_finish_cones if distance < self.track_config.EDGE_DISTANCE_THRESHOLD) \
                        >= StartFinishDetector.START_FINISH_CONES_NEEDED:

                    logging.info(
                        f'Start / Finish detected by Cones!\n\
                        Valid Start Finish Cones: {len(self.start_finish_cones)}\n\
                        Current Position: {current_position}')

                    self.reset_cones()
                    return True

        return False

    def reset_cones(self):
        """Reset start finish cones list."""
        self.start_finish_cones.clear()
