from __future__ import division  # force floating point division when using plain /

import sys

from agent_utils import relative_euclidean_distance

from behaviour_components.sensors import Sensor


class PerceptionProvider(object):
    """
    Objects that holds current perception, provides additional reasoning/postprocessing, and gives easy access to RHBP
    sensors
    """

    def __init__(self):

        self.simulation_step = 0

        self.agent = None

        self.tasks = []

        self.goals = []

        self.dispensers = []

        self.obstacles = []

        # moving things ( entities --> other agents )
        self.entities = []

        self.blocks = []

        self.closest_dispenser = None

        self.closest_dispenser_distance_sensor = Sensor(name="closest_dispenser_distance", initial_value=sys.maxint)

        # Here, we use the most basic Sensor class of RHBP and update manually within the provider to avoid the usage of
        # additional topics and their overhead.
        self.dispenser_visible_sensor = Sensor(name="dispenser_visible", initial_value=False)

        self.number_of_blocks_sensor = Sensor(name="number_of_blocks", initial_value=0)  # TODO this is currently never updated

    def update_perception(self, request_action_msg):
        """
        Has to be called on every simulation step and updated with the current request action message
        :param request_action_msg: request action message
        """

        self._request_action_msg = request_action_msg

        self._update_dispensers(request_action_msg)

        self._update_entities(request_action_msg)

        self.agent = request_action_msg.agent

        self.goals = request_action_msg.goals  # TODO this could be more sophisticated and potentially extracted like above

        self.obstacles = request_action_msg.obstacles  # TODO this could be more sophisticated and potentially extracted like above

        self.blocks = request_action_msg.blocks  # TODO this could be more sophisticated and potentially extracted like above

        self.simulation_step = request_action_msg.simulation_step

        self.tasks = request_action_msg.tasks

    def _update_dispensers(self, request_action_msg):
        """
        Update dispenser perception
        :param request_action_msg: full current request action message object
        """

        self.dispensers = request_action_msg.dispensers

        self.dispenser_visible_sensor.update(newValue=len(self.dispensers) > 0)
        self.dispenser_visible_sensor.sync()

        self._update_closest_dispenser(dispensers=self.dispensers)

    def _update_closest_dispenser(self, dispensers):
        """
        Update information about the closest visible dispenser
        :param dispensers: dispensers perception
        """

        self.closest_dispenser = None
        closest_distance = sys.maxint

        for d in dispensers:
            if self.closest_dispenser is None or closest_distance > relative_euclidean_distance(d.pos):
                self.closest_dispenser = d
                closest_distance = relative_euclidean_distance(d.pos)

        self.closest_dispenser_distance_sensor.update(newValue=closest_distance)
        self.closest_dispenser_distance_sensor.sync()

    def _update_entities(self, request_action_msg):
        """ update entity perception

        Args:
            request_action_msg: full current request action message object

        Returns: void

        """

        self.entities = request_action_msg.entities
