from __future__ import division  # force floating point division when using plain /

from behaviour_components.sensors import TopicSensor

from agent_common.agent_utils import relative_euclidean_distance


class RelDistanceSensor(TopicSensor):
    """
    Distance Sensor for calculating the relative distance to a thing
    """

    def update(self, newValue):

        newValue = relative_euclidean_distance(newValue)

        super(RelDistanceSensor, self).update(newValue=newValue)
