from __future__ import division  # force floating point division when using plain /
import rospy

from behaviour_components.behaviours import BehaviourBase
from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from agent_commons.behaviour_classes.generic_action_behaviour import action_generic_simple

from agent_commons.agent_utils import get_bridge_topic_prefix, pos_to_direction

###################### EXAMPLE CLASS ########################
class MoveToDispenser(BehaviourBase):
    """
    Move to closest dispenser nearby
    """

    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(MoveToDispenser, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name,
                                              **kwargs)

        self._agent_name = agent_name

        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):

        if self._perception_provider.closest_dispenser:

            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)

            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + " request dispense " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no dispenser in range", self.name)
