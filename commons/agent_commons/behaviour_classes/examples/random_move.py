from __future__ import division  # force floating point division when using plain /
import rospy
import random

from behaviour_components.behaviours import BehaviourBase
from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from agent_commons.behaviour_classes.generic_action_behaviour import action_generic_simple

from agent_commons.agent_utils import get_bridge_topic_prefix

###################### EXAMPLE CLASS ########################
class RandomMove(BehaviourBase):
    """
    Move in randomly chosen directions
    """

    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(RandomMove, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        random_move = ['n', 's', 'e', 'w']
        params = [KeyValue(key="direction", value=random.choice(random_move))]
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                              params=params)
