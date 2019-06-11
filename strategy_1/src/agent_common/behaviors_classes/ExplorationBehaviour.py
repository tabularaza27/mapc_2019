from __future__ import division  # force floating point division when using plain /
import rospy

from behaviour_components.behaviours import BehaviourBase

from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction


from ..agent_utils import get_bridge_topic_prefix, pos_to_direction
from ..behaviours_common import action_generic_simple

class ExplorationBehaviour(BehaviourBase):
    """
    Exploration Behaviour
    """

    def __init__(self, name, agent_name, rhbp_agent,**kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param rhbp_agent(RhbpAgent): the agent owner of the behaviour
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(ExplorationBehaviour, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self.rhbp_agent = rhbp_agent
        self.exploration_path_id = None

    def do_step(self):
        path_id, direction= self.rhbp_agent.local_map.get_exploration_move(self.exploration_path_id)
        self.exploration_path_id = path_id
        params = [KeyValue(key="direction", value=direction)]
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(direction))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                              params=params)