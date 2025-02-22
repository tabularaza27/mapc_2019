from __future__ import division  # force floating point division when using plain /
import rospy
import numpy as np

from behaviour_components.behaviours import BehaviourBase
from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from generic_action_behaviour import action_generic_simple

from agent_commons.agent_utils import get_bridge_topic_prefix

import global_variables

class DetachBehaviour(BehaviourBase):

    def __init__(self, name, agent_name, rhbp_agent, **kwargs):
        """Attach to Block

        Args:
            name (str): name of the behaviour
            agent_name (str): name of the agent for determining the correct topic prefix
            rhbp_agent (RhbpAgent): the agent owner of the behaviour
            **kwargs: more optional parameter that are passed to the base class
        """
        super(DetachBehaviour, self).__init__(name=name, requires_execution_steps=True,
                                                       planner_prefix=agent_name,
                                                       **kwargs)

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self.rhbp_agent = rhbp_agent

    def do_step(self):
        active_subtask = self.rhbp_agent.assigned_subtasks[0]  # type: SubTask
        #TODO REFACTOR TO THE ATTACHED BLOCK AND NOT THE CLOSE ONE
        direction_coord = self.rhbp_agent.local_map._attached_blocks[0]._position
        direction ='e'
        for direction_string, coord in global_variables.MOVEMENTS.items():
            if np.array_equal(direction_coord, coord):
                direction = direction_string

        if direction is not None and direction is not False:
            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + "attaching to block in direction " + str(direction))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_DETACH,
                                  params=params)
