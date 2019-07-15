from __future__ import division  # force floating point division when using plain /
import rospy

from behaviour_components.behaviours import BehaviourBase
from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from generic_action_behaviour import action_generic_simple

from agent_commons.agent_utils import get_bridge_topic_prefix


class ConnectBehaviour(BehaviourBase):

    def __init__(self, name, agent_name, rhbp_agent, **kwargs):
        """Move to Dispenser

        Args:
            name (str): name of the behaviour
            agent_name (str): name of the agent for determining the correct topic prefix
            rhbp_agent (RhbpAgent): the agent owner of the behaviour
            **kwargs: more optional parameter that are passed to the base class
        """
        super(ConnectBehaviour, self).__init__(name=name, requires_execution_steps=True,
                                                       planner_prefix=agent_name,
                                                       **kwargs)

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self.rhbp_agent = rhbp_agent

    def do_step(self):
        # active_subtask = self.rhbp_agent.assigned_subtasks[0]  # type: SubTask
        # # TODO REMOVE TEMPORARY CONNECT PARAMS
        # if self.rhbp_agent._agent_name == 'agentA2':
        #     agent_to_connect = 'agentA1'
        # else:
        #     agent_to_connect = 'agentA2'
        #
        # block_position_x = self.rhbp_agent.local_map._attached_blocks[0]._position[1]
        # block_position_y = self.rhbp_agent.local_map._attached_blocks[0]._position[0]
        #
        # params = [KeyValue(key="y", value=str(block_position_y)), KeyValue(key="x", value=str(block_position_x)),
        #           KeyValue(key="agent", value=agent_to_connect)]
        # rospy.logdebug(self._agent_name + "::" + self._name + " connecting with " + str(agent_to_connect))
        # action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_CONNECT,
        #                       params=params)

        active_subtask = self.rhbp_agent.assigned_subtasks[0]  # type: SubTask

        # Agents to connect
        if self.rhbp_agent._agent_name == self.rhbp_agent.first_agent:
            agent_to_connect = self.rhbp_agent.second_agent
        else:
            agent_to_connect = self.rhbp_agent.first_agent

        # Blocks to connect
        block_position_x = self.rhbp_agent.local_map._attached_blocks[0]._position[1]
        block_position_y = self.rhbp_agent.local_map._attached_blocks[0]._position[0]

        params = [KeyValue(key="y", value=str(block_position_y)), KeyValue(key="x", value=str(block_position_x)),
                  KeyValue(key="agent", value=agent_to_connect)]
        rospy.logdebug(self._agent_name + "::" + self._name + " connecting with " + str(agent_to_connect))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_CONNECT,
                              params=params)
