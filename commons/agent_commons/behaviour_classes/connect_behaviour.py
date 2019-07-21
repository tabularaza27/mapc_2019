from __future__ import division  # force floating point division when using plain /
import rospy
import random

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
        # get the subtask
        active_subtask = self.rhbp_agent.assigned_subtasks[0]  # type: SubTask

        # get the task
        active_task = self.rhbp_agent.tasks[active_subtask.parent_task_name]

        # get involved agents
        nearby_agents = self.rhbp_agent.nearby_agents
        # create randomly order copy of nearby agents
        nearby_agents_random = nearby_agents[:]
        random.shuffle(nearby_agents_random)

        # relative to the submitter transform
        relative_submitter_ratio = self.rhbp_agent.local_map._from_matrix_to_relative(active_subtask.position \
                                            ,self.rhbp_agent.local_map._attached_blocks[0]._position)

        agent_to_connect = None

        for close_by_agent in nearby_agents_random:
            for subtask in active_task.sub_tasks:
                if subtask.assigned_agent == close_by_agent and not subtask.complete \
                        and close_by_agent != self._agent_name:
                    for block in self.rhbp_agent.local_map._attached_blocks:
                        # block has to be transform to relative to the submitter
                        # --> matrix and then rel -->submitter
                        block_relative_to_submitter = self.rhbp_agent.local_map._from_relative_to_matrix(block._position, \
                                                relative_submitter_ratio)
                        distance2d = abs(block_relative_to_submitter[0] - subtask.position[0]) \
                                + abs(block_relative_to_submitter[1] - subtask.position[1])
                        distance = abs(distance2d)
                        ##########
                        #print (self._agent_name)
                        #print (str(block_relative_to_submitter))
                        #print (close_by_agent)
                        #print(str(subtask.position))
                        #print (distance)
                        ##########
                        if distance == 1:  # if the blocks 1 step away, that is the one to connect
                            block_position = block._position
                            agent_to_connect = close_by_agent


        if agent_to_connect is not None:
            # connect message
            params = [KeyValue(key="y", value=str(block_position[0])), KeyValue(key="x", value=str(block_position[1])),
                      KeyValue(key="agent", value=agent_to_connect)]
            rospy.logdebug(self._agent_name + "::" + self._name + " connecting with " + str(agent_to_connect))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_CONNECT,
                                  params=params)
