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
        # if self.rhbp_agent._agent_name == self.rhbp_agent.first_agent:
        #     agent_to_connect = self.rhbp_agent.second_agent
        # else:
        #     agent_to_connect = self.rhbp_agent.first_agent
        # agent_to_connect = self.rhbp_agent.nearby_agents[0]

        # get the task
        active_task = self.rhbp_agent.tasks[active_subtask.parent_task_name]

        # # get other agent subtask
        # for subtask in active_task.sub_tasks:
        #     if subtask.assigned_agent == agent_to_connect:
        #         other_block_position = subtask.position
        # block_position = None
        #
        # # block_position for the agent who has more blocks
        # for block in self.rhbp_agent.local_map._attached_blocks:
        #     distance2d = block._position - other_block_position
        #     distance = abs(distance2d[0]) + abs(distance2d[1])
        #     if distance == 1: # if the blocks 1 step away, that is the one to connect
        #         block_position = block._position

        # NEW
        nearby_agents = self.rhbp_agent.nearby_agents
        # create randomly order copy of nearby agents
        nearby_agents_random = nearby_agents[:]
        random.shuffle(nearby_agents_random)
        number_of_blocks_attached = len(self.rhbp_agent.local_map._attached_blocks)

        # relative to the submitter transform
        relative_submitter_ratio = self.rhbp_agent.local_map._from_matrix_to_relative(active_subtask.position \
                                            ,self.rhbp_agent.local_map._attached_blocks[0]._position)


        print (self._agent_name)
        print ("PUTOS BLOCKSSSS" + str(number_of_blocks_attached))
        agent_to_connect = None

        # # get name of agent to connect (in subtask order) and its block position for agent with more
        # # than one block attached
        # if number_of_blocks_attached > 1:
        #     for agent_name in nearby_agents:
        #         for subtask in active_task.sub_tasks:
        #             if subtask.assigned_agent == agent_name:
        #                 # Check if block is already attached
        #                 for block in self.rhbp_agent.local_map._attached_blocks:
        #                     distance2d = abs(block._position[0]) + abs(block._position[1]) \
        #                                  - abs(subtask.position[0]) - abs(subtask.position[1])
        #                     distance = abs(distance2d)
        #                     print (self._agent_name)
        #                     print (agent_name)
        #                     print (distance)
        #                     if distance == 1:  # if the blocks 1 step away, that is the one to connect
        #                         block_position = block._position
        #                         agent_to_connect = agent_name
        #                     # else:
        #                     #     agent_to_connect = None
        #
        # # block_position for the agent who has just one and has to detach
        # else:
        #     agent_to_connect = self.rhbp_agent.nearby_agents[0]
        #     block_position = self.rhbp_agent.local_map._attached_blocks[0]._position
        #     print (self._agent_name)
        #     print (block_position)
        #     print (agent_to_connect)

        # get name of agent to connect (in subtask order) and its block position for agent with more
        # than one block attached
        #if number_of_blocks_attached > 1:
        for close_by_agent in nearby_agents_random:
            for subtask in active_task.sub_tasks:
                if subtask.assigned_agent == close_by_agent and not subtask.complete \
                        and close_by_agent != self._agent_name:
                    # Check if block is already attached
                    #if number_of_blocks_attached > 1:
                    for block in self.rhbp_agent.local_map._attached_blocks:
                        # block has to be transform to relative to the submitter
                        # --> matrix and then rel -->submitter
                        block_relative_to_submitter = self.rhbp_agent.local_map._from_relative_to_matrix(block._position, \
                                                relative_submitter_ratio)
                        distance2d = abs(block_relative_to_submitter[0] - subtask.position[0]) \
                                + abs(block_relative_to_submitter[1] - subtask.position[1])
                        distance = abs(distance2d)
                        ##########
                        print (self._agent_name)
                        print (str(block_relative_to_submitter))
                        print (close_by_agent)
                        print(str(subtask.position))
                        print (distance)
                        ##########
                        if distance == 1:  # if the blocks 1 step away, that is the one to connect
                            block_position = block._position
                            agent_to_connect = close_by_agent
                    # else:
                    #     block = self.rhbp_agent.local_map._attached_blocks[0]
                    #     distance2d = abs(block._position[0]) + abs(block._position[1]) \
                    #                  - abs(subtask.position[0]) - abs(subtask.position[1])
                    #     distance = abs(distance2d)
                    #     print (self._agent_name)
                    #     print (close_by_agent)
                    #     print (distance)
                    #     if distance == 1:  # if the blocks 1 step away, that is the one to connect
                    #         block_position = block._position
                    #         agent_to_connect = close_by_agent

        if agent_to_connect is not None:
            # connect message
            params = [KeyValue(key="y", value=str(block_position[0])), KeyValue(key="x", value=str(block_position[1])),
                      KeyValue(key="agent", value=agent_to_connect)]
            rospy.logdebug(self._agent_name + "::" + self._name + " connecting with " + str(agent_to_connect))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_CONNECT,
                                  params=params)
