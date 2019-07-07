from __future__ import division  # force floating point division when using plain /
import rospy
import numpy as np

from behaviour_components.behaviours import BehaviourBase
from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from generic_action_behaviour import action_generic_simple

from agent_commons.agent_utils import get_bridge_topic_prefix


class ReachMeetingPointBehaviour(BehaviourBase):

    def __init__(self, name, agent_name, rhbp_agent, **kwargs):
        """Move to Dispenser

        Args:
            name (str): name of the behaviour
            agent_name (str): name of the agent for determining the correct topic prefix
            rhbp_agent (RhbpAgent): the agent owner of the behaviour
            **kwargs: more optional parameter that are passed to the base class
        """
        super(ReachMeetingPointBehaviour, self).__init__(name=name, requires_execution_steps=True,
                                                       planner_prefix=agent_name,
                                                       **kwargs)

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self.rhbp_agent = rhbp_agent

    def do_step(self):
        direction = None
        active_subtask = self.rhbp_agent.assigned_subtasks[0]  # type: SubTask
        current_task = self.rhbp_agent.tasks[active_subtask.parent_task_name]
        #temp generation of the meeting point [[agent_position],[block_position]]
        # task_meeting_point = self.rhbp_agent.local_map.goal_top_left
        # if active_subtask.type == "b1":
        #     active_subtask.meeting_point = np.array([task_meeting_point + np.array([0,-1]), task_meeting_point])
        # else:
        #     active_subtask.meeting_point = \
        #         np.array([task_meeting_point + np.array([0, 2]),
        #                  task_meeting_point + np.array([0, 1])])

        # common_meeting_point = self.rhbp_agent.local_map._from_relative_to_matrix(self.rhbp_agent.local_map.goal_top_left)

        agent1, agent2, common_meeting_point = self.rhbp_agent.local_map.get_common_meeting_point(current_task)
        if common_meeting_point is not None:
            task_meeting_point = self.rhbp_agent.local_map.meeting_position(current_task, common_meeting_point )
            active_subtask.meeting_point = task_meeting_point
            path_id, direction = self.rhbp_agent.local_map.get_meeting_point_move(active_subtask)
            active_subtask.path_to_meeting_point_id = path_id

        if direction is not None and direction is not False:
            params = [KeyValue(key="direction", value=direction)]
            if direction == 'cw' or direction == 'ccw':
                rospy.logdebug(
                    self._agent_name + "::" + self._name + " executing move to meeting point, direction: " + str(
                        direction))
                action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ROTATE,
                                      params=params)
            else:
                params = [KeyValue(key="direction", value=direction)]
                rospy.logdebug(self._agent_name + "::" + self._name + " executing move to meeting point, direction: " + str(direction))
                action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                                      params=params)
