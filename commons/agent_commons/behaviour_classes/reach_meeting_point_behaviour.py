from __future__ import division  # force floating point division when using plain /
import rospy

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
        active_subtask = self.rhbp_agent.assigned_tasks[0]  # type: SubTask
        #temp generation of the meeting point
        task_meeting_point = self.rhbp_agent.local_map.goal_top_left
        active_subtask._meeting_point = [task_meeting_point + active_subtask.position] # this is really the block position not the agent position

        direction = self.rhbp_agent.local_map.get_meeting_point_move(active_subtask)

        if direction is not None and direction is not False:
            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + " executing move to meeting point, direction: " + str(direction))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                                  params=params)
