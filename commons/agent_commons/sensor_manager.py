from __future__ import division  # force floating point division when using plain /

import rospy

from behaviour_components.sensors import TopicSensor

from behaviour_components.sensors import Sensor

import global_variables


class SensorManager():
    def __init__(self, rhbp_agent):
        """The SensorManager keeps track of all sensors of an agent and updates them every simulation step

        Args:
            rhbp_agent (RhbpAgent): RhbpAgent object
        """

        self.rhbp_agent = rhbp_agent

        # sensors for move to dispenser & dispense behaviour
        self.assigned_task_list_empty = Sensor(name="assigned_task_list_empty", initial_value=True)

        self.attached_to_block = Sensor(name="attached_to_block", initial_value=False) # for current subtask

        self.at_the_dispenser = Sensor(name="at_the_dispenser", initial_value=False)

        # sensors for attach behaviour
        self.next_to_block = Sensor(name="next_to_block", initial_value=False) # that has the type of the current task

        self.fully_attached = Sensor(name="all_positions_attached", initial_value=False) # True if agent is attached to block in all directions

        # sensors for go_to_meet behaviour
        self.at_meeting_point = Sensor(name="at_meeting_point", initial_value=False)

        self.can_submit = Sensor(name="can_submit", initial_value=False)

        # sensors for connect behaviour

        # sensors for detach behaviour

        # sensors for submit behaviour

    def update_sensors(self):
        """updates sensor values"""

        ### Get required data from agent ###

        # assume the current subtask is the first one in the list
        if len(self.rhbp_agent.assigned_subtasks) > 0:
            current_subtask = self.rhbp_agent.assigned_subtasks[0]
        else:
            current_subtask = None

        attached_blocks = self.rhbp_agent.local_map._attached_blocks

        ### Update Sensors ###

        # check if agent is not assigned to at least one subtask
        self.assigned_task_list_empty.update(current_subtask is None)

        # if agent is not assigned to a sub task yet, there is no need most of the sensors, since they are all
        # related to the current subtask
        if current_subtask is not None:
            current_task = self.rhbp_agent.tasks[current_subtask.parent_task_name]

            # check if agent has already the block attached
            block_attached = False
            for block in attached_blocks:
                if block._type == current_subtask.type:
                    block_attached = True
            self.attached_to_block.update(block_attached)

            # check if the agent is 1 step away from the dispenser
            direction_to_closest_dispenser = self.rhbp_agent.local_map.get_direction_to_close_dispenser(current_subtask.type)
            if direction_to_closest_dispenser in global_variables.string_directions:
                rospy.loginfo('AT THE DISPENSER = TRUE!!!!!!!')
                self.at_the_dispenser.update(True)
            else:
                self.at_the_dispenser.update(False)

            # check if the agent is 1 step away from a block of the type of the current task
            direction_to_closest_block = self.rhbp_agent.local_map.get_direction_to_close_block(current_subtask.type)
            if direction_to_closest_block:
                rospy.loginfo('Block of Type {} in Direction {}'.format(current_subtask.type, direction_to_closest_block))
                self.next_to_block.update(True)
            else:
                self.next_to_block.update(False)

            # check if agent has already attached blocks in all available slots (check unique x values)
            # TODO refactor this function
            #occupied_slots = len({block._position[1] for block in attached_blocks})
            #if occupied_slots >= 4:
            #    self.fully_attached.update(True)
            #else:
            #    self.fully_attached.update(False)
            self.fully_attached.update(False)
            #check if agent is at the meeting point
            at_meeting_point = self.rhbp_agent.local_map.is_at_point(current_subtask.meeting_point)
            if at_meeting_point:
                rospy.loginfo('Reached meeting point!')
                self.at_meeting_point.update(True)
            else:
                self.at_meeting_point.update(False)

            # check if agent can submit the task because it is assigned and all the connections has taken place
            can_submit = current_subtask.submit_behaviour and current_task.is_submittable()
            if can_submit:
                rospy.loginfo('Reached meeting point!')
                self.can_submit.update(True)
            else:
                self.can_submit.update(False)
