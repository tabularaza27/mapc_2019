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

        self.next_to_block_with_free_attach_slot = Sensor(name="next_to_block_with_free_attach_slot", initial_value=False)

        self.all_positions_attached = Sensor(name="all_positions_attached", initial_value=False) # True if agent is attached to block in all directions

        # sensors for go_to_meet behaviour

        self.at_meeting_point = Sensor(name="at_meeting_point", initial_value=False)

        # sensors for connect behaviour

        # sensors for detach behaviour

        # sensors for submit behaviour

    def update_sensors(self):
        """updates sensor values"""

        ### Get required data from agent ###

        # assume the current subtask is the first one in the list
        if len(self.rhbp_agent.assigned_tasks) > 0:
            current_subtask = self.rhbp_agent.assigned_tasks[0]
        else:
            current_subtask = None

        attached_blocks = self.rhbp_agent.local_map._attached_blocks

        ### Update Sensors ###

        # check if agent is not assigned to at least one subtask
        self.assigned_task_list_empty.update(current_subtask is None)

        if current_subtask is not None:
            # check if agent has already the block attached
            tmp_attached = False
            block_type = current_subtask.type
            for block in attached_blocks:
                if block._subtask_id == current_subtask.sub_task_name:
                    tmp_attached = True
            self.attached_to_block.update(tmp_attached)

            # check if the agent is 1 step away from the dispenser
            if self.rhbp_agent.local_map.get_direction_to_close_dispenser(block_type) in global_variables.string_directions:
                rospy.loginfo('AT THE DISPENSER = TRUE!!!!!!!')
                self.at_the_dispenser.update(True)
            else:
                self.at_the_dispenser.update(False)

            # check if the agent is 1 step away from a block of the type of the current task
            self.next_to_block.update(self.rhbp_agent.local_map.get_direction_to_close_block(current_subtask.type) != False)
