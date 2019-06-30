from __future__ import division  # force floating point division when using plain /

from behaviour_components.sensors import TopicSensor

from behaviour_components.sensors import Sensor


class SensorManager():
    def __init__(self, rhbp_agent):
        """The SensorManager keeps track of all sensors of an agent and updates them every simulation step

        Args:
            rhbp_agent (RhbpAgent): RhbpAgent object
        """

        self.rhbp_agent = rhbp_agent

        # sensors for move to dispenser
        self.assigned_task_list_empty = Sensor(name="assigned_task_list_empty", initial_value=True)

        self.attached_to_block = Sensor(name="attached_to_block", initial_value=False)

        self.at_the_dispenser = Sensor(name="at_the_dispenser", initial_value=False)

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

        # check if agent has already the block attached
        if current_subtask is not None:
            tmp_attached = False
            for block in attached_blocks:
                if block._subtask_id == current_subtask.sub_task_name:
                    tmp_attached = True
            self.attached_to_block.update(tmp_attached)

        # check if the agent is 1 step away from the dispenser
        self.at_the_dispenser.update(self.rhbp_agent.local_map.is_close_to_dispenser(current_subtask.type))
