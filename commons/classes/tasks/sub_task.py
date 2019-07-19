""" This module contains the class representing a single task """

import numpy as np


class SubTask:
    """This class represents a subtask. A subtask is one requirement of the parent task

    Each subtask gets auctioned among all available agents and agent with lowest bid gets assigned to the subtask.
    Note that an agent can only be assigned to one subtask of the same parent task

    Several behaviours are needed for completion of a subtask. ( see behaviour model )
    One Agent of the whole task needs to do submit behaviour. The agent that does the subtask at the origin of the parent task
    (in the GUI the block next to the red dot), does the submit. All other agents detach after connecting

    A Subtask is uniquely identifiable by its name. The naming convention is taskname_x-coord_y-coord
    example:
        task22_1_1
    """

    def __init__(self, task_requirement, parent_task_name):
        """
        Args:
            task_requirement (dict): requirements of the subtask with keys pos, details, type
                example:
                    {"pos": {"x": 0, "y": 3}, "details": '', "type": "b1"}
            parent_task_name (str): unique id of parent task (e.g. task22)
        """
        self.parent_task_name = parent_task_name
        self.position = np.array([task_requirement.pos.y, task_requirement.pos.x])
        self.type = task_requirement.type
        self.sub_task_name = '{}_{}_{}'.format(parent_task_name, self.position[0], self.position[1])

        # subtask is complete, when all required behaviours have been conducted
        self.complete = False

        # name of assigned agent
        self.assigned_agent = None

        # if submit is True, the Agent completing this subtask, has to submit the parent task, else agent detaches
        # after connect
        self.submit_behaviour = False


        # data necessary for meeting
        self.distance_to_dispenser = None
        self.closest_dispenser_position = None
        self.path_to_dispenser_id = None
        self.path_to_meeting_point_id = None
        self.meeting_point = None

        # necessary for connecting and submitting
        self.is_connected = False