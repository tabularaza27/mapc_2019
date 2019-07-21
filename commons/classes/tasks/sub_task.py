""" This module contains the class representing a single task """

import numpy as np


class SubTask:
    """This class represents a subtask. A subtask is one requirement of the parent task

    Each subtask gets auctioned among all available agents and agent with lowest bid gets assigned to the subtask.
    Note that an agent can only be assigned to one subtask of the same parent task

    Several behaviours are needed for completion of a subtask.
    The behaviours listed in COMMON_SUB_TASK_BEHAVIOURS every agent has to fulfill
    One Agent of the whole task needs to do the Behaviours in the SUBMIT_BEHAVIOUR, the others the DETACH_BEHAVIOUR

    For now as an easy solution: The Agent with the lowest agent name (e.g. agent1) does the submit behaviour
    the others are doing the detach behaviour


    A Subtask is uniquely identifiable by its name. The naming convention is taskname_x-coord_y-coord
    example:
        task22_1_1
    """

    # Note: another possibility ( what Ale suggested )  to keep track of subtask progress is via sensors of the agent
    # COMMON_SUB_TASK_BEHAVIOURS = ["go_to_dispenser", "dispense_attach", "meet_to_connect", "connect"]
    # SUBMIT_BEHAVIOUR = ["go_to_goal_area", "submit"]
    # DETACH_BEHAVIOUR = ["detach"]

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

        # data necessary for attaching, connecting and submitting
        self.is_dispensed = False
        self.is_connected = False
