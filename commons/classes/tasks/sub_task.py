""" This module contains the class representing a single task """

import numpy as np


class SubTask:
    """
    Note:
        Behaviours that are needed for completion of a subtask are listed below.
        * The behaviours listed in COMMON_SUB_TASK_BEHAVIOURS every agent has to fulfill
        One Agent of the whole task needs to do the Behaviours in the SUBMIT_BEHAVIOUR, the others the DETACH_BEHAVIOUR

        For now as an easy solution: The Agent with the lowest agent name (e.g. agent1) does the submit behaviour
        the others are doing the detach behaviour

    Attributes:
        task_requirement ():
        parent_task_name
    """

    COMMON_SUB_TASK_BEHAVIOURS = ["go_to_dispenser", "dispense_attach", "meet_to_connect", "connect"]
    SUBMIT_BEHAVIOUR = ["go_to_goal_area", "submit"]
    DETACH_BEHAVIOUR = ["detach"]

    def __init__(self, task_requirement, parent_task_name):
        """
        Args:
            task_requirement (dict): requirements of the subtask with keys pos, details, type
                example:
                    {"pos": {"x": 0, "y": 3}, "details": '', "type": "b1"}
            parent_task_name (str): unique id of parent task (e.g. task22)
        """
        self.task_requirement = task_requirement

        # Sub Task name consists of taskname_x-coord_y-coord. Example task22_1_1
        self.parent_task_name = parent_task_name

        self.position = [task_requirement.pos.x, task_requirement.pos.y]
        self.type = task_requirement.type
        self.sub_task_name = '{}_{}_{}'.format(parent_task_name, self.position[0], self.position[1])

        # subtask is complete, when all required behaviours have been conducted
        self.complete = False

        # ToDo do the task subdivision here

        # is the sub task assigned to me ( agent that created the sub tasks object )
        self.assigned_to_me = False

        # ToDo decide on which behaviours are needed based on who are the other agents completing the parent task
        # self.required_behaviours = [list_of_behaviours from above]


        # next step required
        self.next_step = self.required_behaviours[0]

    def sub_task_auction(self):
        """Auction Process

        ToDo: Implement the Auction Mechanism

        Note:
            make sure that one agent can only win one subtask in a whole task
        """

        # # for now randomly win auction every
        # won_auction = False
        # if np.random.random(1)[0] < 0.3:
        #     won_auction = True
        #     names_of_other_agents = get_agent_names()
