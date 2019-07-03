""" This module contains the class representing a single task """

import numpy as np

from sub_task import SubTask


class Task:
    """This class represents one single task in the massim environment.

    The task is decomposed in its subtask when it is initialized. Each requirement of the task ( see example below )
    is a subtask, that needs to be completed by an agent. When all sub tasks are marked complete, the task is complete.

    """

    def __init__(self, task_percept):
        """
        Args:
            task_percept (dict): perception of a task.

        Examples:
            deadline: 155
            name: "task0"
            reward: 90
            requirements:
              -
                pos:
                  x: 0
                  y: 2
                details: ''
                type: "b0"
              -
                pos:
                  x: 0
                  y: 3
                details: ''
                type: "b0"
              -
                pos:
                  x: 0
                  y: 1
                details: ''
                type: "b0"


        """
        # task is complete, when all sub tasks are complete
        self.complete = False
        # task is auctioned if all sub tasks are auctioned and assigned to an agent
        self.auctioned = False
        self.expired = False

        self.task_percept = task_percept
        self.name = task_percept.name
        self.deadline = task_percept.deadline
        self.reward = task_percept.reward

        self.sub_tasks = []
        self._decompose_task()

    def _decompose_task(self):
        """Decomposes the tasks into sub tasks, based on the requirements in the task percept"""
        self.sub_tasks = [SubTask(task_requirement=requirement, parent_task_name=self.name) for requirement in
                          self.task_percept.requirements]

    def check_sub_task_completness(self):
        """Checks if all subtasks are complete, if so marks whole task as complete"""
        sub_task_completeness = [sub_task.complete for sub_task in self.sub_tasks]
        if False in sub_task_completeness:
            return False
        else:
            self.mark_task_complete()
            return True

    def check_auctioning(self):
        """Checks if all sub tasks have been auctioned and assigned to an agent, if so marks task as complete"""
        sub_tasks_auctioned = [sub_task.assigned_agent for sub_task in self.sub_tasks]
        if None in sub_tasks_auctioned:
            return False
        else:
            self.mark_task_auctioned()
            self.delegate_submit_behaviour()
            return True

    def delegate_submit_behaviour(self):
        """delegates the submit behaviour to the agent that is assigned to the block next to the origin in the task requirements
        (red dot in the GUI of the simulation).

        Note:
            * the block is next to the origin if abs(position.x) + abs(position(.y) == 1
            * If there are several blocks next to the origin ( so far we never experienced that in the simulation ), the
              agent with the lowest id assigned to one of these subtasks is going to do the submit behaviour
        """
        submit_agent_candidates = []
        for sub_task in self.sub_tasks:
            if abs(sub_task.position.x) + abs(sub_task.position.y) == 1:
                submit_agent_candidates.append(sub_task)

        if len(submit_agent_candidates) == 0:
            raise ValueError, 'There is no block next to the origin of the task. Check calculations this should not ' \
                              'be possible '
        elif len(submit_agent_candidates) == 1:
            submit_agent_candidates[0].submit_behaviour = True
        else:
            # if there are more than 2 blocks next to the origin of the task, select the agent to do the submit
            # that has the lowest agent_id
            index = np.argmin([sub_task.assigned_agent for sub_task in submit_agent_candidates])
            submit_agent_candidates[index].submit_behaviour = True

    def mark_task_complete(self):
        self.complete = True

    def mark_task_auctioned(self):
        self.auctioned = True
