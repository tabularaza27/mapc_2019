""" This module contains the class representing a single task """

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
        elif 'invalid' in sub_tasks_auctioned:
            return False
        else:
            self.mark_task_auctioned()
            self.delegate_submit_behaviour()
            return True

    def delegate_submit_behaviour(self):
        """delegates the submit behaviour of the task to the agent with the lowest id

        ToDo: This heuristic should be replaced by a more sophisticated approach later
        """
        # ToDo implement this in a cleaner way
        sub_task_agents = [sub_task.assigned_agent for sub_task in self.sub_tasks]
        submit_agent = min(sub_task_agents)
        for sub_task in self.sub_tasks:
            if sub_task.assigned_agent == submit_agent:
                sub_task.submit_behaviour = True

    def mark_task_complete(self):
        self.complete = True

    def mark_task_auctioned(self):
        self.auctioned = True
