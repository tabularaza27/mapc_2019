""" This module contains the class representing a single task """

from sub_task import SubTask

class Task:
    def __init__(self, task_percept):
        self.task_complete = False
        self.expired = False

        self.task_percept = task_percept
        self.name = task_percept.name
        self.deadline = task_percept.deadline
        self.reward = task_percept.reward

        self.sub_tasks = []
        self._decompose_task()

    def _decompose_task(self):
        """Decomposes the tasks into sub tasks, based on the requirements in the task percept"""
        self.sub_tasks = [SubTask(task_requirement=requirement) for requirement in self.task_percept.requirements]

    def check_sub_task_completness(self):
        """Checks if all subtasks are complete, if so marks whole task as complete"""
        sub_task_completeness = [sub_task.complete for sub_task in self.sub_tasks]
        if False in sub_task_completeness:
            return False
        else:
            self.mark_task_complete()
            return True

    def mark_task_complete(self):
        self.task_complete = True