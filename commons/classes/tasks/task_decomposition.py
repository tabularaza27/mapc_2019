""" This module contains functionalities for creating and keeping track of tasks based on the percept """

from classes.tasks.task import Task


# ToDo these functions either go in the agent commons folder or the rhbp_agent.py itself

def update_tasks(current_tasks, tasks_percept, simulation_step):
    """updates task representation based on new percept.

    * adds tasks, that are new
    * mark expired tasks as expired
    * marks complete tasks as complete

    Args:
        current_tasks (dict): task names as keys and task objects as values
        tasks_percept (dict): perception of tasks
        simulation_step (int): current simulation step

    Returns:
        updated list of tasks containing all tasks
    """
    new_tasks = current_tasks

    # add new tasks
    for task in tasks_percept:
        if task.name not in current_tasks.keys():
            new_tasks[task.name] = Task(task_percept=task)

    # mark expired tasks & mark complete tasks
    # ToDo implement way so that completed and expired tasks don't get looped through again every time
    for task_name, task_object in new_tasks.iteritems():
        # expired
        if task_object.deadline > simulation_step:
            task_object.expired = True
        # complete
        elif not task_object.complete:
            task_object.check_sub_task_completness()

    return new_tasks
