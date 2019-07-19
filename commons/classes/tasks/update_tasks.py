""" This module contains functionalities for creating and keeping track of tasks based on the percept """

from classes.tasks.task import Task


def update_tasks(current_tasks, tasks_percept, simulation_step):
    """updates task representation based on new percept.

    * adds tasks, that are new
    * remove tasks, that are no longer in the perception
    * marks tasks that are fully auctioned ( all subtasks are assigned to an agent ). Task is ready to be executed
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

    # select tasks that are not in the percept anymore
    to_remove_tasks = list()
    for current_task in current_tasks:
        to_remove = True
        for task in tasks_percept:
            if task.name == current_task:
                to_remove = False
        if to_remove:
            to_remove_tasks.append(current_task)

    # delete those tasks
    for task_name in to_remove_tasks:
        del current_tasks[task_name]

    # mark expired tasks & mark complete tasks
    # ToDo implement way so that completed and expired tasks don't get looped through again every time
    for task_name, task_object in new_tasks.iteritems():
        # skip expired and complete tasks
        if task_object.expired or task_object.complete:
            continue
        # check if task is expired and mark it if that is true
        elif task_object.deadline < simulation_step:
            task_object.expired = True
        # check if task has been fully auctioned and mark it, if true.
        # When the task is auctioned the submit behaviour can be delegated to the agent fulfilling a sub task
        # The execution of the task can now start
        elif not task_object.auctioned:
            task_object.check_auctioning()
        # check if task is complete and mark it if its true
        elif not task_object.complete:
            task_object.check_sub_task_completness()

    return new_tasks
