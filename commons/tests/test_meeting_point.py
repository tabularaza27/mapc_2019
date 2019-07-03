import pytest
import numpy as np
from classes.grid_map import GridMap

import pickle


def load_map(map_name, origin):
    my_map = GridMap('Agent1', 5)
    my_map._representation = np.loadtxt(open("test_maps/{}.txt".format(map_name), "rb"), delimiter=",")
    my_map._path_planner_representation = my_map._representation
    my_map.origin = np.array(origin, dtype=np.int)
    # Agent position equal to agent origin
    my_map._agent_position = my_map.origin
    my_map._set_goal_top_left()
    return my_map

@pytest.fixture
def map1():
    return load_map('04_test_map', [10, 15])

@pytest.fixture
def map2():
    return load_map('04_test_map', [26, 20])


@pytest.fixture
def task_dictionary1():
    """
    { 'Agent_name': ( distance_to_dispenser, dispenser_position, block_position_in_figure)}

    dispensers: b0 : 1, b1 : 2, b2 : 3
    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/dumped_class.dat", "rb")
    task_dictionary1 = pickle.load(task)

    return task_dictionary1

def test_meeting_point_1(map1, task_dictionary1, task_dictionary2, task_dictionary3):
    """Cases:
            1: 2 agents
            2: 3 agents, Agent1 and Agent 2 are the closest
            3: 3 agents, Agent2 and Agent 3 are the closest

    Args:
        map1: Map representation in matrix notation
        task_dictionary1: { 'Agent_name': ( distance_to_dispenser, dispenser_position, block_position_in_figure)}

    Returns:

    """

    # case_number = 1
    # task_dictionary = []
    #
    # if case_number == 1:
    #     task_dictionary = task_dictionary1
    #     desired_meeting_point = np.array([[15, 24]])
    # if case_number == 2:
    #     task_dictionary = task_dictionary2
    #     desired_meeting_point = np.array([[15, 24]])
    # if case_number == 3:
    #     task_dictionary = task_dictionary3
    #     desired_meeting_point = np.array([[29, 24]])

    desired_meeting_point = np.array([[15, 24]])
    actual_meeting_point = map1.get_common_meeting_point(task_dictionary1)

    np.testing.assert_array_equal(actual_meeting_point, desired_meeting_point)





