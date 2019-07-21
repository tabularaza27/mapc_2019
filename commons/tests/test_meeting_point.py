import pytest
import numpy as np
from classes.mapping.grid_map import GridMap

import pickle


def load_map(map_name, origin):
    my_map = GridMap(map_name, 5)
    my_map._representation = np.loadtxt(open("test_maps/{}.txt".format(map_name), "rb"), delimiter=",")
    my_map._representation = my_map._representation
    # TODO we consider no obstacles in this case, we now it works with obstacles
    my_map._path_planner_representation = my_map._representation
    my_map.origin = np.array(origin, dtype=np.int)
    # Agent position equal to agent origin
    my_map._agent_position = my_map._from_matrix_to_relative(my_map.origin)
    my_map._set_goal_top_left()
    return my_map

# move 7 8 agentA1
# move 10 10 agentA2

@pytest.fixture
def map1():
    return load_map('agentA1', [13, 12])

@pytest.fixture
def map2():
    return load_map('agentA3', [17, 11])

@pytest.fixture
def map5():
    return load_map('agentA3', [18, 18])


@pytest.fixture
def task_dictionary_2_001():
    """ Task for two agents with different blocks

    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_2_001.dat", "rb")
    complete_task_list = pickle.load(task)
    task_dictionary_2_001 = complete_task_list['task1']

    return task_dictionary_2_001

@pytest.fixture
def task_dictionary_3_001():
    """ Task for two agents with different blocks

    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_3_001.dat", "rb")
    complete_task_list = pickle.load(task)
    task_dictionary_3_001 = complete_task_list['task1']

    return task_dictionary_3_001

@pytest.fixture
def task_dictionary_5_001():
    """ Task for two agents with different blocks

    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_5_001.dat", "rb")
    complete_task_list = pickle.load(task)
    task_dictionary_5_001 = complete_task_list['task1']

    return task_dictionary_5_001

@pytest.fixture
def task_dictionary_5_003():
    """ Task for two agents with different blocks

    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_5_003.dat", "rb")
    complete_task_list = pickle.load(task)
    task_dictionary_5_003 = complete_task_list['task1']

    return task_dictionary_5_003

@pytest.fixture
def task_dictionary_5_fixed():
    """ Task for two agents with different blocks

    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_5_fixed.dat", "rb")
    complete_task_list = pickle.load(task)
    task_dictionary_5_fixed = complete_task_list['task1']

    return task_dictionary_5_fixed

# def test_meeting_point_for_2_001(map1, task_dictionary_2_001):
#     """
#
#     """
#     figure_old = map1.create_figure(task_dictionary_2_001)
#     # np.testing.assert_array_equal(actual_figure, desired_figure)
#     agent1, agent2, actual_meeting_point = map1.get_common_meeting_point(task_dictionary_2_001)
#     # # np.testing.assert_array_equal(actual_meeting_point, desired_meeting_point)
#     meeting_position = map1.meeting_position(task_dictionary_2_001, actual_meeting_point)
#     print (map1.agent_name, meeting_position)
#     #
#     # #np.testing.assert_array_equal(actual_figure, desired_figure)

def test_meeting_point_for_3_001(map2, task_dictionary_3_001):

    actual_figure, submit_agent_index = map2.create_figure(task_dictionary_3_001)
    actual_meeting_point = map2.get_common_meeting_point(task_dictionary_3_001)
    nearby_agents, meeting_position = map2.meeting_position(task_dictionary_3_001, actual_meeting_point)
    print (map2.agent_name, meeting_position)

    print ("dioca")

# def test_meeting_point_for_5_001(map5, task_dictionary_5_00):
#
#     actual_figure, submit_agent_index = map5.create_figure(task_dictionary_5_001)
#     agent1, agent2, actual_meeting_point = map5.get_common_meeting_point(task_dictionary_5_001)
#     meeting_position = map5.meeting_position(task_dictionary_5_001, actual_meeting_point)
#     print (map2.agent_name, meeting_position)
#
#     print ("dioca")

# def test_meeting_point_for_5_003(map5, task_dictionary_5_003):
#
#     actual_figure, submit_agent_index = map5.create_figure(task_dictionary_5_003)
#     agent1, agent2, actual_meeting_point = map5.get_common_meeting_point(task_dictionary_5_003)
#     meeting_position = map5.meeting_position(task_dictionary_5_003, actual_meeting_point)
#     print (map5.agent_name, meeting_position)
#
#     print ("dioca")

# def test_meeting_point_for_5_fixed(map5, task_dictionary_5_fixed):
#     actual_figure, submit_agent_index = map5.create_figure(task_dictionary_5_fixed)
#     agent1, agent2, actual_meeting_point = map5.get_common_meeting_point(task_dictionary_5_fixed)
#     meeting_position = map5.meeting_position(task_dictionary_5_fixed, actual_meeting_point)
#     print (map5.agent_name, meeting_position)
#
#     print ("dioca")




