import pytest
import numpy as np
from classes.grid_map import GridMap

import pickle


def load_map(map_name, origin):
    my_map = GridMap('agentA1', 5)
    my_map._representation = np.loadtxt(open("test_maps/{}.txt".format(map_name), "rb"), delimiter=",")
    my_map._representation = my_map._representation
    # TODO we consider no obstacles in this case, we now it works with obstacles
    my_map._path_planner_representation = my_map._representation
    my_map.origin = np.array(origin, dtype=np.int)
    # Agent position equal to agent origin
    my_map._agent_position = my_map.origin
    my_map._set_goal_top_left()
    return my_map

# move 7 8 agentA1
# move 10 10 agentA2

@pytest.fixture
def map1():
    return load_map('agentA1', [13, 12])

@pytest.fixture
def map2():
    return load_map('agentA2', [17, 15])


@pytest.fixture
def task_dictionary_2_001():
    """ Task for two agents with different blocks

    """
    task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_2_001.dat", "rb")
    complete_task_list = pickle.load(task)
    task_dictionary_2_001 = complete_task_list['task1']

    return task_dictionary_2_001

# @pytest.fixture
# def task_dictionary_3_001():
#     """ Task for two agents with different blocks
#
#     """
#     task = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/task_assignment_for_3_001.dat", "rb")
#     complete_task_list = pickle.load(task)
#     task_dictionary_3_001 = complete_task_list['task1']
#
#     return task_dictionary_3_001

def test_meeting_point_for_2_001(map1, task_dictionary_2_001):
    """

    """
    figure_old = map1.create_figure(task_dictionary_2_001)
    # np.testing.assert_array_equal(actual_figure, desired_figure)
    agent1, agent2, actual_meeting_point = map1.get_common_meeting_point(task_dictionary_2_001)
    # # np.testing.assert_array_equal(actual_meeting_point, desired_meeting_point)
    meeting_position = map1.meeting_position(task_dictionary_2_001, actual_meeting_point)
    print (meeting_position)
    #
    # #np.testing.assert_array_equal(actual_figure, desired_figure)

# def test_meeting_point_for_3_001(map1, task_dictionary_3_001):
#
#     actual_figure = map1.create_figure(task_dictionary_2_001)
#     # np.testing.assert_array_equal(actual_figure, desired_figure)
#     print ("dioca")





