import pytest
import numpy as np
from classes.mapping.grid_map import GridMap
from classes.mapping.block import Block

def load_map(map_name, origin):
    my_map = GridMap('Agent1', 5)
    my_map._representation = np.loadtxt(open("test_maps/{}.txt".format(map_name), "rb"), delimiter=",")
    my_map._path_planner_representation = my_map._representation
    my_map._origin = np.array(origin, dtype=np.int)
    my_map._set_goal_top_left()
    return my_map

@pytest.fixture
def map1():
    map1 = load_map('01_test_map', [5, 5])
    map1._attached_blocks.append(Block(block_type='b1', position=np.array([0, 1])))
    map1._attached_blocks.append(Block(block_type='b1', position=np.array([0, 2])))
    return map1

@pytest.fixture
def map5():
    map5 = load_map('05_test_map', [5, 5])
    map5._attached_blocks.append(Block(block_type='b1', position=np.array([0, 1])))
    map5._attached_blocks.append(Block(block_type='b1', position=np.array([0, 2])))
    return map5


def test_possible_configurations_free(map1):
    """
    test possible configurations for the goal area destination
    Args:
        my_map: a GridMap instance for testing
    """
    goal_area_point = np.array(map1.goal_top_left) + np.array([1, 1])
    possible_configurations = map1.get_possible_configurations_in_point(goal_area_point)
    desired_configurations = []
    desired_configurations.append(np.array([[-2, -1], [-2, 0], [-2, 1]]))
    desired_configurations.append(np.array([[-2, -1], [-1, -1], [0, -1]]))
    desired_configurations.append(np.array([[-2, -1], [-2, -2], [-2, -3]]))
    desired_configurations.append(np.array([[-2, -1], [-3, -1], [-4, -1]]))

    np.testing.assert_array_equal(possible_configurations, desired_configurations)

def test_possible_configurations_obstacle1(map5):
    """
    test possible configurations for the goal area destination
    Args:
        my_map: a GridMap instance for testing
    """
    goal_area_point = np.array(map5.goal_top_left) + np.array([1, 1])
    possible_configurations = map5.get_possible_configurations_in_point(goal_area_point)
    desired_configurations = list()
    desired_configurations.append(np.array([[-2, -1], [-2, 0], [-2, 1]]))
    desired_configurations.append(np.array([[-2, -1], [-1, -1], [0, -1]]))
    desired_configurations.append(np.array([[-2, -1], [-3, -1], [-4, -1]]))

    np.testing.assert_array_equal(possible_configurations, desired_configurations)
