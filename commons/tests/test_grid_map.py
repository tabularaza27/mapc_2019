import pytest
import numpy as np
from classes.mapping.grid_map import GridMap

@pytest.fixture
def my_map():
    my_map = GridMap('Agent1', 5)
    my_map._representation = np.loadtxt(open("test_maps/01_test_map.txt", "rb"), delimiter=",")
    my_map._update_distances()
    my_map._origin = np.array([4, 14], dtype=np.int)
    my_map._agent_position = np.array([0, 0], dtype=np.int)
    return my_map

def test_left_corner(my_map):
    """
    Function that test the function GridMap._set_goal_top_left
    Args:
        my_map: a GridMap instance for testing
    """
    my_map._set_goal_top_left()
    print(my_map.goal_top_left)
    np.testing.assert_array_equal(my_map.goal_top_left, np.array([-2,-11]))



