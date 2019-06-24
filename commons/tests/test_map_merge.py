import pytest
import numpy as np
from classes.grid_map import GridMap
from classes.map_merge import mapMerge

def load_map(map_name, origin):
    my_map = GridMap('Agent1', 5)
    my_map._representation = np.loadtxt(open("test_maps/{}.txt".format(map_name), "rb"), delimiter=",")
    my_map._origin = np.array(origin, dtype=np.int)
    my_map._set_goal_top_left()
    return my_map

@pytest.fixture
def map1():
    return load_map('01_test_map', [4, 14])

@pytest.fixture
def map2():
    return load_map('02_test_map', [4, 14])

@pytest.fixture
def map3():
    return load_map('03_test_map', [4, 14])

def test_left_corner(map1, map2, map3):
    """
    Function that test the function mapMerge
    Args:
        my_map: a GridMap instance for testing
    """
    #TODO return shift to add to the origin point of map2 and test it
    merged_map = mapMerge(map1._representation, map2._representation, map1.goal_top_left, map2.goal_top_left)
    print(merged_map)
    np.testing.assert_array_equal(merged_map, map3._representation)



