import unittest
import numpy as np
from classes.grid_map import GridMap

class TestGridMap(unittest.TestCase):
    def setUp(self):
        self.my_map = GridMap('Agent1', 5)
        self.my_map._representation = np.loadtxt(open("test_maps/01_test_map.txt", "rb"), delimiter=",")
        self.my_map._update_distances()
        self.my_map._origin = np.array([4, 14], dtype=np.int)
        self.my_map._agent_position = np.array([0, 0], dtype=np.int)
    def test_(self):
        self.my_map._set_goal_top_left()
        print(self.my_map.goal_top_left)
        self.assertItemsEqual(self.my_map.goal_top_left, np.array([2,3]))


if __name__ == '__main__':

    unittest.main()