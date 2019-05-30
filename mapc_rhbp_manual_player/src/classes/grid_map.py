import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

class GridMap:
    """
    Class that represent the local map of an agent like in the scenario.

    MAP SPECIFICATION
    Matrix of integers. Each value has the following meaning:
    #  0: empty cell
    # -1: unknown cell
    # -2: wall, obstacle
    # -3: goal area
    # -4: enemy entity agent ??necessary??temporary??
    # -5: ally entity agent ??necessary??temporary??
    # 1,2,3,4,...: dispenser of type 1,2,3,4,...
    # 101,102,103,104,...: block of type 1,2,3,4,...

    author: Alessandro
    """
    EMPTY_CELL = 0
    UNKNOWN_CELL = -1
    WALL_CELL = -2
    GOAL_CELL = -3
    BLOCK_CELL_STARTING_NUMBER = 101

    ### PUBLIC METHODS ###
    def __init__(self, perception):
        """
        Initialization of the map. The agent is at the center of an unknown map
        """
        self._representation = np.full((11,11), -1) # init the map with unknown cells
        self._origin = (5,5) # the origin of the agent is at the center of the map
        self._dispensers = []
        self._goal_areas = []
        self._agents = []
        self._temporary_obstacles = []

    def updateMap(self, agent, destination, perception):
        """
        Update the map according to the movement of an agent and the new perception.
        Args:
            agent: the agent proprietary of the perception
            destination: the direction in which the agent moved {n,s,e,w, None}
            perception: the new perception of the agent

        Returns: void

        """
        print(perception)

    ### PRIVATE METHODS ###
    def _from_relative_to_matrix(self, relative_coord):
        """
        Function that translate the coordinate with respect to the origin of the map to the
        origin of the matrix

        Args:
            relative_coord: (x,y) with respect to the origin of the map

        Returns:
            matrix_coord: (x',y') with respect to the origin of the matrix
        """
        return relative_coord + self._origin

    def _from_matrix_to_relative(self, matrix_coord):
        """
        Reverse function of from_relative_to_matrix
        Args:
            matrix_coord: (x,y) with respect to the origin of the matrix

        Returns:
            relative_coord: (x',y') with respect to the origin of the map
        """
        return matrix_coord - self._origin


    #### DEBUG FUNCTIONALITIES ####

    def show_map(self):
        """
        Debug function to show a colored map
        Returns: None

        """
        cmap = mpl.colors.ListedColormap(['red', 'black', 'blue', 'white'])
        plt.matshow(self._representation, cmap=cmap, vmin=-3, vmax=1)
        plt.show(block=False)