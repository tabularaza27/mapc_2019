import numpy as np
import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import random
import os

from helpers import get_utils_location
from helpers import add_coord
from helpers import manhattan_distance


# ToDo: Implement blocks
# ToDo: Keep track of blocks / entities when they are moving
# ToDo: Identify if other entity is enemy or ally


class GridMap:
    """
    Class that represent the local map of an agent like in the scenario.

    The map is a two dimensional np.array, initialized with size (11,11), that is the vision of the agent.
    If the agent moves in a direction, where is vision exceeds the border of the map, a new row / column is added to the np.array
    Note, that the first index of the np.array is for the y value and second is for the x value

    MAP SPECIFICATION
    Matrix of integers. Each value has the following meaning:

    # -1: unknown cell
    # -2: wall, obstacle
    # -3: goal area
    # -4: current agent
    # -5: other agents

    # enemy entity agent ??necessary??temporary??
    # ally entity agent ??necessary??temporary??
    # 10,11,12,13,14,...: dispenser of type 0,1,2,3,4,...
    # 100,101,102,103,104,...: block of type 0,1,2,3,4,...

    author: Alessandro
    """
    # create plot every x steps
    PLOT_FREQUENCY = 1
    # counter variable
    STEP = 0

    EMPTY_CELL = 0
    UNKNOWN_CELL = -1
    WALL_CELL = -2
    GOAL_CELL = -3
    AGENT_CELL = -4
    ENTITY_CELL = -5
    BLOCK_CELL_STARTING_NUMBER = 101

    def __init__(self, agent_name, live_plotting=False):
        """
        Initialization of the map. The agent is at the center of an unknown map
        """
        self.agent_name = agent_name
        self.live_plotting = live_plotting
        self.data_directory = self._get_data_directory()

        # fixed for now, but could be made dynamic later
        self.agent_vision = 5

        # map
        self._representation = np.full((11, 11), -1)  # init the map with unknown cells
        self._origin = (5, 5)  # the origin of the agent is at the center of the map

        # info about agent in map
        self._agent_position = self._origin
        self._representation[self._agent_position[1]][self._agent_position[0]] = -4

        # objects in map
        self._dispensers = []
        self._goal_areas = []
        self._agents = []
        self._temporary_obstacles = []

    def update_map(self, agent, perception):
        """
        Update the map according to the movement of an agent and the new perception.
        Args:
            agent (dict): info about agent. dict is received from the percept
            perception (rhbp.perception_provider): the new perception of the agent

        Returns: void

        """
        # if last action was move update agent position and expand map size if sight is out of bounds
        if agent.last_action == "move" and agent.last_action_result == "success":
            self._update_agent_position(move=agent.last_action_params[0])

        # update empty cells (all cells that are in vision range, get overwritten below if they are occupied)
        for i in range(0, self.agent_vision + 1):
            for j in range(0, self.agent_vision + 1):
                if 0 < manhattan_distance(self._agent_position, add_coord(self._agent_position, (i, j))) <= 5:
                    self._representation[self._agent_position[1] + i][self._agent_position[0] + j] = 0
                    self._representation[self._agent_position[1] - i][self._agent_position[0] - j] = 0
                    self._representation[self._agent_position[1] - i][self._agent_position[0] + j] = 0
                    self._representation[self._agent_position[1] + i][self._agent_position[0] - j] = 0


        # update obstacles
        for obs in perception.obstacles:
            pos = (obs.pos.x, obs.pos.y)
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value
            self._representation[matrix_pos[1]][matrix_pos[0]] = -2

        # update goals
        for goal in perception.goals:

            # add to local map
            pos = (goal.pos.x, goal.pos.y)
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value
            self._representation[matrix_pos[1]][matrix_pos[0]] = -3

            # add to goals variable
            if pos not in self._goal_areas:
                self._goal_areas.append(pos)

        # update entities (other agents)
        for entity in perception.entities:
            if entity.pos.x == 0 and entity.pos.y == 0: continue
            pos = (entity.pos.x, entity.pos.y)
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value
            self._representation[matrix_pos[1]][matrix_pos[0]] = -5

            # ToDo update agents state variable, including info which team the agent belong

        # update dispensers
        for dispenser in perception.dispensers:
            pos = (dispenser.pos.x, dispenser.pos.y)
            matrix_pos = self._from_relative_to_matrix(pos)
            # get dispenser type
            for i in range(4):
                if str(i) in dispenser.type:
                    self._representation[matrix_pos[1]][matrix_pos[0]] = 10 + i

            # add to state variable as dict
            # {pos: {x: 3, y: 3}, type: 'b1'}
            if dispenser not in self._dispensers:
                self._dispensers.append(dispenser)

        # update blocks

        # write data to file, used for live plotting plotting
        if self.live_plotting and self.STEP % self.PLOT_FREQUENCY == 0:
            self._write_data_to_file()

        self.STEP += 1

    def _from_relative_to_matrix(self, relative_coord):
        """
        Function that translate the coordinate with respect to the origin of the map to the
        origin of the matrix

        Args:
            relative_coord: (x,y) with respect to the origin of the map

        Returns:
            matrix_coord: (x',y') with respect to the origin of the matrix
        """
        return add_coord(relative_coord, self._agent_position)

    def _from_matrix_to_relative(self, matrix_coord):
        """
        Reverse function of from_relative_to_matrix
        Args:
            matrix_coord: (x,y) with respect to the origin of the matrix

        Returns:
            relative_coord: (x',y') with respect to the origin of the map
        """

        return add_coord(matrix_coord, self._agent_position, operation='sub')

    def _update_agent_position(self, move=None):
        """update agents position in map and expand grid if sight is out of bounds

        Args:
            move:

        Returns:

        """
        assert move in [None, 'n', 'e', 's',
                        'w'], "Move direction needs to be 'n','e','s','w'. '{}' was provided".format(move)

        if move is not None:
            self._representation[self._agent_position[1]][self._agent_position[0]] = -1

            if move == 'n':
                #
                if self._agent_position[1] - 5 <= 0:
                    self._expand_map('n')
                else:
                    self._agent_position = add_coord(self._agent_position, (0, -1))


            elif move == 'e':
                self._agent_position = add_coord(self._agent_position, (1, 0))
                if self._agent_position[0] + 5 >= self._representation.shape[1]:
                    self._expand_map('e')

            elif move == 's':
                self._agent_position = add_coord(self._agent_position, (0, 1))
                if self._agent_position[1] + 5 >= self._representation.shape[0]:
                    self._expand_map('s')

            elif move == 'w':
                if self._agent_position[0] - 5 <= 0:
                    self._expand_map('w')
                else:
                    self._agent_position = add_coord(self._agent_position, (-1, 0))

            self._representation[self._agent_position[1]][self._agent_position[0]] = -4

        else:
            pass

    def _expand_map(self, direction):
        """Expands map in given direction with vector of unknowns

        Adds row / column to numpy array

        Args:
            direction (str): {'n', 'e', 's', 'w'} direction in which the map should be expanded

        Returns: void

        """
        assert direction in ['n', 'e', 's',
                             'w'], "Expansion direction needs to be 'n','e','s','w'. '{}' was provided".format(
            direction)

        old_map_shape = self._representation.shape

        # this is the fastest way to add a row / column to a numpy array
        # see 'https://stackoverflow.com/questions/8486294/how-to-add-an-extra-column-to-a-numpy-array' if interested
        if direction == 'n':
            helper_map = np.full((old_map_shape[0] + 1, old_map_shape[1]), fill_value=-1)
            helper_map[1:, :] = self._representation
        if direction == 's':
            helper_map = np.full((old_map_shape[0] + 1, old_map_shape[1]), fill_value=-1)
            helper_map[:-1, :] = self._representation
        if direction == 'e':
            helper_map = np.full((old_map_shape[0], old_map_shape[1] + 1), fill_value=-1)
            helper_map[:, :-1] = self._representation
        if direction == 'w':
            helper_map = np.full((old_map_shape[0], old_map_shape[1] + 1), fill_value=-1)
            helper_map[:, 1:] = self._representation

        self._representation = helper_map

    def _get_data_directory(self):
        """Returns Directory where map data is stored for plotting purposes"""
        utils_dir = get_utils_location()
        return os.path.join(utils_dir, 'generatedMaps', 'tmp_maps')

    def _write_data_to_file(self):
        """writes two dimensional np.array to .txt file named after agent and in directory /utils/generatedMaps/tmp_maps"""
        np.savetxt(os.path.join(self.data_directory, '{}.txt'.format(self.agent_name)), self._representation, fmt='%i',
                   delimiter=',')