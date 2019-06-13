import numpy as np
import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import random
import os

from helpers import get_utils_location
from helpers import add_coord
from helpers import manhattan_distance
from path_planner import GridPathPlanner

import global_variables
#import rospy #for debug logs


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
    # counter variable
    STEP = 0

    EMPTY_CELL = 0
    UNKNOWN_CELL = -1
    WALL_CELL = -2
    GOAL_CELL = -3
    AGENT_CELL = -4
    ENTITY_CELL = -5
    BLOCK_CELL_STARTING_NUMBER = 101

    def __init__(self, agent_name):
        """
        Initialization of the map. The agent is at the center of an unknown map
        """
        self.agent_name = agent_name
        self.data_directory = self._get_data_directory()

        # fixed for now, but could be made dynamic later
        self.agent_vision = 5

        # map
        self._representation = np.full((11, 11), -1)  # init the map with unknown cells
        self._origin = (self.agent_vision, self.agent_vision)  # the origin of the agent is at the center of the map

        # info about agent in map
        self._agent_position = self._origin
        self._representation[self._agent_position[1]][self._agent_position[0]] = -4

        # objects in map
        self._dispensers = []
        self._goal_areas = []
        self._agents = []
        self._temporary_obstacles = []

        # path_planner
        self.path_planner = GridPathPlanner()
        self.paths = {}

        #### DEBUG ####
        if global_variables.DEBUG_MODE:
            self.PLOT_MAP = True
            # create plot every x steps
            self.PLOT_FREQUENCY = 1
            self.live_plotting = global_variables.LIVE_PLOTTING


    ### PUBLIC METHODS ###
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
        # for entity in perception.entities:
        #     if entity.pos.x == 0 and entity.pos.y == 0: continue
        #     pos = (entity.pos.x, entity.pos.y)
        #     matrix_pos = self._from_relative_to_matrix(pos)
        #     # first index --> y value, second  --> x value
        #     self._representation[matrix_pos[1]][matrix_pos[0]] = -5

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

    def get_exploration_move(self, path_id):
        if not self.paths.has_key(path_id):
            # TODO do we need only the path?
            best_point, best_path, current_high_score = self._get_point_to_explore()
            path_id = self._save_path(best_path)

        direction = self.path_planner.next_move_direction(
            self._from_relative_to_matrix(self._agent_position),
            self.paths[path_id])

        # TODO IMPROVE CODE QUALITY
        if direction == 'end':
            best_point, best_path, current_high_score = self._get_point_to_explore()
            path_id = self._save_path(best_path)

            direction = self.path_planner.next_move_direction(
                self._from_relative_to_matrix(self._agent_position),
                self.paths[path_id])

        rospy.logdebug("Best path: " + str(self.paths[path_id]))
        rospy.logdebug("direction: " + direction)
        return path_id, direction

    ### PRIVATE METHODS ###
    def _save_path(self, path):
        path_id = random.randint(1,9999999)
        while self.paths.has_key(path_id):
            path_id = random.randint(1, 9999999)
        self.paths[path_id] = path
        return path_id

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

    ### EXPLORATION FUNCTIONALITIES ###
    def _get_unknown_amount(self, position):
        """calculate amount of unknown cells around a given cell

        Args:
            map_representation (np.array):
            position (tuple): tuple containing x,y coordinates

        Returns:
            int: amount of unknown cells around given position
        """
        vision_range = 5
        unknown_count = 0
        # loop through all cells that are in (theoretical) vision from specified position
        # not considering vision hindering through obstacles
        for y in range(-vision_range, vision_range + 1):
            for x in range(-vision_range, vision_range + 1):
                cell_index = (position[0] + y, position[1] + x)
                # omit cells that are out of bounds
                if cell_index not in np.ndindex(self._representation.shape): continue
                # if unknown, increase unknown count
                if self._representation[cell_index[0], cell_index[1]] == -1:
                    unknown_count += 1

        return unknown_count

    def _get_point_to_explore(self):
        """Calculates point that is most suited for exploring and path to it

        Args:
            map_representation (np.array):
            current_position (tuple): position of the agent

        Returns:
            tuple: tuple containing best_point (tuple), best_path list(tuples), amount of unkown cells to be explored when this point is reached by the agent (int)
        """
        # map indices
        lower_bound = 0
        upper_bound = self._representation.shape[0]

        # keep track of best suitable points for exploration
        best_points = []
        current_high_score = 0

        # select all the points that are close to an unknown cell
        possible_points = []
        for y, x in np.ndindex(self._representation.shape):
            if self._representation[y][x] == -1:
                for direction in global_variables.moving_directions:
                    cell_coord = (y + direction[0], x + direction[1])
                    # Make sure in range
                    if self._representation.shape[0] - 1 > cell_coord[0] >= 0 \
                            and self._representation.shape[1] - 1 > cell_coord[1] >= 0:
                        # Adherence cell is walkable
                        if self._representation[cell_coord] not in (self.UNKNOWN_CELL, self.WALL_CELL, self.ENTITY_CELL):
                            possible_points.append(cell_coord)

        # loop through all points in the map
        for (y, x) in possible_points:
            # consider all points that are 5 cells away from a wall
            """
            if ((x == lower_bound + 5 and lower_bound + 5 < y < upper_bound - 5) or (
                    x == upper_bound - 5 and lower_bound + 5 < y < upper_bound - 5) or (
                        y == lower_bound + 5 and lower_bound + 5 < x < upper_bound - 5) or (
                        y == upper_bound - 5 and lower_bound + 5 < x < upper_bound - 5)) and (
                    self._representation[x][y] == 0):
            """
            # calculate the amount of unknown cells around this cell
            unknown_count = self._get_unknown_amount((y, x))
            '''
            if unknown_count == current_high_score:
                best_points.append([(y, x)])            
            elif unknown_count > current_high_score:
                current_high_score = unknown_count
                best_points = [[(y, x)]]
            '''
            if unknown_count > current_high_score:
                current_high_score = unknown_count
                best_points.append([(y, x)])


        # calculate path length between current position and potential exploration points and choose the one with shortest path
        shortest_path = np.inf
        best_point = None
        best_path = None
        print ("points:" + str(best_points))
        for point in best_points:
            #print(point)
            path = self.path_planner.astar(self._representation, self._agent_position, point)
            #print(path)
            length = len(path)
            print (length)
            if path is not None:
                if length < shortest_path:
                    best_point = point
                    best_path = path
                    shortest_path = length

        return best_point, best_path, current_high_score


def main():
    import time

    my_map = GridMap('Agent1')
    my_map._representation = np.loadtxt(open("generatedMaps/00/partial.csv", "rb"), delimiter=",")
    my_agent = [[4, 14]]
    my_map._agent_position = np.array(my_agent, dtype=np.int)

    start_time = time.time()
    best_point, best_path, current_high_score = my_map._get_point_to_explore()
    print ("---%s seconds ---" % (time.time() - start_time))
    print ("Best point:" + str(best_point))
    #print ("Best path:" + str(best_path))
    #print ("Current high score:" + str(current_high_score))




if __name__ == '__main__':
    main()