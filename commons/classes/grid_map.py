import numpy as np
from collections import deque
import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import random
import os

from helpers import get_commons_location
from path_planner import GridPathPlanner

import global_variables
import rospy #for debug logs


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
    DISPENSER_STARTING_NUMBER = 1

    def __init__(self, agent_name, agent_vision):
        """
        Initialization of the map. The agent is at the center of an unknown map
        """
        self.agent_name = agent_name
        self.data_directory = self._get_data_directory()

        # fixed for now, but could be made dynamic later
        self.agent_vision = agent_vision

        # map
        self._representation = np.full((11, 11), -1)  # init the map with unknown cells
        self._origin = (self.agent_vision, self.agent_vision)  # the origin of the agent is at the center of the map

        # info about agent in map
        self._agent_position = np.array([0, 0])
        # IS IT USEFUL TO DRAW THE AGENT IN THE MAP?
        #agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        #self._representation[agent_in_matrix[1]][agent_in_matrix[0]] = -4
        self._distances = np.array([]) # the matrix of all the distances from the agent

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
        agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        self._representation[agent_in_matrix[0],agent_in_matrix[1]] = 0
        # update empty cells (all cells that are in vision range, get overwritten below if they are occupied)
        for j in range(-self.agent_vision , self.agent_vision + 1):
            for i in range(-self.agent_vision, self.agent_vision + 1):
                cell = agent_in_matrix + np.array([j, i])
                if 0 < GridMap.manhattan_distance(agent_in_matrix, cell) <= self.agent_vision:
                    self._representation[cell[0]][cell[1]] = 0


        # update obstacles
        for obs in perception.obstacles:
            pos = np.array([obs.pos.y, obs.pos.x]) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value

            self._representation[matrix_pos[0]][matrix_pos[1]] = -2

        # update goals
        for goal in perception.goals:

            # add to local map
            pos = (goal.pos.y, goal.pos.x)
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value
            self._representation[matrix_pos[0]][matrix_pos[1]] = -3

            # add to goals variable
            if pos not in self._goal_areas:
                self._goal_areas.append(pos)

        # update entities (other agents)
        # for entity in perception.entities:
        #     if entity.pos.x == 0 and entity.pos.y == 0: continue
        #     pos = (entity.pos.x, entity.pos.y)
        #     matrix_pos = self._from_relative_to_matrix(pos)
        #     # first index --> y value, second  --> x value
        #     self._representation[matrix_pos[1]][matrix_pos[0]] = -self.agent_vision

            # ToDo update agents state variable, including info which team the agent belong

        # update dispensers
        for dispenser in perception.dispensers:
            pos = (dispenser.pos.y, dispenser.pos.x)
            matrix_pos = self._from_relative_to_matrix(pos)
            # get dispenser type
            for i in range(4):
                if str(i) in dispenser.type:
                    self._representation[matrix_pos[0]][matrix_pos[1]] = global_variables.DISPENSER_STARTING_NUMBER + i

            # add to state variable as dict
            # {pos: {x: 3, y: 3}, type: 'b1'}
            if dispenser not in self._dispensers:
                self._dispensers.append(dispenser)

        # update blocks
            # TODO

        # update distances
        self._update_distances()

        # write data to file, used for live plotting plotting
        if self.live_plotting and self.STEP % self.PLOT_FREQUENCY == 0:
            self._write_data_to_file()

        self.STEP += 1

    def _update_distances(self):
        """
        Update the matrix of distances from the agent
        Returns: void

        """
        dist_shape = self._representation.shape
        self._distances = np.full((dist_shape[0], dist_shape[1]), -1, dtype=int)
        print(self._distances.shape)
        print(self._origin)
        print(self._agent_position)
        agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        queue = deque([(agent_in_matrix, 0)])
        while len(queue) > 0:
            pos, dist = queue.popleft()
            if self._distances[pos[0], pos[1]] == -1: #to avoid infinite loop
                self._distances[pos[0], pos[1]] = dist
                for direction in global_variables.moving_directions: # ADD ALSO ROTATIONS?
                    new_pos = direction + pos
                    if GridMap.coord_inside_matrix(new_pos, dist_shape):
                        if self._distances[new_pos[0], new_pos[1]] == -1:
                            cell_value = self._representation[new_pos[0], new_pos[1]]
                            if cell_value != self.WALL_CELL \
                                    and cell_value != self.UNKNOWN_CELL:
                                queue.append((new_pos, dist+1))



    def get_exploration_move(self, path_id):
        if not self.paths.has_key(path_id):
            # TODO do we need only the path?
            best_point, best_path, current_high_score = self._get_point_to_explore()
            path_id = self._save_path(best_path)

        direction = self.path_planner.next_move_direction(
            self._agent_position,
            self.paths[path_id])

        # TODO IMPROVE CODE QUALITY
        if direction == 'end':
            best_point, best_path, current_high_score = self._get_point_to_explore()
            path_id = self._save_path(best_path)

            direction = self.path_planner.next_move_direction(
                self._agent_position,
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
        matrix_coord = np.copy(relative_coord)
        matrix_coord = matrix_coord + self._origin

        return matrix_coord

    def _from_matrix_to_relative(self, matrix_coord):
        """
        Reverse function of from_relative_to_matrix
        Args:
            matrix_coord: (x,y) with respect to the origin of the matrix

        Returns:
            relative_coord: (x',y') with respect to the origin of the map
        """

        relative_coord = np.copy(matrix_coord)
        relative_coord = relative_coord + self._origin

        return relative_coord

    def _update_agent_position(self, move=None):
        """update agents position in map and expand grid if sight is out of bounds

        Args:
            move: n,s,w or e as strings

        Returns:

        """
        assert move in [None, 'n', 'e', 's',
                        'w'], "Move direction needs to be 'n','e','s','w'. '{}' was provided".format(move)

        agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        if move is not None:
            self._representation[agent_in_matrix[0]][agent_in_matrix[1]] = 0 # ??
            move_array = global_variables.movements[move]
            self._agent_position = self._agent_position + move_array
            agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
            if (agent_in_matrix <= self.agent_vision).any() \
                    or agent_in_matrix[1] + self.agent_vision >= self._representation.shape[1] \
                    or agent_in_matrix[0] + self.agent_vision >= self._representation.shape[0]:
                self._expand_map(move)

            agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
            self._representation[agent_in_matrix[0]][agent_in_matrix[1]] = -4

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
            helper_map = np.full((old_map_shape[0] + 1, old_map_shape[1]), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[1:, :] = self._representation
            self._origin = self._origin + np.array([1, 0])
        if direction == 's':
            helper_map = np.full((old_map_shape[0] + 1, old_map_shape[1]), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[:-1, :] = self._representation
        if direction == 'e':
            helper_map = np.full((old_map_shape[0], old_map_shape[1] + 1), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[:, :-1] = self._representation
        if direction == 'w':
            helper_map = np.full((old_map_shape[0], old_map_shape[1] + 1), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[:, 1:] = self._representation
            self._origin = self._origin + np.array([0, 1])

        self._representation = helper_map

    def _get_data_directory(self):
        """Returns Directory where map data is stored for plotting purposes"""
        commons_dir = get_commons_location()
        return os.path.join(commons_dir, 'generatedMaps', 'tmp_maps')

    def _write_data_to_file(self):
        """writes two dimensional np.array to .txt file named after agent and in directory /commons/generatedMaps/tmp_maps"""
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
        unknown_count = 0
        # loop through all cells that are in (theoretical) vision from specified position
        # not considering vision hindering through obstacles
        for j in range(-self.agent_vision , self.agent_vision + 1):
            for i in range(-self.agent_vision, self.agent_vision + 1):
                cell = position + np.array([j, i])
                if 0 < GridMap.manhattan_distance(position, cell) <= self.agent_vision:
                    if not GridMap.coord_inside_matrix(cell, self._representation.shape):
                        unknown_count += 1
                    else:
                        if self._representation[cell[0], cell[1]] == self.UNKNOWN_CELL:
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


        # select all the points that are close to an unknown cell
        possible_points = []
        unknown_counts = []
        for y, x in np.ndindex(self._distances.shape):
            if self._distances[y][x] == -1:
                for direction in global_variables.moving_directions:
                    cell_coord = np.array([y + direction[0], x + direction[1]])
                    # Make sure in range
                    if GridMap.coord_inside_matrix(cell_coord, self._distances.shape):
                        # Adherence cell is walkable
                        if self._distances[cell_coord[0], cell_coord[1]] > 0:
                            possible_points.append(cell_coord)

        # remove duplicates
        possible_points.sort(key=lambda x:(x[0], x[1]))
        i = 0
        while i < len(possible_points)-1:
            a = possible_points[i]
            b = possible_points[i+1]
            if a[0] == b[0] and a[1] == b[1]:
                del possible_points[i]
            else:
                i += 1
        for point in possible_points:
            unknown_counts.append(float(self._get_unknown_amount(point)))

        # calculate path length between current position and potential exploration points and choose the one with shortest path
        shortest_path = 1000000
        best_point = None
        best_score = -1
        print ("points:" + str(possible_points))
        for i in range(len(possible_points)):
            #print(point)
            #print(path)
            length = self._distances[possible_points[i][0],possible_points[i][1]]
            if length == 0:
                lenght = 100
            new_score = unknown_counts[i]/(length+10)
            print (length)
            update_new_highscore = False
            if new_score > best_score:
                update_new_highscore = True
            elif new_score == best_score and length <= shortest_path: # we prefer a shorter path
                if length == shortest_path:
                    if bool(random.getrandbits(1)):
                        update_new_highscore = True
                else:
                    update_new_highscore = True
            if update_new_highscore:
                best_point = possible_points[i]
                shortest_path = length
                best_score = new_score
            
        best_path = self.path_planner.astar(
            maze=self._representation,
            origin=self._origin,
            start=np.array([self._from_relative_to_matrix(self._agent_position)]),
            end=np.array([best_point]))
        # TODO somewhere if best_score = 0 always we should set the exploring sensor to 0?
        return best_point, best_path, best_score


    ### static methods ###

    @staticmethod
    def add_coord(coord_a, coord_b, operation='add'):
        """Add x and y values of two coordinates in tuple form

        Args:
            coord_a (tuple): coordinate in tuple form. first value is x, second value is y
            coord_b (tuple): coordinate in tuple form. first value is x, second value is y
            operation (str): 'add' for addition / 'sub' for subtraction
        Returns:
            tuple: resulting coord

        """
        assert isinstance(coord_a, tuple) and len(coord_a) == 2, 'Coordinate needs to be a tuple of length 2'
        assert isinstance(coord_b, tuple) and len(coord_b) == 2, 'Coordinate needs to be a tuple of length 2'
        assert operation in ['add', 'sub'], 'The operation needs to be "add" or "sub"'

        if operation == 'add':
            return coord_a[0] + coord_b[0], coord_a[1] + coord_b[1]
        elif operation == 'sub':
            return coord_a[0] - coord_b[0], coord_a[1] + coord_b[1]

    @staticmethod
    def manhattan_distance(coord1, coord2):
        """Calculate the manhattan distance between two coordinates in tuple form

        Args:
            coord1 (tuple): first coordinate
            coord2 (tuple): second coordinate

        Returns:
            int: euclidean distance

        """
        return abs((coord2[0] - coord1[0])) + abs((coord2[1] - coord1[1]))

    @staticmethod
    def coord_inside_matrix(coord, shape):
        if coord[0] >= 0 and coord[0] < shape[0] \
                and coord[1] >= 0 and coord[1] < shape[1]:
            return True
        return False


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