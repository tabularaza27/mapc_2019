import numpy as np
from collections import deque
import random
import copy


import os

from helpers import get_data_location
from map_live_plotting import cleanup
from grid_path_planner import GridPathPlanner
from block import Block

import global_variables
import rospy  # for debug logs

import itertools


# ToDo: Implement blocks
# ToDo: Keep track of blocks / entities when they are moving
# ToDo: Identify if other entity is enemy or ally


class GridMap():
    """
    Class that represent the local map of an agent like in the scenario.

    The map is a two dimensional np.array, initialized with size (11,11), that is the vision of the agent.
    If the agent moves in a direction, where is vision exceeds the border of the map, a new row / column is added to the np.array
    Note, that the first index of the np.array is for the y value and second is for the x value

    MAP SPECIFICATION
    Matrix of integers. Each value has the following meaning:

    author: Alessandro
    """
    # counter variable
    STEP = 0

    def __init__(self, agent_name, agent_vision):
        """initialization of the map. The agent is at the center of an unknown map"""
        self.agent_name = agent_name
        self.data_directory = self._get_data_directory()

        # fixed for now, but could be made dynamic later
        self.agent_vision = agent_vision

        # map
        self._representation = np.full((11, 11), -1)  # init the map with unknown cells
        self.origin = (self.agent_vision, self.agent_vision)  # the origin of the agent is at the center of the map
        self._path_planner_representation = np.copy(self._representation)  # map with fixed and temporary stuffs

        # info about agent in map
        self._agent_position = np.array([0, 0])
        # IS IT USEFUL TO DRAW THE AGENT IN THE MAP?
        # agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        # self._representation[agent_in_matrix[1]][agent_in_matrix[0]] = -4
        self._distances = np.array([])  # the matrix of all the distances from the agent
        self.is_at_goal_area = False
        self.failed_last_move = False
        # objects in map
        self._dispensers = []
        self._goal_areas = []
        self._agents = []
        self._temporary_obstacles = []

        # the list of attached blocks
        # attached blocks are objects of class Block
        self._attached_blocks = []

        # goal area discovery
        self.goal_area_fully_discovered = False
        self.goal_top_left = None
        self._start_discovering_goal_area = False

        # path_planner
        self.path_planner = GridPathPlanner()
        self.paths = {}

        # agents for connect
        # self.first_agent = None
        # self.second_agent = None
        self.nearby_agents = []

        #### DEBUG ####
        if global_variables.DEBUG_MODE:
            self.PLOT_MAP = True

        # delete old live plotting files
        cleanup()

        # create plot every x steps
        self.PLOT_FREQUENCY = 1
        self.live_plotting = global_variables.LIVE_PLOTTING

    ### PUBLIC METHODS ###
    def update_map(self, perception):
        """Update the map according to the movement of the agent and the new perception.
        Args:
            perception (rhbp.perception_provider): the new perception of the agent
        """
        # TODO CHECK IF THIS IS WORKING, create tests for rotate function
        if perception.agent.last_action == "rotate" and perception.agent.last_action_result == "success":
            for block in self._attached_blocks:
                block.rotate(rotate_direction=perception.agent.last_action_params[0])

        # if last action was `move` update agent position and expand map size if sight is out of bounds
        if perception.agent.last_action == "move" and perception.agent.last_action_result == "success":
            self._update_agent_position(move=perception.agent.last_action_params[0])
        elif perception.agent.last_action == "move" and perception.agent.last_action_result != "success":
            self.failed_last_move = True
            # TODO randomly change moving direction if continue to fail moving

        # if last action was `attach` update the attached_blocks list and update
        if perception.agent.last_action == "attach" and perception.agent.last_action_result == "success":
            # TODO CAN'T JUST CHECK THE ACTIVE SUBTASK?
            attach_direction = perception.agent.last_action_params[0]
            relative_block_position = global_variables.MOVEMENTS[attach_direction]
            # find out type of block that was attached
            block_type = None
            for block in perception.blocks:
                if block.pos.x == relative_block_position[1] and block.pos.y == relative_block_position[0]:
                    block_type = block.type

            attached_block = Block(block_type=block_type, position=relative_block_position)
            self._attached_blocks.append(attached_block)
            rospy.loginfo('{} attached to block of type {} in direction {}'.format(self.agent_name, block_type, attach_direction))

        agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        self._representation[agent_in_matrix[0], agent_in_matrix[1]] = global_variables.AGENT_CELL
        # update empty cells (all cells that are in vision range, get overwritten below if they are occupied)
        for j in range(-self.agent_vision, self.agent_vision + 1):
            for i in range(-self.agent_vision, self.agent_vision + 1):
                cell = agent_in_matrix + np.array([j, i])
                if 0 < GridMap.manhattan_distance(agent_in_matrix, cell) <= self.agent_vision:
                    self._representation[cell[0]][cell[1]] = 0

        # update obstacles
        for obs in perception.obstacles:
            pos = np.array([obs.pos.y, obs.pos.x]) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value

            self._representation[matrix_pos[0]][matrix_pos[1]] = global_variables.WALL_CELL

        # update goal cells
        self.is_at_goal_area = False
        for goal in perception.goals:
            # save if the agent is in the goal area
            if goal.pos.x == 0 and goal.pos.y == 0:
                self.is_at_goal_area = True
            # to activate the goal area discovering if the agent spawn in the middle of it
            if self.STEP == 0:
                self._start_discovering_goal_area = True
            # add to local map
            pos = np.array([goal.pos.y, goal.pos.x]) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value
            self._representation[matrix_pos[0]][matrix_pos[1]] = global_variables.GOAL_CELL

            # add to goals variable
            # if not self._goal_areas.__contains__(pos):
            # self._goal_areas.append(pos)

        # update dispensers
        for dispenser in perception.dispensers:
            pos = np.array([dispenser.pos.y, dispenser.pos.x]) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # get dispenser type
            for i in range(9):
                if str(i) in dispenser.type:
                    self._representation[matrix_pos[0]][matrix_pos[1]] = global_variables.DISPENSER_STARTING_NUMBER + i
        self.update_dispsenser_list()




        # write data to file, used for live plotting plotting
        if self.live_plotting and self.STEP % self.PLOT_FREQUENCY == 0:
            self._write_data_to_file()

        self.STEP += 1

    def update_dispsenser_list(self):
        """update the dispensers in the dispenser_list"""
        new_list = []
        for y in range(self._representation.shape[0]):
            for x in range(self._representation.shape[1]):
                pos_matrix = np.array([y,x])
                cell_value = self._get_value_of_cell(pos_matrix)
                dispenser_type = self.get_dispenser_type(cell_value=cell_value)
                if dispenser_type:
                    pos_relative = self._from_matrix_to_relative(pos_matrix)
                    new_list.append({'pos': pos_relative, 'type': dispenser_type})
        self._dispensers = new_list

    def _update_path_planner_representation(self, perception):
        # Update temporary map used by path_planner to avoid obstacles
        self._path_planner_representation = np.copy(self._representation)
        # add agent position
        matrix_pos = self._from_relative_to_matrix(self._agent_position)
        self._path_planner_representation[matrix_pos[0]][matrix_pos[1]] = global_variables.AGENT_CELL

        # update blocks
        # todo take blocks attached to the agent into account --> alvaro
        for block in perception.blocks:
            block_rel_to_agent = np.array([block.pos.y, block.pos.x])
            attached = False
            for attached_block in self._attached_blocks:
                if np.array_equal(attached_block._position, block_rel_to_agent):
                    attached = True
            if not attached:
                pos = self._from_relative_to_matrix(block_rel_to_agent, self._agent_position)

                matrix_pos = self._from_relative_to_matrix(pos)
                # first index --> y value, second  --> x value
                self._path_planner_representation[matrix_pos[0]][
                    matrix_pos[1]] = global_variables.BLOCK_CELL_STARTING_NUMBER + int(block.type[1])

        # TO SOLVE BUG WHEN AGENT THINKS TO HAVE BLOCKS ATTACHED THAT DO NOT EXIST
        for attached_block in self._attached_blocks[:]:
            block_exist = False
            for block in perception.blocks:
                block_rel_to_agent = np.array([block.pos.y, block.pos.x])
                if np.array_equal(attached_block._position, block_rel_to_agent):
                    block_exist = True
            if not block_exist:
                self._attached_blocks.remove(attached_block)

        # updates entities
        for entity in perception.entities:
            # It detects itself as an entity
            if entity.pos.y == 0 and entity.pos.x == 0:
                continue

            pos = np.array([entity.pos.y, entity.pos.x]) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value

            self._path_planner_representation[matrix_pos[0]][
                matrix_pos[1]] = global_variables.ENTITY_CELL

            # rospy.logdebug("temporary map: " + str(self._path_planner_representation))

    def _update_distances(self):
        """update the matrix of distances from the agent"""
        dist_shape = self._representation.shape
        self._distances = np.full((dist_shape[0], dist_shape[1]), -1, dtype=int)
        # print(self._distances.shape)
        # print(self._origin)
        # print(self._agent_position)
        agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        queue = deque([(agent_in_matrix, 0)])
        while len(queue) > 0:
            pos, dist = queue.popleft()
            if self._distances[pos[0], pos[1]] == -1:  # to avoid infinite loop
                self._distances[pos[0], pos[1]] = dist
                for direction in global_variables.MOVING_DIRECTIONS:  # ADD ALSO ROTATIONS?
                    new_pos = direction + pos
                    if GridMap.coord_inside_matrix(new_pos, dist_shape):
                        if self._get_value_of_cell(new_pos,self._distances) == global_variables.UNKNOWN_CELL:
                            cell_value = self._get_value_of_cell(new_pos, self._path_planner_representation)
                            if GridPathPlanner.is_walkable(cell_value):
                                queue.append((new_pos, dist + 1))

    def get_move_direction(self, path_id, path_creation_function, parameters=None):
        """get n,s,e,w to move the agent along the path.
        If the path is ended, or invalid, it generate a new path using path_creation_function
        Args:
            path_id: the id of the path the agent wants to move along to
            path_creation_function: the function that generate the path if the direction is invalid
            parameters: additional parameters needed by the path_creation_function

        Returns(tuple): path_id, direction
            path_id(int): the id of the path used to move
            direction(str): n,s,e or w, None if not possible
        """
        if not self.paths.has_key(path_id):
            # TODO do we need only the path?
            best_path = path_creation_function(parameters)
            path_id = self._save_path(best_path)
        else:
            best_path = self.paths[path_id]

        # Check if the map has been fully discovered
        valid_direction = False
        # Number of times you may try find a valid direction
        try_counter = 0
        max_trials = 10
        # Compute next direction if map is not fully discovered (path = -1)
        if best_path != -1 and best_path is not None and best_path != 'invalid end':
            while not valid_direction and best_path is not None and try_counter < max_trials:
                # this point is never reached
                if best_path == 'invalid end':
                    print ("invalid end")
                # increase counter
                try_counter += 1
                configuration_free = False
                # Compute direction
                direction = self.path_planner.next_move_direction(
                    self.get_agent_pos_and_blocks_array(),
                    self.paths[path_id])
                if direction is not None:
                    # Calculate next cell value if position  of the agent is not unknown
                    if direction != 'unknown position':
                        next_configuration = None
                        if direction == 'cw' or direction == 'ccw':
                            # TODO CHECK IF THE NEXT ROTATION IS POSSIBLE
                            # copy of the blocks
                            temp_blocks = copy.deepcopy(self._attached_blocks)
                            for block in temp_blocks:
                                block.rotate(rotate_direction=direction)
                            next_configuration = self.get_agent_pos_and_blocks_array(attached_blocks=temp_blocks)
                        else:
                            temp_agent_pos = self._agent_position + global_variables.MOVEMENTS[direction]
                            next_configuration = self.get_agent_pos_and_blocks_array(agent_position=temp_agent_pos)
                        if self.is_configuration_free(next_configuration):
                            configuration_free = True
                        else:  # out of bounds
                            configuration_free = False
                        # TODO CHECK UNKNOWN POSITION IN THIS CASE
                        # Check if the next_cell is still inside the path (unknown position error)
                        # next_cell_rel = self._from_matrix_to_relative(next_cell_matrix)
                        if not (next_configuration == best_path).any():
                            configuration_free = False
                    else:
                        # if position is unknown we want to recalculate path
                        configuration_free = False
                    # Check if agent has reached the end or the next cell is blocked
                    if direction == 'end' or not configuration_free:
                        self._remove_path(path_id)
                        # Recalculate path
                        best_path = path_creation_function(parameters)
                        path_id = self._save_path(best_path)
                        # TODO this function is now unreadable, we need to refactor it and create a logic
                    else:
                        # direction is valid
                        valid_direction = True
            # Set direction to none if run out of tries
            if not try_counter < max_trials:
                direction = None
        # TODO DELETE THIS WHEN WE ADD THE SENSOR FOR MAP EXPLORATION COMPLETED
        # Map discovered
        else:
            direction = None
            path_id = None
            print (best_path)
            rospy.loginfo(str(self.agent_name) + ":   MAP DISCOVERED COMPLETED! or... wait, I am lost :(")

        return path_id, direction

    def get_exploration_move(self, path_id):
        """get the move direction for the exploration behaviour
        Args:
            path_id: the path_id saved for exploration

        Returns: n,s,e or w or None
        """
        return self.get_move_direction(path_id, self._get_path_to_explore)

    def get_go_to_dispenser_move(self, subtask):
        """get the move direction for the go_to_dispenser behaviour
        Args:
            subtask(): the subtask needed to recompute the path to the closest dispenser if needed

        Returns:n,s,e or w or None
        """
        parameters = dict()
        parameters["dispenser_pos"] = self._from_relative_to_matrix(subtask.closest_dispenser_position, self.goal_top_left)
        return self.get_move_direction(subtask.path_to_dispenser_id, self._get_path_to_reach_dispenser, parameters)

    def get_go_to_goal_area_move(self, path_id):
        """get the move direction for the reach_goal_area behaviour"""
        return self.get_move_direction(path_id, self._get_path_to_reach_goal_area)

    def get_meeting_point_move(self, subtask, meeting_position):
        """get the move direction for the go_to_meeting_point behaviour
        Args:
            subtask(): the subtask needed to recompute the path to the closest dispenser if needed

        Returns:n,s,e or w or None
        """
        parameters = dict()
        # TODO change it back
        #parameters["final_pos"] = meeting_position
        parameters["final_pos"] = subtask.meeting_point
        return self.get_move_direction(subtask.path_to_meeting_point_id, self._get_path_to_meeting_point, parameters)

    def get_direction_to_close_dispenser(self, dispenser_type):
        """check if the agent is one step away from a dispenser of a certain type
        Args:
            dispenser_type(str): the type of the dispenser

        Returns: The direction of the dispenser that is 1 step away, False otherwise

        """
        for direction in global_variables.MOVEMENTS:
            if direction != 'end':
                close_cell_matrix = self._from_relative_to_matrix(
                    self._agent_position + global_variables.MOVEMENTS[direction]
                )
                dispenser_value = GridMap.get_dispenser_type(
                    self._get_value_of_cell(close_cell_matrix)
                )
                if dispenser_value == dispenser_type:
                    return direction
        return False

    def get_direction_to_close_block(self, block_type):
        """check if agent is one step away from a block of a certain type

        Args:
            block_type(str): the type of the block

        Returns:
            str: The direction of the block that is 1 step away, False otherwise
        """
        for direction in global_variables.MOVEMENTS:
            close_cell_matrix = self._from_relative_to_matrix(
                self._agent_position + global_variables.MOVEMENTS[direction]
            )
            block_value = GridMap.get_block_type(
                self._get_value_of_cell(close_cell_matrix, maze=self._path_planner_representation)
            )
            if block_value == block_type:
                return direction

        return False

    # def is_at_point(self, position):
    #     """ Check if agent is arrived in the point with coordinates=relative_coord"""
    #     if np.array_equal(self.get_agent_pos_and_blocks_array(), position):
    #         return True
    #     else:
    #         return False

    # TODO this shit is to avoid an error with different dimensionality between blocks_attached and meeting_point
    def is_at_point(self, position):
        """ Check if agent is arrived in the point with coordinates=relative_coord"""
        agent_and_blocks = self.get_agent_pos_and_blocks_array()
        meeting_point_elements = agent_and_blocks[0:2]
        if np.array_equal(meeting_point_elements, position):
            return True
        else:
            return False


    ### PRIVATE METHODS ###
    def _get_value_of_cell(self, coord, maze=None):
        """get the value of the cell in a map=matrix
        Args:
            coord(np.array): the matrix coordinates
            maze: the maze to use. It is set to the representation by default

        Returns:

        """
        if maze is None:
            maze = self._representation
        return maze[coord[0], coord[1]]

    def _remove_path(self, path_id):
        """ Remove a path from the dictionary of paths"""
        if self.paths.has_key(path_id):
            del self.paths[path_id]

    def _save_path(self, path, path_id=-1):
        """ Save the path into the dictionary of paths and return the id"""
        if path_id == -1:
            path_id = random.randint(1, 9999999)
            while self.paths.has_key(path_id):
                path_id = random.randint(1, 9999999)
        self.paths[path_id] = path
        return path_id

    def _from_relative_to_matrix(self, relative_coord, coord=None):
        """translates the coordinate with respect to the origin of the map to the
        origin of the matrix

        Args:
            relative_coord: (x,y) with respect to the origin of the map
            coord: the relative point of the coordinates

        Returns:
            matrix_coord: (x',y') with respect to the origin of the matrix
        """

        matrix_coord = np.copy(relative_coord)

        if coord is None:  # by default origin
            matrix_coord = matrix_coord + self.origin
        else:
            matrix_coord = matrix_coord + coord

        return matrix_coord

    def _from_matrix_to_relative(self, matrix_coord, coord=None):
        """Reverse function of from_relative_to_matrix

        Args:
            matrix_coord: (x,y) with respect to the origin of the matrix
            coord: the relative point of the coordinates
        Returns:
            relative_coord: (x',y') with respect to the origin of the map
        """

        relative_coord = np.copy(matrix_coord)
        if coord is None:
            relative_coord = relative_coord - self.origin
        else:
            relative_coord = relative_coord - coord

        return relative_coord

    def list_from_relative_to_matrix(self, relative_coord_list):
        new_list = []
        for coord in relative_coord_list:
            new_list.append(self._from_relative_to_matrix(coord))
        return np.array(new_list)

    def list_from_matrix_to_relative(self, matrix_coord_list):
        new_list = []
        for coord in matrix_coord_list:
            new_list.append(self._from_matrix_to_relative(coord))
        return np.array(new_list)

    def _update_agent_position(self, move=None):
        """update agents position in map and expand grid if sight is out of bounds

        Args:
            move (str): n,s,w or e as strings

        Returns:

        """
        assert move in [None, 'n', 'e', 's',
                        'w'], "Move direction needs to be 'n','e','s','w'. '{}' was provided".format(move)

        agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
        if move is not None:
            # Delete previous position of agent in map
            self._representation[agent_in_matrix[0]][agent_in_matrix[1]] = 0
            move_array = global_variables.MOVEMENTS[move]
            self._agent_position = self._agent_position + move_array
            agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
            if (agent_in_matrix <= self.agent_vision).any() \
                    or agent_in_matrix[1] + self.agent_vision >= self._representation.shape[1] \
                    or agent_in_matrix[0] + self.agent_vision >= self._representation.shape[0]:
                self._expand_map(move)

            agent_in_matrix = self._from_relative_to_matrix(self._agent_position)
            self._representation[agent_in_matrix[0]][agent_in_matrix[1]] = global_variables.AGENT_CELL

        else:
            pass

    def _expand_map(self, direction):
        """Expands map in given direction with vector of unknowns

        Adds rows / columns to numpy array

        Args:
            direction (str): {'n', 'e', 's', 'w'} direction in which the map should be expanded
        """
        assert direction in ['n', 'e', 's',
                             'w'], "Expansion direction needs to be 'n','e','s','w'. '{}' was provided".format(
            direction)

        old_map_shape = self._representation.shape

        rospy.loginfo('old origin:{}'.format(self.origin))
        # this is the fastest way to add a row / column to a numpy array
        # see 'https://stackoverflow.com/questions/8486294/how-to-add-an-extra-column-to-a-numpy-array' if interested
        if direction == 'n':
            helper_map = np.full((old_map_shape[0] + 1, old_map_shape[1]), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[1:, :] = self._representation
            self.origin = self.origin + np.array([1, 0])
        if direction == 's':
            helper_map = np.full((old_map_shape[0] + 1, old_map_shape[1]), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[:-1, :] = self._representation
        if direction == 'e':
            helper_map = np.full((old_map_shape[0], old_map_shape[1] + 1), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[:, :-1] = self._representation
        if direction == 'w':
            helper_map = np.full((old_map_shape[0], old_map_shape[1] + 1), fill_value=global_variables.UNKNOWN_CELL)
            helper_map[:, 1:] = self._representation
            self.origin = self.origin + np.array([0, 1])
        rospy.loginfo('new origin:{}'.format(self.origin))
        self._representation = helper_map

    def _get_data_directory(self):
        """Returns Directory where map data is stored for plotting purposes"""
        data_path = get_data_location()
        return os.path.join(data_path, 'generatedMaps', 'tmp_maps')

    def _write_data_to_file(self):
        """writes two dimensional np.array to .txt file named after agent and in data directory"""
        map_copy = np.copy(self._representation)
        # TODO PRINT LIST OF DISPENSERS
        map_copy[self.origin[0],self.origin[1]] = 0
        for d in self._dispensers:
            pos = self._from_relative_to_matrix(d['pos'])
            map_copy[pos[0], pos[1]] = 10
        np.savetxt(os.path.join(self.data_directory, '{}.txt'.format(self.agent_name)), map_copy, fmt='%i',
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
        for j in range(-self.agent_vision, self.agent_vision + 1):
            for i in range(-self.agent_vision, self.agent_vision + 1):
                cell = position + np.array([j, i])
                if 0 < GridMap.manhattan_distance(position, cell) <= self.agent_vision:
                    update_counter = False
                    if not GridMap.coord_inside_matrix(cell, self._representation.shape):
                        unknown_count += 1
                    else:
                        if self._representation[cell[0], cell[1]] == global_variables.UNKNOWN_CELL:
                            unknown_count += 1

        return unknown_count

    def _set_goal_top_left(self):
        """Set the goal_top_left variable with the relative coordinates of the top left corner of the goal area.
        With this we can merge maps using this fixed common point.

        Returns: void

        """
        top_discovered = False
        start_area = False
        top = 10000
        left = 10000
        for i in range(self._representation.shape[0]):
            goal_present_in_row = False
            for j in range(self._representation.shape[1]):
                value = self._get_value_of_cell(np.array([i, j]))
                if value == global_variables.GOAL_CELL:
                    start_area = True
                    goal_present_in_row = True
                    if i < top:
                        top = i
                    if j < left:
                        left = j
                    break
            if start_area and not goal_present_in_row:
                break

        self.goal_top_left = self._from_matrix_to_relative(np.array([top, left]))


    def get_distance_and_path(self, a, b, return_path=False):
        """returns the distance and path from a point to another

        Args:
            a: first point in relative coordinates
            b: second point in relative coordinates
            return_path(bool): if the path is needed

        Returns:
            tuple: (distance (int), path (list))
        """
        b_matrix_representation = self._from_relative_to_matrix(b)
        a_matrix_representation = self._from_relative_to_matrix(a)
        dist = -1

        if self._distances[a_matrix_representation[0], a_matrix_representation[1]] == 0:
            dist = self._distances[b_matrix_representation[0], b_matrix_representation[1]]
        elif self._distances[b_matrix_representation[0], b_matrix_representation[1]]:
            dist = self._distances[a_matrix_representation[0], a_matrix_representation[1]]

        if dist != -1 or return_path:
            path = self.path_planner.astar(
                maze=self._path_planner_representation,
                origin=self.origin,
                start=np.array([a_matrix_representation]),
                end=np.array([b_matrix_representation]))
            if path is not None:
                dist = len(path)
                if not return_path:
                    path = None
            return dist, path
        else:
            return dist, None

    def _get_path_to_explore(self, params=None):
        """Calculates point that is most suited for exploring and path to it

        Args:
            map_representation (np.array):
            current_position (tuple): position of the agent

        Returns:
            tuple: tuple containing best_point (tuple), best_path list(tuples), amount of unkown cells to be explored when this point is reached by the agent (int)
            int: -1 if map is fully discovered
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
                for direction in global_variables.MOVING_DIRECTIONS:
                    cell_coord = np.array([y + direction[0], x + direction[1]])
                    # Make sure in range
                    if GridMap.coord_inside_matrix(cell_coord, self._distances.shape):
                        # Adherence cell is walkable
                        if self._distances[cell_coord[0], cell_coord[1]] > 0:
                            possible_points.append(cell_coord)

        # remove duplicates
        possible_points.sort(key=lambda x: (x[0], x[1]))
        i = 0
        while i < len(possible_points) - 1:
            a = possible_points[i]
            b = possible_points[i + 1]
            if a[0] == b[0] and a[1] == b[1]:
                del possible_points[i]
            else:
                i += 1
        goal_area_in_border = False
        interesting_points = []
        for point in possible_points:
            unknown_count = float(self._get_unknown_amount(point))
            if self._get_value_of_cell(point) == global_variables.GOAL_CELL:
                unknown_count *= 10000
                goal_area_in_border = True
            if unknown_count > 0:
                interesting_points.append(point)
                unknown_counts.append(unknown_count)

        # DISCOVERING GOAL AREA
        # TODO IF THE AGENT IS BORN IN THE GOAL AREA THIS IS NOT WORKING
        if not self.goal_area_fully_discovered:
            if goal_area_in_border:
                self._start_discovering_goal_area = True
            if self._start_discovering_goal_area and not goal_area_in_border:
                self.goal_area_fully_discovered = True
                self._set_goal_top_left()
        # calculate path length between current position and potential exploration points and choose the one with shortest path
        shortest_path = 1000000
        best_point = None
        best_score = -1
        #print ("points:" + str(possible_points))
        for i in range(len(interesting_points)):
            #print(point)
            #print(path)
            length = self._distances[interesting_points[i][0],interesting_points[i][1]]
            if length == 0: # to avoid division per zero
                lenght = 100
            new_score = unknown_counts[i]/length
            #print (length)
            update_new_highscore = False
            if new_score > best_score:
                update_new_highscore = True
            elif new_score == best_score and length <= shortest_path:  # we prefer a shorter path
                if length == shortest_path:
                    if bool(random.getrandbits(1)):
                        update_new_highscore = True
                else:
                    update_new_highscore = True
            if update_new_highscore:
                best_point = interesting_points[i]
                shortest_path = length
                best_score = new_score

        # Avoid stuck behavior
        if best_point is not None:
            best_path = self.path_planner.astar(
                maze=self._path_planner_representation,
                origin=self.origin,
                start=np.array([self._from_relative_to_matrix(self._agent_position)]),
                end=np.array([best_point]))
            # TODO somewhere if best_score = 0 always we should set the exploring sensor to 0?
        else:
            # TODO this will be used as a flag to avoid the agent to get stuck when there is no best point (map has been fully discovered)
            # Map fully discovered
            best_path = -1

        return best_path

    def get_common_meeting_point(self, task):
        """
        Compute a common meeting point for performing a connection between several agents.

        Args:
            task (key): task assigned to the agent
        Returns:
            common_meeting_point (np.array): common meeting point (in relative coordinates to the top left corner of
                the goal area.
        """

        lowest_dist_disp = 10000
        lowest_dist_goal = 10000
        dispenser_position = []
        dist_to_dispenser = []
        assigned_agents = []
        possible_meeting_points = []

        for sub in task.sub_tasks:
            if True:    # Right now we don't change common meeting point
            #if sub.complete is not True:
                # transform from relative to top_left to matrix
                dispenser_rel = self._from_relative_to_matrix(sub.closest_dispenser_position, self.goal_top_left)
                # transform from relative to matrix
                dispenser_position.append(self._from_relative_to_matrix(dispenser_rel))
                # distance to dispensers
                dist_to_dispenser.append(sub.distance_to_dispenser)
                # name of agents assigned
                assigned_agents.append(sub.assigned_agent)

        # # Check if dispensers are actually in its position
        # for dispenser in dispenser_position:
        #     cell_value = self._get_value_of_cell(dispenser)
        #     if not cell_value >= global_variables.DISPENSER_STARTING_NUMBER \
        #             and cell_value < global_variables.BLOCK_CELL_STARTING_NUMBER:
        #         return None

        # calculate distance matrix for each dispenser
        # TODO dont calculate twice distance d1-d2 and d2-d1 (how?)
        for i, disp_i in enumerate(dispenser_position):
            dist_matrix = self.distance_matrix(disp_i)
            # get the distance from each pair of dispenser
            for j, disp_j in enumerate(dispenser_position):
                if i != j:
                    dist = self._get_value_of_cell(disp_j, dist_matrix) + dist_to_dispenser[i]
                    if dist < lowest_dist_disp:
                        lowest_dist_disp = dist  # lowest distance
                        first_agent = i    # first couple of closest agents
                        second_agent = j    # second couple of closest agents
                        lowest_dist_matrix = dist_matrix    # distance matrix from 1st to 2nd dispenser

        # Meeting point distance = (distance between dispenser + distance agent2 to dispenser)/2 - distance agent1 to dispenser
        middle_point_distance = int((lowest_dist_disp + dist_to_dispenser[second_agent])/2 - \
                                    dist_to_dispenser[first_agent])

        # Take into account only points between dispensers
        first_dispenser = dispenser_position[first_agent]
        second_dispenser = dispenser_position[second_agent]
        # Get row bounds
        if first_dispenser[0] <= second_dispenser[0]:
            upper_row_limit = first_dispenser[0]
            lower_row_limit = second_dispenser[0]
        else:
            upper_row_limit = second_dispenser[0]
            lower_row_limit = first_dispenser[0]
        # Get column bounds
        if first_dispenser[1] <= second_dispenser[1]:
            upper_col_limit = first_dispenser[1]
            lower_col_limit = second_dispenser[1]
        else:
            upper_col_limit = second_dispenser[1]
            lower_col_limit = first_dispenser[1]

        # Check if the distance between dispensers is enough to fit the meeting point
        # E.g when two agents dispense from the same dispenser
        goal_top_left_matrix = self._from_relative_to_matrix(self.goal_top_left)
        # maximum distance between dispensers
        max_distance = lower_row_limit - upper_row_limit + lower_col_limit - upper_col_limit
        if max_distance < middle_point_distance:    # common meeting point out of bounds
            # Establish new upper and lower limits
            added_row_and_col = middle_point_distance - max_distance
            upper_row_limit -= added_row_and_col
            lower_row_limit += added_row_and_col
            upper_col_limit -= added_row_and_col
            lower_col_limit += added_row_and_col

        # Get points as the same distance as middle_point_distance
        for row_index, row in enumerate(lowest_dist_matrix):
            # only points inside the row bounds
            if lower_row_limit >= row_index >= upper_row_limit:
                # only points inside the column bounds
                for col_index, cell_value in enumerate(row):
                    if lower_col_limit >= col_index >= upper_col_limit:
                        if cell_value == middle_point_distance:
                            possible_meeting_points.append([row_index, col_index])

        # Get the point with the lowest distance to the goal top left corner
        if possible_meeting_points is not None:
            for point in possible_meeting_points:
                dist_to_goal = abs(point[0] - goal_top_left_matrix[0]) + abs(point[1] - goal_top_left_matrix[1])
                if dist_to_goal < lowest_dist_goal:
                    lowest_dist_goal = dist_to_goal
                    common_meeting_point = point
        else:
            common_meeting_point = None

        #return closest_agent_1, closest_agent_2, common_meeting_point

        #return common_meeting_point
        return common_meeting_point

    def meeting_position(self, task, common_meeting_point):
        """ #TODO change description
        It returns a position for the agent and blocks attached around a common meeting point associated to a task

        Args:
            task (key): task assigned to the agent
            common_meeting_point (np.array): Common meeting point for several agents to connect

        Returns:
            meeting_position (np.array): Position of the agent and blocks attached around the meeting point or
                                            recomputed common meeting point
        """
        ordered_assigned_agents = []
        task_figure, submitting_agent_index = self.create_figure(task)
        multiple_agent_meeting_position = self.agent_position_in_figure(task_figure, \
                                                                        submitting_agent_index, common_meeting_point)

        # return position only for actual agent
        for sub_index, sub in enumerate(task.sub_tasks):
            if sub.assigned_agent == self.agent_name:
                # Agent and block position
                agent_index = 2*sub_index
                block_index = agent_index + 1
                single_agent_meeting_position = multiple_agent_meeting_position[agent_index:block_index + 1]
                # # Nearby agent names (to connect)
                # block_position = abs(sub.position[0]) + abs(sub.position[1])
                # for sub_task in task.sub_tasks:
                #     if abs(sub_task.position[0]) + abs(sub_task.position[1]) == block_position - 1:
                #         nearby_agents.append(sub_task.assigned_agent)
                #     elif abs(sub_task.position[0]) + abs(sub_task.position[1]) == block_position + 1:
                #         nearby_agents.append(sub_task.assigned_agent)


        # Transform to relative
        single_agent_meeting_position = self._from_matrix_to_relative(single_agent_meeting_position)

        # Save agents assigned in order from further to closer to the submitter
        for index, element in enumerate(task_figure):
            if index % 2 == 0:      # agent
                ordered_assigned_agents.append(element)

        return ordered_assigned_agents, single_agent_meeting_position

        #return single_agent_meeting_position

    def create_figure(self, task):
        """ Create a list of agents and relative positions of blocks (to the submitting agent) associated to a
        particular task

        Args:
            task: task assigned

        Returns:
        Returns:
            figure (np.array): [Agent_name + relative block position]
        """

        figure_length = 2*len(task.sub_tasks)
        figure = [None]*figure_length
        last_agent_index = - 1


        # # Figure is a list of pairs (agent_name,block position) from further to closer to the submitter agent
        # if task.sub_tasks is not None:      # Ensure there are subtasks
        #     for sub_index, sub in enumerate(task.sub_tasks):
        #         # Save agent name
        #         figure.append(sub.assigned_agent)
        #         # Save block position
        #         figure.append(sub.position)
        #         # Save submitting agent index
        #         if sub.submit_behaviour:    # True
        #             submitting_agent_index = sub_index
        # else:
        #     # No figure for task
        #     return None

        # Figure is a list of pairs (agent_name,block position) from further to closer to the submitter agent
        if task.sub_tasks is not None:  # Ensure there are subtasks
            for sub_index, sub in enumerate(task.sub_tasks):
                # Save submitting agent index
                if sub.submit_behaviour:  # True
                    submitting_agent_index = sub_index
                # Check block position distance to submitter
                agent_index = abs(sub.position[0]) + abs(sub.position[1]) - 1
                # Index indicates distance to submitter, if it is the same, save it on the next free spot in figure
                if last_agent_index == agent_index:
                    agent_index += 1
                # Calculate index for each agent and block.
                # Order between blocks and agent is swap so it is correct when we reversed at the end
                figure_block_index = 2*agent_index
                figure_agent_index = figure_block_index + 1

                # update the last block index
                last_agent_index = agent_index

                # Save agent name
                figure[figure_agent_index] = sub.assigned_agent
                # Save block position
                figure[figure_block_index] = sub.position

        else:
            # No figure for task
            return None

        # Swap the order so the figure order is from further to closer distance to the submitter
        figure = list(reversed(figure))

        return figure, submitting_agent_index

    def agent_position_in_figure(self, figure_rel, submitting_agent_index, common_meeting_point):

        free_figure_basic = figure_rel[:]
        mp_shift_times = 1
        mp_shift_values = np.array([[0, 0], [-1, 0], [0, 1], [1, 0], [0, -1]])  # same, up, right, down, left
        agent_shift_values = np.array([[-1, 0], [1, 0], [0, 1], [0, -1]])  # up, down, right, left

        # common meeting point already in matrix notation
        common_mp_matrix = common_meeting_point

        # check the numbers of agents involved in this task
        figure_size = len(free_figure_basic)
        number_of_agents_for_task = figure_size / 2

        # calculate cartesian product of all possible positions of agents around a block
        product_list = []
        all_possible_shifts = []

        if number_of_agents_for_task == 2:
            all_possible_shifts = agent_shift_values
        else:
            # create product list depending on the number of agents
            for unused in range(number_of_agents_for_task - 1):
                product_list.append(agent_shift_values)
            # calculate cartesian product
            for element in itertools.product(*product_list):
                all_possible_shifts.append(element)

        # Figure composition
        # Submitting agent and blocks have fixed position
        while True:
            # counter = 0
            mp_shift_amount = mp_shift_values * mp_shift_times  # if there is no position around meeting point, shift
            for mp_shift in mp_shift_amount:
                # Establish common meeting point value
                submitting_agent_meeting_position = common_mp_matrix + mp_shift
                # Check if the position of the submitting agent is reachable (inside the wall boundaries)
                path_to_submitting_agent_position = self.path_planner.astar(
                    # TODO could it be better to use path_planner_representation? possible errors?
                    maze=self._representation,
                    origin=self.origin,
                    start=np.array([self._from_relative_to_matrix(self._agent_position)]),
                    end=np.array([submitting_agent_meeting_position]))
                # TODO is there a function for invalid paths?
                if path_to_submitting_agent_position is None or path_to_submitting_agent_position == 'invalid end':
                    continue   # next shift for common meeting point
                # Submitting agent meeting position is fixed
                free_figure_basic[2*submitting_agent_index] = submitting_agent_meeting_position
                # Blocks are fixed
                for blocks_position in range(figure_size):
                    # Blocks are in even positions
                    if blocks_position % 2 != 0:
                        free_figure_basic[blocks_position] = submitting_agent_meeting_position + \
                                                             figure_rel[blocks_position]
                    elif blocks_position != 2 * submitting_agent_index:
                        free_figure_basic[blocks_position] = np.array([-1, -1])

                # Iterate through different agent position
                combinations = all_possible_shifts
                for shifts in combinations:
                    # Reset variables
                    free_figure_shifted = free_figure_basic[:]
                    invalid_figure_composition = False

                    # Apply shifts
                    block_counter = 0
                    for blocks_position in range(figure_size):
                        # Blocks are in even positions and submitting agent is fixed
                        if blocks_position % 2 != 0 and blocks_position != 2 * submitting_agent_index + 1:
                            # for more than 2 agents, shifts is a list of arrays (else is an array)
                            if number_of_agents_for_task == 2:
                                shift_value = shifts
                            else:
                                shift_value = shifts[block_counter]
                            block_counter += 1
                            # Possible agent position by applying shift to attached block position
                            possible_agent_position = free_figure_shifted[blocks_position] + shift_value
                            # Check duplicate position in figure composition
                            for index, position in enumerate(free_figure_shifted):
                                # duplicated
                                if index != blocks_position and (position == possible_agent_position).all():
                                    invalid_figure_composition = True
                                    break
                            # Composition not valid, try next shift combination
                            if invalid_figure_composition:
                                break
                            else:
                                # save agent position in figure composition
                                free_figure_shifted[blocks_position - 1] = possible_agent_position

                    # If valid figure composition, check if figure position is feasible in the map
                    if not invalid_figure_composition:
                        # TODO Refactor free_spot function
                        if self.free_spot_for_meeting(free_figure_shifted):
                            # Figure composition found
                            return free_figure_shifted

            # Increase the shifts values of common meeting point by 1
            mp_shift_times += 1

    #TODO this has to be rethink, right now it is the same as is_walkable. We need to find a way to track if a cell
    # is occupied by an ally and the block we expect or not
    def free_spot_for_meeting(self, composition, maze=None):
        """Right now this function is as is_walkable but ignore agents and blocks, so it is a
        temporal fix for a problem with the end position of figure composition

        Args:
            composition:
            maze:

        Returns:

        """

        if maze is None:
            map_representation = self._representation
        else:
            map_representation = maze

        for cell in composition:
            cell_value = self._get_value_of_cell(cell, map_representation)
            #if GridPathPlanner.is_walkable(cell_value):
            # We are avoiding meeting on top of dispensers due to it leads to possible wrong behaviors
            if cell_value in (global_variables.EMPTY_CELL, global_variables.GOAL_CELL, global_variables.AGENT_CELL):
                continue
            else:
                return False

        return True

    ### GO TO DISPENSER FUNCTIONS ###
    def _get_path_to_reach_dispenser(self, parameters):
        """ Get path from agent to the dispenser """
        # TODO add attached blocks to the agent position
        if 'dispenser_pos' not in parameters:
            return None
        else:
            dispenser_pos = parameters['dispenser_pos']
        agent_pos = [self._from_relative_to_matrix(self._agent_position)]
        for direction in global_variables.MOVING_DIRECTIONS:
            dispenser_pos_in_matrix = [self._from_relative_to_matrix(dispenser_pos+direction)]
            path = self.path_planner.astar(
                maze=self._path_planner_representation,
                origin=self.origin,
                start=np.array(agent_pos),
                end=np.array(dispenser_pos_in_matrix)
            )
            if GridPathPlanner.is_valid_path(path):
                return path
        # TODO IF PATH IS NOT VALID? CHANGE DISPENSER LOCATION?
        return None

    def get_closest_dispenser_position(self, required_type):
        """get the closest dispenser position (in relative coord) of the required_type
        Args:
            required_type(str): the type of the dispenser needed

        Returns(tuple): (position,distance), (None,9999) if didn't find a valid dispenser
        """
        min_dist = 9999
        pos = None
        for dispenser in self._dispensers:
            if dispenser['type'] == required_type:  # check if the type is the one we need
                pos_matrix = self._from_relative_to_matrix(dispenser['pos'])
                dist = self._distances[pos_matrix[0], pos_matrix[1]]

                if dist < min_dist:  # see if the distance is minimum and save it
                    min_dist = dist
                    pos = pos_matrix
        if pos is not None:
            return self._from_matrix_to_relative(pos), min_dist
        else:
            return None, min_dist

    def distance_matrix(self, start_point):
        """Calculate a distances matrix between a starting point and everywhere else

        Args:
            start_point (np.array): starting point (in the same coordinates as the dispenser position)

        Returns:
            distance_matrix (np.array): distances matrix from starting point
        """
        #dist_shape = self._path_planner_representation.shape
        dist_shape = self._representation.shape
        distances = np.full((dist_shape[0], dist_shape[1]), -1, dtype=int)
        #start_point_matrix = self._from_relative_to_matrix(start_point)
        # Dispenser are already in matrix notations
        start_point_matrix = start_point
        queue = deque([(start_point_matrix, 0)])
        while len(queue) > 0:
            pos, dist = queue.popleft()
            if distances[pos[0], pos[1]] == -1:  # to avoid infinite loop
                distances[pos[0], pos[1]] = dist
                for direction in global_variables.MOVING_DIRECTIONS:
                    new_pos = direction + pos
                    if self.coord_inside_matrix(new_pos, dist_shape):
                        if distances[new_pos[0], new_pos[1]] == -1:
                            #cell_value = self._path_planner_representation[new_pos[0], new_pos[1]]
                            cell_value = self._representation[new_pos[0], new_pos[1]]
                            if GridPathPlanner.is_walkable(cell_value):
                                queue.append((new_pos, dist + 1))
        return distances

    ### GO TO MEETING POINT FUNCTIONS ###

    def _get_path_to_meeting_point(self, parameters):
        """ Get path from agent to the dispenser

        Args:
            parameters:

        """
        if 'final_pos' not in parameters:
            return None
        else:
            final_pos = parameters['final_pos']
        agent_pos = self.get_agent_pos_and_blocks_array()
        agent_pos = self.list_from_relative_to_matrix(agent_pos)
        final_pos_in_matrix = self.list_from_relative_to_matrix(final_pos)
        # TODO THIS FINAL POS SHOULD ALREADY BE A LIST, CHANGE IT WHEN IT IS
        # TODO put in the list of the agent pos the attached blocks
        # if (final_pos_in_matrix[0] == agent_pos[0]).all():  # already in meeting point
        #     return None
        print (agent_pos)
        print (final_pos_in_matrix)
        path = self.path_planner.astar(
            maze=self._path_planner_representation,
            origin=self.origin,
            start=agent_pos,
            end=final_pos_in_matrix
        )

        return path

    def get_meeting_point(self, dispenser_distance, dispenser_name, agent_names):
        return

    def _get_path_to_reach_goal_area(self, parameters):
        """get path from agent to the goal area"""
        best_path = None
        agent_pos = self.get_agent_pos_and_blocks_array()
        agent_pos_matrix = self.list_from_relative_to_matrix(agent_pos)
        goal_area = np.array(self.goal_top_left) + np.array([1, 1])
        # TODO should check for all the goal area cells (starting from the closest)
        possible_ends = self.get_possible_configurations_in_point(goal_area)
        for end in possible_ends:
            goal_area_matrix = self.list_from_relative_to_matrix(end)
            path = self.path_planner.astar(
                maze=self._path_planner_representation,
                origin=self.origin,
                start=agent_pos_matrix,
                end=goal_area_matrix
            )
            if GridPathPlanner.is_valid_path(path):
                best_path = path
                break
        return best_path

    def get_possible_configurations_in_point(self, point):
        """get all the possible configuration of the agents and blocks attached in a point
            point(np.array): the point that the agent must reach
        """
        possible_configurations = []
        # copy the blocks_attached array
        blocks = copy.deepcopy(self._attached_blocks)

        # create a configuration and add it if it is free
        for i in range(4):
            configuration = self.get_agent_pos_and_blocks_array(agent_position=point, attached_blocks=blocks)
            if self.is_configuration_free(configuration):
                possible_configurations.append(configuration)
            for block in blocks: # the last rotation is not needed
                block.rotate(rotate_direction='cw')
        return possible_configurations

    def is_configuration_free(self, configuration):
        """check if a configuration of the agents and blocks attached in a point is a valid end point"""
        for coord in configuration:
            matrix_coord = self._from_relative_to_matrix(coord)
            if not self.coord_inside_matrix(matrix_coord, self._path_planner_representation.shape):
                return False
            if not GridPathPlanner.is_walkable(self._get_value_of_cell(matrix_coord,maze=self._path_planner_representation)):
                return False
        return True


    def get_agent_pos_and_blocks_array(self, agent_position=None, attached_blocks=None):
        """get agent with blocks array in relative coordinates"""
        if attached_blocks is None:
            attached_blocks = self._attached_blocks
        if agent_position is None:
            agent_position = self._agent_position
        list = [np.copy(agent_position)]
        for block in attached_blocks:
            # transform the coordinates of the block in coordinates relative to the agent
            block_in_relative = self._from_relative_to_matrix(block._position, agent_position)
            list.append(block_in_relative)

        return np.array(list)

    ### static methods ###

    @staticmethod
    def swap(arr):
        a = arr[0]
        b = arr[1]

        return [b,a]

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
        if 0 <= coord[0] < shape[0] \
                and 0 <= coord[1] < shape[1]:
            return True
        return False

    @staticmethod
    def get_dispenser_type(cell_value):
        """returns the type of the dispenser if the cell is a dispenser, False otherwise"""
        if global_variables.DISPENSER_STARTING_NUMBER <= cell_value < global_variables.BLOCK_CELL_STARTING_NUMBER:
            return "b{}".format(cell_value - global_variables.DISPENSER_STARTING_NUMBER)
        else:
            return False

    @staticmethod
    def get_block_type(cell_value):
        if cell_value >= global_variables.BLOCK_CELL_STARTING_NUMBER:
            return "b{}".format(cell_value - global_variables.BLOCK_CELL_STARTING_NUMBER)
        else:
            return False

def main():
    import time

    my_map = GridMap('Agent1', 5)
    my_map._representation = np.loadtxt(open("../../data/generatedMaps/00/partial.csv", "rb"), delimiter=",")
    my_map._update_distances()
    my_map.origin = np.array([4, 14], dtype=np.int)
    my_map._agent_position = np.array([0, 0], dtype=np.int)

    start_time = time.time()
    best_path = my_map._get_path_to_explore()
    print ("---%s seconds ---" % (time.time() - start_time))
    # print ("Best point:" + str(best_point))
    print ("Best path:" + str(best_path))
    # print ("Current high score:" + str(current_high_score))


if __name__ == '__main__':
    main()
