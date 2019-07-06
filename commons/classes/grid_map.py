import numpy as np
from collections import deque
import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import random
import os

from helpers import get_data_location
from map_live_plotting import cleanup
from grid_path_planner import GridPathPlanner

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

        # objects in map
        self._dispensers = []
        self._goal_areas = []
        self._agents = []
        self._temporary_obstacles = []

        # the list of attached blocks
        # {type:'b1', pos:[y,x], subtask_id:None}
        self._attached_blocks = []

        # goal area discovery
        self.goal_area_fully_discovered = False
        self.goal_top_left = None
        self._start_discovering_goal_area = False

        # path_planner
        self.path_planner = GridPathPlanner()
        self.paths = {}

        #### DEBUG ####
        if global_variables.DEBUG_MODE:
            self.PLOT_MAP = True

        # delete old live plotting files
        cleanup()

        # create plot every x steps
        self.PLOT_FREQUENCY = 1
        self.live_plotting = global_variables.LIVE_PLOTTING

    ### PUBLIC METHODS ###
    def update_map(self, agent, perception):
        """Update the map according to the movement of the agent and the new perception.
        Args:
            agent (dict): info about agent. dict is received from the percept
            perception (rhbp.perception_provider): the new perception of the agent
        """
        # TODO CHECK IF THIS IS WORKING, create tests for rotate function
        if agent.last_action == "rotate" and agent.last_action_result == "success":
            for block in self._attached_blocks:
                block.rotate(rotate_direction=agent.last_action_params[0])

        # if last action was move update agent position and expand map size if sight is out of bounds
        if agent.last_action == "move" and agent.last_action_result == "success":
            self._update_agent_position(move=agent.last_action_params[0])

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
        for goal in perception.goals:
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

        # TODO COPY IN MAZE_MAP AND PUT BLOCKS(NOT ATTACHED) AND ENTITIES

        # update dispensers
        for dispenser in perception.dispensers:
            pos = (dispenser.pos.y, dispenser.pos.x) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # get dispenser type
            for i in range(4):
                if str(i) in dispenser.type:
                    self._representation[matrix_pos[0]][matrix_pos[1]] = global_variables.DISPENSER_STARTING_NUMBER + i

            # add to state variable as dict
            # {pos: {x: 3, y: 3}, type: 'b1'}
            dispenser.pos = pos
            flag_equal = False
            for d in self._dispensers:
                if np.array_equal(d.pos, dispenser.pos):
                    flag_equal = True
            if not flag_equal:
                self._dispensers.append(dispenser)

        # Update temporary map used by path_planner to avoid obstacles
        self._path_planner_representation = np.copy(self._representation)
        # add agent position
        matrix_pos = self._from_relative_to_matrix(self._agent_position)
        self._path_planner_representation[matrix_pos[0]][matrix_pos[1]] = global_variables.AGENT_CELL

        # update blocks
        for block in perception.blocks:
            pos = np.array([block.pos.y, block.pos.x]) + self._agent_position
            matrix_pos = self._from_relative_to_matrix(pos)
            # first index --> y value, second  --> x value

            self._path_planner_representation[matrix_pos[0]][
                matrix_pos[1]] = global_variables.BLOCK_CELL_STARTING_NUMBER

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

        # update distances
        self._update_distances()

        # write data to file, used for live plotting plotting
        if self.live_plotting and self.STEP % self.PLOT_FREQUENCY == 0:
            self._write_data_to_file()

        self.STEP += 1

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
                for direction in global_variables.moving_directions:  # ADD ALSO ROTATIONS?
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
        max_trials = 1000
        # Compute next direction if map is not fully discovered (path = -1)
        if best_path != -1 and best_path is not None:
            while not valid_direction and best_path is not None or not try_counter < max_trials:
                # this point is never reached
                if best_path == 'invalid end':
                    print ("invalid end")
                # increase counter
                try_counter += 1
                # Compute direction
                direction = self.path_planner.next_move_direction(
                    [self._agent_position],
                    self.paths[path_id])
                if direction is not None:
                    # Calculate next cell value if position  of the agent is not unknown
                    if direction != 'unknown position':
                        next_cell_matrix = self._from_relative_to_matrix(self._agent_position) \
                                    + global_variables.movements[direction]
                        # Check if next_cell is out of bounds (synchronization problem?) # TODO out of bounds error in get_value_of_cell just after merging the map, this does not solve the problem idky
                        if next_cell_matrix[0] < self._path_planner_representation.shape[0] - 1 \
                                or next_cell_matrix[1] < self._path_planner_representation.shape[1] - 1:
                            walkable_cell = GridPathPlanner.is_walkable \
                                (self._get_value_of_cell(next_cell_matrix, self._path_planner_representation))
                        else:  # out of bounds
                            walkable_cell = False
                        # Check if the next_cell is still inside the path (unknown position error)
                        next_cell_rel = self._from_matrix_to_relative(next_cell_matrix)
                        if not (next_cell_rel == best_path).any():
                            walkable_cell = False
                    else:
                        # if position is unknown we want to recalculate path
                        walkable_cell = False
                    # Check if agent has reached the end or the next cell is blocked
                    if direction == 'end' or not walkable_cell:
                        self._remove_path(path_id)
                        # Recalculate path
                        best_path = path_creation_function(parameters)
                        path_id = self._save_path(best_path)
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
        parameters["dispenser_pos"] = subtask._closest_dispenser_position
        return self.get_move_direction(subtask._path_to_dispenser_id, self._get_path_to_reach_dispenser, parameters)

    def get_direction_to_close_dispenser(self, dispenser_type):
        """check if the agent is one step away from a dispenser of a certain type
        Args:
            dispenser_type(str): the type of the dispenser

        Returns: The direction of the dispenser that is 1 step away, False otherwise

        """
        for direction in global_variables.movements:
            if direction != 'end':
                close_cell_matrix = self._from_relative_to_matrix(
                    self._agent_position + global_variables.movements[direction]
                )
                dispenser_value = GridMap.get_dispenser_type(
                    self._get_value_of_cell(close_cell_matrix)
                )
                if dispenser_value == dispenser_type:
                    return direction
        return False

    def get_direction_to_close_block(self, block_type):
        """check if the agent is one step away from a dispenser of a certain type
        Args:
            block_type(str): the type of the dispenser

        Returns:The direction of the dispenser that is 1 step away, False otherwise
        """
        for direction in global_variables.movements:
            close_cell_matrix = self._from_relative_to_matrix(
                self._agent_position + global_variables.movements[direction]
            )
            block_value = GridMap.get_block_type(
                self._get_value_of_cell(close_cell_matrix)
            )
            if block_value == block_type:
                return direction
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

    def _from_relative_to_matrix(self, relative_coord, coord = None):
        """translates the coordinate with respect to the origin of the map to the
        origin of the matrix

        Args:
            relative_coord: (x,y) with respect to the origin of the map

        Returns:
            matrix_coord: (x',y') with respect to the origin of the matrix
        """

        matrix_coord = np.copy(relative_coord)
        if coord is None:   # by default origin
            matrix_coord = matrix_coord + self.origin
        else:
            matrix_coord = matrix_coord + coord

        return matrix_coord

    def _from_matrix_to_relative(self, matrix_coord, coord = None):
        """Reverse function of from_relative_to_matrix

        Args:
            matrix_coord: (x,y) with respect to the origin of the matrix

        Returns:
            relative_coord: (x',y') with respect to the origin of the map
        """

        relative_coord = np.copy(matrix_coord)
        if coord is None:
            relative_coord = relative_coord - self.origin
        else:
            relative_coord = relative_coord - coord

        return relative_coord

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
            move_array = global_variables.movements[move]
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

        self._representation = helper_map

    def _get_data_directory(self):
        """Returns Directory where map data is stored for plotting purposes"""
        data_path = get_data_location()
        return os.path.join(data_path, 'generatedMaps', 'tmp_maps')

    def _write_data_to_file(self):
        """writes two dimensional np.array to .txt file named after agent and in data directory"""
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
                for direction in global_variables.moving_directions:
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
            if length == 0:
                lenght = 100
            new_score = unknown_counts[i]/(length)
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
        """ # TODO USE ONLY RELATIVE TO THE GOAL
            # TODO RETURN ALSO THE ORDER OF THE CONNECT (base on the lowest distance)
        Compute a common meeting point for performing a connection between several agents.

        Args:
            task (key): task assigned to the agent
        Returns:
            common_meeting_point (np.array): common meeting point (in relative coordinates to the top left corner of
                the goal area.
        """

        lowest_dist = 10000
        dispenser_position = []
        dist_to_dispenser = []
        assigned_agents = []

        # For each agent
        # save dispensers and distances to lists
        # TODO check the order received when integrating it in the rhbp (first should be agent1?)
        # for key, value in agent_ids.items():
        #     # it goes from the last element of the dictionary
        #     dist_to_dispenser.insert(0, value[0])
        #     dispenser_position.insert(0, value[1])

        for sub in task.sub_tasks:
            if sub.complete is not True:
                # TODO check if the dispenser is actually where it should be.
                # TODO dispenser has to be in relative to the top left goal area corner
                dispenser_position.append(self._from_relative_to_matrix(sub._closest_dispenser_position, self.goal_top_left))
                # Check if dispensers are actually in its position
                for dispenser in dispenser_position:
                    if not GridPathPlanner.is_walkable(self._get_value_of_cell(dispenser)):
                        return None
                # distance to dispensers
                dist_to_dispenser.append(sub.distance_to_dispenser)
                # name of agents assigned
                assigned_agents.append(sub.assigned_agent)

        # calculate distance matrix for each dispenser
        # TODO dont calculate twice distance d1-d2 and d2-d1 (how?)
        for i, disp_i in enumerate(dispenser_position):
            dist_matrix = self.distance_matrix(disp_i)
            # get the distance from each pair of dispenser
            for j, disp_j in enumerate(dispenser_position):
                if i != j:
                    dist = self._get_value_of_cell(disp_j, dist_matrix) + dist_to_dispenser[i]
                    if dist < lowest_dist:
                        lowest_dist = dist  # lowest distance
                        first_agent = i    # first couple of closest agents
                        second_agent = j    # second couple of closest agents

        # Calculate a path from first (closest agent) dispenser to second (second closest agent) dispenser
        path = self.path_planner.astar(self._path_planner_representation, self.origin, \
                                     np.array([dispenser_position[first_agent]]), \
                                     np.array([dispenser_position[second_agent]]))

        # Meeting point = (distance between dispenser + distance agent2 to dispenser)/2 - distance agent1 to dispenser
        meet_index = int((lowest_dist + dist_to_dispenser[second_agent])/2 - dist_to_dispenser[first_agent])
        common_meeting_point = path[meet_index]     # in relative
        # Transform into matrix notation
        common_meeting_point = self._from_relative_to_matrix(common_meeting_point)

        # save names of closest agents
        closest_agent_1 = assigned_agents[first_agent]
        closest_agent_2 = assigned_agents[second_agent]

        return closest_agent_1, closest_agent_2, common_meeting_point

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

        agent_position = []
        task_figure = self.create_figure(task)
        all_agent_position = self.agent_position_in_figure(task_figure, common_meeting_point)

        # return position only for actual agent
        for sub in task.sub_tasks:
            if sub.assigned_agent == self.agent_name:
                index = abs(sub.position[0]) + abs(sub.position[1])
                if sub.submit_behaviour:  # True
                    agent_position = all_agent_position[:index+1]
                else:
                    agent_position = all_agent_position[index:index+1]

        # Transform to relative
        agent_position = self._from_matrix_to_relative(agent_position)

        return agent_position

    def create_figure(self, task):
        """ Create a list of agents and relative positions of blocks (to the submitting agent) associated to a
        particular task

        Args:
            task: task assigned

        Returns:
        Returns:
            figure (np.array): [Agent_name + relative block position]
        """
        max_pairs = 10  # Agents + blocks
        figure_tmp = [None]*max_pairs
        figure = []

        # Add agent position as [0,0]
        # figure_tmp[0] = [0, 0]
        # Get all the blocks
        # TODO check if there are any subtasks? there should be if task exists tho
        for sub in task.sub_tasks:
            # Sum of abs value of each coordinate is equal to the distance to the agent position
            index = abs(sub.position[0]) + abs(sub.position[1])
            # Save the agent names and blocks of figure in order
            if index == 1:  # submit agent
                figure_tmp[index - 1] = sub.assigned_agent
                figure_tmp[index] = sub.position
            else:
                figure_tmp[index] = sub.assigned_agent
                figure_tmp[index + 1] = sub.position

        # Eliminate extra elements of figure
        for i, element in enumerate(figure_tmp):
            if element is not None:
                figure.append(element)

        return figure

    def agent_position_in_figure(self, figure_rel, common_meeting_point):
        max_pairs = 10  # Agents + blocks
        free_figure_basic = figure_rel[:]
        #walkable = False
        mp_shift = 1
        mp_shift_values = np.array([[0, 0], [-1, 0], [0, 1], [1, 0], [0, -1]])  # same, up, right, down, left
        agent_shift_values = [[-1, 0], [0, 1], [1, 0], [0, -1]]  # up, right, down, left

        # TODO check common meeting point is in the right format
        common_mp_matrix = common_meeting_point
        # Transform common meeting point to matrix
        # common_mp_matrix = self._from_relative_to_matrix(common_meeting_point, self.goal_top_left)

        # Size of the figure list
        figure_size = len(free_figure_basic)
        # minimum for meeting is 4
        if figure_size == 4:
            all_possible_shifts = agent_shift_values
        elif figure_size == 6:
            all_possible_shifts = list(itertools.product(agent_shift_values, agent_shift_values, agent_shift_values))
        elif figure_size == 8:
            all_possible_shifts = list(itertools.product(agent_shift_values, agent_shift_values, agent_shift_values, agent_shift_values))
        elif figure_size == 10:
            all_possible_shifts = list(itertools.product(agent_shift_values, agent_shift_values, agent_shift_values, agent_shift_values, agent_shift_values))

        # Figure composition
        # Submitting agent and blocks have fixed position
        while True:
            mp_shift_values = mp_shift_values*mp_shift  #if there is no position around meeting point, shift
            for mp_shift in mp_shift_values:
                # Submitting agent position
                agent_submit_pos = common_mp_matrix + mp_shift  # submitting agent is in common meeting point
                free_figure_basic[0] = agent_submit_pos
                #free_figure_basic[1] = agent_submit_pos + figure_rel[1]
                # Blocks are fixed
                for i in range(1, figure_size):
                    # Block is in even position in the list
                    if i % 2 != 0:
                        free_figure_basic[i] = agent_submit_pos + figure_rel[i]
                # Iterate through different agent position
                free_figure_shifted = free_figure_basic[:]
                if figure_size == 4:
                    combinations = all_possible_shifts
                    # sum shift to figure (blocks are in even position)
                    # TODO this for is useless when only 2 agents
                    for j in range(3, figure_size):  # start at 3 because agent 1 is fixed
                        if j % 2 != 0:
                            for shifts in combinations:
                                free_figure_shifted[j-1] = free_figure_shifted[j] + shifts
                                # Check if figure composition is colliding with nearby blocks of the figure
                                if (free_figure_shifted[j-1] == free_figure_shifted[j-2]).all() or (free_figure_shifted[j-1] == free_figure_shifted[j]).all():
                                    continue
                                else:
                                    # Check if figure composition is blocked with an element of the map
                                    for element in free_figure_shifted:
                                        if GridPathPlanner.is_walkable(self._get_value_of_cell(element, self._path_planner_representation)):
                                            continue
                                        else:
                                            break
                                    # Figure composition found
                                    return free_figure_shifted

            else:
                print("TODO")

    ### GO TO DISPENSER FUNCTIONS ###
    def _get_path_to_reach_dispenser(self, parameters):
        """ Get path from agent to the dispenser """
        # TODO add attached blocks to the agent position
        # TODO delete last point in the path, not needed, we need to move 1 step away from dispenser
        if 'dispenser_pos' not in parameters:
            return None
        else:
            dispenser_pos = parameters['dispenser_pos']
        agent_pos = [self._from_relative_to_matrix(self._agent_position)]
        dispenser_pos_in_matrix = [self._from_relative_to_matrix(dispenser_pos)]
        path = self.path_planner.astar(
            maze=self._path_planner_representation,
            origin=self.origin,
            start=np.array(agent_pos),
            end=np.array(dispenser_pos_in_matrix)
        )
        path.pop() # path.techno() because we are in Berlin
        # TODO IF PATH IS NOT VALID? CHANGE DISPENSER LOCATION?
        return path

    def get_closest_dispenser_position(self, required_type):
        """get the closest dispenser position (in relative coord) of the required_type
        Args:
            required_type(str): the type of the dispenser needed

        Returns(tuple): (position,distance), (None,9999) if didn't find a valid dispenser
        """
        min_dist = 9999
        pos = None
        for dispenser in self._dispensers:
            if dispenser.type == required_type:  # check if the type is the one we need
                pos_matrix = self._from_relative_to_matrix(dispenser.pos)
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
        dist_shape = self._path_planner_representation.shape
        distances = np.full((dist_shape[0], dist_shape[1]), -1, dtype=int)
        #start_point_matrix = self._from_relative_to_matrix(start_point)
        # Dispenser are already in matrix notations
        start_point_matrix = start_point
        queue = deque([(start_point_matrix, 0)])
        while len(queue) > 0:
            pos, dist = queue.popleft()
            if distances[pos[0], pos[1]] == -1:  # to avoid infinite loop
                distances[pos[0], pos[1]] = dist
                for direction in global_variables.moving_directions:
                    new_pos = direction + pos
                    if self.coord_inside_matrix(new_pos, dist_shape):
                        if distances[new_pos[0], new_pos[1]] == -1:
                            cell_value = self._path_planner_representation[new_pos[0], new_pos[1]]
                            if GridPathPlanner.is_walkable(cell_value):
                                queue.append((new_pos, dist + 1))
        return distances



    ### static methods ###

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
        if global_variables.DISPENSER_STARTING_NUMBER <= cell_value < global_variables.BLOCK_CELL_STARTING_NUMBER:
            return cell_value - global_variables.DISPENSER_STARTING_NUMBER
        else:
            return -1
    @staticmethod
    def get_block_type(cell_value):
        if cell_value >= global_variables.BLOCK_CELL_STARTING_NUMBER:
            return cell_value - global_variables.DISPENSER_STARTING_NUMBER
        else:
            return -1


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
