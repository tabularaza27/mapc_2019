import numpy as np
import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import random

# ToDo: Implement blocks
# ToDo: Convert Unknown Cell to empty cell
# ToDo: Keep track of blocks / entities when they are moving
# ToDo: Identify if other entity is enemy or ally
# ToDo: Live Plotting Functionality, now a new plot get created every time


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
    # 0,1,2,3,4,...: dispenser of type 1,2,3,4,...
    # 100,101,102,103,104,...: block of type 1,2,3,4,...

    author: Alessandro
    """
    PLOT_MAP = True
    # create plot every x steps
    PLOT_FREQUENCY = 50
    # counter variable
    STEP = 0

    # EMPTY_CELL = 0
    UNKNOWN_CELL = -1
    WALL_CELL = -2
    GOAL_CELL = -3
    AGENT_CELL = -4
    ENTITY_CELL = -5
    BLOCK_CELL_STARTING_NUMBER = 101

    ### PUBLIC METHODS ###
    def __init__(self):
        """
        Initialization of the map. The agent is at the center of an unknown map
        """
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

        # generate colors for mapping purposes
        number_of_colors = 12
        colors = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)]) for i in range(number_of_colors)]
        self.cmap = mpl.colors.ListedColormap(colors)



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
                    self._representation[matrix_pos[1]][matrix_pos[0]] = i

            # add to state variable as dict
            # {pos: {x: 3, y: 3}, type: 'b1'}
            if dispenser not in self._dispensers:
                self._dispensers.append(dispenser)

        # update blocks

        if self.PLOT_MAP and self.STEP % self.PLOT_FREQUENCY == 0:
            self.show_map()

        self.STEP += 1

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
    
    def getLocalMap(self):
        return self._representation


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
        return GridMap.add_coord(relative_coord, self._agent_position)

    def _from_matrix_to_relative(self, matrix_coord):
        """
        Reverse function of from_relative_to_matrix
        Args:
            matrix_coord: (x,y) with respect to the origin of the matrix

        Returns:
            relative_coord: (x',y') with respect to the origin of the map
        """

        return GridMap.add_coord(matrix_coord, self._agent_position, operation='sub')

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
                    self._agent_position = GridMap.add_coord(self._agent_position, (0, -1))


            elif move == 'e':
                self._agent_position = GridMap.add_coord(self._agent_position, (1, 0))
                if self._agent_position[0] + 5 >= self._representation.shape[1]:
                    self._expand_map('e')

            elif move == 's':
                self._agent_position = GridMap.add_coord(self._agent_position, (0, 1))
                if self._agent_position[1] + 5 >= self._representation.shape[0]:
                    self._expand_map('s')

            elif move == 'w':
                if self._agent_position[0] - 5 <= 0:
                    self._expand_map('w')
                else:
                    self._agent_position = GridMap.add_coord(self._agent_position, (-1, 0))

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


    #### DEBUG FUNCTIONALITIES ####

    def show_map(self):
        """
        Debug function to show a colored map
        Returns: None

        """

        fig = plt.figure()
        ax = fig.add_subplot(111)
        im = ax.matshow(self._representation, cmap=self.cmap, vmin=-6, vmax=5)
        fig.show()