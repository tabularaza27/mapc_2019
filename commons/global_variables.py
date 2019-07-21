import numpy as np

#debug variables
DEBUG_MODE = False
LIVE_PLOTTING = True
DUMP_CLASS = False

# movements and directions useful variables
MOVING_DIRECTIONS = [[-1, 0], [1, 0], [0, 1], [0, -1]]
MOVEMENTS = {
    'n': np.array([-1, 0]),
    's': np.array([1, 0]),
    'w': np.array([0, -1]),
    'e': np.array([0, 1]),
    'end': np.array([0, 0])
}
string_directions = ['n', 's', 'w', 'e']

# Cells values
EMPTY_CELL = 0
UNKNOWN_CELL = -1
WALL_CELL = -2
GOAL_CELL = -3
AGENT_CELL = -4
ENTITY_CELL = -5
BLOCK_CELL_STARTING_NUMBER = 100
DISPENSER_STARTING_NUMBER = 10
