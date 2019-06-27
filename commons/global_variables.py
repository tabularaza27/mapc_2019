import numpy as np

DEBUG_MODE=False
LIVE_PLOTTING=True

moving_directions = [[-1, 0], [1, 0], [0, 1], [0, -1]]
movements = {
    'n': np.array([-1, 0]),
    's': np.array([1, 0]),
    'w': np.array([0, -1]),
    'e': np.array([0, 1]),
    'end': np.array([0, 0])
}

# Cells values
EMPTY_CELL = 0
UNKNOWN_CELL = -1
WALL_CELL = -2
GOAL_CELL = -3
AGENT_CELL = -4
ENTITY_CELL = -5
BLOCK_CELL_STARTING_NUMBER = 101
DISPENSER_STARTING_NUMBER = 1
