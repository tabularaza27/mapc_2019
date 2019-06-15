import numpy as np

DEBUG_MODE=True
LIVE_PLOTTING=True

moving_directions = [[-1, 0], [1, 0], [0, 1], [0, -1]]
movements = {
    'n': np.array([-1, 0]),
    's': np.array([1, 0]),
    'w': np.array([0, -1]),
    'e': np.array([0, 1])
}
