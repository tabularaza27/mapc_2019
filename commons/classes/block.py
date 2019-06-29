import numpy as np
class Block():
    def __init__(self, type, position, subtask_id=None):
        '''
        Representation of a block attached to an agent
        Args:
            type(str): string b1,b2,...
            position(np.array): couple of coordinates [y,x] relative to the agent position
            subtask_id(str): the id of the subtask for which this block is reserved. None if it is free
        '''
        self._type = type
        self._position = position
        self._subtask_id = subtask_id

    def rotate(self, rotate_direction):
        '''
        Function that rotates the coordinates of the blocks attached to the agents
        given the rotation direction
        Args:
            rotate_direction(str): cw or ccw

        Returns:

        '''
        switch_sign = False
        if self._position[0] == 0:
            if rotate_direction == 'ccw':
                switch_sign = True
        else:
            if rotate_direction == 'cw':
                switch_sign = True
        self._position = self._switch_coordinates(self._position, switch_sign)


    def assign_block(self, subtask_id):
        self._subtask_id = subtask_id

    def unassign_block(self):
        self._subtask_id = None



    #private methods

    def _switch_coordinates(self,a, change_sign=False):
        if change_sign:
            return np.array(-a[1], -a[0])
        else:
            return np.array(a[1], a[0])