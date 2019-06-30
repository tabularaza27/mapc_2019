import numpy as np


class Block:
    def __init__(self, block_type, position, subtask_id=None):
        """Representation of a block attached to an agent

        Args:
            block_type(str): string b1,b2,...
            position(np.array): coordinates [y,x] relative to the agent position
            subtask_id(str): the id of the subtask for which this block is reserved. None if it is free
        """
        self._type = block_type
        self._position = position
        self._subtask_id = subtask_id

    def rotate(self, rotate_direction):
        """rotate the coordinates of the blocks attached to the agents given the rotation direction

        Args:
            rotate_direction(str): cw or ccw
        """
        switch_sign = False
        if self._position[0] == 0:
            if rotate_direction == 'ccw':
                switch_sign = True
        else:
            if rotate_direction == 'cw':
                switch_sign = True
        self._position = Block.switch_coordinates(position=self._position, change_sign=switch_sign)

    def assign_block(self, subtask_id):
        """assigns the attached block to a specific subtask

        Args:
            subtask_id (str): is of the assigned subtask
        """
        self._subtask_id = subtask_id

    def unassign_block(self):
        """unassign block.

        Note:
            this is for example necessary if the task was completed by the enemy team
        """
        self._subtask_id = None

    # static methods

    @staticmethod
    def switch_coordinates(position, change_sign=False):
        """switches x and y coordinates of a 2d point ( in this case block position relative to agent )

        Note:
            This is necessary when updating the blocks position when the agent rotates

        Args:
            position (np.array): coordinates [y,x] relative to the agent position
            change_sign (bool): if true the sign is also changed

        Returns:
            position (np.array): updated position

        """
        if change_sign:
            return np.array(-position[1], -position[0])
        else:
            return np.array(position[1], position[0])
