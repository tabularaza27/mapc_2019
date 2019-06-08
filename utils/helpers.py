"""This File contains little helper functions that can be useful for different parts of the project"""
import os
import re
import math


def get_utils_location():
    """returns absolute path of utils folder

    Note:
        this only works when the curdir is beneathe group5 ( which should be always the case )

    Returns:
        str: absolute path to the utils folder

    Raises:
        ValueError: if group5 folder could not be found ( reason is the the script was executed outside of /group5
    """
    pattern = '^(.*?group5)'
    path = '/home/ale/Desktop/EIT/AppAI/mapc_workspace/src/group5/mapc_rhbp_manual_player/src'
    group_5_path = re.search(pattern, path)

    if group_5_path:
        utils_path = os.path.join(group_5_path.group(0), 'utils')
        return  utils_path
    else:
        raise ValueError('Could not find group5 dir, file was executed outside of /group5')


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


def manhattan_distance(coord1, coord2):
    """Calculate the manhattan distance between two coordinates in tuple form

    Args:
        coord1 (tuple): first coordinate
        coord2 (tuple): second coordinate

    Returns:
        int: euclidean distance

    """
    return abs((coord2[0] - coord1[0])) + abs((coord2[1] - coord1[1]))