from __future__ import division  # force floating point division when using plain /

import math

from mapc_ros_bridge.msg import Position


def get_bridge_topic_prefix(agent_name):
    """
    Determine the topic prefix for all topics of the bridge node corresponding to the agent (name)
    :param agent_name: current agents name
    :return: prefix just before the topic name of the bridge
    """
    return '/bridge_node_' + agent_name + '/'


def euclidean_distance(pos1, pos2):
    """
    Calculate the euclidean distance between two positions
    :param pos1: position 1
    :type pos1: Position
    :param pos2: position 2
    :type pos2: Position
    :return: euclidean distance
    """
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)


def relative_euclidean_distance(pos1):
    """
    Calculate the relative euclidean distance to a point
    :param pos1: position 1
    :type pos1: Position
    :return: relative euclidean distance
    """
    return math.sqrt(pos1.x ** 2 + pos1.y ** 2)


def pos_to_direction(pos):
    """
    Determine a direction from a given relative position
    :param pos: relative position with x and y
    :return: direction string like 'n', 's', 'e', 'w'
    """

    if pos.x == 0 and pos.y > 0:
        return 's'
    elif pos.x == 0 and pos.y < 0:
        return 'n'
    elif pos.y == 0 and pos.x > 0:
        return 'e'
    elif pos.y == 0 and pos.x < 0:
        return 'w'
    else:  # if not directly on the same column/row as the position, we try to reduce the smaller axis value (x/y) first
        if pos.x > pos.y:
            if pos.y > 0:
                return 's'
            else:
                return 'n'
        else:
            if pos.x > 0:
                return 'e'
            else:
                return 'w'
    return None
