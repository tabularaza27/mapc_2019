"""
ToDo: Consider how far you need to travel to explore x amount new cells. Example travel 1 to discover 3 new cells vs. travel 5 to discover 7 new cells
ToDo: Consider Sight can also be blocked by obstacles --> double check if agents can see throug walls or not
ToDo: Index of Map is not necessarily index of array where map is stored, array just adds indices when new stuff gets appended
ToDo: Choose more sophisticated approach for points considering for exploration, now all point in inner square that are 5 points away from wall are chosen

Notes:
    We always have a quadratic map, unknown fields are just filled up with -1
    What if two points are best

Questions:
    Is diagonal vision also 5
    Should the agent anticipate, what is behind the unknown, maybe there are signs that there is more unknown, i.e. if we would know the absolute position of our agent
    What will be the reference of out frame, where is the origin --> e.g. where agent was initialized | will origin change or stay fixed


"""
import numpy as np

from path_planner import get_path


def get_unknown_amount(map_representation, position):
    """calculate amount of unknown cells around a given cell

    Args:
        map_representation (np.array):
        position (tuple): tuple containing x,y coordinates

    Returns:
        int: amount of unknown cells around given position
    """
    vision_range = 5
    unknown_count = 0
    # loop through all cells that are in (theoretical) vision from specified position
    # not considering vision hindering through obstacles
    for x in range(-vision_range, vision_range + 1):
        for y in range(-vision_range, vision_range + 1):
            cell_index = (position[0] + x, position[1] + y)
            # omit cells that are out of bounds
            if cell_index not in np.ndindex(map_representation.shape): continue
            # if unknown, increase unknown count
            if map_representation[cell_index[0], cell_index[1]] == -1:
                unknown_count += 1

    return unknown_count


def get_point_to_explore(map_representation, current_position):
    """Calculates point that is most suited for exploring and path to it

    Args:
        map_representation (np.array):
        current_position (tuple): position of the agent

    Returns:
        tuple: tuple containing best_point (tuple), best_path list(tuples), amount of unkown cells to be explored when this point is reached by the agent (int)
    """
    # map indices
    lower_bound = 0
    upper_bound = map_representation.shape[0]

    # keep track of best suitable points for exploration
    best_points = []
    current_high_score = 0

    # loop through all points in the map
    for x, y in np.ndindex(map_representation.shape):
        # consider all points that are 5 cells away from a wall
        if ((x == lower_bound + 5 and lower_bound + 5 < y < upper_bound - 5) or (
                x == upper_bound - 5 and lower_bound + 5 < y < upper_bound - 5) or (
                    y == lower_bound + 5 and lower_bound + 5 < x < upper_bound - 5) or (
                    y == upper_bound - 5 and lower_bound + 5 < x < upper_bound - 5)) and (
                map_representation[x][y] == 0):
            # calculate the amount of unknown cells around this cell
            unknown_count = get_unknown_amount(map_representation, (x, y))
            if unknown_count == current_high_score:
                best_points.append((x, y))
            elif unknown_count > current_high_score:
                current_high_score = unknown_count
                best_points = [(x, y)]

    # calculate path length between current position and potential exploration points and choose the one with shortest path
    shortest_path = np.inf
    best_point = None
    best_path = None
    for point in best_points:
        print(point)
        path = get_path(map_representation, current_position, point)
        print(path)
        if len(path) < shortest_path:
            best_point = point
            best_path = path

    return best_point, best_path, current_high_score


if __name__ == '__main__':
    map_representation = np.loadtxt(open("generatedMaps/1/partial7.csv", "rb"), delimiter=",")
    current_point = (5,4)
    best_point, best_path, amount_unknowns = get_point_to_explore(map_representation, current_point)

    print 'best_point: ', best_point
    print 'best_path: ', best_path
    print 'amount unknowns that will be explored', amount_unknowns
