class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  # G is the distance between the current node and the start node.
        self.h = 0  # H is the heuristic : estimated distance from the current node to the end node.
        self.f = 0  # F is the total cost of the node. F= G + H

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    '''

    Args:
        maze: given local or global map
        start: agent position in the map
        end: goal of the agent

    Returns: path (in matrix coordinates)

    '''

    # Check if the end point is a free cell, else exit
    if maze[end[0]][end[1]] != 0:
        print ("invalid End point")
        return

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []          # include adjacent nodes that has to be evaluated (f value)
    closed_list = []        # include nodes from the open_list which has been evaluated (lowest f value)

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):  # search for the minimum f value among all nodes in open_list
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current (lowest f value node) off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []

        # Check adjacent cells (n, s, e , w)
        for new_position in [(1, 0), (-1, 0), (0, 1), (0, -1)]:  # maze(rows, col) = agent(y, x)

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    break
            else:
                # Create the f, g, and h values
                child.g = current_node.g + 1
                # H: Manhattan distance to end point
                child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    # check if the new path to children is worst or equal than
                    # one already in the open_list (by measuring g value)
                    if child == open_node and child.g >= open_node.g:
                        break
                else:
                    # Add the child to the open list
                    open_list.append(child)


def path_relative_coord(path_matrix):
    '''

    Args:
        path_matrix: path to end point from start point in matrix notation (row, col)

    Returns: sequence of steps to reach the end point from the start point in cardinal notation ( n, s, e, w)

    '''

    path_rel = []  # path in relative coordinates
    cardinal_points = ['n', 's', 'e', 'w']
    direction = [(-1, 0), (1, 0), (0, 1), (0, -1)]  # direction = next position - actual position
    next_step = []

    for index, node_matrix in enumerate(path_matrix):

        # Check if the end has been reached
        if len(path_matrix) - 1 == index:
            break
        else:
            actual_pos = list(node_matrix)  # actual position from where it moves
            next_pos = list(path_matrix[index + 1])  # next position to where it moves

        # Check direction of next step
        next_step.append(next_pos[0] - actual_pos[0])
        next_step.append(next_pos[1] - actual_pos[1])

        # Transform step into cardinal points (n, s, e , w)
        for coord, movement in enumerate(direction):
            if next_step == list(movement):
                path_rel.append(cardinal_points[coord])  # Append next step in cardinal point into path
                # Remove old next steps
                del next_step[:]
                break

    return path_rel



def main():

    # Load map from examples
    import numpy as np
    maze = np.loadtxt(open("generatedMaps/00/map.csv", "rb"), delimiter=",")
    '''

    maze = [[0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 0, 1, 0, 0, 1, 1, 1],
            [0, 1, 0, 0, 1, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
            [0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]

    '''
    # Example of start and end path with load map
    start = (2, 4)
    end = (21, 34)
    '''
    start = (2, 1)
    end = (0, 7)
    '''
    path = astar(maze, start, end)
    print(path)  # (row, col) format: n=(-1,0), s=(1,0), e=(0,1), w=(0,-1)
    path = path_relative_coord(path)
    print(path)



if __name__ == '__main__':
    main()