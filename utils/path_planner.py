import matplotlib.pyplot as plt
import matplotlib as mpl


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
    """ Return a path list in map coordinates given a map (maze), an starting point and an end point

    Args:
        maze (np.array): given local or global map
        start (list): starting position in the map of the object compose by the agent and all the blocks attached to it
        end (list): ending position in the map of the object compose by the agent and all the blocks attached to it

    Returns:
        list: path in map coordinates

    """

    # Check if the end point is not a free cell
    for element in end:
        end_pos = maze[element[0]][element[1]]
        if end_pos != 0:  # ( ADD OTHER TYPES OF BLOCKED CELLS HERE )
            print ("invalid End point")
            return

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []  # include adjacent nodes that has to be evaluated (f value)
    closed_list = []  # include nodes from the open_list which has been evaluated (lowest f value)

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
            return path[::-1]  # return reversed path

        # Generate children
        children = []

        # Check adjacent cells (n, s, e , w) and rotations (left, right)
        for new_position in [(-1, 0), (1, 0), (0, 1), (0, -1), 'left', 'right']:  # maze(rows, col) = agent(y, x)
            # Get node position
            if new_position == 'left' or new_position == 'right':
                node_position = rotation(current_node.position, new_position)
            else:
                node_position = translation(current_node.position, new_position)

            for element in node_position:
                # Make sure in range
                if element[0] > (len(maze) - 1) or element[0] < 0 or element[1] > (len(maze[len(maze) - 1]) - 1) or \
                        element[1] < 0:
                    break
                # Make sure walkable terrain
                elif maze[element[0]][element[1]] != 0:
                    break
            else:
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
                agent_current_pos = child.position[0]
                agent_end_pos = end_node.position[0]
                child.h = abs(agent_current_pos[0] - agent_end_pos[0]) + abs(agent_end_pos[1] - agent_current_pos[1])
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


def translation(node, direction):
    """Apply a translation (n, s, e, w) to a node (Agent + blocks attached) and return its new position in
        map coordinates

    Args:
        node (list): Node to be translated
        direction (list): Direction of the translation (n, s, e, w)

    Returns:
        list: Node translated in map coordinates

    """
    # Apply transformation
    node_translated = []
    for element in node:
        node_y = element[0] + direction[0]
        node_x = element[1] + direction[1]
        node_translated.append((node_y, node_x))

    return node_translated


def rotation(node, direction):
    """ Apply a rotation (left, right) to a node (Agent + blocks attached) and return its new position in
        map coordinates

    Args:
        node (list): Node to be rotated
        direction (string): Direction of the rotation (Right = Clockwise or Left = Counterclockwise)

    Returns:
        list: Node rotated in map coordinates

    """
    # Rotation transformations
    rotate_left = (-1, 1)
    rotate_right = (1, -1)
    # Check relative position of node
    node_rotated = []
    origin = node[0]
    node_rel = transform_matrix_node_to_relative(node, origin)
    for element in node_rel:
        if element == (0, 0):
            # Agent pos doesn't rotate : Transform to matrix notation and append
            element_rotated_y = element[0] + origin[0]
            element_rotated_x = element[1] + origin[1]
            element_rotated = (element_rotated_y, element_rotated_x)
            node_rotated.append(element_rotated)
            continue

        # Check direction of rotation
        if direction == 'left':
            rot_transf = rotate_left
        else:  # right
            rot_transf = rotate_right

        # Element_rotated = swap (y,x) * rotate_left/right + origin
        element_y = element[0]
        element_x = element[1]
        # Swap y and x
        element_swap = (element_x, element_y)
        # Apply rotation + conversion to matrix notation
        element_rotated_y = element_swap[0] * rot_transf[0] + origin[0]
        element_rotated_x = element_swap[1] * rot_transf[1] + origin[1]
        element_rotated = (element_rotated_y, element_rotated_x)

        # Add rotated element to the node
        node_rotated.append(element_rotated)

    return node_rotated


def transform_matrix_list_to_relative(node_matrix, pos_init):
    """Transform a list of nodes (agent + block) position in map coordinates to
        relative coordinates (agent position = (0,0))


    Args:
        node_matrix (list): list of nodes in map coordinates
        pos_init (tuple): position at the node initialization of the agent node in map coordinates

    Returns:
        list: List of nodes in relative coordinates (agent position = (0,0))

    """
    node_rel_list = []
    node_rel_element = []
    for list_matrix in node_matrix:
        for element in list_matrix:
            node_rel_y = element[0] - pos_init[0]
            node_rel_x = element[1] - pos_init[1]
            node_rel_element.append((node_rel_y, node_rel_x))
        node_rel_list.append(node_rel_element)
        node_rel_element = []  # reset list

    return node_rel_list


def transform_matrix_node_to_relative(node_matrix, pos_init):
    """Transform a node (agent + block) position in map coordinates to relative coordinates (agent position = (0,0))


    Args:
        node_matrix (tuple): node in map coordinates
        pos_init(tuple): position at the node initialization in map coordinates

    Returns:
        tuple: node in relative coordinates (agent position = (0,0)

    """
    node_rel = []
    for element in node_matrix:
        node_rel_y = element[0] - pos_init[0]
        node_rel_x = element[1] - pos_init[1]
        node_rel.append((node_rel_y, node_rel_x))

    return node_rel


def transform_relative_list_to_matrix(node_rel, pos_init):
    """Transform a list of nodes (agent + block) position in relative coordinates (agent position = (0,0)) to
        map coordinates

    Args:
        node_rel (list): list of nodes in relative coordinates
        pos_init (tuple): position at the node initialization of the agent node in map coordinates

    Returns:
        list: List of nodes in map coordinates

    """
    node_matrix_list = []
    node_matrix_element = []
    for list_rel in node_rel:
        for element in list_rel:
            node_matrix_y = element[0] + pos_init[0]
            node_matrix_x = element[1] + pos_init[1]
            node_matrix_element.append((node_matrix_y, node_matrix_x))
        node_matrix_list.append(node_matrix_element)
        node_matrix_element = []

    return node_matrix_list


def transform_relative_node_to_matrix(node_rel, pos_init):
    """Transform a node (agent + block) position in relative coordinates (agent position = (0,0)) to
        map coordinates

    Args:
        node_rel (tuple): node in relative coordinates
        pos_init (tuple): position at the node initialization in map coordinates

    Returns:
        tuple: node in map coordinates

    """
    node_matrix = []
    for element in node_rel:
        node_matrix_y = element[0] + pos_init[0]
        node_matrix_x = element[1] + pos_init[1]
        node_matrix.append((node_matrix_y, node_matrix_x))

    return node_matrix

'''
def path_relative_coord(path):
    """Calculates commands ('n','e','s','w','ccw','cw') to fulfill path

    Args:
        path (list): path to end point from start point in matrix notation (row, col)

    Returns:
        list: list of steps to reach the end point from the start point in cardinal notation ( n, s, e, w)

    """

    path_rel = []  # path in relative coordinates
    cardinal_points = ['n', 's', 'e', 'w']
    direction = [(-1, 0), (1, 0), (0, 1), (0, -1)]  # direction = next position - actual position
    next_step = []

    for index, node_matrix in enumerate(path):

        # Check if the end has been reached
        if len(path) - 1 == index:
            break
        else:
            actual_pos = list(node_matrix)  # actual position from where it moves
            next_pos = list(path[index + 1])  # next position to where it moves

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


def get_path(maze, start, end):
    """Calculates path from start to end and returns commands to ('n','e','s','w') fulfill the path

    Args:
        maze (np.array): given local or global map
        start (tuple): agent position in the map
        end (tuple): goal of the agent

    Returns:
        list: list of steps to reach the end point from the start point in cardinal notation ( n, s, e, w)
    """
    coordinates_path = astar(maze, start, end)
    command_path = path_relative_coord(coordinates_path)

    return command_path
'''

def next_move(node_pos, path, map):
    """ Return the next move or rotate direction to perform with respect to a given path and map

    Args:
        node_pos (list): position of the agent + blocks at the moment
        path (list): path to follow by the agent
        map (np.array: given local or global map

    Returns:
        string: ( 'n', 's', 'e', 'w', 'ccw', 'cw' ) - Next move or rotate direction
                'end' - current position is the end point
                'unknown' - it is not possible to reach next position in path

    """

    # (move) north, south, east, west, counter-clockwise (left), clockwise (right)
    action_list = ['n', 's', 'e', 'w', 'ccw', 'cw']
    direction_list = [(-1, 0), (1, 0), (0, 1), (0, -1), (1, -1), (-1, 1)]
    path_index = 0

    # Get agent and first block current position
    current_agent_pos = node_pos[0]
    first_block_pos = node_pos[1]

    for index, path_step in enumerate(path):
        # Discard previous steps from path
        path_pos = path_step[0]
        agent_pos = path_pos
        #print(path_pos)
        #print(agent_pos)
        #print(path_pos == agent_pos)
        if agent_pos == current_agent_pos:
        # Check if the end has been reached --> return 'end'
            path_length = len(path)
            if index == path_length - 1:
                next_action = 'end'
                return next_action
            else:
                # Save the index of the current position in path
                path_index = index
                break   # Get out of the for loop

    # Next position of the node
    next_node_pos = path[path_index + 1]

    # Check if there is any block attached
    if first_block_pos is None:    # No blocks attached
        current_pos = current_agent_pos   # Tracking block is the agent
        next_pos = next_node_pos[0]
    else:
        current_pos = first_block_pos   # Tracking block is the first block attached
        next_pos = next_node_pos[1]

    # Calculate relative position of the next step w.r.t current position
    current_pos_y = current_pos[0]
    current_pos_x = current_pos[1]
    next_pos_y = next_pos[0]
    next_pos_x = next_pos[1]
    # direction = next position - actual position
    direction_y = next_pos_y - current_pos_y
    direction_x = next_pos_x - current_pos_x
    direction = (direction_y, direction_x)

    # Get the next action
    for coord, movement in enumerate(direction_list):
        if direction != movement:
            continue
        if coord > len(direction_list) + 1:
            print ("action is unknown")
            next_action = 'unknown'
            return next_action
        else:
            next_action = action_list[coord]
            break

    return next_action


def show_path(maze, path_matrix, length=-1, pause=1.0):
    """Generate a representation of the map with the path of the agent coloured in orange and the blocks in yellow

    Args:
        maze (np.array): map matrix
        path_matrix (tuple): path in matrix notation
        lenght(int): the length of the path to show
        pause(float): the pause between each frame
    Returns:
        void: Nothing

    """
    # integer for path cell
    agent_path_cell = -4
    block_path_cell = -5
    last_agent = -3
    last_block = -1
    # make a copy of maze
    map_path = maze
    # create list out of path_matrix
    if length == -1:
        length = len(path_matrix)
    counter = 0
    for position in path_matrix[0:length]:
        path = list(position)

        # add path values to map
        for index, element in enumerate(path):
            i = element[0]
            j = element[1]
            agent_color = agent_path_cell
            block_color = block_path_cell
            if counter == length-1: #change color of the current agent position
                agent_color = last_agent
                block_color = last_block
            if index == 0:
                map_path[i][j] = agent_color
            elif map_path[i][j] != agent_color:  # Agent cells overlapped block cells
                map_path[i][j] = block_color
        counter += 1

    cmap = mpl.colors.ListedColormap(['yellow', 'orange', 'red', 'black', 'blue', 'white'])
    plt.imshow(map_path, cmap=cmap, vmin=-5, vmax=1)
    
    plt.draw()
    plt.pause(pause)
    if(length < len(path_matrix)-1): # don't clear the last image
        plt.clf()


def main():
    """ Load map
    import numpy as np
    maze = np.loadtxt(open("generatedMaps/00/map.csv", "rb"), delimiter=",")
    """

    # Simple maze
    maze = [[ 0, 0, 0, -2, 0,-2,-2, 0, 0, 0],
            [-2, 0, 0, -2, 0, 0, 0, 0, 0, 0],
            [ 0, 0, 0, -2, 0, 0, 0, 0, 0, 0],
            [ 0, 0,-2,  0,-2,-2, 0, 0, 0, 0],
            [ 0, 0, 0,  0, 0,-2, 0, 0, 0, 0],
            [-2, 0, 0,  0, 0, 0, 0, 0,-2,-2],
            [ 0,-2, 0,  0, 0, 0, 0,-2, 0, 0],
            [ 0, 0, 0,  0,-2, 0,-2, 0,-2, 0],
            [ 0, 0, 0,  0,-2, 0,-2, 0, 0, 0],
            [ 0, 0, 0,  0,-2, 0, 0, 0, 0, 0]]

    # Example with 2 blocks attached (L shape)
    start = [(0, 0), (0, 1), (1, 1)]
    end = [(1, 5), (1, 4), (0, 4)]

    """
    # Example without blocks attached
    start = [(1, 1)]
    end = [(6, 8)]
    """
    path = astar(maze, start, end)  # path in matrix notation
    print (path)
    '''
    # Animation of path
    if path is not None:
        plt.ion()
        for i in range(1,len(path)+1):
            show_path(maze, path, i, 0.5)
    else:
        print ("There is no path")
        
    raw_input("Press Enter to continue...")

    '''
    # Next_move print
    if path is not None:
        for next_step in path:
            # Print next action
            move = next_move(next_step, path, maze)
            print (move)
            '''
            # Plot next step
            plt.ion()
            show_path(maze, next_step, index + 1, 0.5)
            '''
            raw_input("Press Enter to continue...")
    else:
        print ("There is no path")

    raw_input("Press Enter to continue...")


if __name__ == '__main__':

    main()
