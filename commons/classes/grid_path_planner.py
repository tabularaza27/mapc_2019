import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import global_variables

direction_values = [[-1, 0], [1, 0], [0, 1], [0, -1], 'ccw', 'cw']  # maze(rows, col) = agent(y, x)
direction_list = ['n', 's', 'e', 'w', 'ccw', 'cw']


class GridPathPlanner():
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

    def astar(self, maze, origin, start, end):
        """ Return a path list in relative coordinates given a map (maze), an starting point and an end point

        Args:
            maze (np.array): given map
            origin (np.array): given origin of the agent
            start (np.array): starting position in the map of the object compose by the agent and all the blocks
                attached to it
            end (np.array): ending position in the map of the object compose by the agent and all the blocks
                attached to it

        Returns:
            list: path in relative coordinates

        """

        # Check if the end point is not a free cell
        for element in end:
            end_pos = maze[element[0]][element[1]]
            if not GridPathPlanner.is_walkable(end_pos):
                print ("invalid End point")
                return 'invalid end'

        # Create start and end node
        start_node = self.Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = self.Node(None, end)
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
            if (current_node == end_node).all():
                path = []
                current = current_node
                while current is not None:
                    # Transform path to relative
                    relative_pos = self.transform_matrix_node_to_relative(current.position, origin)
                    path.append(relative_pos)
                    current = current.parent
                return path[::-1]  # return reversed path

            # Generate children
            children = []

            # Check adjacent cells (n, s, e , w) and rotations (left, right)
            for new_position in direction_values:

                # Get node position
                if new_position == 'ccw' or new_position == 'cw':
                    node_position = self.rotation(current_node.position, new_position)
                else:
                    node_position = self.translation(current_node.position, new_position)

                for element in node_position:
                    # Make sure in range
                    if element[0] > (len(maze) - 1) or element[0] < 0 or element[1] \
                            > (len(maze[len(maze) - 1]) - 1) or element[1] < 0:
                        break
                    # Make sure walkable terrain
                    elif not GridPathPlanner.is_walkable(maze[element[0], element[1]]):
                        break
                else:
                    # Create new node
                    new_node = self.Node(current_node, node_position)

                    # Append
                    children.append(new_node)

            # Loop through children
            for child in children:
                # Child is on the closed list
                for closed_child in closed_list:
                    if (child == closed_child).all():
                        break
                else:
                    # Create the f, g, and h values
                    child.g = current_node.g + 1
                    # H: Manhattan distance to end point
                    agent_current_pos = child.position[0]
                    agent_end_pos = end_node.position[0]
                    child.h = abs(agent_current_pos[0] - agent_end_pos[0]) \
                              + abs(agent_end_pos[1] - agent_current_pos[1])
                    child.f = child.g + child.h

                    # Child is already in the open list
                    for open_node in open_list:
                        # check if the new path to children is worst or equal than
                        # one already in the open_list (by measuring g value)
                        if (child == open_node).all() and child.g >= open_node.g:
                            break
                    else:
                        # Add the child to the open list
                        open_list.append(child)

    @staticmethod
    def is_walkable(cell):
        """ Check if the cell given is walkable

        Args:
            cell (int): value of given cell

        Returns:
            Bool: True if cell is walkable, False otherwise

        """
        if cell in (global_variables.EMPTY_CELL, global_variables.GOAL_CELL) or \
                global_variables.DISPENSER_STARTING_NUMBER <= cell < global_variables.BLOCK_CELL_STARTING_NUMBER:
            return True

        return False

    def translation(self, node, direction):
        """Apply a translation (n, s, e, w) to a node (Agent + blocks attached) and return its new position

        Args:
            node (np.array): Node to be translated
            direction (list): Direction of the translation (n, s, e, w)

        Returns:
            np.array: Node translated coordinates in the same type as given (relative or matrix)

        """
        # Create a copy of node
        node_translated = np.copy(node)

        # Apply translation
        for i in range(0, node_translated.shape[0]):
            node_translated[i] = node_translated[i] + direction

        return node_translated

    def rotation(self, node, direction):
        """ Apply a rotation (counterclockwise, clockwise) to a node (Agent + blocks attached) and return its
        new position

        Args:
            node (np.array): Node to be rotated
            direction (string): Direction of the rotation (cw = Clockwise or ccw = Counterclockwise)

        Returns:
            np.array: Node rotated in map coordinates

        """
        # Rotation transformations
        rotate_left = [-1, 1]
        rotate_right = [1, -1]

        # Transform to relative coordinates
        origin = node[0]
        node_rotated = self.transform_matrix_node_to_relative(node, origin)

        # Check direction of rotation
        if direction == 'ccw':
            rotation = rotate_left
        else:   # cw
            rotation = rotate_right

        # Apply rotation: swap (y,x) * rotate_left/right + origin
        for i in range(1, node_rotated.shape[0]):
            # swap
            node_element = node_rotated[i]
            node_element = node_element[::-1]
            # Multiply by rotation transformation
            node_rotated_y = node_element[0]*rotation[0]
            node_rotated_x = node_element[1]*rotation[1]
            node_rotated[i] = [node_rotated_y, node_rotated_x]

        # Convert back to map coordinates
        node_rotated = self.transform_relative_node_to_matrix(node_rotated, origin)

        return node_rotated

    def transform_matrix_node_to_relative(self, node_matrix, pos_init):
        """Transform a node (agent + block) position in matrix coordinates to relative coordinates
            (agent position = (0,0))

        Args:
            node_matrix (np.array): node in matrix coordinates
            pos_init(np.array): position at the node initialization in matrix coordinates

        Returns:
            np.array: node in relative coordinates (agent position = (0,0)

        """

        node_rel = np.copy(node_matrix)
        node_rel = node_rel - pos_init

        return node_rel

    def transform_relative_node_to_matrix(self, node_rel, pos_init):
        """Transform a node (agent + block) position in relative coordinates (agent position = (0,0)) to
            map coordinates

        Args:
            node_rel (np.array): node in relative coordinates
            pos_init (np.array): position at the node initialization in matrix coordinates

        Returns:
            np.array: node in matrix coordinates

        """
        node_matrix = np.copy(node_rel)
        node_matrix = node_matrix + pos_init

        return node_matrix

    def next_move_direction(self, actual_pos, path):
        """ Return the next move or rotate direction to perform with respect to a given path and agent position

        Args:
            actual_pos (np.array): position of the agent + blocks at the moment
            path (list or -1): path to follow by the agent

        Returns:
            string: ( 'n', 's', 'e', 'w', 'ccw', 'cw' ) - Next move or rotate direction
                    'end' - current position is the end point
                    'unknown position' - position is not included in path
                    'unknown rotation' - rotation is not valid
                    'unknown translation' - translation is not valid
                    None - No move

        """
        same_position = False
        path_index = 0

        # Check is path is None
        if path is None:
            return None

        # Check if path is not just the agent position
        if path != -1 and path != 'invalid end':  # Map fully discovered
            length = len(path)
            if length > 1:
                # Index of actual position in path
                for index, node in enumerate(path):
                    if (node == actual_pos).all():
                        path_index = index
                        break   # get out
                    else:
                        path_index = -1
            else:
                same_position = True
        else:
            return None     # No move

        # Check if position doesn't exist in path
        if path_index == -1:
            next_action = 'unknown position'
            return next_action

        # Check if the end has been reached
        if path_index == length - 1 or same_position:
            next_action = 'end'
            return next_action

        # Save next position from path
        next_pos = np.copy(path[path_index + 1])

        # Calculate difference between next and actual position
        diff = next_pos - actual_pos

        # Check if diff is rotation
        if np.sum(diff[0]) == 0:    # first node is equal in a rotation
            for rotation in ['ccw', 'cw']:
                block_rotate = self.rotation(actual_pos, rotation)
                if (block_rotate == next_pos).all():
                    next_action = rotation
                    return next_action

            print ("unknown rotation")
            next_action = 'unknown rotation'
            return next_action

        else:   # translation
            for coord, movement in enumerate(direction_values):
                if (movement == diff[0]).all():     # Check agent, equal for all the blocks too
                    next_action = direction_list[coord]
                    return next_action
                elif coord == len(direction_values) - 1:
                    print ("action is unknown")
                    next_action = 'unknown translation'
                    return next_action

    def show_path(self, maze, path_matrix, length = -1, pause=1.0):
        """Generate a representation of the map with the path of the agent coloured in orange and the blocks in yellow

        Args:
            maze (np.array): map matrix
            path_matrix (list): path in matrix notation
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
                if counter == length-1: # change color of the current agent position
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
        #if(length < len(path_matrix)-1): # don't clear the last image
        if (length < len(path_matrix) - 1):  # don't clear the last image
            plt.clf()


def main():
    """ Load map
    import numpy as np
    maze = np.loadtxt(open("generatedMaps/00/map.csv", "rb"), delimiter=",")
    """

    # Simple maze
    maze = [[ 0, 0, 0, -2, 0,-2,-2, 0, 0, 0],
            [-2, 2, 0, -2, 5, 0, 0, 0, 0, 0],
            [ 0, 0, 0, -2, 0, 0, 0, 0, 0, 0],
            [ 0, 0,-2,  0,-2,-2, 0, 0, 0, 0],
            [ -3, 0, 0,  0, 0,-2, 0, 0, 0, 0],
            [-2, 0, 0,  0, 0, 0, 0, 0,-2,-2],
            [ 0,-2, 0,  0, 0, 0, 0,-2, 0, 0],
            [ 0, 0, 0,  0,-2, 0,-2, 0,-2, 0],
            [ 0, 0, 0,  0,-2, 0,-2, 0, 0, 0],
            [ 0, 0, 0,  0,-2, 0, 0, 0, 0, 0]]


    # Origin
    origin = [[0, 0]]
    # Single agent
    start_point = [[1, 1]]
    #end_point = [[6, 8]]
    end_point = start_point

    """
    # Agent + blocks (L shape)
    start_point = [[0, 0], [0, 1], [1, 1]]
    end_point = [[1, 5], [1, 4], [0, 4]]
    """

    start = np.array(start_point, dtype=np.int)
    end = np.array(end_point, dtype=np.int)
    
    path_planner = GridPathPlanner()
    path = path_planner.astar(maze, origin, start, end)  # path in matrix notation
    print (path)

    """
    # Animation of path
    if path is not None:
        plt.ion()
        for i in range(1, len(path)+1):
            path_planner.show_path(maze, path, i, 0.5)
    else:
        print ("There is no path")
        
    raw_input("Press Enter to continue...")
    """

    # Next_move print
    if path is not None:
        for next_step in path:
            # Print next action
            move = path_planner.next_move_direction(next_step, path)
            print (move)
            raw_input("Press Enter to continue...")
    else:
        print ("There is no path")
        raw_input("Press Enter to continue...")


if __name__ == '__main__':

    main()
