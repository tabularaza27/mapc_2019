"""
USAGE go into the util folder and run:
python mapGeneration.py

At the end of the file change the number of the example to save new examples with map and partial maps

Feel free to copy and paste the code if necessary.
I'm sorry if it's not commented very well, but it took me already 5 hours of work.
Feel free to add meaningful comments and please tell me if I should change style of coding

Author: Alessandro Amici
"""
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import random

"""
    MAP SPECIFICATION
    Matrix of integers. Each value has the following meaning:
    #  0: empty cell
    # -1: unknown cell
    # -2: wall, obstacle
    # -3: goal area
    # -4: enemy entity agent ??necessary??temporary??
    # -5: ally entity agent ??necessary??temporary??
    # 1,2,3,4,...: dispenser of type 1,2,3,4,...
    # 101,102,103,104,...: block of type 1,2,3,4,...
"""

EMPTY_CELL = 0
UNKNOWN_CELL = -1
WALL_CELL = -2
GOAL_CELL = -3



def generateMap(lenght, width, blockDensity, n_types, n_dispenser):
    """
    Args:
        lenght: the x dimension of the map
        width: the y diemension of the map
        blockDensity: the probability for an empty cell to be a wall
        n_types: the number of block types
        n_dispenser: number of dispenser to be placed

    Returns:
        map: a matrix of integers that follows the MAP SPECIFICATION
    """
    map = np.zeros((lenght,width), dtype=int)
    #add borders
    map[0, :] = WALL_CELL
    map[-1, :] = WALL_CELL
    map[:, 0] = WALL_CELL
    map[:, -1] = WALL_CELL

    # add dispenser randomly
    extra_dispenser = (n_dispenser) % n_types
    dispenser_to_place = (n_dispenser - extra_dispenser) / n_types
    while extra_dispenser > 0:
        random_type = random.randint(1, n_types)
        x = random.randint(1, lenght - 2)
        y = random.randint(1, width - 2)
        if (map[x, y] == EMPTY_CELL):
            map[x, y] = random_type
            extra_dispenser -= 1
    for i in range(n_types):
        placed = 0
        while placed < dispenser_to_place:
            x = random.randint(1, lenght - 2)
            y = random.randint(1, width - 2)
            if (map[x, y] == EMPTY_CELL):
                map[x, y] = i+1
                placed += 1

    # add walls randomly
    for i in range(1,lenght-2):
        for j in range(1, width -2):
            if map[i, j] == EMPTY_CELL:
                if random.uniform(0,1 ) < blockDensity:
                    map[i, j] = WALL_CELL

    return map

def sumCoordinates(a,b):
    return a[0]+b[0], a[1]+b[1]

def cellIsInMap(map, cell_coordinates):
    if (cell_coordinates[0] >= 0
        and cell_coordinates[0] < map.shape[0]
        and cell_coordinates[1] >= 0
        and cell_coordinates[1] < map.shape[1]):
        return True
    return False

def countNearUnknown(map, cell_coordinates):
    unknown_counter = 0
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for d in directions:
        neighbor_cell = sumCoordinates(cell_coordinates, d)
        if cellIsInMap(map, neighbor_cell):
            if map[neighbor_cell] == UNKNOWN_CELL:
                unknown_counter +=1
        else:
            unknown_counter += 1
    return unknown_counter

def cellCanBeUnknown(map,cell_coordinates):
    unknown_counter = countNearUnknown(map, cell_coordinates)
    if unknown_counter >= 2:
        return True
    else:
        return False

def getAgentPartialMap(map, lenght, width, unknown_percentage):

    leftCornerX = random.randint(0, map.shape[0] - lenght)
    leftCornerY = random.randint(0, map.shape[1] - width)

    partialMap = map[leftCornerX:leftCornerX+lenght, leftCornerY:leftCornerY+width].copy()
    print(partialMap.shape)
    #add unknown cells
    if (unknown_percentage > 0.8):
        print("unknown_percentage is too high, it must be between 0 and 0.8")
        return partialMap
    possible_unknowns = []
    for i in range(lenght):
        possible_unknowns.append((0, i))
        possible_unknowns.append((width-1, i))
    for i in range(width):
        possible_unknowns.append((i, 0))
        possible_unknowns.append((i, lenght-1))
    #print possible_unknowns
    unknown_placed = 0
    necessary_unknown = lenght*width*unknown_percentage
    while unknown_placed < necessary_unknown:
        # FOR DEBUG
        #print unknown_placed
        #print necessary_unknown
        #print "number of possibilities: "
        #print partialMap
        #print(len(possible_unknowns))
        # choose random coordinates in the list of the possible unknowns
        coordinates = possible_unknowns[random.randint(0, len(possible_unknowns)-1)]

        if cellCanBeUnknown(partialMap, coordinates):
            if partialMap[coordinates] != UNKNOWN_CELL: # set cell to unknown
                unknown_placed += 1
                partialMap[coordinates] = UNKNOWN_CELL
                if countNearUnknown(partialMap, coordinates) >= 2:
                    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
                    for d in directions:
                        neighbor_cell = sumCoordinates(coordinates, d)
                        #print("cell: " + coordinates.__str__())
                        #print("neighbor_cell: " + neighbor_cell.__str__())
                        if cellIsInMap(partialMap, neighbor_cell):
                            if partialMap[neighbor_cell] != UNKNOWN_CELL:
                                possible_unknowns.append(neighbor_cell) # add possible unkown to the list
            possible_unknowns.remove(coordinates)


    return partialMap

def saveMap(map, example_number, is_partial=False, partial_number=0):
    name = 'partial{}'.format(partial_number) if is_partial else 'map'
    np.savetxt('generatedMaps/{}/{}.csv'.format(example_number.__str__(), name)
               , map, fmt='%i', delimiter=',')

def showMap(map):
    cmap = mpl.colors.ListedColormap(['red', 'black', 'blue', 'white'])
    plt.matshow(map, cmap=cmap, vmin=-3, vmax=1)
    plt.show(block=False)


#first example
example_numbler = 1
lenght = 40
width = 40
map = generateMap(lenght, width, blockDensity=0.2, n_types=4, n_dispenser=6)
saveMap(map, example_numbler)
showMap(map)
#print map

for i in range(10):
    partial_lenght = 20
    partial_width = 20
    partialMap = getAgentPartialMap(map, partial_lenght, partial_width, 0.1)
    saveMap(partialMap, example_numbler, is_partial=True,partial_number=i)
    print("done")
    showMap(partialMap)



#to read from csv
map = np.loadtxt(open("generatedMaps/00/map.csv", "rb"), delimiter=",")
print(map)
data = input("Enter something to quit: ") # to leave the plots open
#print partialMap









