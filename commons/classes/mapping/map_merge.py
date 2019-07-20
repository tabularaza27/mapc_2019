import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import rcParams
import rospy


def mapMerge(external_map, my_map, external_land_mark, my_land_mark, my_origin):
    """
    merges the external_map and my_map and returns the merged_map + the updated origin
    Args:
        external_map(np.array): the map of the other agent
        my_map(np.array): my map
        external_land_mark(np.array): the top left of the goal area of external_map
        my_land_mark(np.array): the top left of the goal area of my_map
        my_origin(np.array): origin coordinate in matrix my_map

    Returns:
        tuple:(merged_map, new_origin)
    """
    # TODO PASS AND USE TOP_LEFT COORDINATES
    # TODO copy only the unknown in m2
    # goal_landmark = 3

    # Check rows and columns added in the merge map
    top_rows_to_add = int(external_land_mark[0] - my_land_mark[0])
    if top_rows_to_add < 0:
        top_rows_to_add = 0

    bottom_rows_to_add = int(external_map.shape[0] - external_land_mark[0]) - int(my_map.shape[0] - my_land_mark[0])
    if bottom_rows_to_add < 0:
        bottom_rows_to_add = 0

    left_columns_to_add = int(external_land_mark[1] - my_land_mark[1])
    if left_columns_to_add < 0:
        left_columns_to_add = 0

    right_columns_to_add = int(external_map.shape[1] - external_land_mark[1]) - int(my_map.shape[1] - my_land_mark[1])
    if right_columns_to_add < 0:
        right_columns_to_add = 0

    # Fill extra columns and rows with -1
    fill_top = np.full((top_rows_to_add, my_map.shape[1]), -1)
    my_map = np.r_[fill_top, my_map]

    fill_bot = np.full((bottom_rows_to_add, my_map.shape[1]), -1)
    my_map = np.r_[my_map, fill_bot]

    fill_left = np.full((my_map.shape[0], left_columns_to_add), -1)
    my_map = np.c_[fill_left, my_map]

    fill_right = np.full((my_map.shape[0], right_columns_to_add), -1)
    my_map = np.c_[my_map, fill_right]

    # showSingleMap(m2)

    # get new coordinates of landmarks of m2
    my_land_mark = (my_land_mark[0] + top_rows_to_add, my_land_mark[1] + left_columns_to_add)

    # get new coordinates of origin
    new_origin = np.array([my_origin[0] + top_rows_to_add, my_origin[1] + left_columns_to_add])

    # print(lm1)
    # print(lm2)
    overlap_shift = my_land_mark - external_land_mark
    # print("overlap_shift: " + str(overlap_shift))
    # print ("SHAAAAPEEEEEEEEEEEEE M1!!!!!!!!!!!!!!!!!!!!!" + str(m1.shape))
    # print ("SHAAAAPEEEEEEEEEEEEE M2!!!!!!!!!!!!!!!!!!!!!" + str(m2.shape))

    # showSingleMap(m2)
    # showSingleMap(m1)

    for i in range(external_map.shape[0]):
        for j in range(external_map.shape[1]):
            cell_value = external_map[i, j]

            if my_map[i + overlap_shift[0], j + overlap_shift[1]] == -1:
                my_map[i + overlap_shift[0], j + overlap_shift[1]] = cell_value

    return my_map, new_origin
    # showSingleMap(m2)


def showSingleMap(map):
    cmap = 'cool'  # mpl.colors.ListedColormap(['grey','white', 'black', 'blue', 'red'])
    plt.matshow(map, cmap=cmap, vmin=-1, vmax=4)
    plt.show()


def showAllMaps(m1, m2, m2_m):
    rcParams['axes.titlepad'] = 20
    cmap = mpl.colors.ListedColormap(['grey', 'white', 'black', 'blue', 'red'])

    ax1 = plt.subplot(221)
    ax1.matshow(m1, cmap=cmap, vmin=-1, vmax=4)
    ax1.set_title("Map 1")

    ax2 = plt.subplot(222)
    ax2.matshow(m2, cmap=cmap, vmin=-1, vmax=4)
    ax2.set_title("Map 2")

    ax3 = plt.subplot(212)
    ax3.matshow(m2_m, cmap=cmap, vmin=-1, vmax=4)
    ax3.set_title("Merged Map")

    plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.5)

    plt.show()


def main():
    '''run for local testing

    m1 = [ [0,0,0,0],
           [0,4,0,0],
           [0,1,1,1],
           [0,0,0,0] ]
    
    m2 = [ [1,0,1,0,0],
           [1,1,1,0,0],
           [0,0,0,0,4],
           [0,1,0,0,1],
           [0,1,1,0,0] ]
    '''

    m1 = np.loadtxt(open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/map_merge/agentA1.txt", "rb"),
                    delimiter=",", dtype=int)
    m2 = np.loadtxt(open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/map_merge/agentA2.txt", "rb"),
                    delimiter=",", dtype=int)

    m2_m = mapMerge(m1, m2, (8, 23), (6, 22))

    m1 = np.array(m1)
    m2 = np.array(m2)

    showAllMaps(m1, m2, m2_m)
    showSingleMap(m1)
    showSingleMap(m2)

    showSingleMap(m2_m)


if __name__ == '__main__':
    main()
