import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import rcParams

def mapMerge(m1, m2, lm1, lm2):
    # TODO PASS AND USE TOP_LEFT COORDINATES
    # TODO copy only the unknown in m2
    goal_landmark = 3

    
    top_rows_to_add = int(lm1[0] - lm2[0])
    if top_rows_to_add < 0:
        top_rows_to_add = 0


    bottom_rows_to_add = int(m1.shape[0] - lm1[0]) - int(m2.shape[0] - lm2[0])
    if bottom_rows_to_add < 0:
        bottom_rows_to_add = 0

    
    left_columns_to_add = int(lm1[1] - lm2[1])
    if left_columns_to_add < 0:
        left_columns_to_add = 0

    right_columns_to_add = int(m1.shape[0] - lm1[1]) - int(m2.shape[0] - lm2[1])
    if right_columns_to_add < 0:
        right_columns_to_add = 0
    
    print(str(top_rows_to_add))
    print(str(bottom_rows_to_add))
    print(str(left_columns_to_add))
    print(str(right_columns_to_add))


    fill_top = np.full((top_rows_to_add, m2.shape[1]),-1)
    m2 = np.r_[fill_top,m2]

    fill_bot = np.full((bottom_rows_to_add, m2.shape[1]), -1)
    m2 = np.r_[m2, fill_bot]

    fill_left = np.full((m2.shape[0], left_columns_to_add), -1)
    m2 = np.c_[fill_left, m2]

    fill_right = np.full((m2.shape[0], right_columns_to_add), -1)
    m2 = np.c_[m2, fill_right]

    #showSingleMap(m2)

    # get new coordinates of landmarks of m2
    lm2 = (lm2[0]+top_rows_to_add, lm2[1]+left_columns_to_add)

    print(lm1)
    print(lm2)
    shift = sum_tuple(lm2, lm1, minus=True)
    print("shift: " + str(shift))

    #showSingleMap(m2)
    #showSingleMap(m1)

    for i in range(m1.shape[0]):
        for j in range(m1.shape[1]):
            index_m2 = (i, j)
            cell_value = m1[index_m2]

            index_m2 = sum_tuple(index_m2, shift)
            if m2[index_m2] == -1:
                m2[index_m2] = cell_value

    #showSingleMap(m2)
    """
    if (top_rows_to_add == 0):
        i_m2_zeroval = int(pos_landmark_m2[0] - pos_landmark_m1[0])
    else: # added on the top
        i_m2_zeroval = int(pos_landmark_m2[0] - pos_landmark_m1[0]) + n_rows

    if (left_columns_to_add == 0):
        j_m2_zeroval = int(pos_landmark_m2[1] - pos_landmark_m1[1])
    else: # added on the left
        j_m2_zeroval = int(pos_landmark_m2[1] - pos_landmark_m1[1]) + n_columns
    
    

    i_m2 = i_m2_zeroval

    print(i_m2_zeroval)
    print(j_m2_zeroval)
    
    for i in range(0,m1.shape[0]):
        j_m2 = j_m2_zeroval

        for j in range(0,len(m1[0])):
            m2[i_m2,j_m2] = m1[i,j]
            j_m2 += 1
        
        i_m2 += 1

    print("---")
    print(str(m2))
    return m2
    """

def sum_tuple(a,b,minus=False):
    if not minus:
        return a[0] + b[0], a[1] + b[1]
    else:
        return a[0] - b[0], a[1] - b[1]

def showSingleMap(map):
    cmap = 'cool'#mpl.colors.ListedColormap(['grey','white', 'black', 'blue', 'red'])
    plt.matshow(map, cmap=cmap, vmin=-1, vmax=4)
    plt.show()

def showAllMaps(m1,m2,m2_m):
    rcParams['axes.titlepad'] = 20 
    cmap = mpl.colors.ListedColormap(['grey','white', 'black', 'blue', 'red'])

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

'''m1 = [ [0,0,0,0,1], 
	   [0,0,0,0,2],
       [0,0,0,0,3],
	   [0,7,1,0,0],
	   [0,1,0,4,0],
       [0,0,3,0,0],
       [0,1,2,3,4] ]

m2 = [ [0,0,0,0,0], 
	   [0,1,0,0,0],
	   [0,0,0,7,1],
	   [0,0,0,1,0],
	   [0,3,0,0,3] ]'''

'''
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

'''
m1 = np.loadtxt(open("agentA1.txt", "rb"), delimiter=",", dtype=int)
m2 = np.loadtxt(open("agentA2.txt", "rb"), delimiter=",", dtype=int)

m2_m = mapMerge(m1, m2, (8,23),(6,22))
'''

'''
m1 = np.array(m1)
m2 = np.array(m2)



showAllMaps(m1,m2,m2_m)
showSingleMap(m1)
showSingleMap(m2)

showSingleMap(m2_m)
'''