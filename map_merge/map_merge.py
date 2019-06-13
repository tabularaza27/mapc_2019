import numpy
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import rcParams

def mapMerge(m1, m2):
    goal_landmark = 3

    # these should be saved in a variable during exploration to avoid computational waste
    pos_landmark_m1 = numpy.where(m1 == goal_landmark)
    pos_landmark_m2 = numpy.where(m2 == goal_landmark)

    print(pos_landmark_m1)

    '''

    # calculate rows to add
    
    top_rows_to_add = int(pos_landmark_m1[0] - pos_landmark_m2[0]) 
    if top_rows_to_add < 0:
        top_rows_to_add = 0
        
    bottom_rows_to_add = int(len(m1) - pos_landmark_m1[0]) - int(len(m2) - pos_landmark_m2[0])
    if bottom_rows_to_add < 0:
        bottom_rows_to_add = 0

    left_columns_to_add = int(pos_landmark_m1[1] - pos_landmark_m2[1])
    if left_columns_to_add < 0:
        left_columns_to_add = 0

    right_columns_to_add = int(len(m1) - pos_landmark_m1[1]) - int(len(m2) - pos_landmark_m2[1])
    if right_columns_to_add < 0:
        right_columns_to_add = 0
    
    print(str(top_rows_to_add))
    print(str(bottom_rows_to_add))
    print(str(left_columns_to_add))
    print(str(right_columns_to_add))

    n_rows = top_rows_to_add if top_rows_to_add > 0 else bottom_rows_to_add
    n_columns = left_columns_to_add if left_columns_to_add > 0 else right_columns_to_add

    if (n_rows != 0 or n_columns != 0):
        print("---")
        print(str(n_rows))
        print(str(n_columns))

        fill_c = numpy.full((len(m2),n_columns),-1)
        m2 = numpy.c_[fill_c,m2] if right_columns_to_add == 0 else numpy.c_[m2,fill_c]
        
        fill_r = numpy.full((n_rows,len(m2[0])),-1)
        m2 = numpy.r_[m2,fill_r] if top_rows_to_add == 0 else numpy.r_[fill_r,m2]
        print(m2)
        
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
        
        for i in range(0,len(m1)):
            j_m2 = j_m2_zeroval

            for j in range(0,len(m1[0])):
                m2[i_m2,j_m2] = m1[i,j]
                j_m2 += 1
            
            i_m2 += 1

        print("---")
        print(str(m2))
        return m2
    else:
        print("eh no dc")
    
    '''

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

m1 = numpy.loadtxt(open("agentA1.txt", "rb"), delimiter=",", dtype=int)
m2 = numpy.loadtxt(open("agentA2.txt", "rb"), delimiter=",", dtype=int)

m2_m = mapMerge(m1,m2)

'''
m1 = numpy.array(m1)
m2 = numpy.array(m2)



showAllMaps(m1,m2,m2_m)
showSingleMap(m1)
showSingleMap(m2)

showSingleMap(m2_m)
'''