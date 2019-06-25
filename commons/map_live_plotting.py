"""Live Plotting Functionalities.
See https://matplotlib.org/3.1.0/gallery/images_contours_and_fields/multi_image.html#sphx-glr-gallery-images-contours-and-fields-multi-image-py for details on plotting multiple images
"""

from matplotlib import colors
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
import glob

from helpers import get_data_location


def cleanup(agent_name):
    """Deletes old map files

    Args:
        agent_name (str): name of agent, e.g. agentA1
    """
    data_path = get_data_location()
    file_path = os.path.join(data_path,'generatedMaps/tmp_maps/{}.txt'.format(agent_name))
    if os.path.isfile(file_path):
        os.remove(file_path)


def load_map_data():
    """loads current map data

    Returns:
        dict: agent names as keys, map data ( 2-dim np.array as values )
    """
    map_data ={}
    data_path = get_data_location()
    file_paths = glob.glob(os.path.join(data_path,"generatedMaps/tmp_maps/*.txt"))
    for file in file_paths:
        agent_name = file[file.rindex('/')+1:].split('.')[0]
        '''
        if agent_name in ('agentA1', 'agentA2'):
            print(agent_name)
            agent_map = np.loadtxt(open(file, "rb"), delimiter=",", dtype=int)
            map_data.update({agent_name: agent_map})
        '''
        agent_map = np.loadtxt(open(file, "rb"), delimiter=",", dtype=int)
        map_data.update({agent_name: agent_map})
    return map_data


def live_plotting():
    """

    Returns:

    """

    map_data = load_map_data()
    no_agents = len(map_data.keys())

    n_rows = 1
    n_columns = no_agents

    fig, axs = plt.subplots(n_rows, n_columns)
    cmap = 'cool'

    # initialize maps
    images = []
    if no_agents == 1:
        images.append(axs.imshow(map_data.values()[0], cmap=cmap))
        axs.set_title(map_data.keys()[0])
    else:
        i = 0
        for agent, data in map_data.iteritems():
            try:
                images.append(axs[i].imshow(data, cmap=cmap))
            except TypeError:
                continue
            axs[i].set_title(agent)
            # axs[i].label_counter()
            i += 1
    print(images)
    # Find the min and max of all colors for use in setting the color scale.
    vmin = min(image.get_array().min() for image in images)
    vmax = max(image.get_array().max() for image in images)
    norm = colors.Normalize(vmin=vmin, vmax=vmax)
    for im in images:
        im.set_norm(norm)

    def animate(j):
        map_data = load_map_data()

        if no_agents == 1:
            images[0] = axs.imshow(map_data.values()[0], cmap=cmap, vmin=vmin, vmax=vmax)
        else:
            for index, image in enumerate(images):
                try:
                    images[index] = axs[index].imshow(map_data.values()[index], cmap=cmap, vmin=vmin, vmax=vmax)
                except TypeError:
                    # empty np array was loaded, probably because it was written to it at that moment
                    continue


    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

if __name__ == '__main__':
    live_plotting()




# Nr = 1
# Nc = 1
# fig, axs = plt.subplots()
# cmap = 'cool'
#
#
# graph_data = np.loadtxt(open("generatedMaps/tmp_maps/map.txt", "rb"), delimiter=",", dtype=int)
# # graph_data = ((1 + 1 + 1) / 10) * np.random.rand(10, 20) * 1e-6
#
# images = []
#
# if not isinstance(axs, list):
#     images.append(axs.imshow(graph_data, cmap=cmap))
#     # axs.label_counter()
# else:
#     for i in range(Nr):
#         for j in range(Nc):
#             images.append(axs[i, j].matshow(graph_data, cmap=cmap))
#             axs[i, j].label_counter
#
# # Find the min and max of all colors for use in setting the color scale.
# vmin = min(image.get_array().min() for image in images)
# vmax = max(image.get_array().max() for image in images)
# norm = colors.Normalize(vmin=vmin, vmax=vmax)
# for im in images:
#     im.set_norm(norm)
#
# print(vmin, vmax)
#
#
#
# def animate(i):
#     print(i)
#     graph_data = np.loadtxt(open("generatedMaps/tmp_maps/map.txt", "rb"), delimiter=",", dtype=int)
#     print(graph_data)
#     for index, image in enumerate(images):
#         images[index] = axs.imshow(graph_data, cmap=cmap,vmin=vmin,vmax=vmax)
#
# ani = animation.FuncAnimation(fig, animate, interval=1000)
# plt.show()