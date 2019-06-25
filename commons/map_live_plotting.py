"""Live Plotting Functionalities.
See https://matplotlib.org/3.1.0/gallery/images_contours_and_fields/multi_image.html#sphx-glr-gallery-images-contours-and-fields-multi-image-py for details on plotting multiple images
"""
from __future__ import division

from matplotlib import colors
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
import glob

from helpers import get_data_location


def cleanup():
    """Deletes old map files"""
    data_path = get_data_location()
    file_paths = glob.glob(os.path.join(data_path,"generatedMaps/tmp_maps/*.txt"))
    for file_path in file_paths:
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
        agent_map = np.loadtxt(open(file, "rb"), delimiter=",", dtype=int)
        map_data.update({agent_name: agent_map})

    return map_data


def live_plotting(no_agents=6):
    """Plots Maps of Agents from data/generatedMaps/tmp_maps and updates them automatically

    Args:
        no_agents (int): number of agents to be plotted
    """

    map_data = load_map_data()
    if not no_agents:
        no_agents = len(map_data.keys())

    n_columns = 2
    n_rows = int(np.ceil(no_agents / n_columns))
    fig, axs = plt.subplots(n_rows, n_columns)

    cmap = 'cool'

    # initialize maps
    images = []
    if no_agents == 1:
        images.append(axs.imshow(map_data.values()[0], cmap=cmap))
        axs.set_title(map_data.keys()[0])
    else:
        i = 0
        j = 0
        k = 1
        for agent, data in map_data.iteritems():
            if k > no_agents:
                break
            try:
                images.append(axs[i][j].imshow(data, cmap=cmap))

                axs[i][j].set_title(agent)
                # axs[i].label_counter()
                if j == 0:
                    j += 1
                else:
                    i += 1
                    j = 0
                k += 1
            except TypeError:
                continue

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
            i = 0
            j = 0
            for index, image in enumerate(images):
                try:
                    images[index] = axs[i][j].imshow(map_data.values()[index], cmap=cmap, vmin=vmin, vmax=vmax)

                    if j == 0:
                        j += 1
                    else:
                        i += 1
                        j = 0
                except TypeError:
                    # empty np array was loaded, probably because it was written to it at that moment
                    continue


    ani = animation.FuncAnimation(fig, animate, interval=2000)
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