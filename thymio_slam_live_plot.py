#!/usr/local/bin/python3.6


# live_plot_thymio_and_obstacles.py


## Setting the current working directory automatically
import os
project_path = os.getcwd()  # getting the path leading to the current working directory
os.getcwd()  # printing the path leading to the current working directory
os.chdir(project_path)  # setting the current working directory based on the path leading to the current working directory
# Alternative way to get the path of the current working directory
#from pathlib import Path
#project_path = str(Path('live_plot_thymio_and_obstacles.py').parent.absolute())


## Required packages
import os
import sys
import time
import numpy as np
import matplotlib
from time import sleep
from time import perf_counter
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as patches
from matplotlib._png import read_png
from matplotlib.image import BboxImage
from matplotlib.transforms import TransformedBbox, Bbox
from matplotlib.legend_handler import HandlerBase
from matplotlib.offsetbox import OffsetImage, AnnotationBbox




# Defining the useful ImageHandler class for the legend
class ImageHandler(HandlerBase):
    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize,
                       trans):

        # enlarge the image by these margins
        sx, sy = self.image_stretch

        # create a bounding box to house the image
        bb = Bbox.from_bounds(xdescent - sx,
                              ydescent - sy,
                              width + sx,
                              height + sy)

        tbb = TransformedBbox(bb, trans)
        image = BboxImage(tbb)
        image.set_data(self.image_data)

        self.update_prop(image, orig_handle, legend)

        return [image]

    def set_image(self, image_path, image_stretch=(0, 0)):
        self.image_data = read_png(image_path)
        self.image_stretch = image_stretch





# Initializing the figure of square dimension 5x5
fig, ax = plt.subplots(figsize=(6,5))

# Organisation of the map
# Cf. Asciiflow Infinity (Ascii online editor to draw shapes in plain text file
# (ASCII/Unicode)), http://asciiflow.com
#
#   ------> x
#  |   +--------------+
#  |   |              |
#  v   |              |
#  y   |              |
#      |              |
#      |              |
#      +--------------+
#


# Initializing the text file (with the pose of the Thymio (location + orientation), no path and no obstacles)
x_thymio_init, y_thymio_init = 0, 0
orient_init = 'S'
list_of_lines =\
    ["-- Thymio" + "\n",\
    str(x_thymio_init)+','+str(y_thymio_init)+'\n',\
    orient_init + "\n",\
    "-- A* path" + "\n",\
    "-- Obstacles"]
a_file = open("thymio_slam_obstacles_coordinates.txt", "w")
a_file.writelines(list_of_lines)
a_file.close()






# Infinite loop
def animate(i):


    #------
    # Extracting data from the text file
    text_file_to_open = project_path+'/thymio_slam_obstacles_coordinates.txt'
    pullData = open(text_file_to_open,'r').read()
    dataArray = pullData.split('\n')
    # Getting the information about the Thymio
    x_thymio_str, y_thymio_str = dataArray[1].split(',')
    x_thymio, y_thymio = int(x_thymio_str), int(y_thymio_str)
    orient = dataArray[2]
    # Getting A* path coordinates
    dataArrayPath = dataArray[dataArray.index('-- A* path')+1 : dataArray.index('-- Obstacles')]
    xarPath = []
    yarPath = []
    for eachLine in dataArrayPath:
        if len(eachLine)>1:
            x, y = eachLine.split(',') # As classically: 1st coord. is "x", 2nd coord. is "y"
            xarPath.append(int(x))
            yarPath.append(int(y))
    # Getting the new obstacles to place on the map
    dataArrayObst = dataArray[dataArray.index('-- Obstacles')+1 : len(dataArray)]
    xarObst = []
    yarObst = []
    for eachLine in dataArrayObst:
        if len(eachLine)>1:
            x, y = eachLine.split(',') # As classically: 1st coord. is "x", 2nd coord. is "y"
            xarObst.append(int(x))
            yarObst.append(int(y))

    # First, we completely clear the previous version of "ax"
    ax.clear()

    # Scattering the A* path
    size = [item*300/len(xarPath) for item in xarPath] # For scaling up the dots along 'x' until the goal location
    path = ax.scatter(xarPath, yarPath, s=size, c='red', marker='o', edgecolors='blue')
    # Scattering the obstacles
    obst = ax.scatter(xarObst, yarObst, s=500, c='yellow', marker='s', edgecolors='green')
    #------

    # axes limits
    ax.set_xlim([-2, 8])
    ax.set_ylim([-2, 6])

    # reversing the built-in defined y-axis of matplotlib
    plt.gca().set_ylim(ax.get_ylim()[::-1])

    # setting up a grid
    # major grid
    plt.grid(color='k', which='major', linestyle='-', linewidth=0.2)
    # minor grid
    plt.minorticks_on()
    plt.grid(which='minor', linestyle=':', linewidth='0.1', color='red')

    # setting the title of the figure
    fig.suptitle('Map')

    # drawing the arena
    begin_on_x_axis = -1
    begin_on_y_axis = -1
    W = 6
    H = 6
    rect = patches.Rectangle((begin_on_x_axis, begin_on_y_axis), W, H, linewidth=3,
                             edgecolor='k', facecolor="none", fill=False)
    ax.add_patch(rect)

    # Adding the start and end flags
    arr_flag_start = plt.imread('flag_start.png')
    imagebox_flag_start = OffsetImage(arr_flag_start, zoom=0.15)
    ab_f_s = AnnotationBbox(imagebox_flag_start, (0, 0),
                        bboxprops=dict(edgecolor='white'))  # x axis, y axis
    ax.add_artist(ab_f_s)
    arr_flag_end = plt.imread('flag_end.png')
    imagebox_flag_end = OffsetImage(arr_flag_end, zoom=0.15)
    ab_f_e = AnnotationBbox(imagebox_flag_end, (4, 4),
                        bboxprops=dict(edgecolor='white'))  # x axis, y axis
    ax.add_artist(ab_f_e)

    # Adding the Thymio
    # Initial position on the top left-hand corner
    if orient == 'N': # North
        arr_thymio = plt.imread('thymio_north.png')
    elif orient == 'S': # South
        arr_thymio = plt.imread('thymio_south.png')
    elif orient == 'E': # East
        arr_thymio = plt.imread('thymio_east.png')
    else: # == if orient == 'W' # West
        arr_thymio = plt.imread('thymio_west.png')
    imagebox = OffsetImage(arr_thymio, zoom=0.17)
    ab = AnnotationBbox(imagebox, (x_thymio, y_thymio),
                        bboxprops=dict(edgecolor='white'))  # x axis, y axis
    ax.add_artist(ab)

    # defining the labels of the axes
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Defining the legend by inserting custom images
    # (cf.: https://stackoverflow.com/questions/26029592/insert-image-in-matplotlib-legend)
    # 1. Setup the handler instances for the scattered data
    custom_handler_obstacles = ImageHandler()
    custom_handler_obstacles.set_image("obstacles.png",
                             image_stretch=(0, 12))
    custom_handler_path = ImageHandler()
    custom_handler_path.set_image("A*path.png",
                                       image_stretch=(0, 12))
    custom_handler_thymio = ImageHandler()
    custom_handler_thymio.set_image("thymio_north.png",
                                       image_stretch=(0, 10))
    # 2. Add the legend for the scattered data, mapping the
    # scattered points to the custom handler
    plt.legend([obst, path, ab],
               ['Obstacles', 'A* path', 'Thymio'],
               handler_map={obst: custom_handler_obstacles, path: custom_handler_path, ab: custom_handler_thymio},
               labelspacing=2,
               frameon=True)




ani = anim.FuncAnimation(fig, animate, interval = 10)
plt.show()


