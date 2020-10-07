#!/usr/local/bin/python3.6


# thymio_slam.py


# /!\ To make this code run on the Thymio, first stop all other processes and
# either run part by part in the corresponding Jupyter Notebook or run this
# whole file thymio_slam.py by going to
# Run > Run 'thymio_slam'



#******************************************
# Conditions of working:
# - Start position: (ys,xs) = (0,0), orient = 'S'
# - The box (0,0) has to be free (start position)
# - The box (4,4) has to be free as well (end position, i.e. goal)
# - No obstacle has to be placed on the boundary of the arena
#******************************************



## Setting the current working directory automatically
import os
from typing import List

project_path = os.getcwd()  # getting the path leading to the current working directory
os.getcwd()  # printing the path leading to the current working directory
os.chdir(project_path)  # setting the current working directory based on the path leading to the current working directory


## Required packages
import os
import sys
import time
import pyglet
import numpy as np
from time import sleep
from time import perf_counter
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as patches
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import serial
from thymio_slam_path_planning import astar, computePath


# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

from Thymio import Thymio

print(sys.executable)
# /Library/Frameworks/Python.framework/Versions/3.7/bin/python3.6
# /usr/local/bin/python3.6


## 0. Drawing the map by calling live_plot_thymio_and_obstacles.py
# 1st try
#os.system("python3.6 live_plot_thymio_and_obstacles.py") # This works but the execution of both programs from the same shell seems to block the following of the code from this point
# 2nd try
#exec(open('live_plot_thymio_and_obstacles.py').read())
# 3rd try
# import subprocess
# subprocess.call('python3.6 live_plot_thymio_and_obstacles.py', shell=True)
# 4th try
import os
os.system("open /Applications/iTerm.app")
from pynput.keyboard import Key, Controller
keyboard = Controller()
# 1) Moving to the project directory
keyboard.type('cd' + ' ' + project_path)
sleep(0.1)
keyboard.press(Key.enter)
# 2) Executing the python live plotting file
file_to_run = 'thymio_slam_live_plot.py'
keyboard.type('python3.6' + ' ' + file_to_run)
sleep(0.1)
keyboard.press(Key.enter)


## 1. Connecting the Thymio
# To connect to the Thymio, you first have to start by connecting it to a USB
# port. Then you will need to identify the ID of the serial port it is connected
# to using a specific terminal command. Open terminal and write:
#
# ls /dev/cu.usb*
#
# Or write:
#
# ls /dev/cu.usbmodem*
#
# To get the serial portt
#
# Look for cu.usbmodemXXXX, where XXXX is the number attributed by your
# computer. You should find one ID.
# Note : Virtual serial port numbering on Mac depends on the physical USB port
# used and the device. If you want to keep the same names, you must connect to
# the SAME USB port each time.
# (/!\/!\/!\ For me it was the top left USB port when facing my computer /!\/!\/!\)
th = Thymio.serial(port="/dev/cu.usbmodem14301", refreshing_rate=0.1)

## 2. Constants
# Ints and Doubles
xs, ys = 0, 0                  # initial position of Thymio for the current set of commands
xe, ye = 4, 4                  # ⚠⚠⚠ final goal position for the Thymio ⚠⚠⚠
x_thymio, y_thymio = 0, 0      # current position of Thymio
x_new_obst, y_new_obst = 0, 0  # initialization of the position of the first new obstacle
orient = 'S'
iter_in_commands = 0
pauseDuration = 1
pauseDurationAfterComputingNewPath = 1.5
#------ go_forward
time_go_forward = 4.7  # [s]
left_adjustment_go_forward = 3
right_adjustment_go_forward = 5
#------ turn_left
time_turn_left = 2.37  # [s] 2.5514, 2.3514, 2.4
#------ turn_right
time_turn_right = 2.33  # [s] 2.3459, 2.2459, 2.3, 2.31
#------ turn_back
time_turn_back = time_turn_right*2 # [s]
#------
prox_intensity: int = 0
prox_intensity_threshold = 2000

# Booleans
access_go_forward = False
first_time_entering_go_forward = True
access_turn_left = False
first_time_entering_turn_left = True
access_turn_right = False
first_time_entering_turn_right = True
access_turn_back = False
first_time_entering_turn_back = True

# Strings
list_of_commands: List[str]
list_orient = [orient]

# Sound
sound = pyglet.media.load("sound.wav", streaming=False)

# Matrix
map = [[0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0]]


## 3. Functions
def stepForward():
    # Setting the left and right wheel to a certain value...
    th.set_var("motor.left.target", 100 + left_adjustment_go_forward)
    th.set_var("motor.right.target", 100 + right_adjustment_go_forward)


def turnLeft90():
    th.set_var("motor.left.target", 2**16-100)
    th.set_var("motor.right.target", 100)


def turnRight90():
    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 2**16-100)


def turnBack180():
    th.set_var("motor.left.target", 100)
    th.set_var("motor.right.target", 2**16-100)


def stopMotors():
    # Stopping both motors
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)


def updateOrientInGraph(orient):
    # Cf. "changing specific lines in a text file": https://www.kite.com/python/answers/how-to-edit-a-specific-line-in-a-text-file-in-python
    # Reading in the text file
    a_file = open("thymio_slam_obstacles_coordinates.txt", "r")
    list_of_lines = a_file.readlines()
    # Updating the line with the orientation of the robot (i.e. the 3rd line of the text file):
    list_of_lines[2] = orient+"\n"
    # Writing in the text file
    a_file = open("thymio_slam_obstacles_coordinates.txt", "w")
    a_file.writelines(list_of_lines)
    # Closing the text file
    a_file.close()


def updatePosInGraph(x_thymio, y_thymio):
    # Reading in the text file
    a_file = open("thymio_slam_obstacles_coordinates.txt", "r")
    list_of_lines = a_file.readlines()
    # Updating the line with the position of the robot:
    list_of_lines[1] = str(x_thymio) + ',' + str(y_thymio) + '\n'
    # Writing in the text file
    a_file = open("thymio_slam_obstacles_coordinates.txt", "w")
    a_file.writelines(list_of_lines)
    # Closing the text file
    a_file.close()


def drawPath(path):
    # Reading in the text file
    a_file = open("thymio_slam_obstacles_coordinates.txt", "r")
    # Getting the lines in the text file
    list_of_lines = a_file.readlines()
    # Getting the beginning and and location for inserting the path coordinates
    # - First line
    line_begin = list_of_lines.index("-- A* path\n")
    # - Last line
    for i in range(len(list_of_lines)):
        if 'Obstacles' in list_of_lines[i]:
            line_end = i
            break
    # Defining the different parts of the text file
    # - First lines
    list_of_lines_begin = list_of_lines[0:line_begin+1]
    # - Middle lines (coordinates of the A* path)
    list_of_lines_path_coord = []
    for i in range(len(path)):
        list_of_lines_path_coord.append(str(path[i][1])+','+str(path[i][0])+'\n')
    # - Last lines
    list_of_lines_end = list_of_lines[line_end:]
    # Recomposing the list of lines
    list_of_lines_new = list_of_lines_begin
    list_of_lines_new.extend(list_of_lines_path_coord)
    list_of_lines_new.extend(list_of_lines_end)
    # Writing in the text file
    a_file = open("thymio_slam_obstacles_coordinates.txt", "w")
    a_file.writelines(list_of_lines_new)
    # Closing the text file
    a_file.close()


## 4. Passing single values to the Thymio
# Computing the A* path for the Thymio to reach the goal
path = astar(map, (ys, xs), (ye, xe))
# Drawing the initial A* path on the map
sleep(pauseDurationAfterComputingNewPath)
drawPath(path)
# Computing the initial set of commands for the Thymio
# (i.e. from the start position to the goal position)
list_of_commands, _ = computePath(ys, xs, ye, xe, orient, map)

time.sleep(2)  # Sleeping a little bit before sending the first commands

# For testing the correct displacements of the robot w.r.t. the animated graph:
#list_of_commands = ['turn_right','turn_left','go_forward','turn_right','go_forward','turn_left']
#list_of_commands = ['go_forward']

while True:

    # Determining accesses
    if list_of_commands[
        iter_in_commands] == 'go_forward' and prox_intensity < prox_intensity_threshold:
        access_go_forward = True
    elif list_of_commands[
        iter_in_commands] == 'turn_left' and prox_intensity < prox_intensity_threshold:
        access_turn_left = True
    elif list_of_commands[
        iter_in_commands] == 'turn_right' and prox_intensity < prox_intensity_threshold:
        access_turn_right = True
    elif list_of_commands[
        iter_in_commands] == 'turn_back' and prox_intensity < prox_intensity_threshold:
        access_turn_back = True

    #⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠
    # else: OBSTACLE DETECTED in front (the intensity is so high that it means that an obstacle is situated in front of the robot)
    #⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠
    else:  # meaning that "prox_intensity > prox_intensity_threshold"
        # We stop the motors while the obstacle as long as the obstacle obstructs the path
        stopMotors()
        access_go_forward = False
        access_turn_left =  False
        access_turn_right = False
        access_turn_back =  False
        print("\n⚠️ Obstacle detected ⚠️")
        # Adding the obstacle to the graph (updating the map)
        if orient == 'N':
            x_new_obst, y_new_obst = x_thymio, y_thymio - 1
        elif orient == 'S':
            x_new_obst, y_new_obst = x_thymio, y_thymio + 1
        elif orient == 'E':
            x_new_obst, y_new_obst = x_thymio + 1, y_thymio
        else:  # == if orient == 'W'
            x_new_obst, y_new_obst = x_thymio - 1, y_thymio
        # Appending the new obstacle values at the very end of the text file
        text_file = open("thymio_slam_obstacles_coordinates.txt","a") # "a" is the append mode ("w" is the write mode and overwrites everything in the text file)
        text_file.write("\n"+str(x_new_obst)+','+str(y_new_obst))
        text_file.close()
        # Adding the obstacles to the map
        map[y_new_obst][x_new_obst] = 1
        # Playing a sound
        sound.play()
        # Printing the current state of the map
        for i in range(len(map)):
            print(map[i])
        # Computing a new path to get to the goal
        print("\nComputing new path...")
        path = astar(map, (y_thymio, x_thymio), (ye, xe))
        sleep(pauseDurationAfterComputingNewPath)
        drawPath(path)
        iter_in_commands = 0
        # Computing the new set of commands
        list_of_commands, _ = computePath(y_thymio, x_thymio, ye, xe, orient, map)
        print('New list of commands:')
        print(list_of_commands)
        # Resetting the measurement of the proximity sensor (in order not to get stopped directly)
        prox_intensity = 0
        # Short pause before performing the next command
        sleep(pauseDuration)



    #-----------------------------------------
    # Executing the current command
    #-----------------------------------------

    # Accessing the "stepForward()" method
    if access_go_forward:

        # FIRST time in "stepForward"
        if first_time_entering_go_forward:
            # Triggering the time counter
            t_start = perf_counter()  # [s]
            first_time_entering_go_forward = False

        stepForward()
        t_current = perf_counter()  # [s]
        t_diff = t_current - t_start  # [s]
        # /!\ With this logic of discrete movements (i.e. on a grid with "fixed" displacement units), the thymio CAN NOT sense obstacle while it is executing the command of a movement!! (I.e. in the middle of a movement, a displacement)

        # LAST time in "stepForward"
        if (t_diff > time_go_forward):
            stopMotors()
            # Updating the x and y of the robot on the graph depending on the orientation of the robot
            if orient == 'N': # North
                y_thymio -= 1
            elif orient == 'S': # South
                y_thymio += 1
            elif orient == 'E': # East
                x_thymio += 1
            else: # == if orient == 'W' # West
                x_thymio -= 1
            # Updating the location of the Thymio on the graph (updating the map by changing the text file)
            # (The orientation has not to be uptadated since, by moving forward, the robot conserves the same orientation)
            updatePosInGraph(x_thymio, y_thymio)

            # Short pause before performing the next command
            sleep(pauseDuration)
            print(("\niter_in_commands n°{0}. Went FORWARD a bit and then stopped for 'pauseDuration'[s]").format(iter_in_commands))
            access_go_forward = False
            first_time_entering_go_forward = True
            iter_in_commands += 1
            # Checking the value of the middle front proximity IR sensor
            prox_intensity = th["prox.horizontal"][3]
            print("Intensity of the proximity sensor:", prox_intensity)


    # Accessing the "turnLeft90()" method
    elif access_turn_left:

        # FIRST time in "turnLeft90"
        if first_time_entering_turn_left:
            # Triggering the time counter
            t_start = perf_counter() # [s]
            first_time_entering_turn_left = False

        turnLeft90()
        t_current = perf_counter() # [s]
        t_diff = t_current - t_start # [s]

        # LAST time in "turnLeft90"
        if (t_diff > time_turn_left):
            stopMotors()
            # Updating the orientation of the robot on the graph
            if orient == 'N': # North
                orient = 'W'
            elif orient == 'S':
                orient = 'E'
            elif orient == 'E':
                orient = 'N'
            else: # == if orient == 'W' # West
                orient = 'S'
            # Updating the orientation of the Thymio on the graph (updating the map by changing the text file)
            updateOrientInGraph(orient)

            # Short pause before performing the next command
            sleep(pauseDuration)
            print(("\niter_in_commands n°{0}. Turned LEFT by 90° and then stopped for 'pauseDuration'[s]").format(iter_in_commands))
            access_turn_left = False
            first_time_entering_turn_left = True
            iter_in_commands += 1
            # Checking the value of the middle front proximity IR sensor
            prox_intensity = th["prox.horizontal"][3]
            print("Intensity of the proximity sensor:", prox_intensity)


    # Accessing the "turnRight90()" method
    elif access_turn_right:

        # FIRST time in "turnRight90"
        if first_time_entering_turn_right:
            # Triggering the time counter
            t_start = perf_counter()  # [s]
            first_time_entering_turn_right = False

        turnRight90()
        t_current = perf_counter()  # [s]
        t_diff = t_current - t_start  # [s]

        # LAST time in "turnRight90"
        if (t_diff > time_turn_right):
            stopMotors()
            # Updating the orientation of the robot on the graph
            if orient == 'N':  # North
                orient = 'E'
            elif orient == 'S':
                orient = 'W'
            elif orient == 'E':
                orient = 'S'
            else:  # == if orient == 'W' # West
                orient = 'N'
            # Updating the orientation of the Thymio on the graph (updating the map by changing the text file)
            updateOrientInGraph(orient)

            # Short pause before performing the next command
            sleep(pauseDuration)
            print(("\niter_in_commands n°{0}. Turned RIGHT by 90° and then stopped for 'pauseDuration'[s]").format(iter_in_commands))
            access_turn_right = False
            first_time_entering_turn_right = True
            iter_in_commands += 1
            # Checking the value of the middle front proximity IR sensor
            prox_intensity = th["prox.horizontal"][3]
            print("Intensity of the proximity sensor:", prox_intensity)


    # Accessing the "turnBack180()" method
    elif access_turn_back:

        # FIRST time in "turnBack180"
        if first_time_entering_turn_back:
            # Triggering the time counter
            t_start = perf_counter()  # [s]
            first_time_entering_turn_back = False

        turnBack180()
        t_current = perf_counter()  # [s]
        t_diff = t_current - t_start  # [s]

        # LAST time in "turnBack180"
        if (t_diff > time_turn_back):
            stopMotors()
            # Updating the orientation of the robot on the graph
            if orient == 'N':  # North
                orient = 'S'
            elif orient == 'S':
                orient = 'N'
            elif orient == 'E':
                orient = 'W'
            else:  # == if orient == 'W' # West
                orient = 'E'
            # Updating the orientation of the Thymio on the graph (updating the map by changing the text file)
            updateOrientInGraph(orient)

            # Short pause before performing the next command
            sleep(pauseDuration)
            print(("\niter_in_commands n°{0}. Turned BACK by 90° and then stopped for 'pauseDuration'[s]").format(iter_in_commands))
            access_turn_back = False
            first_time_entering_turn_back = True
            iter_in_commands += 1
            # Checking the value of the middle front proximity IR sensor
            prox_intensity = th["prox.horizontal"][3]
            print("Intensity of the proximity sensor:", prox_intensity)


    # Stopping the program when the final location has been reached (the location (4,4))
    if (x_thymio, y_thymio) == (4, 4):
        print("\n------\nGoal reached and obstacles on the path "
              "completely detected."
              "\nProgram finished!")
        break

