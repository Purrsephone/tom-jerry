#!/usr/bin/env python3

# necessary libraries
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from itertools import zip_longest
from tom_and_jerry_project.msg import QLearningReward, RobotCoord, GameState
from fake_data import * 
import copy 
from itertools import product

# Given an index and info about map, compute its real coordinate
# returns an array [x, y]
def convert_to_real_coords(indx, height, orx, ory, res):

    # Convert the x and y indexes from row-major order
    x_val = indx % height
    y_val = math.floor(indx/height)

    # Scale our x and y indexes to the size and resolution of our map
    x_coord = orx + (x_val * res)
    y_coord = ory + (y_val * res)

    # Create a new coordinate
    coords = [x_coord, y_coord]

    return(coords)

# define a position class which is composed of an x, y coordinate
# pair plus an orientation (0, 1, 2, or 3)
# let 0 = North, 1 = East, 2 = South, 3 = West
class Position:
    def __init__(self, x, y, z):
        self.x = x #float
        self.y = y #float
        self.z = z #int 1,2,3,4

# define a state class which includes the mouse position and the cat position
# both are of the Position class
class State:
    def __init__(self, catpos, mousepos):
        self.catpos = catpos
        self.mousepos = mousepos

# define our grid, which corresponds to our states
class Grid:

    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False

        # initialize the node
        rospy.init_node('state')

        # side length of squares for states, 1 for now, can change
        self.square_side_len = 60

        # set the topic names and frame names
        self.map_topic = "map"

        # inialize our map
        self.map = OccupancyGrid()

        # initialize squares
        self.squares = []

        # initialize states
        self.states = []

        # initialize actions
        self.actions = []

        # initialze action matrix
        self.action_matrix = []

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_grid)
        
        # publish states 
        self.state_pub = rospy.Publisher("/q_learning/state", GameState, queue_size=10)

        # wait for things to be set up
        rospy.sleep(1)
        print("I HATE IT HERE")

        # set up is complete
        self.initialized = True
    def get_grid(self, data):
        data = self.mapd.data 
    def run(self):
        print(self.map.data)

if __name__ == "__main__": 
    node = Grid()
    node.run()
