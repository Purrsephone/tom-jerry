#!/usr/bin/env python3

# necessary libraries
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from itertools import zip_longest
from q_learning_project.msg import RobotCoord


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

        # initialize this particle filter node
        #rospy.init_node('turtlebot3_particle_filter')

        # side length of squares for states, 1 for now, can change
        self.square_side_len = 1

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
        self.state_pub = rospy.Publisher("/q_learning/reward", GameState, queue_size=10)

        # wait for things to be set up
        rospy.sleep(1)

        # set up is complete
        self.initialized = True

    # helper function that groups elements into groups of size n
    def group_elements(n, iterable, padvalue='x'):
        return zip_longest(*[iter(iterable)]*n, fillvalue=padvalue)

    # given an occupancy grid, get info from it used to cut space into squares
    def get_grid(self):
        # get resolution, height, and width
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        # get origin coordinates
        x_origin = self.map.info.origin.position.x
        y_origin = self.map.info.origin.position.y
        # determine square dimensions
        sq_x = (height * resolution)/self.square_side_len
        sq_y = (width * resolution)/self.square_side_len
        # delimeters for chopping space into squares
        delim = (self.square_side_len/2)/resolution
        delim2 = self.square_side_len/resolution

        # get list of indexes (flatten 2D array into 1D array)
        flat_map = []
        for i in range(width):
            for j in range(height):
                indx = (i*width+j)
                flat_map.append(indx)

        # regroup indexes into groups of size delim2
        temp_squares = group_elements(flat_map, delim2)

        # keep only the midpoint of the square
        for square in temp_squares:
            for cell in square:
            if (cell != 0) and (cell % delim-1 == 0) and (cell % delim2-1 != 0):
                self.squares.append(cell)

        # convert squares list to 2D numpy array
        self.squares = np.array(self.squares.reshape(-1, sq_y))

        # convert indexes into real coordinates
        for el in self.squares:
            coord = convert_to_real_coords(cell, height, x_origin, y_origin, resolution)
            el = [el, coord]

        valid_squares = []
        # remove any invalid squares from our list of squares
        for el in self.squares:
            for sub in el:
                indx = sub[0]
                if(self.map.data[indx] == 0):
                    sub.pop(0)
                    # package coordinates plus orientation into Position class
                    sqr = Position(sub[0][0], sub[0][1], -1)
                    valid_squares.append(sqr)

        # now we should have a states list with only valid square midpoints, yay!
        # permuate the possible states based on the squares and orientations
        state_list = []
        for square in valid_squares:
            temp_sqr = square
            for x in range(4):
                temp_sqr.z = x
                state_list.append(temp_sqr)

        # make the state include all possible combos of tom and jerry positions
        num_states = len(state_list)
        outer_loop_counter = 0
        for state in state_list:
            inner_loop_counter = 0
            for state2 in state_list:
                tom = state_list[outer_loop_counter]
                jerry = state_list[inner_loop_counter]
                new_state = State(tom, jerry)
                self.states.append(new_state)
                inner_loop_counter += 1
            outer_loop_counter += 1

    # helper function, checks if it is possible for a single agent to move
    # from one state to the next
    def possible_transition_helper(self, state1, state2):
        #if both square and direction are different, then no
        diffsquare = True
        if ((state1.x == state2.x) and (state1.y == state2.y)):
            diffsquare = False
        diffdirection = True
        if (state1.z == state2.z):
            diffdirection = False
        if(diffsquare and diffdirection):
            return False
        #if both square and direction are the same, then yes
        if(not(diffsquare) and not(diffdirection)):
            return True
        #if squares are different, are they adjacent?
        if(diffsquare):
            diffx = abs(state1.x - state2.x)
            diffy = abs(state1.y - state2.y)
            #are adjacent
            if((diffx == self.square_side_len) or (diffy == self.square_side_len)):
                return True
            else:
                return False
        #if directions are different, are they adjacent?
        if(diffdirection):
            #are adjacent
            if((state1.z % 2) == 0):
                if((state2.z % 2) != 0):
                    return True
            #are adjacent
            if((state1.z % 2) != 0):
                if((state2.z % 2) == 0):
                    return True
            else
                return False

    # checks if it is possible for both agents to move from one state to the next
    def possible_transition(self, state1, state2):
        cat_possible = possible_transition_helper(state1.catpos, state2.catpos)
        mouse_possible = possible_transition_helper(state1.mousepos, state2.mousepos)
        if(cat_possible and mouse_possible):
            return True
        else
            return False

    # helper function (for single agent)
    # given two states, finds the action needed to transition from state1 to state2
    # does not consider invalid transitions, only call once you have checked validity
    #actions are:
    #0 = go forward
    #1 = 90 degree turn CW
    #2 = 90 degree turn CCW
    #3 = do nothing
    def necessary_action_helper(self, state1, state2):
        diffsquare = True
        if ((state1.x == state2.x) and (state1.y == state2.y)):
            diffsquare = False
        diffdirection = True
        if (state1.z == state2.z):
            diffdirection = False
        if ((not diffsquare) and (not diffdirection)):
            return 3
        if diffsquare:
            return 0
        else:
            if(state2.z > state1.z):
                if(state1.z != 0):
                    return 1
                else:
                    return 2
            else:
                if(state2.z < state1.z):
                    if(state1.z != 3):
                        return 2
                    else:
                        return 1

    # for both agents
    # given two states, finds the action needed to transition from state1 to state2
    # does not consider invalid transitions, only call once you have checked validity
    #actions are:
    #0 = go forward
    #1 = 90 degree turn CW
    #2 = 90 degree turn CCW
    #3 = do nothing
    # returns an array with [cat_action, mouse_action]
    def necessary_action(self, state1, state2):
        cat_action = necessary_action_helper(state1.catpos, state2.catpos)
        mouse_action = necessary_action_hlper(state1.mousepos, state2.mousepos)
        return [cat_action, mouse_action]

    # make 2D array action matrix by pairing off state1 vs. state2 combos
    # value at action_matrix[state1][state2] should be either
    # the array of actions [cat, mouse] OR -1 if not possible transition
    def make_action_matrix(self):
        num_states = len(self.states)
        self.action_matrix = np.array((num_states, num_states))
        outer_loop_counter = 0
        for state in self.states:
            inner_loop_counter = 0
            for state2 in self.states:
                valid = possible_transition(state, state2)
                if(valid):
                    action_needed = necessary_action(state, state2)
                    self.action_matrix[outer_loop_counter][inner_loop_counter] = action_needed
                else:
                     self.action_matrix[outer_loop_counter][inner_loop_counter] = -1
                inner_loop_counter += 1
            outer_loop_counter += 1

    # only 4 possible actions
    #0 = go forward
    #1 = 90 degree turn CW
    #2 = 90 degree turn CCW
    #3 = do nothing
    def make_action_list(self):
        for x in range(4):
            self.actions.append(x)
            
    def publish_states(self):
        count = 0 
        for state in self.states:
               # publish all states
                state_msg = RobotCoord()
                state_msg.move_num = count 
                state_msg.tom_coord = state.catpos
                state_msg.jerry_coord = state.mousepos 
                self.state_pub.publish(state_msg) 
                count += 1

