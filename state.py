#!/usr/bin/env python3

import rospy

import math 
import numpy as np 
from nav_msgs.msg import OccupancyGrid
from itertools import zip_longest


# Given an index and info about map, compute its real coordinate 
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


class Grid:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        

        # initialize this particle filter node
        #rospy.init_node('turtlebot3_particle_filter')

        #side length of squares for states, 1 for now, can change 
        self.square_side_len = 1 

        # set the topic names and frame names
        self.map_topic = "map"

        # inialize our map
        self.map = OccupancyGrid()

        #initialize squares 
        self.squares = []

        #initialize states 
        self.states = []

        #initialize actions
        self.actions = []

        #initialze action matrix 
        self.action_matrix = []

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        rospy.sleep(1)

        self.initialized = True
    def group_elements(n, iterable, padvalue='x'):
        return zip_longest(*[iter(iterable)]*n, fillvalue=padvalue)

    def get_grid(self):
        resolution = self.map.info.resolution
        width = self.map.info.width 
        height = self.map.info.height 
        x_origin = self.map.info.origin.position.x
        y_origin = self.map.info.origin.position.y
        sq_x = (height * resolution)/self.square_side_len
        sq_y = (width * resolution)/self.square_side_len
        delim = (self.square_side_len/2)/resolution 
        delim2 = self.square_side_len/resolution
        #need to group into squares, then get rid of invalid squares, then convert to actual coordinates
        flat_map = []
        for i in range(width):
            for j in range(height):
                indx = (i*width+j)
                flat_map.append(indx)
        
        temp_squares = group_elements(flat_map, delim2)
                
        for square in temp_squares: 
            for cell in square:
            #want only the midpoint of the square 
            if (cell != 0) and (cell % delim-1 == 0) and (cell % delim2-1 != 0):
                self.squares.append(cell)
        
        #convert squares list to 2D numpy array 
        self.squares = np.array(self.squares.reshape(-1, sq_y))

        #want to set N, E, S, W restrictions 
        #let 0 = N, 1 = E, 2 = S, 3 = W

        #need to check adjacent squares 
        for el in self.squares:
            if(self.map.data[el] == 0):
                result = np.where(self.squares == el)
                rowindx = result[0][0]
                colindx = result[1][0]
                north = rowindx + 1
                south = rowindx - 1
                east = colindx + 1
                west = colindx - 1
                valid_directions = []
                #try north 
                try:
                    val = self.squares[north][colindx]
                except:
                    pass 
                else: 
                    if(self.map.data[val] == 0):
                        valid_directions.append(0)
                #try south 
                try:
                    val = self.squares[south][colindx]
                except:
                    pass 
                else: 
                    if(self.map.data[val] == 0):
                        valid_directions.append(2)
                #try east 
                try:
                    val = self.squares[rowindx][east]
                except:
                    pass 
                else: 
                    if(self.map.data[val] == 0):
                        valid_directions.append(1)
                #try west 
                try:
                    val = self.squares[rowindx][west]
                except:
                    pass 
                else: 
                    if(self.map.data[val] == 0):
                        valid_directions.append(3)
                coord = convert_to_real_coords(cell, height, x_origin, y_origin, resolution)
                el = [el, coord, valid_directions]
        valid_squares = []
        #now let's remove any invalid squares from our list of squares 
        for el in self.squares:
            for sub in el:
                indx = sub[0]
                if(self.map.data[indx] == 0):
                    sub.pop(0)
                    valid_squares.append(sub)
        #now we should have a states list with only valid square midpoints, yay! 
        #finally, we'll want to permuate the possible states based on the squares and orientations
        for square in valid_squares: 
            for x in range(4):
                square.append(x)
                self.states.append(square)
        #so the state has 3 components:
            #1 coordinates of midpoint of the square 
            #2 directions that the robot can move from that state 
            #3 current orientation of the robot 
    def possible_transition(self, state1, state2):
        #if both square and direction are different, then no 
        diffsquare = True 
        if ((state1[0][0] == state2[0][0]) and (state1[0][1] == state2[0][1])):
            diffsquare = False 
        diffdirection = True 
        if (state1[2] == state2[2]):
            diffdirection = False 
        if(diffsquare and diffdirection):
            return False 
        #if both square and direction are the same, then yes 
        if(not(diffsquare) and not(diffdirection)):
            return True 
        #if squares are different, are they adjacent? 
        if(diffsquare):
            diffx = abs(state1[0][0] - state2[0][0])
            diffy = abs(state1[0][1] - state2[0][1])
            #are adjacent 
            if((diffx == self.square_side_len) or (diffy == self.square_side_len)):
                return True 
            else:
                return False 
        #if directions are different, are they adjacent? 
        if(diffdirection):
            #are adjacent 
            if((state1[2] % 2) == 0):
                if((state2[2] % 2) != 0):
                    return True 
            #are adjacent 
            if((state1[2] % 2) != 0):
                if((state2[2] % 2) == 0):
                    return True 
            else 
                return False 

    #actions are:
    #0 = go forward 
    #1 = 90 degree turn CW 
    #2 = 90 degree turn CCW 
    #3 = do nothing 
    def necessary_action(self, state1, state2):
        diffsquare = True 
        if ((state1[0][0] == state2[0][0]) and (state1[0][1] == state2[0][1])):
            diffsquare = False 
        diffdirection = True 
        if (state1[2] == state2[2]):
            diffdirection = False 
        if ((not diffsquare) and (not diffdirection)):
            return 3 
        if diffsquare:
            return 0
        else:
            if(state2[2] > state1[2]):
                if(state1[2] != 0):
                    return 1
                else:
                    return 2 
            else: 
                if(state2[2 < state1[2]]):
                    if(state1[2] != 3):
                        return 2
                    else:
                        return 1 
                 
    #make 2D array (state1 vs. state2)
    def make_action_matrix(self):
        num_states = len(self.states)
        self.action_matrix = np.zeros((num_states, num_states))
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
        self.action_matrix
    
    def make_action_list(self):
        for x in range(4):
            self.actions.append(x)

    
            
            

        
            
