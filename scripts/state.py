#!/usr/bin/env python3

# necessary libraries
import rospy
import math, time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from itertools import zip_longest
from tom_and_jerry_project.msg import QLearningReward, RobotCoord, GameState
from fake_data import * 
import copy 
from itertools import product

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
        self.square_side_len = 25

        # set the topic names and frame names
        self.map_topic = "map"

        # inialize our map
        self.map = OccupancyGrid()

        # initialize state arrays 
        self.coors_array = []
        self.possible_states = []

        # initialize squares
        self.squares = []

        # initialize states
        self.states = []

        # initialize actions
        self.actions = []

        # initialze action matrix
        self.action_matrix = []

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        
        # publish states 
        self.state_pub = rospy.Publisher("/q_learning/state", GameState, queue_size=10)

        # wait for things to be set up
        rospy.sleep(1)
     
        # set up is complete
        self.initialized = True

        self.run()
    # get map data from occupancy grid 
    def get_map(self, data):
        self.map = data 

    # given an occupancy grid, get info from it used to cut space into squares
    def get_grid(self):
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        x_origin = self.map.info.origin.position.x
        y_origin = self.map.info.origin.position.y

        # get a 2d array x by y where each entry is ((real_x, real_y), valid)
        x_coors_array = math.ceil(width / self.square_side_len)
        y_coors_array = math.ceil(height / self.square_side_len)

        coors_array = np.zeros((x_coors_array, y_coors_array), dtype=(float,3))
        # get midpoints of squares 
        def get_midpoint(x, y, edge):
            x_mid = x*self.square_side_len+.5*self.square_side_len
            y_mid = y*self.square_side_len+.5*self.square_side_len
            if edge == 'x':
                x_mid = x*self.square_side_len+.5*(width - x*self.square_side_len)
            elif edge == 'y':
                y_mid = y*self.square_side_len+.5*(height - y*self.square_side_len)
            elif edge == 'xy':
                x_mid = x*self.square_side_len+.5*(width - x*self.square_side_len)
                y_mid = y*self.square_side_len+.5*(height - y*self.square_side_len)
            x_coord = x_origin + (x_mid * resolution)
            y_coord = y_origin + (y_mid * resolution)
            return (x_coord, y_coord)
        # determine if square is valid, return 1 if yes, else 0 
        def get_valid(x,y, edge):
            x_end = x*self.square_side_len+self.square_side_len
            y_end = y*self.square_side_len+self.square_side_len
            if edge == 'x':
                x_end = width
            elif edge == 'y':
                y_end = height
            elif edge == 'xy':
                x_end = width
                y_end = height
            for x in range(x*self.square_side_len, x_end):
                for y in range(y*self.square_side_len, y_end):
                    ind = x + y*width
                    if self.map.data[ind] != 0:
                        return 0
            return 1

        # set real midpoint coordintes and valid state
        for x in range(x_coors_array):
            for y in range(y_coors_array):
                edge = ''
                if x == x_coors_array-1 and y == y_coors_array-1: edge = 'xy'
                elif x == x_coors_array-1: edge = 'x'
                elif y == y_coors_array-1: edge = 'y'
                x_mid, y_mid = get_midpoint(x, y, edge)

                coors_array[x][y] = (x_mid, y_mid, get_valid(x, y, edge))

        self.coors_array = coors_array
        
        # get all possible states ((tom_coors_x, tom_coors_y, tom_coors_z), (jerry_coors_x, jerry_coors_y, jerry_coors_z))
        possible_states_single = []

        # permute based on all possible directions (N, E, S, W)
        for x in range(x_coors_array):
            for y in range(y_coors_array):
                if coors_array[x][y][2] == 1:
                    possible_states_single.append((x,y,0))
                    possible_states_single.append((x,y,1))
                    possible_states_single.append((x,y,2))
                    possible_states_single.append((x,y,3))

        # states are cross product between cat positions and mouse positions 
        possible_states_double = list(product(possible_states_single, possible_states_single))

        self.possible_states = possible_states_double
        self.state_index_dict = {}
        state_counter = 0
        for p_state in self.possible_states:
            self.state_index_dict[(p_state[0][0], p_state[0][1], p_state[0][2], p_state[1][0], p_state[1][1], p_state[1][2])] = state_counter
            state_counter += 1
       


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
            else:
                return False

    # checks if it is possible for both agents to move from one state to the next
    def possible_transition(self, state1, state2):
        cat_possible = self.possible_transition_helper(state1.catpos, state2.catpos)
        mouse_possible = self.possible_transition_helper(state1.mousepos, state2.mousepos)
        if(cat_possible and mouse_possible):
            return True
        else:
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
        cat_action = self.necessary_action_helper(state1.catpos, state2.catpos)
        mouse_action = self.necessary_action_helper(state1.mousepos, state2.mousepos)
        return [cat_action, mouse_action]
    
    # given a starting state and an action pair, determines which state results 
    # from taking that action 
    def resulting_state(self, state1, action1, action2):
        cat_pos_res = state1.catpos 
        if action1 == 0: 
            cat_pos_res.y = state1.catpos.y + self.square_side_len
        if action1 == 1:
            if state1.catpos.z == 3:
                cat_pos_res.z = 0
            else: 
                cat_pos_res.z += 1 
        if action1 == 2: 
            if state1.catpos.z == 0:
                cat_pos_res.z = 3
            else: 
                cat_pos_res.z -= 1 
        mouse_pos_res = state1.mousepos  
        if action2 == 0: 
            mouse_pos_res.y = state1.mousepos.y + self.square_side_len
        if action2 == 1:
            if state1.mousepos.z == 3:
                mouse_pos_res.z = 0
            else: 
                mouse_pos_res.z += 1 
        if action2 == 2: 
            if state1.mousepos.z == 0:
                mouse_pos_res.z = 3
            else: 
                mouse_pos_res.z -= 1 
        ret_state = State(cat_pos_res, mouse_pos_res)
        if ret_state in self.states:
            return ret_state
        else:
            return -1 
   
    
    # makes a matrix of where states are rows and actions are columns
    # with entries corresponding to the state following from the action or
    # -1 if there is no resulting state
    def make_action_matrix_other(self):
        self.state_action_matrix = []
        for p_state in self.possible_states:
            state = State(Position(p_state[0][0], p_state[0][1], p_state[0][2]), Position(p_state[1][0], p_state[1][1], p_state[1][2]))
            state_action_row = []
            for max_action in self.actions:
                for min_action in self.actions:
                    next_state = self.state_transition(state, max_action, min_action)
                    if next_state == -1:
                        state_action_row.append(next_state)
                    else:
                        state_action_row.append(self.get_index_of_state(next_state))
            self.state_action_matrix.append(state_action_row)


 

    # determines if cat can monch mouse 
    # cat can monch mouse if and only if 
    # 1 cat is in an adjacent square and 
    # 2 cat is not facing the opposite direction from mouse
    def snack_time(self, curr_state):
        # Ignore resolution, state should be given as the integer board
        adjacent_square = False 
        # check if adjacent 
        if(curr_state.catpos.x == curr_state.mousepos.x):
            if(abs(curr_state.catpos.y - curr_state.mousepos.y) <= 1):
                adjacent_square = True 
            else:
                adjacent_square = False 
        if(curr_state.catpos.y == curr_state.mousepos.y):
            if(abs(curr_state.catpos.x - curr_state.mousepos.x) <= 1):
                adjacent_square = True 
            else:
                adjacent_square = False                
        if(adjacent_square == True):
            # cat is west of mouse
            if(curr_state.catpos.x < curr_state.mousepos.x):
                if(curr_state.catpos.z == 3):
                    return False 
            # cat is east of mouse
            if(curr_state.catpos.x > curr_state.mousepos.x):
                if(curr_state.catpos.z == 1):
                    return False 
              # cat is north of mouse
            if(curr_state.catpos.y > curr_state.mousepos.y):
                if(curr_state.catpos.z == 0):
                    return False 
              # cat is south of mouse
            if(curr_state.catpos.y > curr_state.mousepos.y):
                if(curr_state.catpos.z == 2):
                    return False 
            return True 
        # if not adjacent, return false 
        else: 
            return False 

    # runs a couple test cases for the snack time function 
    def test_snack_time(self):
        # test 1, should return False bc not adjacent 
        catpos1 = Position(5, 4, 0)
        mousepos1 = Position(10, 5, 2)
        state1 = State(catpos1, mousepos1)
        test1 = self.snack_time(state1)
        # test 2, should return False bc facing wrong way 
        catpos2 = Position(3, 5, 1)
        mousepos2 = Position(2, 5, 3)
        state2 = State(catpos2, mousepos2)
        test2 = self.snack_time(state2)
        # test 3, should return True 
        catpos3 = Position(3, 5, 3)
        mousepos3 = Position(2, 5, 3)
        state3 = State(catpos3, mousepos3)
        test3 = self.snack_time(state3)
        print(test1)
        print(test2)
        print(test3)
     
    # only 4 possible actions, make list of them 
    #0 = go forward
    #1 = 90 degree turn CW
    #2 = 90 degree turn CCW
    #3 = do nothing
    def make_action_list(self):
        for x in range(4):
            self.actions.append(x)
            
    def run(self):
        # if ready, run all functions 
        if self.initialized: 
            self.get_grid()
            self.make_action_list()
            self.make_action_matrix_other()
            self.test_snack_time()
        else: 
            rospy.sleep(1)

            

if __name__ == "__main__": 
    g = Grid()
    #rospy.spin()
