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

        # set up is complete
        self.initialized = True

    # helper function that groups elements into groups of size n
    def group_elements(n, iterable, padvalue='x'):
        return zip_longest(*[iter(iterable)]*n, fillvalue=padvalue)

    # given an occupancy grid, get info from it used to cut space into squares
    def get_grid(self):
        # get resolution, height, and width
        #HARD CODE FOR NOW 
        resolution = 0.05
        #resolution = self.map.info.resolution
        
        #HARD CODE FOR NOW 
        width = 384 
        height = 384 
        #width = self.map.info.width
        #height = self.map.info.height

        # get origin coordinates
        #HARD CODE FOR NOW 
        x_origin = -10
        y_origin = -10 
        #x_origin = self.map.info.origin.position.x
        #y_origin = self.map.info.origin.position.y
     
        # delimeters for chopping space into squares
        delim = (self.square_side_len/2)/resolution
        delim2 = int(self.square_side_len/resolution)

        # get list of indexes (flatten 2D array into 1D array)
        flat_map = []
        for i in range(width):
            for j in range(height):
                indx = (i*width+j)
                flat_map.append(indx)

        # regroup indexes into groups of size delim2
        # temp_squares = self.group_elements(delim2, flat_map)
        temp_squares = [flat_map[n:n+delim2] for n in range(0, len(flat_map), delim2)]

        # keep only the midpoint of the square
        for square in temp_squares:
            for cell in square:
                if (cell != 0) and (cell % delim-1 == 0) and (cell % delim2-1 != 0):
                    self.squares.append(cell)

        # convert indexes into real coordinates
        for el in self.squares:
            coord = convert_to_real_coords(el, height, x_origin, y_origin, resolution)
            indx = self.squares.index(el)
            self.squares[indx] = [el, coord]

        valid_squares = []
        # remove any invalid squares from our list of squares
        for el in self.squares:
            indx = el[0]
            #should be self.map.data[indx] but hard coding in alt for now 
            #if(self.map.data[indx] == 0):
            if(map_data[indx] == 0):
                el.pop(0)
                # package coordinates plus orientation into Position class
                sqr = Position(el[0][0], el[0][1], -1)
                valid_squares.append(sqr)

        # now we should have a states list with only valid square midpoints, yay!
        # permuate the possible states based on the squares and orientations
        state_list = []
        print("BEFORE PERMUTING DIRECTIONS")
        print(len(valid_squares))
        for square in valid_squares:
            for x in range(4):
                temp_sqr = copy.deepcopy(square)
                temp_sqr.z = x
                state_list.append(temp_sqr)
        print("BEFORE PAIRING UP CAT MOUSE")
        print(len(state_list))
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
        print("AFTER PAIRING")
        print(len(self.states))
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
    
    def make_action_matrix_other(self):
        new_states = copy.deepcopy(self.states)
        num_states = len(self.states)
        self.action_matrix = np.empty((num_states, 16), dtype=object)
        outer_loop_counter = 0
        for state in new_states: 
            inner_loop_counter = 0
            for x in range(4):
                for y in range(4):
                    res_state = self.resulting_state(state, x, y)
                    self.action_matrix[outer_loop_counter][inner_loop_counter] = res_state 
                    inner_loop_counter += 1 
            outer_loop_counter += 1
            #print(outer_loop_counter)

    # determines if cat can monch mouse 
    # cat can monch mouse if and only if 
    # 1 cat is in an adjacent square and 
    # 2 cat is not facing the opposite direction from mouse
    def snack_time(self, curr_state, sq_len):
        adjacent_square = False 
        print(curr_state.catpos.y)
        print(curr_state.mousepos.y)
        if(curr_state.catpos.x == curr_state.mousepos.x):
            if(abs(curr_state.catpos.y - curr_state.mousepos.y) == sq_len):
                return True 
            else:
                return False 
        if(curr_state.catpos.y == curr_state.mousepos.y):
            if(abs(curr_state.catpos.x - curr_state.mousepos.x) == sq_len):
                return True 
            else:
                return False                
        if(adjacent_square == True):
            # cat is west of mouse
            if(curr_state.catpos.x < curr_state.mousepos.x):
                if(curr_state.catpos.z == 3):
                    return False 
            # cat is east of mouse
            if(curr_state.catpos.x > curr_state.mousepos.x):
                if(curr.state.catpos.z == 1):
                    return False 
              # cat is north of mouse
            if(curr_state.catpos.y > curr_state.mousepos.y):
                if(curr.state.catpos.z == 0):
                    return False 
              # cat is south of mouse
            if(curr_state.catpos.y > curr_state.mousepos.y):
                if(curr.state.catpos.z == 2):
                    return False 
            return True 
        else: 
            print("CHEESE")
            return False 

    # runs a couple test cases for the snack time function 
    def test_snack_time(self):
        # test 1, should return False bc not adjacent 
        catpos1 = Position(5, 4, 0)
        mousepos1 = Position(10, 5, 2)
        state1 = State(catpos1, mousepos1)
        test1 = self.snack_time(state1, 1)
        # test 2, should return False bc facing wrong way 
        catpos2 = Position(3, 5, 1)
        mousepos2 = Position(2, 5, 3)
        state2 = State(catpos2, mousepos2)
        test2 = self.snack_time(state2, 1)
        # test 3, should return True 
        catpos3 = Position(3, 5, 3)
        mousepos3 = Position(2, 5, 3)
        state3 = State(catpos3, mousepos3)
        test3 = self.snack_time(state3, 1)
        print(test1)
        print(test2)
        print(test3)

    # make 2D array action matrix by pairing off state1 vs. state2 combos
    # value at action_matrix[state1][state2] should be either
    # the array of actions [cat, mouse] OR -1 if not possible transition
    def make_action_matrix(self):
        num_states = len(self.states)
        self.action_matrix = np.empty((num_states, num_states), dtype=object)
        outer_loop_counter = 0
        for state in self.states:
            inner_loop_counter = 0
            for state2 in self.states:
                valid = self.possible_transition(state, state2)
                if(valid):
                    action_needed = self.necessary_action(state, state2)
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
    #THIS IS FOR YOU CARLOS 
    def run(self):
        if self.initialized: 
            self.get_grid()
            print(len(self.states))
            for x in range(40):
                print(self.states[x].catpos.x)
                print(self.states[x].catpos.y)
                print(self.states[x].catpos.z)
            #print(self.states)

            #self.make_action_matrix_other()
            self.make_action_list()
            print("DONE HOMIE")
            self.test_snack_time()
            #print(len(self.action_matrix))
            #self.publish_states()
            #print(self.action_matrix)
            #print(self.states)
            #print(self.actions)
            #print(self.states)

        else: 
            rospy.sleep(1)

            
    def publish_states(self):
        count = 0 
        for state in self.states:
               # publish all states
                cat_msg = RobotCoord()
                cat_msg.x_coord = state.catpos.x 
                cat_msg.y_coord = state.catpos.y 
                cat_msg.z_coord = state.catpos.z 
                mouse_msg = RobotCoord()
                mouse_msg.x_coord = state.mousepos.x 
                mouse_msg.y_coord = state.mousepos.y 
                mouse_msg.z_coord = state.mousepos.z 
                state_msg = GameState()
                state_msg.move_num = count 
                state_msg.tom_coord = cat_msg  
                state_msg.jerry_coord = mouse_msg
                #TODO: fix the publishing 
                #self.state_pub.publish(state_msg) 
                count += 1

if __name__ == "__main__": 
    node = Grid()
    node.run()
