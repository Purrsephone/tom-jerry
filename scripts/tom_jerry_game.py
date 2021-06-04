#!/usr/bin/env python3
import state as ST
import os
import copy
import csv

class TJGame(object):
    """
    A class encapsulating the information in a tom&jerry game for the purposes
    of q_learning, i.e., states, actions, and the state action matrix;
    Acts as an interface between state.py and minimax_qlearning.py
    Note that the cat is the maximizer and the mouse is the minimizer
    """
    def __init__(self, compute_game_info_from_state:bool, compute_game_info_abstractly:bool, load_from_file:bool, game_info_prefix:str, cheese_locations):
        self.path_prefix = os.path.dirname(__file__) + '/state_action_matrices/' + game_info_prefix
        self.grid = ST.Grid()
        if compute_game_info_from_state:
            self.grid.run()
            self.states = self.grid.states
            self.maximizer_actions = copy.deepcopy(self.grid.actions) 
            self.minimizer_actions = copy.deepcopy(self.grid.actions)
            self.state_action_matrix = []
            for row in self.grid.action_matrix:
                sam_row = []
                for resulting_state in row:
                    if resulting_state != -1:
                        # self.grid.action_matrix gives the states themselves; we want the indices for qlearning
                        sam_row.append(self.states.index(resulting_state))
                    else:
                        sam_row.append(-1)
                self.state_action_matrix.append(sam_row)
            self.save_info()
        elif compute_game_info_abstractly:
            self.board_size = 6
            self.positions = []
            for x in range(self.board_size):
                for y in range(self.board_size):
                    for direction in range(4):
                        self.positions.append(ST.Position(x, y, direction))
            
            self.states = []
            for catpos in self.positions:
                for mousepos in self.positions:
                    new_state = ST.State(catpos, mousepos)
                    self.states.append(new_state)
            
            self.maximizer_actions = [0,1,2,3]
            self.minimizer_actions = [0,1,2,3]
            self.state_action_matrix = []
            for state in self.states:
                state_action_row = []
                for max_action in self.maximizer_actions:
                    for min_action in self.minimizer_actions:
                        next_state = self.state_transition(state, max_action, min_action)
                        if next_state == -1:
                            state_action_row.append(next_state)
                        else:
                            state_action_row.append(self.get_index_of_state(next_state))
                self.state_action_matrix.append(state_action_row)
            self.save_info()
        elif load_from_file:
            self.load_info()

        self.state_index_dict = {}
        state_counter = 0
        for state in self.states:
            self.state_index_dict[(state.catpos.x, state.catpos.y, state.catpos.z, state.mousepos.x, state.mousepos.y, state.mousepos.z)] = state_counter
            state_counter += 1

        
        # a list of pairs (x, y) where (x, y) is in the list
        # if we cheese is at the location (x, y)
        self.cheese_locations = cheese_locations


    def save_info(self):
        with open(self.path_prefix + 'states.csv', 'w') as state_csv:
            csv_writer = csv.writer(state_csv)
            for state in self.states:
                csv_writer.writerow([state.catpos.x, state.catpos.y, state.catpos.z, state.mousepos.x, state.mousepos.y, state.mousepos.z])
        with open(self.path_prefix + 'max_action.csv', 'w') as max_action_csv:
            csv_writer = csv.writer(max_action_csv)
            for action in self.maximizer_actions:
                csv_writer.writerow([action])
        with open(self.path_prefix + 'min_action.csv', 'w') as min_action_csv:
            csv_writer = csv.writer(min_action_csv)
            for action in self.minimizer_actions:
                csv_writer.writerow([action])
        with open(self.path_prefix + 'state_action_matrix.csv', 'w') as sam_csv:
            csv_writer = csv.writer(sam_csv)
            for state_row in self.state_action_matrix:
                csv_row = []
                for resulting_state_index in state_row:
                    csv_row.append(resulting_state_index)
                csv_writer.writerow(csv_row)


    def load_info(self):
        self.states = []
        self.maximizer_actions = []
        self.minimizer_actions = []
        self.state_action_matrix = []
        with open(self.path_prefix + 'states.csv', 'r') as state_csv:
            csv_reader = csv.reader(state_csv)
            for row in csv_reader:
                row = [float(x) for x in row]
                new_state = ST.State(ST.Position(row[0], row[1], row[2]), ST.Position(row[3], row[4], row[5]))
                self.states.append(new_state)
                
        with open(self.path_prefix + 'max_action.csv', 'r') as max_action_csv:
            csv_reader = csv.reader(max_action_csv)
            for row in csv_reader:
                self.maximizer_actions.append(int(row[0]))
        with open(self.path_prefix + 'min_action.csv', 'r') as min_action_csv:
            csv_reader = csv.reader(min_action_csv)
            for row in csv_reader:
                self.minimizer_actions.append(int(row[0]))
        with open(self.path_prefix + 'state_action_matrix.csv', 'r') as sam_csv:
            csv_reader = csv.reader(sam_csv)
            for row in csv_reader:
                state_row = []
                for resulting_state in row:
                    state_row.append(int(resulting_state))
                self.state_action_matrix.append(state_row)
                

    def reward(self, state, cat_action, mouse_action):
        next_state_index = self.state_action_matrix[state][cat_action*len(self.maximizer_actions) + mouse_action]
        next_state = self.states[next_state_index]
        if (next_state.mousepos.x, next_state.mousepos.y) in self.cheese_locations:
            return -100
        elif self.grid.snack_time(next_state):
            return 100
        else: # pressure cat to go after mouse
            return -5
        

    def game_over(self, state, cat_action, mouse_action):
        if self.reward(state, cat_action, mouse_action) == 100:
            return True
        elif self.reward(state, cat_action, mouse_action) == -100:
            return True
        else:
            return False

    def print_board_positions(self):
        board_dict_x = {}
        board_dict_y = {}
        for state in self.states:
            board_dict_x[(state.catpos.x)] = 1
            board_dict_y[(state.catpos.y)] = 1
        print(board_dict_x, board_dict_y)


    def states_equal(self, state1, state2):
        cat_equal = state1.catpos.x == state2.catpos.x and state1.catpos.y == state2.catpos.y and state1.catpos.z == state2.catpos.z
        mouse_equal = state1.mousepos.x == state2.mousepos.x and state1.mousepos.y == state2.mousepos.y and state1.mousepos.z == state2.mousepos.z
        return cat_equal and mouse_equal


    def state_index_from_positions(self, cat_x, cat_y, cat_z, mouse_x, mouse_y, mouse_z):
        if (cat_x, cat_y, cat_z, mouse_x, mouse_y, mouse_z) in self.state_index_dict:
            return self.state_index_dict[(cat_x, cat_y, cat_z, mouse_x, mouse_y, mouse_z)]


    def get_index_of_state(self, state):
        state_key = (state.catpos.x, state.catpos.y, state.catpos.z, state.mousepos.x, state.mousepos.y, state.mousepos.z)
        if state_key in self.state_index_dict:
            return self.state_index_dict[state_key]
        return -1

    #actions are:
    #0 = go forward
    #1 = 90 degree turn CW
    #2 = 90 degree turn CCW
    #3 = do nothing
    # directions are # 0 = North, 1 = East, 2 = South, 3 = West
    # given a position and an action, returns the resulting position when 
    # applying the given action, possibly returning an out of bounds state
    def position_action_transition(self, position, action):
        new_pos = copy.deepcopy(position)
        if action == 0:
            if position.z == 0:
                new_pos.y += 1
            elif position.z == 1:
                new_pos.x += 1
            elif position.z == 2:
                new_pos.y += -1
            elif position.z == 3:
                new_pos.x += -1
        elif action == 1:
            new_pos.z = (position.z + 1) % 4
        elif action == 2:
            new_pos.z = (position.z - 1) % 4
        return new_pos


    # returns the state from the resulting action pair, or -1 if it is not valid
    # self.states needs to be initialized.
    def state_transition(self, state, max_action, min_action):
        next_state = copy.deepcopy(state)
        next_state.catpos = self.position_action_transition(state.catpos, max_action)
        next_state.mousepos = self.position_action_transition(state.mousepos, min_action)
        if self.in_bounds(next_state):
            return next_state
        else:
            return -1

    
    # uses self.board_size to check if a state is in bounds
    def in_bounds(self, state) -> bool:
        cat_in_bounds = state.catpos.x <= self.board_size and state.catpos.y <= self.board_size
        cat_in_bounds = cat_in_bounds and (state.catpos.x >= 0 and state.catpos.y >= 0)
        mouse_in_bounds = state.mousepos.x <= self.board_size and state.mousepos.y <= self.board_size
        mouse_in_bounds = mouse_in_bounds and (state.mousepos.x >= 0 and state.mousepos.y >= 0)
        return cat_in_bounds and mouse_in_bounds


if __name__ == "__main__":
    game_info_calc = TJGame(True, False, load_from_file=False, game_info_prefix="TJGame1", cheese_locations={})
    #game_info_calc = TJGame(False, False, load_from_file=True, game_info_prefix="TJGame1", cheese_locations={})
    game_info_calc.print_board_positions()