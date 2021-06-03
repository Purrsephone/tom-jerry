#!/usr/bin/env python3
import state
import os
import copy
import pickle
import time

class TJGame(object):
    """
    A class encapsulating the information in a tom&jerry game for the purposes
    of q_learning, i.e., states, actions, and the state action matrix;
    Acts as an interface between state.py and minimax_qlearning.py
    """
    def __init__(self, compute_game_info:bool, load_from_file:bool, game_info_prefix:str, mouse_on_cheese):
        self.path_prefix = os.path.dirname(__file__) + '/state_action_matrices/' + game_info_prefix
        self.grid = state.Grid()
        if compute_game_info:
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
        elif load_from_file:
            self.load_info()
        
        # a dictionary of states where a state is present 
        # omlu if the mouse is in a square with cheese
        self.mouse_on_cheese = mouse_on_cheese


    def save_info(self):
        with open(self.path_prefix + 'states.pckl', 'wb') as state_file:
            pickle.dump(self.states, state_file)

        with open(self.path_prefix + 'max_actions.pckl', 'wb') as max_actions:
            pickle.dump(self.maximizer_actions, max_actions)

        with open(self.path_prefix + 'min_actions.pckl', 'wb') as min_actions:
            pickle.dump(self.minimizer_actions, min_actions)

        with open(self.path_prefix + 'state_action_matrix.pckl', 'wb') as sam:
            pickle.dump(self.state_action_matrix, sam)


    def load_info(self):
        with open(self.path_prefix + 'states.pckl', 'rb') as state_file:
            self.states = pickle.load(state_file)

        with open(self.path_prefix + 'max_actions.pckl', 'rb') as max_actions:
            self.maximizer_actions = pickle.load(max_actions)

        with open(self.path_prefix + 'max_actions.pckl', 'rb') as min_actions:
            self.minimizer_actions = pickle.load(min_actions)

        with open(self.path_prefix + 'state_action_matrix.pckl', 'rb') as sam:
            self.state_action_matrix = pickle.load(sam)
    

    def reward(self, state, cat_action, mouse_action ):
        next_state = self.state_action_matrix[state][cat_action*len(self.maximizer_actions) + mouse_action]
        if next_state in self.mouse_on_cheese:
            return -100
        elif self.grid.snack_time(next_state):
            return 100
        else: # pressure cat to go after mouse
            return -5
        

    def game_over(self, state, cat_action, mouse_action):
        if self.reward(self, state, cat_action, mouse_action) == 100:
            return True
        elif self.reward(self, state, cat_action, mouse_action) == -100:
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


if __name__ == "__main__":
    game_info_calc = TJGame(compute_game_info=True, load_from_file=False, game_info_prefix="TJGame1", mouse_on_cheese={})
    #game_info_calc = TJGame(compute_game_info=False, load_from_file=True, game_info_prefix="TJGame1", mouse_on_cheese={})
    game_info_calc.print_board_positions()