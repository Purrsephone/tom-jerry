#!/usr/bin/env python3
import state
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
                csv_writer.writerow([csv_row])


    def load_info(self):
        self.states = []
        self.maximizer_actions = []
        self.minimizer_actions = []
        self.state_action_matrix = []
        with open(self.path_prefix + 'states.csv', 'r') as state_csv:
            csv_reader = csv.reader(state_csv)
            for row in csv_reader:
                row = [float(x) for x in row]
                new_state = state.State(state.Position(row[0], row[1], row[2]), state.Position(row[3], row[4], row[5]))
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
    # game_info_calc = TJGame(compute_game_info=False, load_from_file=True, game_info_prefix="TJGame1", mouse_on_cheese={})
    game_info_calc.print_board_positions()