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
    def __init__(self, compute_game_info:bool, load_from_file:bool, game_info_prefix:str):
        self.path_prefix = os.path.dirname(__file__) + '/state_action_matrices/' + game_info_prefix
        if compute_game_info:
            node = state.Grid()
            node.run()
            self.states = node.states
            self.maximizer_actions = copy.deepcopy(node.actions) 
            self.minimizer_actions = copy.deepcopy(node.actions)
            self.state_action_matrix = []
            for row in node.action_matrix:
                sam_row = []
                for resulting_state in row:
                    if resulting_state != -1:
                        # node.action_matrix gives the states themselves; we want the indices
                        sam_row.append(self.states.index(resulting_state))
                    else:
                        sam_row.append(-1)
                self.state_action_matrix.append(sam_row)
            self.save_info()
        elif load_from_file:
            self.load_info()


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


if __name__ == "__main__":
    game_info_calc = TJGame(compute_game_info=True, load_from_file=False, game_info_prefix="TJGame1")
    # game_info_calc = TJGame(compute_game_info=False, load_from_file=True, game_info_prefix="TJGame1")
    for states in game_info_calc.states:
        print(states.catpos.x, states.catpos.y, states.catpos.z, states.mousepos.x, states.mousepos.y, states.mousepos.z)