#!/usr/bin/env python3
import state
import os
import copy
import pickle

class TJGame(object):
    """
    A class encapsulating the information in a tom&jerry game for the purposes
    of q_learning, i.e., states, actions, and the state action matrix;
    Acts as an interface between state.py and minimax_qlearning.py
    """
    def __init__(self, compute_game_info:bool, load_from_file:bool, game_info_prefix:str):
        self.path_prefix = os.path.dirname(__file__) + '/state_action_matrices/'
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
                    # node.action_matrix gives the states themselves; we want the indices
                    sam_row.append(self.states.index(resulting_state))
                self.state_action_matrix.append(sam_row)
            self.save_info(game_info_prefix)
        elif load_from_file:
            self.load_info(game_info_prefix)


    def save_info(self, game_info_prefix):
        with open(game_info_prefix + 'states.pckl', 'w') as state_file:
            pickle.dump(self.states, state_file)

        with open(game_info_prefix + 'max_actions.pckl', 'w') as max_actions:
            pickle.dump(self.maximizer_actions, max_actions)

        with open(game_info_prefix + 'max_actions.pckl', 'w') as min_actions:
            pickle.dump(self.minimizer_actions, min_actions)

        with open(game_info_prefix + 'state_action_matrix.pckl', 'w') as sam:
            pickle.dump(self.state_action_matrix, sam)


    def load_info(self, game_info_prefix):
        with open(game_info_prefix + 'states.pckl', 'r') as state_file:
            self.states = pickle.load(state_file)

        with open(game_info_prefix + 'max_actions.pckl', 'r') as max_actions:
            self.maximizer_actions = pickle.load(max_actions)

        with open(game_info_prefix + 'max_actions.pckl', 'r') as min_actions:
            self.minimizer_actions = pickle.load(min_actions)

        with open(game_info_prefix + 'state_action_matrix.pckl', 'r') as sam:
            self.state_action_matrix = pickle.load(sam)


if __name__ == "__main__":
    game_info_calc = TJGame(compute_game_info=True, load_from_file=False, game_info_prefix="TJGame1")