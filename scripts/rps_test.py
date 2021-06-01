#!/usr/bin/env python3

class RPSGame(object):
    """
    A class representing the game of Rock paper scissors,
    this module is used to test minimax_q_learning
    """
    
    def __init__(self):
        # start, maximizer win, minimizer win, draw
        self.states = [0,1,2,3]
        # rock, paper, scissors
        self.maximizer_actions = [0,1,2]
        self.minimizer_actions = [0,1,2]
        self.state_action_matrix = []
        for state in self.states:
            state_row = []
            if state == 0:
                state_row = [3,2,1,1,3,2,2,1,3]
            else:
                state_row = [state] * 9
            self.state_action_matrix.append(state_row)