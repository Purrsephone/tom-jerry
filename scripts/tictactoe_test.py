#!/usr/bin/env python3
from numpy import base_repr 

class TicTacToe(object):
    """
    A class representing the game of TicTacToe,
    this module is used to test minimax_q_learning
    """
    
    def __init__(self):
        # the digits in the base 3 represntation of the state number correspond
        # to which peices are on the board. 0 is none, 1 is cirlce and 2 is x.
        # the nth digit from the right is the nth position of the board in row major order
        # ex: 000000120 means there is an O in the top right and a 2 in the top middle
        self.states = [x for x in range(3 ** 9)]

        # place a circle on the board: action # corresponds to row major order position on board
        # except 9, which corresponds to no action (turn based game)
        self.maximizer_actions = [0,1,2,3,4,5,6,7,8,9]
        self.minimizer_actions = [0,1,2,3,4,5,6,7,8,9]
        # we follow the convention that the maximizer goes first
        self.state_action_matrix = []
        for state in self.states:
            state_row = []
            for max_action in self.maximizer_actions:
                for min_action in self.minimizer_actions:
                    state_array = self.state_num_to_array(state)
                    if state_array.count(1) == state_array.count(2) and min_action == 9: # maximizer goes
                        if max_action < 9 and state_array[max_action] == 0:
                            state_array[max_action] = 1
                            state_row.append(self.state_array_to_state_num(state_array))
                        else:
                            state_row.append(-1)
                    elif state_array.count(1) == 1 + state_array.count(2) and max_action == 9: # minimizer goes
                        if min_action < 9 and state_array[min_action] == 0:
                            state_array[min_action] = 2
                            state_row.append(self.state_array_to_state_num(state_array))
                        else:
                            state_row.append(-1)
                    else:
                        state_row.append(-1)
            self.state_action_matrix.append(state_row)


    # returns an array of length 9 representing the state of the board
    # in row-major order from left to right
    def state_num_to_array(self, state:int):
        state_str = base_repr(state, base=3)
        if len(state_str) < 9:
            state_str = (9-len(state_str)) * '0' + state_str
        state_array = [int(c) for c in state_str]
        state_array.reverse()
        return state_array


    def state_array_to_state_num(self, state_array) -> int:
        new_state = 0
        state_array.reverse()
        for idx, digit in enumerate(state_array):
            new_state += (3 ** idx) * digit
        return new_state

    # returns 0 if game not over, 1 if maximizer won, 2 if minimizer, 3 for draw
    def game_over(self, state:int) -> int:
        state_array = self.state_num_to_array(state)
        for i in range(3):
            #rows
            three_in_a_row = state_array[0+3*i] == state_array[1+3*i] and state_array[0+3*i] == state_array[2+3*i]
            if three_in_a_row:
                return state_array[0+3*i]
            #columns
            three_in_a_col = state_array[0+i] == state_array[3+i] and state_array[0+i] == state_array[6+i]
            if three_in_a_col:
                return state_array[0+i]
        # check diagonals
        if state_array[0] == state_array[4] and state_array[0] == state_array[8]:
            return state_array[0]
        if state_array[2] == state_array[4] and state_array[2] == state_array[6]:
            return state_array[2]
        for peice in state_array:
            if peice == 0:
                return 0
        return 3

    
if __name__ == "__main__":
    ttt = TicTacToe()
    test1 = ttt.game_over(13) #1
    test2 = ttt.game_over(26) #2
    test3 = ttt.game_over(10416) #2
    test4 = ttt.game_over(10817) #3
    test5 = ttt.game_over(21) #0
    print(test1, test2, test3, test4, test5)