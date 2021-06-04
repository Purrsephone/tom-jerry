#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv, time
import qmatrix, tom_jerry_game
from rospy.core import rospyinfo #, rps_test, tictactoe_test
from tom_and_jerry_project.msg import Action, GameState


class QLearning(object):
    """
    A class representing the Qlearning algorithm for a 2-agent adversarial game 
    """
    def __init__(self):

        # path of directory on where q_matrices are saved
        self.path_prefix = os.path.dirname(__file__) + "/q_matrices/"
        self.q_matrix = None

        # Choose a game to train
        # self.TTTGame = tictactoe_test.TicTacToe()
        # self.RPSGame = rps_test.RPSGame()
        self.TJGame = tom_jerry_game.TJGame(False, False, True, "TJGame1", [(3,3), (5,5)])
        self.csv_name = 'q_matrixTJ.csv'

        # set training paramters
        self.learning_rate = 1
        self.learning_rate_decay_factor = 0.99999
        self.discount_factor = 0.5
        self.explore_prob = 0.5
        self.max_iteration = 10000 # change this later
        # TODO change later if need be
        self.initial_state = self.TJGame.state_index_from_positions(5,0,0,0,0,0)

        # TODO set up subscribers and publishers for actual execution
        self.action_pub = rospy.Publisher("/action", Action, queue_size=200)

        # wait for publishers and subscribers to initialize
        time.sleep(1)


    def choose_maximizer_action(self, state):
        explore = np.random.uniform()
        if explore < self.explore_prob:
            action = np.random.choice(self.q_matrix.maximizer_actions)
            while not self.q_matrix.valid_max_action(state, action):
                action = np.random.choice(self.q_matrix.maximizer_actions)
            return action
        else: # choose action according to policy
            probabilities = self.q_matrix.get_max_policy_probabilities(state)
            try:
                action = np.random.choice(self.q_matrix.maximizer_actions, p=probabilities)
                while not self.q_matrix.valid_max_action(state, action):
                    action = np.random.choice(self.q_matrix.maximizer_actions, p=probabilities)
                return action
            except: # probabilites don't sum to 1, choose random action instead
                action = np.random.choice(self.q_matrix.maximizer_actions)
                while not self.q_matrix.valid_max_action(state, action):
                    action = np.random.choice(self.q_matrix.maximizer_actions)
                return action


    def choose_minimizer_action(self, state):
        explore = np.random.uniform()
        if explore < self.explore_prob:
            action = np.random.choice(self.q_matrix.minimizer_actions)
            while not self.q_matrix.valid_min_action(state, action):
                action = np.random.choice(self.q_matrix.minimizer_actions)
            return action
        else: # choose action according to policy
            try:
                probabilities = self.q_matrix.get_min_policy_probabilities(state)
                action = np.random.choice(self.q_matrix.minimizer_actions, p=probabilities)
                while not self.q_matrix.valid_min_action(state, action):
                    action = np.random.choice(self.q_matrix.minimizer_actions, p=probabilities)
                return action
            except: # probabilities don't sum to 1, choose random action instead
                action = np.random.choice(self.q_matrix.minimizer_actions)
                while not self.q_matrix.valid_min_action(state, action):
                    action = np.random.choice(self.q_matrix.minimizer_actions)
                return action

    
    # given a state, returns its reward, dependent on game
    def reward(self, state, max_action, min_action) -> int:
        return self.TJGame.reward(state, max_action, min_action)
        # TTT reward
        """
        if self.TTTGame.game_over(self.q_matrix.next_state(state, max_action, min_action)) == 1:
            return 100
        elif self.TTTGame.game_over(self.q_matrix.next_state(state, max_action, min_action)) == 2:
            return -100
        else:
            return 0
        """
        # RPS reward
        """
        if self.q_matrix.next_state(state, max_action, min_action) == 1:
            return 100
        elif self.q_matrix.next_state(state, max_action, min_action) == 2:
            return -100
        else:
            return 0
        """

    
    # Checks if game is over, dependent on game
    def check_game_over(self, state, max_action, min_action) -> bool:
        return self.TJGame.game_over(state, max_action, min_action)
        # TTT game over
        """
        return self.TTTGame.game_over(self.q_matrix.next_state(state, max_action, min_action)) > 0
        """
        # RPS game over
        """
        if self.q_matrix.next_state(state, max_action, min_action) != 0:
            return True
        else:
            return False
        """


    def train_q_matrix(self, rps:bool, ttt:bool, new_q_matrix:bool=True, random_opponent:bool=True):
        if new_q_matrix and rps:
            rps_info = self.RPSGame
            self.q_matrix = qmatrix.QMatrix(rps_info.states, rps_info.maximizer_actions, rps_info.minimizer_actions, rps_info.state_action_matrix)
        if new_q_matrix and ttt:
            ttt_info = self.TTTGame
            self.q_matrix = qmatrix.QMatrix(ttt_info.states, ttt_info.maximizer_actions, ttt_info.minimizer_actions, ttt_info.state_action_matrix)
        elif new_q_matrix:
            self.q_matrix = qmatrix.QMatrix(self.TJGame.states, self.TJGame.maximizer_actions, self.TJGame.minimizer_actions, self.TJGame.state_action_matrix)
        elif self.q_matrix is None:
            print("'self.q_matrix' should be initialized if calling `train_q_matrix` with 'new_q_matrix=false'; exiting")
            exit(-1)
        
        train:bool = True
        iteration:int = 0
        num_games:int = 0
        maximizer_wins:int = 0
        while train:
            current_state = self.initial_state
            game_over:bool = False
            while not game_over:
                max_action = self.choose_maximizer_action(current_state)
                min_action = self.choose_minimizer_action(current_state)
                next_state = self.q_matrix.next_state(current_state, max_action, min_action)
                old_q_value = (1 - self.learning_rate)*self.q_matrix.get_q_matrix(current_state, max_action, min_action)
                next_state_value = self.discount_factor*self.q_matrix.get_state_value(next_state)
                self.last_reward = self.reward(current_state, max_action, min_action)
                q_value_adjustment = self.learning_rate*(self.last_reward + next_state_value)
                new_q_value = old_q_value + q_value_adjustment
                self.q_matrix.set_q_matrix(current_state, max_action, min_action, new_q_value)
                self.q_matrix.update_maximizer_policy(current_state)
                self.q_matrix.update_value(current_state)
                game_over = self.check_game_over(current_state, max_action, min_action)
                current_state = next_state
                self.learning_rate = self.learning_rate*self.learning_rate_decay_factor
                iteration +=1
            if self.learning_rate < 0.05 or iteration >= self.max_iteration:
                train=False
            num_games += 1
            if self.last_reward > 0:
                maximizer_wins += 1
            print("Game", num_games,  "Maximizer wins:", maximizer_wins, "Iteration:", iteration, "Learning Rate:", self.learning_rate)
        print("Optimal Maximizer policy from initial_state:", self.q_matrix.get_max_policy(self.initial_state))
        self.q_matrix.update_minimizer_policy(self.initial_state)
        print("Optimal Minimizer policy from initial_state:", self.q_matrix.get_min_policy(self.initial_state))
        self.save_q_matrix(self.csv_name)


    # call this after initializing to spin and setup for publishing
    def publish_according_to_state_update(self):
        self.explore_prob = 0
        # state_index = self.TJGame.state_index_from_positions(data.tom_coord.x_coord, data.tom_coord.y_coord, data.tom_coord.z_coord, data.jerry_coord.x_coord, data.jerry_coord.y_coord, data.jerry_coord.z_coord)
        current_state = self.initial_state
        while True:
            self.q_matrix.update_maximizer_policy(current_state)
            self.q_matrix.update_minimizer_policy(current_state)
            max_action = self.choose_maximizer_action(current_state)
            min_action = self.choose_minimizer_action(current_state)
            action_msg = Action(max_action, min_action)
            self.action_pub.publish(action_msg)
            current_state = self.TJGame.state_transition(self.TJGame.states[current_state], max_action, min_action)
            time.sleep(1)


    def save_q_matrix(self, csv_name):
        with open(self.path_prefix + csv_name, 'w') as csvfile:
            q_matrix_writer = csv.writer(csvfile)
            for idx, _ in enumerate(self.q_matrix.states):
                state_row = []
                for max_action in self.q_matrix.maximizer_actions:
                    for min_action in self.q_matrix.minimizer_actions:
                        state_row.append(self.q_matrix.get_q_matrix(idx, max_action, min_action))
                q_matrix_writer.writerow(state_row)
                

    def load_q_matrix(self, csv_name):
        self.q_matrix.q_matrix = []
        with open(self.path_prefix + csv_name, 'r') as csvfile:
            q_matrix_reader = csv.reader(csvfile)
            for row in q_matrix_reader:
                #check this later, not independent of qmatrix.py implementation
                self.q_matrix.q_matrix.append([float(i) for i in row])

if __name__ == "__main__":
    ql = QLearning()
    # ql.train_q_matrix(rps=False, ttt=False)
    ql.publish_according_to_state_update()
    # test loading rps qmatrix and computing policy from it
    """
    rps_info = rps_test.RPSGame()
    ql.q_matrix = qmatrix.QMatrix(rps_info.states, rps_info.maximizer_actions, rps_info.minimizer_actions, rps_info.state_action_matrix)
    ql.load_q_matrix('q_matrixRPS.csv')
    ql.q_matrix.update_maximizer_policy(0)
    print(ql.q_matrix.get_max_policy(0))
    """
    # test loading tictactoe and computing policy from it
    """
    ttt_info = tictactoe_test.TicTacToe()
    ql.q_matrix = qmatrix.QMatrix(ttt_info.states, ttt_info.maximizer_actions, ttt_info.minimizer_actions, ttt_info.state_action_matrix)
    ql.load_q_matrix('q_matrixTTT.CSV)
    ql.q_matrix.update_maximizer_policy(0)
    print("state 0 policy:", ql.q_matrix.get_max_policy(0))
    ql.q_matrix.update_maximizer_policy(13636)
    print("state 13636 policy:", ql.q_matrix.get_max_policy(13636), "Expected action 6 w/ high likelihood")
    ql.q_matrix.update_maximizer_policy(178)
    print("state 178 policy:", ql.q_matrix.get_max_policy(178), "Expected action 7 w/ high likelihood" )
    ql.q_matrix.update_maximizer_policy(13072)
    print("state 13072 policy:", ql.q_matrix.get_max_policy(13072), "Expected action 2 w/ high likelihood" )
    """