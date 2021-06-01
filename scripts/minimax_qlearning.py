#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv, time
import qmatrix, rps_test


class QLearning(object):

    def __init__(self):

        # path of directory on where q_matrices are saved
        self.path_prefix = os.path.dirname(__file__) + "/q_matrices/"
        self.q_matrix = None

        # set training paramters
        self.learning_rate = 1
        self.learning_rate_decay_factor = 0.99999
        self.discount_factor = 0.5
        self.explore_prob = 1
        self.max_iteration = 1000  # change this later
        # TODO change later if need be
        self.initial_state = 0 

        # TODO
        self.action_pub = None #rospy.Publisher(action_topic, action_type, queue_size=10)
        self.state_sub = None #rospy.Subscriber(state_topic, state_type, self.state_update)

        # wait for publishers and subscribers to initialize
        time.sleep(1)


    def choose_maximizer_action(self, state):
        explore = np.random.uniform()
        if explore < self.explore_prob:
            return np.random.choice(self.q_matrix.maximizer_actions)
        else: # choose action according to policy
            print(self.q_matrix.get_max_policy_probabilities(state))
            return np.random.choice(self.q_matrix.maximizer_actions, p=self.q_matrix.get_max_policy_probabilities(state))


    def choose_minimizer_action(self, state):
        explore = np.random.uniform()
        if explore < self.explore_prob:
            return np.random.choice(self.q_matrix.minimizer_actions)
        else: # choose action according to policy
            return np.random.choice(self.q_matrix.minimizer_actions, p=self.q_matrix.get_min_policy_probabilities(state))

    
    # given a state, returns its reward, dependent on game
    def reward(self, state, max_action, min_action) -> int:
        if self.q_matrix.next_state(state, max_action, min_action) == 1:
            return 100
        elif self.q_matrix.next_state(state, max_action, min_action) == 2:
            return -100
        else:
            return 0

    
    # Checks if game is over, dependent on game
    def check_game_over(self, state, max_action, min_action) -> bool:
        if self.q_matrix.next_state(state, max_action, min_action) != 0:
            return True
        else:
            return False


    def train_q_matrix(self, rps:bool, new_q_matrix:bool=True, random_opponent:bool=True):
        if new_q_matrix and not rps:
            self.q_matrix = qmatrix.QMatrix()
        elif new_q_matrix:
            rps_info = rps_test.RPSGame()
            self.q_matrix = qmatrix.QMatrix(rps_info.states, rps_info.maximizer_actions, rps_info.minimizer_actions, rps_info.state_action_matrix)
        elif self.q_matrix is None:
            print("'self.q_matrix' should be initialized if calling `train_q_matrix` with 'new_q_matrix=false'; exiting")
            exit(-1)
        
        train = True
        iteration = 0
        while train:
            current_state = self.initial_state
            game_over:bool = False
            while not game_over:
                max_action = self.choose_minimizer_action(current_state)
                min_action = self.choose_maximizer_action(current_state)
                next_state = self.q_matrix.next_state(current_state, max_action, min_action)
                old_q_value = (1 - self.learning_rate)*self.q_matrix.get_q_matrix(current_state, max_action, min_action)
                next_state_value = self.discount_factor*self.q_matrix.get_state_value(next_state)
                q_value_adjustment = self.learning_rate*(self.reward(current_state, min_action, max_action) + next_state_value)
                new_q_value = old_q_value + q_value_adjustment
                self.q_matrix.set_q_matrix(current_state, max_action, min_action, new_q_value)
                self.q_matrix.update_maximizer_policy(current_state)
                self.q_matrix.update_value(current_state)
                game_over = self.check_game_over(current_state, min_action, max_action)
                current_state = next_state
                self.learning_rate = self.learning_rate*self.learning_rate_decay_factor
                iteration +=1
            if self.learning_rate < 0.05 or iteration == self.max_iteration:
                train=False
            elif (iteration % 1000) == 0:
                print(iteration)
        print("Optimal Maximizer policy:", self.q_matrix.get_max_policy(0))
        self.save_q_matrix()


    def publish_according_to_q_matrix(self):
        #TODO
        pass
    

    def save_q_matrix(self):
        with open(self.path_prefix + 'q_matrix.csv', 'w') as csvfile:
            q_matrix_writer = csv.writer(csvfile)
            for state in self.q_matrix.states:
                state_row = []
                for max_action in self.q_matrix.maximizer_actions:
                    for min_action in self.q_matrix.minimizer_actions:
                        state_row.append(self.q_matrix.get_q_matrix(state, max_action, min_action))
                q_matrix_writer.writerow(state_row)
                

    def load_q_matrix(self):
        self.q_matrix.q_matrix = []
        with open(self.path_prefix + 'q_matrix.csv', 'r') as csvfile:
            q_matrix_reader = csv.reader(csvfile)
            for row in q_matrix_reader:
                #check this later, not independent of qmatrix.py implementation
                self.q_matrix.q_matrix.append([float(i) for i in row])



if __name__ == "__main__":
    ql = QLearning()
    ql.train_q_matrix(True)
    # test loading rps qmatrix and computing policy from it
    """
    rps_info = rps_test.RPSGame()
    q_matrix = qmatrix.QMatrix(rps_info.states, rps_info.maximizer_actions, rps_info.minimizer_actions, rps_info.state_action_matrix)
    ql.q_matrix = q_matrix
    ql.load_q_matrix()
    ql.q_matrix.update_maximizer_policy(0)
    print(ql.q_matrix.get_max_policy(0))
    """