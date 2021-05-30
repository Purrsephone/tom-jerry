#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv, time, copy
from . import qmatrix


class QLearning(object):

    def __init__(self):

        # TODO
        # path of directory on where state/action matrices are located (or make them every time)
        self.path_prefix = os.path.dirname(__file__) + "/action_states/"
        self.q_matrix = None


        # set learning rate, discount factor, explore factor
        self.learning_rate = 1
        self.learning_rate_decay_factor = 0.99999
        self.discount_factor = 0.5
        self.explore_prob = 0.2

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
            return np.random.choice(self.q_matrix.maximizer_actions, p=self.q_matrix.get_max_policy_probabilities(state))


    def choose_minimizer_action(self, state):
        explore = np.random.uniform()
        if explore < self.explore_prob:
            return np.random.choice(self.q_matrix.minimizer_actions)
        else: # choose action according to policy
            return np.random.choice(self.q_matrix.minimizer_actions, p=self.q_matrix.get_min_policy_probabilities(state))

    
    # given a state, returns its reward
    def reward(self, state, max_action, min_action) -> int:
        pass


    def train_q_matrix(self, new_q_matrix:bool=True, random_opponent:bool=True):
        if new_q_matrix:
            self.q_matrix = qmatrix.QMatrix()
        elif self.q_matrix is None:
            print("'self.q_matrix' should be initialized if calling `train_q_matrix` with 'new_q_matrix=false'; exiting")
            exit(-1)
        
        train = True
        while train:
            #TODO set initial state
            current_state = None
            game_over = False
            while not game_over:
                max_action = self.choose_minimizer_action(current_state)
                min_action = self.choose_maximizer_action(current_state)
                next_state = self.q_matrix.next_state(current_state)
                old_q_value = (1 - self.learning_rate)*self.q_matrix.get_q_value(current_state, max_action, min_action)
                next_state_value = self.discount_factor*self.q_matrix.get_state_value(next_state)
                q_value_adjustment = self.learning_rate*(self.reward(current_state, min_action, max_action) + next_state_value)
                new_q_value = old_q_value + q_value_adjustment
                self.q_matrix.set_q_matrix(current_state, max_action, min_action, new_q_value)
                # TODO update policy
                # TODO update current states value
                current_state = next_state
                self.learning_rate = self.learning_rate*self.learning_rate_decay_factor
            # TODO decide on train=false condition (when learning rate totally decays maybe?)
        # TODO save q_matrix


    def publish_according_to_q_matrix(self):
        #TODO
        pass
    

    def save_q_matrix(self):
        #TODO
        pass


    def load_q_matrix(self):
        #TODO
        pass


if __name__ == "__main__":
    pass
