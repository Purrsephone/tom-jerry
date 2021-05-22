#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv, time, copy
#from gazebo_msgs.msg import ModelStates

# TODO
# path of directory on where state/action matrices are located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):

    def __init__(self):

        self.q_matrix = None

        # TODO
        # get states, actions, and action_matrix



        # set learning rate, discount factor, explore factor
        self.learning_rate = 1
        self.discount_factor = 0.5
        self.explore_prob = 0.2

        # TODO
        self.action_pub = None #rospy.Publisher(action_topic, action_type, queue_size=10)
        self.state_sub = None #rospy.Subscriber(state_topic, state_type, self.state_update)

        # wait for publishers and subscribers to initialize
        time.sleep(1)
    

    def save_q_matrix(self):
        pass


    def load_q_matrix(self):
        pass


    def choose_action(self):
        explore = np.random.uniform()
        if explore < self.explore_prob: #choose a random action
            pass
        else: # choose action according to policy
            pass
            

    def train_q_matrix(self, new_q_matrix:bool=True, random_opponent:bool=True):
        if new_q_matrix:
            #make new q_matrix
            pass
        elif self.q_matrix is None:
            print("'self.q_matrix' should be initialized if calling `train_q_matrix` with 'new_q_matrix=false'; exiting")
            exit(-1)
        train = True
        # TODO initialize policy 
        while train:
            # TODO maybe randomize init state between training loops to explore more
            current_state = None # note this is the initial state
            game_over = False
            while not game_over:
                pass
                # TODO update q_matrix
                #self.q_matrix[current_state][][]
                # TODO update policy
                # TODO update value


    def publish_according_to_q_matrix(self, ):
        pass


if __name__ == "__main__":
    pass
