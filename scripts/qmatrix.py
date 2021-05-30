#!/usr/bin/env python3

class QMatrix(object):
    """
    A class representing a Qmatrix for a 2-agent adversarial game 
    """
    def __init__(self):

        #TODO load in states, actions, action-state matrix
        self.states = None
        self.action_pairs = None
        self.maximizer_actions = None
        self.minimizer_actions = None
        self.action_state_matrix = None

        self.q_matrix = {}
        self.state_value = {}
        self.maximizer_policy = {}
        self.minimizer_policy = {}

        for state in self.states:
            self.q_matrix[state] = {}
            self.maximizer_policy[state] = {}
            self.minimizer_policy[state] = {}
            for action_pair in self.action_pairs:
                # action pair should be a tuple of the maximizer action and minimzer action
                self.q_matrix[state][action_pair] = 1
            self.state_value[state] = 1
            for action in self.maximizer_actions:
                self.maximizer_policy[state][action] = 1/len(self.maximizer_actions)
            for action in self.minimizer_actions:
                self.minimizer_policy[state][action] = 1/len(self.minimizer_actions)
        return


    def get_q_matrix(self, state, max_action, min_action):
        return self.q_matrix[state][(max_action, min_action)]


    def set_q_matrix(self, state, max_action, min_action, value):
        self.q_matrix[state][(max_action, min_action)] = value
        return

    
    def get_state_value(self, state):
        return self.state_value[state]


    def set_state_value(self, state, value):
        self.state_value[state] = value
        return

    
    def get_max_policy(self, state):
        return self.maximizer_policy[state]


    def set_max_policy(self, state, new_policy):
        self.maximizer_policy[state] = new_policy
        return

    
    def get_min_policy(self, state):
        return self.minimizer_policy[state]


    def set_min_policy(self, state, new_policy):
        self.minimizer_policy[state] = new_policy
        return


    def get_max_policy_probabilities(self, state):
        return list(self.maximizer_policy[state].values)


    def get_min_policy_probabilities(self, state):
        return list(self.minimizer_policy[state].values)


    def next_state(self, state, max_action, min_action):
        #TODO
        pass


if __name__ == "__main__":
    pass