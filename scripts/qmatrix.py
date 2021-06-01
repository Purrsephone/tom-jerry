#!/usr/bin/env python3
from scipy.optimize import linprog

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

    
    def get_q_matrix_from_pair(self, state, action_pair):
        return self.q_matrix[state][action_pair]


    def set_q_matrix_from_pair(self, state, action_pair, value):
        self.q_matrix[state][action_pair] = value
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

    
    # Updates the maximizer policy based on the latest Q Values using linear programming 
    def update_maximizer_policy(self, state):
        best_policies = []
        for min_action in self.q_matrix.minimizer_actions:
            objective_coefficients = []
            a_ub = []
            b_ub = []
            a_eq = [[]]
            b_eq = [1]
            for max_action1 in self.q_matrix.maximizer_actions:
                # linprog finds the min, and we want max so we multiply by -1
                objective_coefficients.append(-1*self.get_q_matrix(state, max_action1, min_action))
                # ensure weights sum to 1
                a_eq[0].append(1)
            # conditions to ensure that solution is the minimum over other opponent actions
            for alt_min_action in self.q_matrix.minimizer_actions:
                if min_action != alt_min_action:
                    upper_bound_row = []
                    for max_action2 in self.q_matrix.maximizer_actions:
                        upper_bound_row.append(self.get_q_matrix(state, max_action2, min_action) - self.get_q_matrix(state, max_action2, alt_min_action))
                    a_ub.append(upper_bound_row)
                    b_ub.append(0)

            optimal_sol = linprog(objective_coefficients, a_ub, b_ub, a_eq, b_eq)
            best_policies.append((optimal_sol.x, -1*optimal_sol.fun))

        # pick out best solution
        best_sol_val = best_policies[0][1]
        best_sol_index = 0 
        for index, sol_pair in enumerate(best_policies):
            if sol_pair[1] > best_sol_val:
                best_sol_index = index
                best_sol_val = sol_pair[1]
        
        #construct new policy
        new_policy = {}
        for i, action in enumerate(self.maximizer_actions):
            new_policy[action] = best_policies[best_sol_index][0][i]
        self.set_max_policy(state, new_policy)
        return


    def update_value(self, state):
        possible_values = []
        for min_action in self.maximizer_actions:
            sum = 0
            for max_action in self.maximizer_actions:
                sum += self.get_max_policy(state)[max_action] * self.get_q_matrix(state, max_action, min_action)
            possible_values.append(sum)
        self.set_state_value(state, min(possible_values))



if __name__ == "__main__":
    pass