#!/usr/bin/env python3
from scipy.optimize import linprog

class QMatrix(object):
    """
    A class representing a Qmatrix for a 2-agent adversarial game 
    """
    def __init__(self, s=None, max_a=None, min_a=None, sam=None):

        # a list of the possible states
        self.states = s
        # a list of the maximizer actions
        self.maximizer_actions = max_a
        # a list of the minimizer actions
        self.minimizer_actions = min_a
        # a 2d array of state indices vs action pairs where the 
        # entries are the states that follow from that pair of actions
        # for ex: if maximizer takes action x and minimzer takes action y
        # sam[state][len(minimizer_actions) * x + y] give the resulting state
        # Note that this is distinct from the action-state matrix
        # in the original q_learning project
        self.state_action_matrix = sam

        self.q_matrix = []
        self.state_value = {}
        self.maximizer_policy = {}
        self.minimizer_policy = {}

        for idx, _ in enumerate(self.states):
            q_matrix_row = []
            self.maximizer_policy[idx] = {}
            self.minimizer_policy[idx] = {}
            self.state_value[idx] = 1
            for max_action in self.maximizer_actions:
                for _ in self.minimizer_actions:
                    q_matrix_row.append(1)
                valid_max_actions = []
                if self.valid_max_action(idx, max_action):
                    valid_max_actions.append(max_action)
                self.maximizer_policy[idx][max_action] = 0
            for vmax_action in valid_max_actions:
                self.maximizer_policy[idx][vmax_action] = 1/len(valid_max_actions)

            for min_action in self.minimizer_actions:
                valid_min_actions = []
                if self.valid_min_action(idx, min_action):
                    valid_min_actions.append(min_action)
                self.minimizer_policy[idx][min_action] = 0
            for vmin_action in valid_min_actions:
                self.minimizer_policy[idx][vmin_action] = 1/len(valid_min_actions)
            
            self.q_matrix.append(q_matrix_row)
        return


    def get_q_matrix(self, state, max_action, min_action):
        return self.q_matrix[state][len(self.minimizer_actions) * max_action + min_action]


    def set_q_matrix(self, state, max_action, min_action, value):
        self.q_matrix[state][len(self.minimizer_actions) * max_action + min_action] = value
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
        return list(self.maximizer_policy[state].values())


    def get_min_policy_probabilities(self, state):
        return list(self.minimizer_policy[state].values())


    def next_state(self, state, max_action, min_action):
        action_index = len(self.minimizer_actions) * max_action + min_action
        return self.state_action_matrix[state][action_index]

    
    # Updates the maximizer policy based on the latest Q Values using linear programming 
    def update_maximizer_policy(self, state):
        best_policies = []
        for min_action in self.minimizer_actions:
            if self.valid_min_action(state, min_action):
                objective_coefficients = []
                a_ub = []
                b_ub = []
                a_eq = [[]]
                b_eq = [1]
                for max_action1 in self.maximizer_actions:
                    if self.valid_max_action(state, max_action1):
                        # linprog finds the min, and we want max so we multiply by -1
                        objective_coefficients.append(-1*self.get_q_matrix(state, max_action1, min_action))
                        # ensure weights sum to 1
                        a_eq[0].append(1)
                # conditions to ensure that solution is the minimum over other opponent actions
                for alt_min_action in self.minimizer_actions:
                    if self.valid_min_action(state, alt_min_action) :# and min_action != alt_min_action:
                        upper_bound_row = []
                        for max_action2 in self.maximizer_actions:
                            if self.valid_max_action(state, max_action2):
                                q_difference = self.get_q_matrix(state, max_action2, min_action) - self.get_q_matrix(state, max_action2, alt_min_action)
                                upper_bound_row.append(q_difference)
                        a_ub.append(upper_bound_row)
                        b_ub.append(0)
                try:
                    optimal_sol = linprog(objective_coefficients, a_ub, b_ub, a_eq, b_eq)
                    best_policies.append((optimal_sol.x, -1*optimal_sol.fun))
                except:
                    pass

        if len(best_policies) > 1:
            # pick out best solution
            best_sol_val = best_policies[0][1]
            best_sol_index = 0 
            for index, sol_pair in enumerate(best_policies):
                if sol_pair[1] > best_sol_val:
                    best_sol_index = index
                    best_sol_val = sol_pair[1]
            
            #construct new policy
            new_policy = {}
            valid_action_counter = 0
            value_sum = 0
            for action in self.maximizer_actions:
                if self.valid_max_action(state, action):
                    new_policy[action] = best_policies[best_sol_index][0][valid_action_counter]
                    value_sum += best_policies[best_sol_index][0][valid_action_counter]
                    valid_action_counter += 1
                else:
                    new_policy[action] = 0
            #normalize policy in case of linear programming errors
            for key in new_policy.keys():
                new_policy[key] = new_policy[key]/value_sum
            self.set_max_policy(state, new_policy)
        return

    
    # Updates the minimizer policy based on the latest Q Values using linear programming 
    def update_minimizer_policy(self, state):
        best_policies = []
        for max_action in self.maximizer_actions:
            if self.valid_max_action(state, max_action):
                objective_coefficients = []
                a_ub = []
                b_ub = []
                a_eq = [[]]
                b_eq = [1]
                for min_action1 in self.minimizer_actions:
                    if self.valid_min_action(state, min_action1):
                        objective_coefficients.append(self.get_q_matrix(state, max_action, min_action1))
                        # ensure weights sum to 1
                        a_eq[0].append(1)
                # conditions to ensure that solution is the maximum over other opponent actions
                for alt_max_action in self.maximizer_actions:
                    if self.valid_max_action(state, alt_max_action) :# and min_action != alt_min_action:
                        upper_bound_row = []
                        for min_action2 in self.minimizer_actions:
                            if self.valid_min_action(state, min_action2):
                                q_difference = self.get_q_matrix(state, alt_max_action, min_action2) - self.get_q_matrix(state, max_action, min_action2) 
                                upper_bound_row.append(q_difference)
                        a_ub.append(upper_bound_row)
                        b_ub.append(0)
            try:
                optimal_sol = linprog(objective_coefficients, a_ub, b_ub, a_eq, b_eq)
                best_policies.append((optimal_sol.x, optimal_sol.fun))
            except:
                pass
        if len(best_policies) > 1:
            # pick out best solution
            best_sol_val = best_policies[0][1]
            best_sol_index = 0 
            for index, sol_pair in enumerate(best_policies):
                if sol_pair[1] < best_sol_val:
                    best_sol_index = index
                    best_sol_val = sol_pair[1]
            
            #construct new policy
            new_policy = {}
            valid_action_counter = 0 
            value_sum = 0
            for action in self.minimizer_actions:
                if self.valid_min_action(state, action):
                    new_policy[action] = best_policies[best_sol_index][0][valid_action_counter]
                    value_sum += best_policies[best_sol_index][0][valid_action_counter]
                    valid_action_counter += 1
                else:
                    new_policy[action] = 0
            #normalize policy in case of linear programming errors
            for key in new_policy.keys():
                new_policy[key] = new_policy[key]/value_sum
            self.set_max_policy(state, new_policy)
            self.set_min_policy(state, new_policy)
        return


    def update_value(self, state):
        possible_values = []
        for min_action in self.maximizer_actions:
            if self.valid_min_action(state, min_action):
                sum = 0
                for max_action in self.maximizer_actions:
                    sum += self.get_max_policy(state)[max_action] * self.get_q_matrix(state, max_action, min_action)
                possible_values.append(sum)
        self.set_state_value(state, min(possible_values))

    
    # returns true if the given maximizer action is valid from the given state
    def valid_max_action(self, state, max_action) -> bool:
        for i in range(len(self.minimizer_actions)):
            action_index = len(self.minimizer_actions) * max_action + i 
            if self.state_action_matrix[state][action_index] >= 0:
                return True
        return False

    
    # returns true if the given minimizer action is valid from the given state
    def valid_min_action(self, state, min_action) -> bool:
        for i in range (len(self.maximizer_actions)):
            action_index = min_action + len(self.minimizer_actions)*i
            if self.state_action_matrix[state][action_index] >= 0:
                return True
        return False


if __name__ == "__main__":
    pass