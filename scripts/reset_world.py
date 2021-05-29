#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tom_and_jerry_project.msg import QLearningReward
from std_msgs.msg import Header
from q_learning_project.msg import RobotCoord, GameState

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class ResetWorld(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('reset_world_q_learning')

        # reward amounts
        self.positive_reward_amount = 100
        self.negative_reward_amount = -1


        # reset position and orientation of the big robot
        self.tom_model_name = "tom"
        tom_x, tom_y, tom_z = 0.0
        self.tom_reset_position = Point(x=tom_x, y=tom_y, z=tom_z)
        tom_degree = 1.0
        self.tom_reset_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=tom_degree)

        self.tom_coord = (tom_x, tom_y, tom_degree)

        # TODO: add small robot model reset position and orientation of the small robot
        self.jerry_model_name = "jerry"
        jerry_x, jerry_y, jerry_z = 1.0
        self.jerry_reset_position = Point(x=jerry_x, y=jerry_y, z=jerry_z)
        jerry_degree = 1.0
        self.jerry_reset_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=jerry_degree)

        self.jerry_coord = (jerry_x, jerry_y, jerry_degree)

        # TODO: reset position of cheese (randomize?) and add cheese models
        self.cheese_coords = [(1,2,0), (4,6,0)]

        # flag to keep track of the state of when we're resetting the world and when we're not
        # to avoid sending too many duplicate messages
        self.reset_world_in_progress = False

        # keep track of the iteration number
        self.iteration_num = 0
        
        # ROS subscribers
        rospy.Subscriber("/q_learning/state", GameState, self.state_received)

        # ROS publishers
        self.reward_pub = rospy.Publisher("/q_learning/reward", QLearningReward, queue_size=10)

        self.run()


    def state_received(self, data):

        # get the locations of tom and jerry
        temp_tom = data.tom_coord
        temp_jerry = data.jerry_coord

        self.tom_coord = (temp_tom.x_coord, temp_tom.y_coord, temp_tom.z_coord)
        self.jerry_coord = (temp_jerry.x_coord, temp_jerry.y_coord, temp_jerry.z_coord)

        # based on the locations publish a reward
        is_next_move= False
        if data.move_num != self.iteration_num: #TODO: not sure about this
            is_next_move = True

        if (is_next_move and not self.reset_world_in_progress):
            tom_wins = False
            jerry_wins = False

            # assign reward
            if self.tom_coord[0] == self.jerry_coord[0] and self.tom_coord[1] == self.jerry_coord[1]:
                tom_wins = True 
                tom_reward = self.positive_reward_amount
                print(f"Tom caught jerry at {self.tom_coord[0]},{self.tom_coord[1]}")
            else:
                tom_reward = self.negative_reward_amount
            if (self.jerry_coord[0], self.jerry_coord[1], 0) in self.cheese_coords and not tom_wins:
                jerry_wins = True
                jerry_reward = self.positive_reward_amount
                print(f"Jerry got the cheese at {self.jerry_coord[0]},{self.jerry_coord[1]}")
            else:
                jerry_reward = self.negative_reward_amount

            # publish reward
            reward_msg = QLearningReward()
            reward_msg.header = Header(stamp=rospy.Time.now())
            reward_msg.reward_tom = tom_reward
            reward_msg.reward_jerry = jerry_reward            
            reward_msg.iteration_num = self.iteration_num
            self.reward_pub.publish(reward_msg)
            print(f"Tom reward: {tom_reward}")
            print(f"Jerry reward: {jerry_reward}")

        # if tom has caught jerry or jerry is at a cheese end the game
        if ((tom_wins or jerry_wins) and not self.reset_world_in_progress):

            self.reset_world_in_progress = True

            # reset world
            p = Pose(position=self.tom_reset_position, orientation=self.tom_reset_orientation)
            t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
            tom_model_state = ModelState(model_name=self.tom_model_name, pose=p, twist=t)
            self.model_states_pub.publish(tom_model_state)

            p = Pose(position=self.jerry_reset_position, orientation=self.jerry_reset_orientation)
            t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
            jerry_model_state = ModelState(model_name=self.jerry_model_name, pose=p, twist=t)
            self.model_states_pub.publish(jerry_model_state)

        elif (not (tom_wins or jerry_wins) and self.reset_world_in_progress):
            self.reset_world_in_progress = False
            self.iteration_num += 1

    def run(self):
        rospy.spin()


if __name__=="__main__":

    node = ResetWorld()