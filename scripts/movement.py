#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

#initialize movement class
class Movement():
    def __init__(self):
        rospy.init_node('movement')
        vel_pub = rospy.Publisher('/q_learning/cmd_vel', Twist, queue_size=10)
        # cat publisher 
        self.tom_pub = rospy.Publisher('/tom/cmd_vel', Twist, queue_size=10)
        # mouse publisher 
        self.jerry_pub = rospy.Publisher('/jerry/cmd_vel', Twist, queue_size=10)
        
    # turn given robot 90 degrees at given speed in given direction 
    # let 0 = CW, 1 = CCW 
    # let 0 = tom, 1 = jerry 
    def turn_90(self, speed, direction, robot):
        vel_msg = Twist()
        ang_speed = (speed*2*math.pi)/360 
        angle = (90*2*math.pi)/360 
        # clockwise vs. counterclockwise 
        if direction == 0:
            vel_msg.angular.z = -abs(ang_speed)
        else: 
            vel_msg.angular.z = abs(ang_speed)
        time_start = rospy.Time.now().to_sec()
        curr_ang = 0 
        # continue turning until desired angle reached 
        # computed as time * speed 
        while(curr_ang < angle):
            if(robot == 0):
                self.tom_pub.publish(vel_msg)
            if(robot == 1):
                self.jerry_pub.publish(vel_msg)
            time_next = rospy.Time.now().to_sec()
            curr_ang = ang_speed * (time_next - time_start)
        vel_msg.angular.z = 0
        # determine which robot to publish to 
        if(robot == 0):
             self.tom_pub.publish(vel_msg)
        if(robot == 1):
            self.jerry_pub.publish(vel_msg)
        rospy.spin()
        return 1 
    # move given robot forward at given speed a given distance 
    # let 0 = tom, 1 = jerry 
    def move_forward(self, speed, distance, robot):
        vel_msg = Twist()
        curr_distance = 0 
        time_start = rospy.Time.now().to_sec()
        # continue moving until desired distance reached 
        # computed as time * speed 
        while curr_distance < distance:
            vel_msg.linear.x = speed 
            if(robot == 0):
                self.tom_pub.publish(vel_msg)
            if(robot == 1):
                self.jerry_pub.publish(vel_msg)
            curr_time = rospy.Time.now().to_sec()
            curr_distance = (curr_time - time_start) * speed 
        vel_msg.linear.x = 0
        # determine which robot to publish to 
        if(robot == 0):
             self.tom_pub.publish(vel_msg)
        if(robot == 1):
            self.jerry_pub.publish(vel_msg)
        return 1

    def run(self):
        rospy.sleep(5)
        # test functions 
        self.move_forward(0.1, 1.5, 0)
        self.move_forward(-0.1, 1.5, 1)


#runs functions upon execution 		
if __name__ == '__main__':
    rosnode = Movement()
    rosnode.run()
    rospy.spin()
