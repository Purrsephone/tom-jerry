#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class Movement():
    rospy.init_node('movement')
    vel_pub = rospy.Publisher('/q_learning/cmd_vel', Twist, queue_size=10)

    # turn 90 degrees at given speed in given direction 
    # let 0 = CW, 1 = CCW 
    def turn_90(self, speed, direction):
        vel_msg = Twist()
        ang_speed = (speed*2*math.pi)/360 
        angle = (90*2*math.pi)/360 
        if direction == 0:
            vel_msg.angular.z = -abs(ang_speed)
        else: 
            vel_msg.angular.z = abs(ang_speed)
        time_start = rospy.Time.now().to_sec()
        curr_ang = 0 
        while(curr_ang < angle):
            self.vel_pub.publish(vel_msg)
            time_next = rospy.Time.now().to_sec()
            curr_ang = ang_speed * (time_next - time_start)
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.spin()
        return 1 

    def move_forward(self, speed):
        vel_msg = Twist()
        vel_msg.linear.x = speed 
        self.vel_pub.publish(vel_msg)

    def run(self):
        test = self.turn_90(0.1, 0)
        if test==1: 
            move_forward(0.3)

#runs functions upon execution 		
if __name__ == '__main__':
	rosnode = Movement()
	rosnode.run()