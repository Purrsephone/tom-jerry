#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class Movement():

    def turn_90(self, speed, direction):
        pass

    def move_forward(self, speed):
        pass 

#runs functions upon execution 		
if __name__ == '__main__':
	rosnode = Movement()
	rosnode.run()