#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time

from abstractBot import *
from geometry_msgs.msg import Twist

class SampleBot(AbstractBot):
    def strategy(self):
        r = rospy.Rate(100)

        control_speed = 0
        control_turn = 0

        UPDATE_FREQUENCY = 2
        update_time = 0

        while not rospy.is_shutdown():
            if self.center_bumper or self.left_bumper or self.right_bumper:
                update_time = time.time()
                control_speed = -0.5
                control_turn = 0
            
            elif time.time() - update_time > UPDATE_FREQUENCY:
                update_time = time.time()
                
                value = random.randint(1,1000)
                if value < 500:
                	control_speed = 0.3
                	control_turn = 1
                else:
                	control_speed = 0.3
                	control_turn = -1

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('sample_bot')
    bot = SampleBot('Sample')
    bot.strategy()
