#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import random
import time

from abstractBot import *
from geometry_msgs.msg import Twist

class RandomBot(AbstractBot):
    
    def strategy(self):
        r = rospy.Rate(100)

        #target        
        target_speed = 0
        target_turn  = 0

        #control
        control_speed = 0
        control_turn = 0

        #max
        max_control_speed =  0.5
        min_control_speed = -0.5

        #time
        #UPDATE_FREQUENCY = 1
        UPDATE_FREQUENCY = 0.1
        update_time = 0

        #flag
        bum_flag = False
        yel_flag = False        
        red_flag = False
        rot_flag = False

        th = 0.0
        sp = 0.0

        while not rospy.is_shutdown():
            # step5: bumper            
            if self.center_bumper or self.left_bumper or self.right_bumper:
                self.step = 5                                
                update_time = time.time()
                rospy.loginfo('bumper hit!!')                       

                control_speed = -0.05
                target_speed  = -0.05

                #control_turn = 1
                #target_turn  = 1

                if self.L_val < self.R_val:
		#control_turn  = 1.0                    
		     target_turn  = 1.0
                else:
                    target_turn  = -1.0 

                # init
                th_cnt = 0.0
                bum_flag = True                
                yel_flag = False
                red_flag = False
                rot_flag = False

            # --- use rgb and depth data ---
            # step6: neal wall
            #elif 0.01 > self.L_val or 0.01 > self.R_val or 0.01 > self.M_val:
            elif 0.05 > self.M_val:    
                self.step = 6                
                update_time = time.time()
                rospy.loginfo('near wall ---- !!')

                control_speed = 0
                target_speed  = 0

                #if self.L_val > self.R_val:
                control_turn = -1.0
                target_turn  = -1.0
                #else:
                #    control_turn = -1.0
                #    target_turn  = -1.0

                # init
                th_cnt = 0.0
                bum_flag = True                
                yel_flag = False
                red_flag = False
                rot_flag = False                

            elif time.time() - update_time > UPDATE_FREQUENCY:
                update_time = time.time()
                
                # --- use rgb and depth data ---
                # step1: extract yellow (yellow)
                if self.yel_loc[0] != -1 and self.yel_loc[1] != -1:
                    self.step = 1

                    # init
                    th_cnt = 0.0
                    if bum_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        bum_flag = False                        
                    red_flag = False
                    yel_flag = True
                    if rot_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        rot_flag = False

                    loc_x = self.yel_loc[0]
                    cen_x = self.loc_centor_x
                    val   = self.red_val

                    th = 0.5*np.arctan((1.5*(loc_x - cen_x)/cen_x))
                    target_turn  = -th

                    if   0.01 < val:
                        sp = 0.50
                    else:
                        sp = 0.48
                    target_speed = sp
                    
                # step2: extract red (red)
                elif self.red_loc[0] != -1 and self.red_loc[1] != -1:
                    self.step = 2
                    
                    # init
                    th_cnt = 0.0
                    if bum_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        bum_flag = False                                            
                    yel_flag = False
                    red_flag = True
                    if rot_flag:
                        target_turn  = 0
                        control_turn = 0
                        #target_speed  = max_control_speed
                        #control_speed = max_control_speed
                        rot_flag = False

                    loc_x = self.red_loc[0]
                    cen_x = self.loc_centor_x
                    val   = self.red_val

                    th = 0.5*np.arctan((1.5*(loc_x - cen_x)/cen_x))
                    target_turn  = -th

                    if   0.01 < val:
                        sp = 0.50
                    else:
                        sp = 0.48
                    target_speed = sp

		    #print('loc_x:%f cen_x:%f' %(loc_x, cen_x))

                # step3: check total cnt of rotation th (green)
                elif th_cnt < 15 and (yel_flag or red_flag):
		#elif yel_flag or red_flag:
                    self.step = 3

                    #init
                    rot_flag = True

                    #control_speed = 0
                    target_speed  = -0.1
                    #target_speed  = target_speed - 0.2

                    #control_turn = 1.5
                    if self.L_val > self.R_val:
                        target_turn  = 1.0
                    else:
                        target_turn  = -1.0                        
                    th_cnt = th_cnt +1

                # step4: random (while)
                else:
                    self.step = 4
                    
                    # init
                    th_cnt =   th_cnt - 5
                    if th_cnt < 0:
                        th_cnt = 0

                    yel_flag = False
                    red_flag = False
                    rot_flag = False                    

                    # random  http://www.python-izm.com/contents/application/random.shtml
                    #rand = random.uniform(0.0, 1.0)
                    #if   0.75 < rand:
                    #    target_speed  = -0.5
                    #    target_turn   = -3
                    #elif 0.50 < rand:
                    #    target_speed  = 0.5
                    #    target_turn   = -3
                    #elif 0.25 < rand:
                    #    target_speed  = 0.5
                    #    target_turn   = -3
                    #else:
                    #    target_speed  = 0.5
                    #    target_turn   = 3

		    target_speed  = 0.5
                    target_turn   = -3

            #print http://programming-study.com/technology/python-print/
            #print('speed(%f, %f)_turn(%f, %f)_th(%f)_sp(%f)_(%f, %f, %f)_ave(%f)' %(target_speed, control_speed, target_turn, control_turn, th, sp, self.L_val, self.M_val, self.R_val, self.ave_val))
	    print('speed(%f, %f)_turn(%f, %f)' %(target_speed, control_speed, target_turn, control_turn))

            # def 0.02
            del_speed  = 0.1
            # def 0.1
            del_turn   = 0.2

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + del_speed )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - del_speed )
            else:
                control_speed = target_speed

            #max speed check
            control_speed = min (control_speed, max_control_speed)
            control_speed = max (control_speed, min_control_speed)

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + del_turn )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - del_turn)
            else:
                control_turn = target_turn  

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            #print(twist)
        
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_bot')
    bot = RandomBot('Random')
    bot.strategy()
