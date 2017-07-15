#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.msg import ModelStates 
from sensor_msgs.msg import Image
import message_filters

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class AbstractBot(object):
    __metaclass__ = ABCMeta

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name

        # bumper state
        self.bumper = BumperEvent()
        self.center_bumper = False
        self.left_bumper = False
        self.right_bumper = False

        # for convert image topic to opencv obj
        self.bridge = CvBridge()

        # velocity publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

        # bumper subscrivre
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumperCallback)

        # camera subscriver
        # please uncoment out if you use camera
        # get rgb and depth image  http://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/
        self.image_sub = message_filters.Subscriber("camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)

        # ApproximateTimeSynchronizer http://docs.ros.org/api/message_filters/html/python/
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 0.1)
        self.ts.registerCallback(self.imageDepthcallback)

        # view gui flag
        self.cv_view = True

        # red region near position(loc) and distance(val)
        self.red_loc = np.zeros((2))
        self.red_val = np.array([0.0])

        # yellow region near position(loc) and distance(val)
        self.yel_loc = np.zeros((2))
        self.yel_val = np.array([0.0])

        # center position and max value
        self.loc_centor_x  = np.array([1])
        self.loc_max       = np.zeros((2))

        # average depth
        self.ave_val = np.array([0.0])
        self.M_val = np.array([0.0])
        self.L_val = np.array([0.0])
        self.R_val = np.array([0.0])        

        # step
        self.step = np.array([0])

    # bumper topic call back sample
    # update bumper state
    def bumperCallback(self, data):
        if data.bumper == 0:
            if data.state == 1:
                self.left_bumper = True
            else:
                self.left_bumper = False
                
        if data.bumper == 1:
            if data.state == 1:
                self.center_bumper = True
            else:
                self.center_bumper = False

        if data.bumper == 2:
            if data.state == 1:
                self.right_bumper = True
            else:
                self.right_bumper = False

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageDepthcallback(self, rgb_data, depth_data):
        try:
            rgb_image   = self.bridge.imgmsg_to_cv2(rgb_data,   "bgr8")

            # resize  http://tatabox.hatenablog.com/entry/2013/07/15/164015
            h = rgb_image.shape[0]
            w = rgb_image.shape[1]            
            rgb_image_ = cv2.resize(rgb_image,(w/4,h/4))

            if w <= 0:
                self.loc_centor_x = 640/8;
                self.loc_max[0]   = 640/4;
                self.loc_max[1]   = 480/4;                                
            else:
                self.loc_centor_x = w/8;
                self.loc_max[0]   = w/4;
                self.loc_max[1]   = h/4;

            # reffer http://answers.ros.org/question/58902/how-to-store-the-depth-data-from-kinectcameradepth_registreredimage_raw-as-gray-scale-image/
            #depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")            
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")

            # resize  http://tatabox.hatenablog.com/entry/2013/07/15/164015            
            h = depth_image.shape[0]
            w = depth_image.shape[1]
            depth_image_ = cv2.resize(depth_image,(w/4,h/4))            
        except CvBridgeError, e:
            print e

        #depth_array = np.array(depth_image, dtype=np.float32)
        depth_array = np.array(depth_image_, dtype=np.float32)        
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        #get middle average depth
        #get 1row    http://qiita.com/supersaiakujin/items/d63c73bb7b5aac43898a
        #cal average http://programming.blogo.jp/python/average 

        #mid = depth_array[60,:]
        #if 0 < len(mid) and sum(mid) != 0:
        #    self.ave_val = sum(mid) / len(mid)
        #else:
        #    self.ave_val = 0

        # array access  http://qiita.com/supersaiakujin/items/d63c73bb7b5aac43898a
        
        self.L_val = depth_array[ 60,  2]
        self.M_val = depth_array[ 60, 80]
        self.R_val = depth_array[ 60,158]

        # --- test Red Region ---
        # https://www.blog.umentu.work/python3-opencv3%E3%81%A7%E6%8C%87%E5%AE%9A%E3%81%97%E3%81%9F%E8%89%B2%E3%81%AE%E3%81%BF%E3%82%92%E6%8A%BD%E5%87%BA%E3%81%97%E3%81%A6%E8%A1%A8%E7%A4%BA%E3%81%99%E3%82%8B%E3%80%90%E5%8B%95%E7%94%BB/
        # BGR array
        # extract red region
        lower_red = np.array([0,    0,   64])
        upper_red = np.array([8,    8,  255])       
        red_mask = cv2.inRange(rgb_image_, lower_red, upper_red)
        red_image = cv2.bitwise_and(rgb_image_, rgb_image_,   mask=red_mask) 
        red_depth = cv2.bitwise_and(depth_array, depth_array, mask=red_mask)

        # extract yellow region
        lower_yel = np.array([0,    64,   64])
        upper_yel = np.array([8,    255,  255])       
        yel_mask = cv2.inRange(rgb_image_, lower_yel, upper_yel)
        yel_image = cv2.bitwise_and(rgb_image_, rgb_image_,   mask=yel_mask)
        yel_depth = cv2.bitwise_and(depth_array, depth_array, mask=yel_mask)        

        # extract depth region
        # lower_dep = 0.0
        # upper_dep = 0.2      
        # dep_mask = cv2.inRange(depth_array, lower_dep, upper_dep)
        # dep_image = cv2.bitwise_and(rgb_image_, rgb_image_, mask=dep_mask)

        # extract max value
        # http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_contours/py_contour_properties/py_contour_properties.html#contour-properties
        # --- search red near area ---
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(red_depth, mask=red_mask)
        self.red_loc = min_loc
        self.red_val = min_val
        # if not exit max_loc = [-1, -1]
        #cv2.circle(red_image, self.red_loc, 3, (255, 255, 255), -1)
        r = int((1.0 - self.red_val)*10.0)
        if r < 0 :
            r = 0
        if min_loc[0] == -1 and min_loc[1] == -1 :
            r = 0
        cv2.circle(red_image, self.red_loc, r, (255, 255, 255), -1)        

        # --- search yellow near area ---
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(yel_depth, mask=yel_mask)
        self.yel_loc = min_loc
        self.yel_val = min_val
        # if not exit max_loc = [-1, -1]        
        # cv2.circle(yel_image, self.yel_loc, 3, (255, 255, 255), -1)
        r = int((1.0 - self.yel_val)*10.0)
        if r < 0 :
            r = 0
        if min_loc[0] == -1 and min_loc[1] == -1 :
            r = 0            
        cv2.circle(yel_image, self.yel_loc, r, (255, 255, 255), -1)

        # --- debug ---------------------------------------------
        # step5 (bumper white)
        if self.step == 5:
            cv2.rectangle(rgb_image_, (0,0), (160,120), (255, 0, 0), 3)
        # step6 (near wall)
        if self.step == 6:
            cv2.rectangle(rgb_image_, (0,0), (160,120), (255, 255, 0), 3)            
        # step1 (yellow)            
        elif self.step == 1:
            cv2.rectangle(rgb_image_, (0,0), (160,120), (0, 255, 255), 3)
        # step2 (red)            
        elif self.step == 2:
            cv2.rectangle(rgb_image_, (0,0), (160,120), (0, 0, 255), 3)
        # step3 (turn)            
        elif self.step == 3:
            cv2.rectangle(rgb_image_, (0,0), (160,120), (0, 255, 0), 3) 
        # step4 (random white)            
        elif self.step == 4:
            cv2.rectangle(rgb_image_, (0,0), (160,120), (255, 255, 255), 3)

        if self.cv_view == True:
            cv2.imshow("Image window", rgb_image_)            
            cv2.imshow("Depth window", depth_array)
            cv2.imshow("Red Image window", red_image)
            #cv2.imshow("Red Depth window", red_depth)
            cv2.imshow("Yellow Image window", yel_image)
            #cv2.imshow("Yellow Depth window", yel_depth)                        
            #cv2.imshow("near depth Image window", dep_image)                        
            cv2.waitKey(1)

    @abstractmethod
    def strategy(self):
        pass

