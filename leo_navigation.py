#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 15:24:41 2022

@author: leo
"""

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
import ros_numpy
import numpy as np
from tf.transformations import euler_from_quaternion , quaternion_from_euler


#GENERAL PURPUSES VARIABLES
pub_hz=0.1 #main loop frequency
#ROS PUBLISHER SET UP
pub_img = rospy.Publisher('image_line_mask', Image,queue_size=1)
msg_img = Image()       
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cmd_vel = Twist()


class robot_class:
    def __init__(self): #It is done only the first iteration
        #Variables to store temporally the info of human detectection at each time instant
        self.pos_raw=np.zeros([self.n_human,3]) #robot position from odometry
        self.pos_corrected=np.zeros([self.n_human,3]) #robot position corrected from camera feedback
        self.color_image=np.zeros((848,400,3), np.uint8) #rgb image, initial value
        self.checkpoint_id=0 #identification number of the location along the rows 
        self.vel=[0,0,0] #[linear_x,linear_y,angular]
    
    def image_callback(self,rgb):
        color_image = ros_numpy.numpify(rgb) #replacing cv_bridge
        color_image = color_image[...,[2,1,0]].copy() #from bgr to rgb
        self.line_following(color_image)
   
    def position_callback(self,pose):
        self.pos_raw[0]=pose.position.x
        self.pos_raw[1]=pose.position.y
        quat = pose.orientation    
        # From quaternion to Euler
        angles = euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        theta = angles[2]
        self.pos_raw[2]=np.unwrap([theta])[0]
   
    def line_following(self,image):
        #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        # change below lines to map the color you wanted robot to follow
        lower_yellow = numpy.array([ 50,  10,  10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            #cy = int(M['m01']/M['m00'])
            #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # CONTROL starts
            err = cx - w/2
            self.vel[0] = 0.2
            self.vel[2] = -float(err) / 100
            #self.cmd_vel_pub.publish(self.twist)
            # CONTROL ends
        #Publishing Mask
        msg_img.header.stamp = rospy.Time.now()
        msg_img.height = mask.shape[0]
        msg_img.width = mask.shape[1]
        msg_img.encoding = "rgb8"
        msg_img.is_bigendian = False
        msg_img.step = 3 * mask.shape[1]
        msg_img.data = np.array(mask).tobytes()
        pub_img.publish(msg_img)
        #Publishing control signals
        cmd_vel.linear.x = robot.vel[0]
        cmd_vel.linear.y = robot.vel[1]
        cmd_vel.angular.z = robot.vel[2]
        cmd_pub.publish(cmd_vel) 
        
#MAIN
    
robot=robot_class()  
rospy.init_node('leo_navigation',anonymous=True)
# Setup and call subscription
rospy.Subscriber('/camera/image_raw', Image, robot.image_callback)
rospy.Subscriber('/wheel_pose',)
rospy.Subscriber('/robot_pose', PoseStamped, robot.position_callback)  
#Rate setup
rate = rospy.Rate(1/pub_hz) # main loop frecuency in Hz
while not rospy.is_shutdown():
    #robot.navigation_plan()     
        
    rate.sleep() #to keep fixed the publishing loop rate
