#!/usr/bin/env python
from __future__ import division 

import os 
import rospy
import actionlib
import tf
import time 
from math import hypot, atan2, pi, sqrt, degrees, cos, sin

from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node import SubscriberNode

## THE ORIENTATION OF THE MAP IN RELATION OF THE AXI SHOULD BE GIVEN AS 
## PARAMETER TO THE PROBLEM



class MarkerLocalization:

    def __init__(self):
        
        # robot pose dict
        self.robotPose = {}
        self.robotPose['x'] = 0
        self.robotPose['y'] = 0
        self.robotPose['z'] = 0

        self.robotPose['qx'] = 0
        self.robotPose['qy'] = 0
        self.robotPose['qz'] = 0
        self.robotPose['qw'] = 0
        

        # move Base Msg
        self.subNode = SubscriberNode()
        
        rospy.loginfo("[Main Node] Wait 10 seconds for the subscribers to be ready!")
        # time.sleep(10)
        
        self.tinit = time.time()
        # set up the Publisher
        self.posePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 3, latch=True)

        # Check from marker pose every second
        rospy.Timer(rospy.Duration(1.0), self.calculateRobotPose)

    def calculateRobotPose(self, event):
        # calculates robot Pose 
        markerPose = self.subNode.markerPose
        link_to_camera = self.subNode.markerOffset
        map_to_marker = self.subNode.initPose
	
	

####### map_to_marker quaternion ################
        # store the quat in a list
        # minus sign in W element to inverse the quaternion!!! 
        quatMM = [map_to_marker['qx'], map_to_marker['qy'], map_to_marker['qz'], map_to_marker['qw']] 
####### Marker Quaternion from alvar ############    
        # Get the quat from the transform 
        quatMC = [markerPose['qx'], markerPose['qy'], markerPose['qz'], -markerPose['qw']]
        
####### Transformations #########################
    
        # Get the quat from the transform 
        # quatLC = [link_to_camera['qx'], link_to_camera['qy'], link_to_camera['qz'], link_to_camera['qw']]
	quatLC = [0, 0, 0, 1]
####### determine the robot pose ################
	

 	print 'Link to camera trans'
	print 'x: ', link_to_camera['x']
        print 'y: ', link_to_camera['y']
        print 'z: ', link_to_camera['z']

	print 'markerPose'
	print 'x: ', markerPose['x']
        print 'y: ', markerPose['y']
        print 'z: ', markerPose['z']

	print 'map to marker '
	print 'x: ', map_to_marker['x']
        print 'y: ', map_to_marker['y']
        print 'z: ', map_to_marker['z']


        self.robotPose['x'] = map_to_marker['x'] - markerPose['z'] - link_to_camera['x'] 
        self.robotPose['y'] = map_to_marker['y'] - markerPose['y'] - link_to_camera['y'] 
        self.robotPose['z'] = map_to_marker['z'] - markerPose['x'] - link_to_camera['z']

            
        # this quat is the initial rotation of the map we kinda wann counter
        # orientation for map server map
        # quatInit = tf.transformations.quaternion_from_euler(0, 0, -0.436332)
        # orientation for clould map when node runs from lap top????
        angle = -(9*pi)/18
        #quatInit = tf.transformations.quaternion_from_euler(0, 0, angle)
        # orientation for cloud map 
        quatInit = tf.transformations.quaternion_from_euler(0, 0, -pi/4 -pi/8 - pi/12)
        quatLC = tf.transformations.quaternion_multiply(quatLC, quatInit)
        quat1 = tf.transformations.quaternion_multiply(quatMC, quatLC)
        quatF = tf.transformations.quaternion_multiply(quatMM, quat1)
    	

	# Normalize the quaternion
	quatFN = [0, 0, 0, 1]
	
	quatFN[0] = quatF[0]/sqrt(quatF[0]**2 + quatF[1]**2 + quatF[2]**2 + quatF[3]**2)
	quatFN[1] = quatF[1]/sqrt(quatF[0]**2 + quatF[1]**2 + quatF[2]**2 + quatF[3]**2)
	quatFN[2] = quatF[2]/sqrt(quatF[0]**2 + quatF[1]**2 + quatF[2]**2 + quatF[3]**2)
	quatFN[3] = quatF[3]/sqrt(quatF[0]**2 + quatF[1]**2 + quatF[2]**2 + quatF[3]**2)
	    
        euler_angleF = tf.transformations.euler_from_quaternion(quatFN)
        # transform radians to degrees
        degrees_angleF = map(degrees, euler_angleF)

	
        # chnage translation according to orientation             
        self.robotPose['x'] = self.robotPose['x'] + markerPose['z'] * (1 - cos(euler_angleF[2])) 
        self.robotPose['y'] = self.robotPose['y'] - markerPose['z'] * sin(euler_angleF[2]) 
        
        # print the translation (position) of the robot
        print 'the robot position is:'
        print 'x: ', self.robotPose['x']
        print 'y: ', self.robotPose['y']
        print 'z: ', self.robotPose['z']
        
        # print orientation of the robot (roll, pitch, yaw)
        print 'the orientation of the robot is:'        
        print degrees_angleF
        print quatFN        
        # init the msg
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = '/map'
        # position 
        msg.pose.pose.position.x = self.robotPose['x']
        msg.pose.pose.position.y = self.robotPose['y']
        msg.pose.pose.position.z = 0.0
        #  orientation
	
	# DOES NOT WORK FOR SOME REASON. I GET NOT A NUMBER ERROR	
        msg.pose.pose.orientation.x = quatFN[0]
        msg.pose.pose.orientation.y = quatFN[1]
        msg.pose.pose.orientation.z = quatFN[2]
        msg.pose.pose.orientation.w = quatFN[3]
       
        # init covariance
        msg.pose.covariance = [0] * 36

        # publish the pose msg
        self.posePub.publish(msg)

        dt = self.checkTime()
        # print the time passed
        rospy.logdebug('time passed: %f', dt)
        if dt > 30:
            # shutdown the ar track alvar node
            # hard coded
            os.system('rosnode kill' + ' /ar_track_alvar')
            os.system('rosnode kill' + ' /raspicam_node')
            # sleep for 8 seconds
            time.sleep(8)
            # kill this node  
            rospy.signal_shutdown('Position Determined!!')

 
    def checkTime(self):
        # check the time passed
        dt = time.time() - self.tinit         
        return dt       
