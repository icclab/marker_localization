#!/usr/bin/env python
from __future__ import division 
 
import rospy
import actionlib
import tf
import time 
from math import hypot, atan2, pi, sqrt, degrees, cos, sin

from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from subscriber_node import SubscriberNode
#from ar_track_alvar.msg import AlvarMarkers

## THE ORIENTATION OF THE MAP IN RELATION OF THE AXI SHOULD BE GIVEN AS 
## PARAMETER TO THE PROBLEM



class TrackToGoal:

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
        time.sleep(10)
        
        # set up the Publisher
        self.posePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 3, latch=True)

        # Check For goal every minute???
        rospy.Timer(rospy.Duration(1.0), self.calculateSendGoal)

    def calculateSendGoal(self, event):
        
        markerPose = self.subNode.markerPose
        link_to_camera = self.subNode.markerOffset
        map_to_marker = self.subNode.initPose


####### map_to_marker quaternion #############
        # store the quat in a list
        # minus sign in W element to inverse the quaternion!!! 
        quatMM = [map_to_marker['qx'], map_to_marker['qy'], map_to_marker['qz'], map_to_marker['qw']] 



####### Marker Quaternion from alvar ############    
        # Get the quat from the transform 
        quatMC = [markerPose['qx'], markerPose['qy'], markerPose['qz'], -markerPose['qw']]
        

######### Transformations ###########
    
        # Get the quat from the transform 
        quatLC = [link_to_camera['qx'], link_to_camera['qy'], link_to_camera['qz'], link_to_camera['qw']]

########## determine the robot pose ####
    
    #drift form init pose
    # c for corner
#        c = 0.4363332
#        y = - cos(c) * (markerPose['z'] + link_to_camera['x'])
#        x = sin(c) * (markerPose['z'] + link_to_camera['x'])

        self.robotPose['x'] = map_to_marker['x'] - markerPose['z'] - link_to_camera['x'] 
        self.robotPose['y'] = map_to_marker['y'] - markerPose['y'] - link_to_camera['y'] 
        self.robotPose['z'] = map_to_marker['z'] - markerPose['x'] - link_to_camera['z']

        



#        print 'the robot pose is:'
#        print 'x: ', self.robotPose['x']
#        print 'y: ', self.robotPose['y']            
#        print 'z: ', self.robotPose['z']


        if True:  #abs(self.robotPose['z']) < 0.05: # and self.robotPose['x'] < 2:
            
            # Apply the rotations!!
            #quat1 = tf.transformations.quaternion_multiply(quatMC, quatMM)
            #quatF = tf.transformations.quaternion_multiply(quatLC, quat1)

            # this quat is the initial rotation of the map we kinda wann counter
            # quatInit = tf.transformations.quaternion_from_euler(0, 0, -0.436332)
            quatInit = tf.transformations.quaternion_from_euler(0, 0, -pi/2)
            quatLC = tf.transformations.quaternion_multiply(quatLC, quatInit)
            quat1 = tf.transformations.quaternion_multiply(quatMC, quatLC)
            quatF = tf.transformations.quaternion_multiply(quatMM, quat1)
        
        
            euler_angleF = tf.transformations.euler_from_quaternion(quatF)
            # transform radians to degrees
            degrees_angleF = map(degrees, euler_angleF)
            if True: #10 < degrees_angleF[2] < 170:
                print ('I am Here!!!')
                self.robotPose['x'] = self.robotPose['x'] + markerPose['z'] * (1 - cos(euler_angleF[2])) 
                self.robotPose['y'] = self.robotPose['y'] - markerPose['z'] * sin(euler_angleF[2]) 

            print 'the robot pose is:'
            print 'x: ', self.robotPose['x']
            print 'y: ', self.robotPose['y']
            print 'z: ', self.robotPose['z']

#       
            print 'the pose of the robot is:'        
            print degrees_angleF
            
            # init the msg

            msg = PoseWithCovarianceStamped()

            msg.header.frame_id = '/map'
            # position 
            msg.pose.pose.position.x = self.robotPose['x']
            msg.pose.pose.position.y = self.robotPose['y']
            msg.pose.pose.position.z = 0.0
            #  orientation
            msg.pose.pose.orientation.x = quatF[0]
            msg.pose.pose.orientation.y = quatF[1]
            msg.pose.pose.orientation.z = quatF[2]
            msg.pose.pose.orientation.w = quatF[3]
            # init covariance
            msg.pose.covariance = [0] * 36

            # publish the pose msg
            self.posePub.publish(msg)

    
    def initPoseDrift(quatMC, MarkerPose):
        pass


                
