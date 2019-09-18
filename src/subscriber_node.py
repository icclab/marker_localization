#!/usr/bin/env python

import rospy
import numpy
import math
import random
import tf
import time

from nav_msgs.msg import OccupancyGrid
from ar_track_alvar_msgs.msg import AlvarMarkers

class SubscriberNode:

    def __init__(self):
        
        # dict that holds the ground pose
        self.initPose = {}
        self.initPose['x'] = 0
        self.initPose['y'] = 0
        self.initPose['z'] = 0
        self.initPose['qx'] = 0
        self.initPose['qy'] = 0
        self.initPose['qz'] = 0
        self.initPose['qw'] = 0

        # dict that holds the ground pose
        self.markerOffset = {}
        self.markerOffset['x'] = 0
        self.markerOffset['y'] = 0
        self.markerOffset['z'] = 0
        self.markerOffset['qx'] = 0
        self.markerOffset['qy'] = 0
        self.markerOffset['qz'] = 0
        self.markerOffset['qw'] = 0

        # dict that holds the marker pose
        self.markerPose = {}
        self.markerPose['x'] = 0
        self.markerPose['y'] = 0
        self.markerPose['z'] = 0
        self.markerPose['qx'] = 0
        self.markerPose['qy'] = 0
        self.markerPose['qz'] = 0
        self.markerPose['qw'] = 0
        self.markerPose['id'] = -1 
        self.markerPose['sec'] = -1 

	#  tf listener and read function
        self.robotPoseListener = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.11), self.cameraToBaseLink)
        rospy.Timer(rospy.Duration(0.11), self.map_to_marker)
        
        # Subscribers
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.markerPoseCallback, \
                    queue_size=100, buff_size=2**24)

    def cameraToBaseLink(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.robotPoseListener.lookupTransform\
                    ('summit_xl_base_link', 'summit_xl_front_rgbd_camera_rgb_optical_frame', rospy.Time(0))
                    #('camera_rgb_optical_frame', 'base_link', rospy.Time(0))
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            # Just print the error to try again
            rospy.logwarn("[Main Node] Error in tf transform from robot to map frame!")
            return

        # Updating the markerOffset pose

        self.markerOffset['x'] = translation[0]
        self.markerOffset['y'] = translation[1]
        self.markerOffset['z'] = translation[2]
        
        self.markerOffset['qx'] = rotation[0]
        self.markerOffset['qy'] = rotation[1]
        self.markerOffset['qz'] = rotation[2]
        
    def map_to_marker(self, event):
        if self.markerPose['id'] != -1:
            self.robotPoseListener.waitForTransform('map', 'ar_marker_' + str(self.markerPose['id']), rospy.Time(0), rospy.Duration(10.0))
	    (trans, quat) = self.robotPoseListener.lookupTransform('map', 'ar_marker_' + str(self.markerPose['id']), rospy.Time(0)) 

            self.initPose['x'] = trans[0]
            self.initPose['y'] = trans[1]
            self.initPose['z'] = trans[2]
            
            self.initPose['qx'] = quat[0]
            self.initPose['qy'] = quat[1]
            self.initPose['qz'] = quat[2]
            self.initPose['qw'] = quat[3]
        else:
            rospy.logwarn('Waiting for Marker Transform!!!')


    def markerPoseCallback(self,data):

        try:
            if True:
                # sequence
                self.markerPose['id'] = data.markers[0].id
                self.markerPose['sec'] = data.markers[0].header.stamp.secs
                # position
                self.markerPose['x'] = data.markers[0].pose.pose.position.x
                self.markerPose['y'] = data.markers[0].pose.pose.position.y 
                self.markerPose['z'] = data.markers[0].pose.pose.position.z 
                # rotation
                self.markerPose['qx'] = data.markers[0].pose.pose.orientation.x
                self.markerPose['qy'] = data.markers[0].pose.pose.orientation.y
                self.markerPose['qz'] = data.markers[0].pose.pose.orientation.z
                self.markerPose['qw'] = data.markers[0].pose.pose.orientation.w

        except (IndexError):
            print ('[Sub Node] Cannot Find any Markers!!!!')

