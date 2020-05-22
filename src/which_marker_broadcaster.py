#!/usr/bin/env python
 
import rospy
import tf
from math import hypot, atan2, pi, sqrt, degrees, cos, sin

from subscriber_node import SubscriberNode

class tfBroad:

    def __init__(self):

        # get the stuff from the subClass
        self.subNode = SubscriberNode()
        # call the function
        rospy.Timer(rospy.Duration(0.05), self.broadCastTf)

    def broadCastTf(self, event):
        # get the ID
        markerID = self.subNode.markerPose
        # print it
        # rospy.logdebug('the id is %d', markerID['id'])
        
        # Big Markers 13.4 cm side
        if markerID['id'] == 3: # my desk!!
            # make the translation
            # marker from map in the cloud
            trans = (0.2, 0.9, 0.11)
            # marker from map on map server
            # trans = (-0.54, 2.07, 0.14)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 3.1415926535)
            # set the name of the marker frame
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function
            self.sendTransform(trans, quat, marker)

        # Small Markers
        # I think this way it is more readable
        if markerID['id'] == 11: # west wall
            # make the translation
            # marker from map in the cloud
            trans = (2.9, 6.0, 0.11)
	    # marker from map on map server
            # trans = (-3.6, 7.26, 0.11)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 1.570796325)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)
        
        elif markerID['id'] == 12: # panos's desk
            # make the translation
            # marker from map in the cloud
            trans = (1.85, 4.5, 0.11)
	    # marker from map from map server
            # trans = (-1.95, 5.5, 0.14)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 0)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)

        elif markerID['id'] == 13: # my desk
            # make the translation
 	    # marker from map in the cloud
            trans = (0.2, 0.9, 0.11)
	    # marker from map from map server
            # trans = (-0.54, 2.07, 0.14)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 3.1415926535)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)
        
        elif markerID['id'] == 14: # on ali's desk
            # make the translation
            # marker from map in the cloud
            trans = (3.82, 1.15, 0.11)
	    # marker from map from map server in robot.
            # trans = (1.72, 4.66, 0.11)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 0)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)

        else:
            print 'No Marker Are Found!!!'

    ## ARGS
    # arg1 :class variables
    # arg2 :translation of the transform
    # arg3 :quaternion of the transform
    # arg4 :marker id
    def sendTransform(self, translation, quaternion, marker):
        # init the transform
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(100.0) # 10 hz
        # sendTransform(translation, rotation, time, child, parent)
        br.sendTransform(translation,
                  quaternion,
                  rospy.Time.now(),
                  marker,
                  "map")
        # print "Transform is Sent!!"
        rate.sleep()

