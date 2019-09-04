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
        rospy.Timer(rospy.Duration(0.11), self.broadCastTf)

    def broadCastTf(self, event):
        # get the ID
        markerID = self.subNode.markerPose
        # print it
        rospy.logdebug('the id is %d', markerID['id'])
        
        # I think this way it is more readable
        if markerID['id'] == 11:
            # make the translation
            trans = (-3.6, 7.26, 0.11)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 1.570796325)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)
        
        elif markerID['id'] == 12:
            # make the translation
            # trans = (-1.95, 5.5, 0.14)
            trans = (1.85, 4.4, 0.14)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 0)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)

        elif markerID['id'] == 13:
            # make the translation
            trans = (-0.54, 2.07, 0.14)
            # make a quat for the function
            quat = tf.transformations.quaternion_from_euler(0, -1.570796325, 3.1415926535)
            # set the name of the marker frame 
            marker = 'ar_marker_' + str(markerID['id'])
            # call the function 
            self.sendTransform(trans, quat, marker)
        
        elif markerID['id'] == 14:
            # make the translation
            # trans = (1.72, 4.66, 0.11)
            trans = (3.45, 1.05, 0.11)
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
        print "Transform is Sent!!"
        rate.sleep()

