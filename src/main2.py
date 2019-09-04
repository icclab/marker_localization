#!/usr/bin/env python

import rospy
import time

#from track_to_pose import TrackToGoal
from track_to_pose2 import TrackToGoal
from which_marker_broadcaster import tfBroad


# The main function of the program
if __name__ == '__main__':

    # Wait for simulator and SLAM to initialize
    print "Waiting 5 seconds for initialization"
#    time.sleep(5)
    
    # Initializes the ROS node
    rospy.init_node('main_node', log_level=rospy.DEBUG)
    # Creates a RobotController object
    smbg = TrackToGoal()
    tfb = tfBroad()
    # ROS waits for events
    rospy.spin()
