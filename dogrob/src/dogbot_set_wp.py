#!/usr/bin/env python3

# Name: dogbot_set_wp.py

import rospy
import traceback
import numpy as np

from dogrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

waypoint = ME439WaypointXY()

def talker():
    rospy.init_node("dogbot_set_wp", anonymous=False)

    # Publisher for waypoint
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)

    # Subscribes from waypoint_seeker
    waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, )

    # Subscriber from the openCV
    cv_trans_vec = rospy.Subscriber('/tag_location', ME439WaypointXY, compute_waypoint)

    # Subscriber for going home
    goHome = rospy.Subscriber('/home', Bool, goToHome)

    # Subscriber for spinning
    toSpin = rospy.Subscriber('/toSpin', Bool, startSpin)

    rospy.spin()

def goToHome(home_msg_in):
    if home_msg_in == True:
        waypoint.x = 0
        waypoint.y = 0
        pub_waypoint_xy.publish(waypoint)

def compute_waypoint(tag_loc_msg):
    # do the vector math thing here

    #ensure waypoint_complete is false at first?
    waypoint_complete = False

    #uncomment this once math complete
    #pub_waypoint_xy.publish(waypoint)


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
