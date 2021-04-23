#!/usr/bin/env python3

# Name: dogbot_set_wp.py

import rospy
import traceback
import numpy as np

from mobrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

def talker():
    rospy.init_node("dogbot_set_wp", anonymous=False)

    # Publisher for waypoint
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)

    # Subscribes from waypoint_seeker
    waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, )

    # Subscriber from the openCV
    cv_trans_vec = rospy.Subscriber('/', ME439WaypointXY, )

