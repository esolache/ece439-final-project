#!/usr/bin/env python3

# Name: dogbot_set_wp.py

import rospy
import traceback
import numpy as np

from dogrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

waypoint = ME439WaypointXY()
curr_tag_loc = ME439WaypointXY()

goHome = Bool()
currPose = Pose2D()
spin = Bool()

# Publisher for waypoint
pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)

def talker():
    rospy.init_node("dogbot_set_wp", anonymous=False)

    # Initialize waypoint
    spin = True
    waypoint.x = np.nan
    waypoint.y = np.nan
    pub_waypoint_xy.publish(waypoint)

    # Subscribes from waypoint_seeker
   #waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, )

    # Subscriber from the openCV
    cv_trans_vec = rospy.Subscriber('/tag_location', ME439WaypointXY, update_wp)

    # Subscriber for going home
    sub_goToHome = rospy.Subscriber('/home', Bool, goToHome)

    # Subscriber for spinning
    toSpin = rospy.Subscriber('/toSpin', Bool, startSpin)

    # Subscriber for deadreckoning curr robot pose 
    robotPose = rospy.Subscriber('/robot_pose_estimated', Pose2D, updatePose)

    rospy.spin()

def startSpin(spin_msg_in):
    spin = spin_msg_in.data
    if spin == True:
        print('spin is true')
        #Give waypoint a ridiculous number. or nan?
        waypoint.x = np.nan
        waypoint.y = np.nan
        pub_waypoint_xy.publish(waypoint)
    else:
        compute_waypoint(curr_tag_loc)

def goToHome(home_msg_in):
    goHome = home_msg_in.data

    if goHome == True:
        waypoint.x = 0
        waypoint.y = 0
        pub_waypoint_xy.publish(waypoint)

def updatePose(pose_msg):
    currPose = pose_msg

def update_wp(tag_loc_msg):
    curr_tag_loc.x = tag_loc_msg.x
    curr_tag_loc.y = tag_loc_msg.y

def compute_waypoint(tag_loc):
    global goHome
    # do the vector math thing here
    #tag_log_msg is the transVector
    #tag_loc_msg.x
    #tag_loc_msg.y
    
    waypoint.x = currPose.x + tag_loc.y*np.cos(currPose.theta)
    waypoint.y = currPose.y + tag_loc.y*np.sin(currPose.theta)
    print(waypoint)
    print(goHome)

    if goHome.data == False:
        print('going to publish waypoint')
        waypoint_complete = False
        pub_waypoint_xy.publish(waypoint)

    


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
