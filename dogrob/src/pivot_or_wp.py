#!/usr/bin/python2

# Name: pivot_or_wp.py
# this node decides whether or not we're following a waypoint or spinning.
import rospy
import traceback
import numpy as np
import me439_mobile_robot_class_v02 as m439rbt

from mobrob_util.msg import ME439WaypointXY, ME439PathSpecs, ME439WheelSpeeds
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# Get parameters from rosparam
wheel_width = rospy.get_param('/wheel_width_model') # All you have when planning is a model - you never quite know the truth! 
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0

robot = m439rbt.robot(wheel_width, body_length, wheel_radius)
spin = False


def talker():

    pub_speeds = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=1)

    sub_spin = rospy.Subscriber('/toSpin', Bool, updateSpin)
    sub_wp_specs = rospy.Subscriber('/wheel_speeds_desired_wp', ME439WheelSpeeds, spinOrWP)


def updateSpin(spin_msg_in):
    global spin
    spin = spin_msg_in

def spinOrWP(wp_msg_in):
    global spin, robot, pub_speeds
    wheelspeed = ME439WheelSpeeds()
    if spin == True:
        # Send wheelspeed for pivot
        spin_plan = np.array( [robot.plan_pivot(-1.0, -np.pi)] )
        wheelspeed.v_left = spin_plan[1]
        wheelspeed.v_right = spin_plan[2]
    else:
        wheelspeed = wp_msg_in
    
    # Publish wheel speeds
    pub_speeds.publish(wheelspeed)

        