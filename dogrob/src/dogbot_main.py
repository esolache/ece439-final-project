#!/usr/bin/python2

# Name: dogbot_main
# Import stuff like access to mobrob and servos?
import rospy
import traceback
import numpy as np

from dogrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from std_msgs.msg import Int32

# Dog must spin first to find a tag
spin = True
goHome = False


def talker():
    rospy.init_node("dogbot_main", anonymous=False)

    arm_up = rospy.get_param('/servo_cmd_us_arm_up')
    arm_down = rospy.get_param('/servo_cmd_us_arm_down')

    # Publish home
    pub_goHome = rospy.Publisher('/home', Bool, queue_size = 1)

    # Publish toSpin
    pub_spin = rospy.Publisher('/toSpin', Bool, queue_size = 1)

    # Arm Servo Publisher: use servo_node.py from basic_motors
    pub_arm_servo = rospy.Publisher('/arm_servo_0', Int32, queue_size = 1)

    # Tail Servo Publisher: use servo_node.py from basic_motors
    pub_tail_servo = rospy.Publisher('/tail_servo_1', Int32, queue_size = 1)

    # Electromagnet "Servo" Publisher: use servo_node.py from basic_motors
    pub_electromag = rospy.Publisher('/electromag_servo_3', Int32, queue_size = 1)

    # Subscriber for Whether or not the tag is found
    tag_found = rospy.Subscriber('/tagFound', Bool, finding_tag)

    # Subscriber for whether waypoint is complete
    wp_complete = rospy.Subscriber('/waypoint_complete', Bool, pick_up_put_down)

    rospy.spin()

# Tells robot what to do if an ArUco tag is found or not
def finding_tag(tag_msg_in):
    global spin
    if tag_msg_in.data == False:
        # if Tag is not found and wp is not 0,0: spin to find tag
        if waypoint.any() != 0:
            spin = True
            pub_spin.publish(spin)

        # if Tag is not found and wp is 0,0 (home): don't spin, go home
        else:
            spin = False
            pub_spin.publish(spin)
    # if Tag is found: don't spin, go to tag
    else:
        spin = False
        pub_spin.publish(spin)
        #wag tail
        wag_tail()

# Tells robot what to do when waypoint is complete.
def pick_up_put_down(msg_in):
    global goHome, arm_up, arm_down
    if msg_in.data == True:
        if waypoint.all() == 0:
            #put down
            pub_electromag(0) #just drops it lol
            #wag tail
            wag_tail()
            rospy.sleep(2) # Waits 2 sec before starting again
        else:
            #pick up
            pick_up()
            goHome = True # Publish to set waypoints so that it sets to 0,0
            pub_goHome.publish(goHome)

# Pick up
def pick_up():
    global arm_up, arm_down
    pub_electromag.publish(18000) # turn on electromag
    pub_arm_servo.publish(arm_down)
    rospy.sleep(0.2)
    pub_arm_servo.publish(arm_up)

# Wag Tail needs own function
def wag_tail():
    #left
    pub_tail_servo.publish(750)
    rospy.sleep(0.2)
    #right
    pub_tail_servo.publish(1100)
    rospy.sleep(0.2)
    #left
    pub_tail_servo.publish(750)
    rospy.sleep(0.2)
    #right
    pub_tail_servo.publish(1100)
    rospy.sleep(0.2)
    #center
    pub_tail_servo.publish(925)
    rospy.sleep(0.2)
    


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
