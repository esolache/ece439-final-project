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
spin = Bool()
goHome = Bool()
waypoint = ME439WaypointXY()
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

def talker():
    rospy.init_node("dogbot_main", anonymous=False)

    arm_up = rospy.get_param('/servo_cmd_us_arm_up')
    arm_down = rospy.get_param('/servo_cmd_us_arm_down')
    spin = True
    goHome = False
    waypoint.x = np.nan
    waypoint.y = np.nan
    pub_spin.publish(spin)
    pub_goHome.publish(goHome)
    # Subscriber for Whether or not the tag is found
    tag_found = rospy.Subscriber('/tagFound', Bool, finding_tag)

    # Subscriber for whether waypoint is complete
    wp_complete = rospy.Subscriber('/waypoint_complete', Bool, pick_up_put_down)
    sub_wp = rospy.Subscriber('/waypoint_xy', ME439WaypointXY, update_wp)

    rospy.spin()

# Updates the waypoint variable
def update_wp(wp_msg_in):
    global waypoint
    waypoint.x = wp_msg_in.x
    waypoint.y = wp_msg_in.y

# Tells robot what to do if an ArUco tag is found or not
def finding_tag(tag_msg_in):
    global spin, pub_spin, waypoint
    print(waypoint.x)
    if tag_msg_in.data == False:
        # if Tag is not found and wp is not 0,0: spin to find tag
        if (waypoint.x != 0) or (waypoint.y != 0):
            print('in if wp not 0 for findingtag')
            spin = True
            pub_spin.publish(spin)

        # if Tag is not found and wp is 0,0 (home): don't spin, go home
        else:
            print('in wp is zero for findingtag')
            spin = False
            pub_spin.publish(spin)
    # if Tag is found: don't spin, go to tag
    if tag_msg_in.data == True:
        print('in if tag is found in findingtag')
        spin = False
        pub_spin.publish(spin)
        pub_goHome.publish(goHome)
        #wag tail
        wag_tail()

# Tells robot what to do when waypoint is complete.
def pick_up_put_down(msg_in):
    global goHome, arm_up, arm_down, pub_electromag, pub_goHome
    if msg_in.data == True:
        if (waypoint.x == 0) and (waypoint.y == 0):
            #put down
            pub_electromag.publish(0) #just drops it lol
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
    global arm_up, arm_down, pub_electromag, pub_arm_servo
    pub_electromag.publish(18000) # turn on electromag
    pub_arm_servo.publish(arm_down)
    rospy.sleep(0.2)
    pub_arm_servo.publish(arm_up)

# Wag Tail needs own function
def wag_tail():
    global pub_tail_servo
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
