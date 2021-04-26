#!/usr/bin/python2

# Name: dogbot_main
# Import stuff like access to mobrob and servos?
import rospy
import traceback
import numpy as np

from mobrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# Dog must spin first to find a tag
spin = True
goHome = False

def talker():
    rospy.init_node("dogbot_main", anonymous=False)
    # Publish home
    pub_goHome = rospy.Publisher('/home', Bool, queue_size = 1)

    # Publish toSpin
    pub_spin = rospy.Publisher('/toSpin', Bool, queue_size = 1)

    # Subscriber for Whether or not the tag is found
    tag_found = rospy.Subscriber('/tagFound', Bool, finding_tag)

    # Subscriber for whether waypoint is complete
    wp_complete = rospy.Subscriber('/waypoint_complete', Bool, pick_up_put_down)

    rospy.spin()

# Tells robot what to do if an ArUco tag is found or not
def finding_tag(tag_msg_in):
    if tag_msg_in == False:
        # if Tag is not found and wp is not 0,0: spin to find tag
        if waypoint.any() != 0:
            spin = True
            pub_spin.publish(spin)
            #wag tail
        # if Tag is not found and wp is 0,0 (home): don't spin, go home
        else:
            spin = False
            pub_spin.publish(spin)
    # if Tag is found: don't spin, go to tag
    else:
        spin = False
        pub_spin.publish(spin)
        #set waypoint? and publish?

# Tells robot what to do when waypoint is complete.
def pick_up_put_down(msg_in)
    if msg_in == True:
        if waypoint.all() == 0:
            #put down
            #wag tail
        else:
            #pick up
            goHome = True # Publish to set waypoints so that it sets to 0,0
            pub_goHome.publish(goHome)
            #publish?


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
