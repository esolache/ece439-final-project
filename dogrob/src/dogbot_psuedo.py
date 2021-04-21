#!/usr/bin/env python3


# Write psuedo code/random notes for dogrob features.


# Import stuff like access to mobrob and servos?
import rospy
import traceback
import numpy as np

# TODO: add mobrob_util messages to dogrob_util
#from mobrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# We don't want to copy all the mobrob stuff over.

# Make wag_tail a Bool
