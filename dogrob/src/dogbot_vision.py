#!/usr/bin/python

# Name: dogbot_vision.py
# copy/paste tag_tracking_with_distance.py into this and add publishers??? or just add publisher to tag_tracking_with_distance.py?
# tag_tracking_with_distance.py has a while loop, which we'll need to think about with ROS (subscribe/publish)

# Import stuff like access to mobrob and servos?

import rospy
import traceback
import numpy as np
import cv2
import PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import pickle

from dogrob_util.msg import ME439WaypointXY, ME439PathSpecs
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

#pub_arucoFound
#pub_arucoTransVector
pub_arucoFound = rospy.Publisher('/tagFound', Bool, queue_size = 1)
pub_arucoTransVector = rospy.Publisher('/tag_location', ME439WaypointXY, queue_size = 1)
arucoLocation = ME439WaypointXY()
arucoFound = Bool()


cam_matrix = pickle.load(open("/home/pi/catkin_ws_project/src/dogrob/src/cam_matrix.p","rb"))
dist_matrix = pickle.load(open("/home/pi/catkin_ws_project/src/dogrob/src/dist_matrix.p","rb"))


def talker():
    global pub_arucoFound, pub_arucoTransVector
    rospy.init_node("dogbot_vision", anonymous=False)
    # Publish home
#    pub_arucoFound = rospy.Publisher('/vision_arucoFound', Bool, queue_size = 1)

    # Publish toSpin
 #   pub_arucoTransVector = rospy.Publisher('/tag_location', ME439WaypointXY, queue_size = 1)


    while True: 
        checkArucoTagFound()
    rospy.spin()



    


def checkArucoTagFound():
    global pub_arucoFound, pub_arucoTransVector, arucoLocation, arucoFound, cam_matrix, dist_matrix
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280);
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT,960);

    plt.figure()


    retval, frame = camera.read()
    if frame is None:
        return
    
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    print(gray.shape)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    arucoFound = False
    arucoLocation.x = 0
    arucoLocation.y = 0
    
    for c in corners :
        arucoFound = True

        rot_vec, trans_vec = aruco.estimatePoseSingleMarkers(c,.1016,cam_matrix,dist_matrix);
        axis = np.float32([[4,0,0],[0,4,0],[0,0,-4]]).reshape(-1,3)
        imgpts, jac = cv2.projectPoints(axis,rot_vec,trans_vec,cam_matrix,dist_matrix);
        print(imgpts)
        frame = aruco.drawAxis(frame,cam_matrix,dist_matrix,rot_vec,trans_vec,.1)
        #frame = draw(frame,corners,imgpts)
        print("Rotation Vector:" )
        print(rot_vec)
        print ("translation vector: ")
        print(trans_vec)
        frame = cv2.putText(frame,"X: " + str(trans_vec[0][0][0]) + " Y: " + str(trans_vec[0][0][1]) + " Z: " + str(trans_vec[0][0][2]),(30,30),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),4)
        arucoLocation.x = trans_vec[0][0][0]
        arucoLocation.y = trans_vec[0][0][2]
             
    frame_markers = aruco.drawDetectedMarkers(frame.copy(),corners,ids)
    cv2.imshow("live video", frame_markers)
    cv2.waitKey(1)

    
    pub_arucoFound.publish(arucoFound)
    if arucoFound:
        pub_arucoTransVector.publish(arucoLocation)


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
