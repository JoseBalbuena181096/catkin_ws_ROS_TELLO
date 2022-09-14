#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
from geometry_msgs.msg import  Twist 
from std_msgs.msg import Int8
from std_msgs.msg import Empty
from djitellopy import Tello
import cv2
import numpy as np


myDrone = None

def initializeTello():
    global myDrone
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone. left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone

 
def telloGetFrame(myDrone, w= 360,h=240):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame,(w,h))
    return img

def cb_takeoff(msg):
    global myDrone
    myDrone.takeoff()

def cb_land(msg):
    global myDrone
    myDrone.land()


def driver_tello():
    pub = rospy.Publisher('/tello/image_raw', Image,queue_size=1)
    sub_takeoff = rospy.Subscriber('/tello/takeoff', Empty,  cb_takeoff) 
    sub_land = rospy.Subscriber('/tello/land', Empty,  cb_land)  
    
    rospy.init_node('tello_driver', anonymous=True)
    global myDrone
    w,h = 360,240
    bridge = CvBridge()
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        img = telloGetFrame(myDrone,w,h)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        try:
            pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)
        #rate.sleep()

if __name__ == '__main__':
    try:
        myDrone = initializeTello()
        driver_tello()    
    except rospy.ROSInterruptException:
        pass
    
