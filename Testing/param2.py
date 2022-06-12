#!/usr/bin/env python3

from tracemalloc import start
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

loaddir="/home/sijt/ISBEP/Arm_system/agrobot_ws/src/detection/scripts/camera_data/"
savedir="/home/sijt/ISBEP/Arm_system/Testing/Hough Circle Parameter/"

def detection(image):
    grayscale= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grayscale = cv2.GaussianBlur(grayscale,(3,3),cv2.BORDER_DEFAULT)
    
    #smooth to reduce noise a bit more
    mindist = 300
    minrad = 50
    maxrad = 130
    
    
    
    #changing params
    dp_val = np.arange(1, 2.1, 0.1)
    param1_val = np.arange(20, 300, 10)
    param2_val = np.arange(20, 300, 10)

    #for all test values of dp, param 1 and param 2
    for i in range(np.size(dp_val)):
        for j in range(np.size(param1_val)):
            for k in range(np.size(param2_val)):
                circles = FindCircles(grayscale,dp_val[i],mindist,param1_val[j],param2_val[k],minrad,maxrad)
                print("Circles", circles)
                print("Current value: (dp,param1,param2)", dp_val[i], param1_val[j], param2_val[k])
                print("-------------------------")
                if circles is not None:
                    AddCircles(circles, image)
                
                cv2.imshow("Detection",image)
                cv2.imshow("original",original)
                cv2.waitKey(100)
                image = original #reset the pictuer
                print("reset")

def callback(image_ROS):
    #pub = rospy.Publisher('/circles', Image, queue_size = 1) #publish circles as image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ROS, desired_encoding = 'passthrough')
    detection(image)

def listener():
    rospy.init_node("Detection_improved", anonymous = False)
    rospy.Subscriber('/image', Image, callback) #subscribe to the webcam image from image_publisher.py
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

