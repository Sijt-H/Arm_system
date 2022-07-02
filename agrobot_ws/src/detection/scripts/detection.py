#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def FindCircles(grayscale,dp,mindist,par1,par2,minrad,maxrad):
    # detect circles in the image
    circles = cv2.HoughCircles(grayscale, cv2.HOUGH_GRADIENT,dp,mindist,param1=par1,param2=par2,minRadius=minrad,maxRadius=maxrad)
# ensure at least some circles were found
    if circles is not None:
    # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
    return circles

def AddCircles(circles, image):
    # loop over the (x, y) coordinates and radius of the circles
    if circles is not None:
        for (x, y, r) in circles:
    # draw the circle in the output image, then draw a rectangle
    # corresponding to the center of the circle
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

def detection(image):
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dp = 1.2
    param1 = 70
    param2 = 50
    mindist = 300
    minrad = 61
    maxrad = 130
    circles = FindCircles(grayscale,dp,mindist,param1,param2,minrad,maxrad)
    AddCircles(circles, image)


    #show detected image
    cv2.imshow("Detected Image", image)
    cv2.waitKey(3)

def callback(image_ROS):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ROS, desired_encoding = 'passthrough')
    detection(image)

def listener():
    rospy.init_node("Detection", anonymous = False)
    rospy.Subscriber('/image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

