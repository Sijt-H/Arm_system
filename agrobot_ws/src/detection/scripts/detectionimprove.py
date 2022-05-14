#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def ImageProcessing(frame):
    #add blur
    frame_mblur= cv2.medianBlur(frame, 7)
    hsv = cv2.cvtColor(frame_mblur, cv2.COLOR_BGR2HSV)
    lower = np.array([0,15,0]) #np.array([0,21,0])
    upper = np.array([179,255,254]) #np.array([24,255,171])
    lower = np.array([0,0,0])
    upper = np.array([179,255,255])
    mask = cv2.inRange(hsv, lower, upper)
    masked = cv2.bitwise_and(frame,frame, mask= mask)
    #cv2.imshow("Masked", masked)
    #return masked
    return frame_mblur

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
    image_processed = ImageProcessing(image)
    grayscale_blur = cv2.cvtColor(image_processed, cv2.COLOR_BGR2GRAY)
    grayscale= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #tune param
    
    dp = 1.2
    mindist = 100
    param1 = 80
    param2 = 80
    minrad = 30
    maxrad = 200
    circles = FindCircles(grayscale,dp,mindist,param1,param2,minrad,maxrad)
    circles_blur = FindCircles(grayscale_blur,dp,mindist,param1,param2,minrad,maxrad)
    AddCircles(circles, image)
    AddCircles(circles_blur, image_processed)
    #show detected image
    #cv2.imshow("Grayscale", grayscale)
    cv2.imshow("Detection_blur", image_processed)
    cv2.imshow("Detection normal",image)
    cv2.waitKey(3)

def callback(image_ROS):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ROS, desired_encoding = 'passthrough')
    detection(image)

def listener():
    rospy.init_node("Detection_improved", anonymous = False)
    rospy.Subscriber('/image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

