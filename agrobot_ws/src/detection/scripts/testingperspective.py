#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

savedir = "/home/sijt/ISBEP/Arm_system/agrobot_ws/src/detection/scripts/camera_data/"

cam_mtx=np.load(savedir+'cam_mtx.npy')
dist=np.load(savedir+'dist.npy')
newcam_mtx=np.load(savedir+'newcam_mtx.npy')

def ImageProcessing(frame):
    #add blur
    frame_mblur= cv2.medianBlur(frame, 7)
    hsv = cv2.cvtColor(frame_mblur, cv2.COLOR_BGR2HSV)
    lower = np.array([0,15,0]) #np.array([0,21,0])
    upper = np.array([179,255,254]) #np.array([24,255,171])
    #lower = np.array([0,0,0])
    #upper = np.array([179,255,255])

    #HSV_testing.jpg:
    lower = np.array([0,40,0])
    upper = np.array([48,255,254])

    mask = cv2.inRange(hsv, lower, upper)
    reversemask = 255 - mask
    masked = cv2.bitwise_and(frame,frame, mask= mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    masked = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("Masked", reversemask)
    BlobDetection(frame, reversemask)
    #return masked
    return masked

def FindCircles(grayscale,dp,mindist,par1,par2,minrad,maxrad):
    # detect circles in the image
    circles = cv2.HoughCircles(grayscale, cv2.HOUGH_GRADIENT,dp,mindist,param1=par1,param2=par2,minRadius=minrad,maxRadius=maxrad)
# ensure at least some circles were found
    if circles is not None:
    # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
    return circles #circles is a (n,3) numpy array

def AddCircles(circles, image):
    # loop over the (x, y) coordinates and radius of the circles
    if circles is not None:
        for (x, y, r) in circles:
    # draw the circle in the output image, then draw a rectangle
    # corresponding to the center of the circle
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

def Undistort(image):
    undistorted = cv2.undistort(image, cam_mtx, dist, None, newcam_mtx)

    return undistorted

def BlobDetection(image, reversemask):
    params = cv2.SimpleBlobDetector_Params()

    params.filterByArea = True
    params.minArea = 100

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(reversemask)

    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Blobs Using Area", blobs)

def detection(image):
    #image_processed = ImageProcessing(image)
    #grayscale_blur = cv2.cvtColor(image_processed, cv2.COLOR_BGR2GRAY)
    grayscale= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #undistorted = Undistort(image)
    #BlobDetection(image_processed)
    #tune param
    
    dp = 1.2
    mindist = 100
    param1 = 80
    param2 = 80
    minrad = 30
    maxrad = 200
    global circles 
    circles = FindCircles(grayscale,dp,mindist,param1,param2,minrad,maxrad)
    #circles_blur = FindCircles(grayscale_blur,dp,mindist,param1,param2,minrad,maxrad)
    AddCircles(circles, image)
    #AddCircles(circles_blur, image_processed)
    #show detected image
    #cv2.imshow("Undistorted", undistorted)
    #cv2.imshow("Grayscale", grayscale)
    #cv2.imshow("Detection_blur", image_processed)
    cv2.imshow("Detection normal",image)
    cv2.waitKey(3)

def callback(image_ROS):
    pub = rospy.Publisher('/circles', Image, queue_size = 1) #publish circles as image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ROS, desired_encoding = 'passthrough')
    detection(image)
    if circles is not None:
        print(np.shape(circles))
        print(type(circles[0,1]))
        print(circles)
        msg = bridge.cv2_to_imgmsg(circles, 'mono16') #convert image to ROS image
        #try:t
        #    tpub.publish(msg)

def listener():
    rospy.init_node("Detection_improved", anonymous = False)
    rospy.Subscriber('/image', Image, callback) #subscribe to the webcam image from image_publisher.py
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


