#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

loaddir="/home/sijt/ISBEP/Arm_system/agrobot_ws/src/detection/scripts/camera_data/"

#Camera Parameters
cam_mtx=np.load(loaddir+'cam_mtx.npy')
dist=np.load(loaddir+'dist.npy')
newcam_mtx=np.load(loaddir+'newcam_mtx.npy')
roi=np.load(loaddir+'roi.npy')


cx = int(cam_mtx[0,2])
cy = int(cam_mtx[1,2])
fx = cam_mtx[0,0]
fy = cam_mtx[1,1]

#XYZ at world position of cx,cy
X_center=21.9
Y_center=14.0
Z_center=51.0
worldPoints=np.array([[X_center,Y_center,Z_center], #measured 1 to 9
                       [11.9,6.0,52.0],
                       [23.9,6.2,51.8],
                       [35.9,6.5,53.4],
                       [12.0,15.3,51.4],
                       [24.0, 15.5, 50.4],
                       [36.0, 15.0,53.4],
                       [11.5,24.5,52.5],
                       [23.6,24.9,52.3],
                       [35.6,24.9,54.6]], dtype=np.float32)

imagePoints=np.array([[cx,cy],
                       [395,202],
                       [736,209],
                       [1075,217],
                       [393,467],
                       [729,476],
                       [1063,481],
                       [398,724],
                       [724,730],
                       [1052,737]], dtype=np.float32)

                


retva, rvec, tvec = cv2.solvePnP(worldPoints, imagePoints,cam_mtx, dist)
R, jacobian = cv2.Rodrigues(rvec)
Rt=np.column_stack((R,tvec))
P = cam_mtx.dot(Rt)

def calcXYZalt(s,u,v): #based on the FDXLabs method
    A_inv = np.linalg.inv(cam_mtx)
    R_inv = np.linalg.inv(R)
    uv1 = np.array([[u,v,1]], dtype=np.float32)
    uv1 = uv1.T #transpose array
    suv1 = s*uv1
    XYZ_int = A_inv.dot(suv1)
    XYZ_int2 = XYZ_int - tvec
    XYZ = R_inv.dot(XYZ_int2)
    return XYZ.T 
def calcScaling():
    s = np.empty([np.size(worldPoints,0)+1,1])

    for i in range(0,np.size(worldPoints,0)):
        #print("-----------", i, "------------")
        XYZ1 = np.array([[worldPoints[i,0], worldPoints[i,1], worldPoints[i,2],1 ]], dtype=np.float32) #(X,Y,Z,1)^T
        XYZ1 = XYZ1.T
        #print("XYZ1: ", XYZ1, sep = '\n')
        suv1 = P.dot(XYZ1)
        #print("suv1: ", suv1,sep = '\n')
        s[i,0] = suv1[2,0] #(u,v,1)T.s = (su,sv,s)T -> s = [2,0]
        uv1 = suv1/s[i,0]
        #print("s: ",s,sep = '\n')
    s_mean = np.mean(s)
    s[-1,0] = s_mean 
    #print("Mean s: ",s_mean)
    return s_mean, s
def calcBestScaling(): #calculate coordinates from all scaling factors, find scaling factor with lowest error for all coordinate
    coord_alt = np.empty((np.size(imagePoints,0),3,np.size(s))) 
    error_alt = np.empty((np.size(imagePoints,0),3, np.size(s)))
    print("----Calculation with all s---------")
    for j in range(np.size(s)):
        for i in range(0,np.size(imagePoints,0)):    
            coord_alt[i,:,j] = calcXYZalt(s[j,0] ,imagePoints[i,0], imagePoints[i,1])
            error_alt[:,:,j] = abs(coord_alt[:,:,j] - worldPoints)/worldPoints*100
    error_alt_mean = np.mean(error_alt,axis=0) #row x column = #error: XYZ x s
    s_best = np.argmin(error_alt_mean, axis=1) #indices for lowest error, for XYZ
    s_best = s[s_best[1],0] #the error in the Y coordinates is the highest so we pick the best s for Y coordinates
    return s_best


s_mean, s = calcScaling() #calculate the scaling factor and return mean scaling factor and s for every imagePoint
s_best = calcBestScaling()


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
    undistorted = Undistort(image)
    #BlobDetection(image_processed)
    #tune param
    
    dp = 1.2
    mindist = 100
    param1 = 100
    param2 = 100
    minrad = 40
    maxrad = 250
    global circles 
    circles = FindCircles(grayscale,dp,mindist,param1,param2,minrad,maxrad)
    
    #circles_blur = FindCircles(grayscale_blur,dp,mindist,param1,param2,minrad,maxrad)
    AddCircles(np.array([[cx,cy,20]]), image) #centre point
    AddCircles(circles, image)
  
    if circles is not None:
        XYZ = calcXYZalt(s_best,circles[0,0], circles[0,1])
        print("Image coordinate: ","cX: ",circles[0,0],"cY: ", circles[0,1])
        print("World Coordinate:", "X: ", XYZ[0,0],"Y: ", XYZ[0,1])
    #AddCircles(circles_blur, image_processed)
    #show detected image
    #cv2.imshow("Undistorted", undistorted)
    #cv2.imshow("Grayscale", grayscale)
    #cv2.imshow("Detection_blur", image_processed)
    cv2.imshow("Detection",image)
    cv2.waitKey(3)

def callback(image_ROS):
    #pub = rospy.Publisher('/circles', Image, queue_size = 1) #publish circles as image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ROS, desired_encoding = 'passthrough')
    detection(image)
    #testing sending the circles array as image
    #if circles is not None:
    #    print(np.shape(circles))
    #    print(type(circles[0,1]))
    #    print(circles)
    #    msg = bridge.cv2_to_imgmsg(circles, 'mono16') #convert image to ROS image
    #    #try:t
    #    #    tpub.publish(msg)

def listener():
    rospy.init_node("Detection_improved", anonymous = False)
    rospy.Subscriber('/image', Image, callback) #subscribe to the webcam image from image_publisher.py
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


