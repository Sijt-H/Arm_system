#!/usr/bin/env python3

from tracemalloc import start
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

loaddir="/home/sijt/ISBEP/Arm_system/agrobot_ws/src/detection/scripts/camera_data/"
savedir="/home/sijt/ISBEP/Arm_system/Testing/Detection/Automated_3/"


#Camera Parameters
cam_mtx=np.load(loaddir+'cam_mtx.npy')
dist=np.load(loaddir+'dist.npy')
newcam_mtx=np.load(loaddir+'newcam_mtx.npy')
roi=np.load(loaddir+'roi.npy')

logger = np.empty((0,3))
cx = int(cam_mtx[0,2])
cy = int(cam_mtx[1,2])
fx = cam_mtx[0,0]
fy = cam_mtx[1,1]

#XYZ at world position of cx,cy
X_center=67.0
Y_center=15.3
Z_center=43.5
worldPoints=np.array([[X_center,Y_center+1.3,Z_center], #measured 1 to 9, calibration 08/06/2022, frame flat on ground, no motors
                       [80.1,6.7,47.0],
                       [67.9,6.8,47.0],
                       [56.0,6.8,45.0],
                       [80.1,16.0,45.0],
                       [67.9,16.1,45.5],
                       [56.0,16.2,45.5],
                       [80.1,25.4,45.0],
                       [67.9,25.5,43.5],
                       [56.0, 25.5,45.5]], dtype=np.float32)

worldPoints = worldPoints - np.array([[0,1.3,0]]) #correction for the circle radius, as the outer part of the circle was measured, not the center point, iam = 2.6cm
worldPoints = worldPoints - np.array([[0,4,0]]) #offset for Y-axis measuring point. Offset is the size of 1 frame part, 40mm

imagePoints=np.array([[cx,cy],
                       [246,122],
                       [636,122],
                       [1024,127],
                       [238,420],
                       [634,424],
                       [1028,426],
                       [230,729],
                       [634,733],
                       [1032,734]], dtype=np.float32)

                
#pixel = input("Give pixel number: ") #pixel location
pixel = "2"
pixel_loc = np.array([[15, 50, 90]]) #pixel location of Arduino code
pixel_offset = np.array([[pixel_loc[0,(int(pixel)-1)]-50,0,0]]) #pixel offset


retva, rvec, tvec = cv2.solvePnP(worldPoints, imagePoints,cam_mtx, dist)
R, jacobian = cv2.Rodrigues(rvec)
Rt=np.column_stack((R,tvec))
P = cam_mtx.dot(Rt)
timing = [1,1,1,1,1,2,2,2,2,2,5,5,5,5,5,10,10,10,10,10,20,20,20,20,20]
start_time = time.time() #timout after 10 seconds
time_index = 0


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
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), 1) #thickness -1

def Undistort(image):
    undistorted = cv2.undistort(image, cam_mtx, dist, None, newcam_mtx)

    return undistorted


def detection(image):

    global logger
    global time_index
    global current_time
    global start_time
    grayscale= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dp = 1.3
    mindist = 400
    param1 = 80
    param2 = 80
    minrad = 40
    maxrad = 200
    global circles 
    circles = FindCircles(grayscale,dp,mindist,param1,param2,minrad,maxrad)
    AddCircles(circles, image)
    
    if circles is not None:
        for i in range(np.shape(circles)[0]):
            XYZ = calcXYZalt(s_best,circles[0,0], circles[0,1])
            XYZ = XYZ + pixel_offset #offset the X value for the camera location on the frame
            logger = np.append(logger, XYZ, axis=0)
        #print("Image coordinate: ","cX: ",circles[0,0],"cY: ", circles[0,1])      
        #print("World Coordinate:", "X: ", XYZ[0,0],"Y: ", XYZ[0,1])
        print("---------------------------------------------------------------")
    if time.time()>(start_time + timing[time_index]):
        np.savetxt(savedir+str(timing[time_index])+'_seconds_'+str(time_index)+".txt", logger)
        #print("File saved")
        print("Test: ", time_index)
        print("Mean value [X,Y,Z]: ", np.mean(logger, axis=0))
        
        start_time = time.time()
        time_index = time_index + 1
        if time_index is len(timing):
            rospy.signal_shutdown("Completed") #final thing is done
        
    imS = cv2.resize(image, (800,600))                #
    cv2.imshow("Detection",imS)
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


