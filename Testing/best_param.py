from tracemalloc import start
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math

loaddir="/home/sijt/ISBEP/Arm_system/Testing/Hough Circle Parameter/Images/"
savedir="/home/sijt/ISBEP/Arm_system/Testing/Hough Circle Parameter/"
number = input("Input image number: ")
image = cv2.imread(loaddir+"image_" + number + ".jpg")
original = cv2.imread(loaddir+"image_" + number + ".jpg")

#real_center = np.array([[370,405,50]]) #for picture 1
real_center = np.array([[190,610,50]]) #picture 2
#real_center = np.array([[660,650,50]]) #for picture 3
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

def main():
    global image
    global original
    grayscale= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grayscale = cv2.GaussianBlur(grayscale,(3,3),cv2.BORDER_DEFAULT)
    
    #smooth to reduce noise a bit more
    mindist = 300
    minrad = 61
    maxrad = 130
    accept_range = 80
    log = np.empty([0,5])
    FP = 0
    TP = 0
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
                
                if circles is not None:
                    AddCircles(circles, image) #draw circles in frame
                    
                    for (x,y,r) in circles:
                        distance = math.sqrt((x-real_center[0,0])**2 + (y-real_center[0,0])**2)
                        if distance < real_center[0,2]:
                            TP = TP+1 #TP counter
                        else:
                            FP = FP + 1 #FP counter
                    log = np.append(log, np.array([[dp_val[i], param1_val[j], param2_val[k], TP, FP]]) ,axis=0) #dp, param1, param2, TP, FP
                else:
                    log = np.append(log, np.array([[dp_val[i], param1_val[j], param2_val[k], 0, 0]]), axis = 0)
                    #save data here
                    #save the current parameters
                    #save the amount of good detections
                print("-------------------------")
                FP = 0 #reset counter
                TP = 0 #reset counter
                #AddCircles(np.array([[370,405,40]]), image)
                cv2.imshow("Detection",image)
                cv2.waitKey(10)
                image = original.copy() #reset the pictuer
    #print("Log file:", log)
    np.savetxt(savedir+"hough_parameter_performance.txt", log)

main()