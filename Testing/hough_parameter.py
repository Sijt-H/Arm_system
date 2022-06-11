#!/usr/bin/env python3

import cv2
import numpy as np

loaddir = "/home/sijt/ISBEP/Arm_system/Testing/Hough Circle Parameter/Images/"
image = cv2.imread(loaddir+"image_10.jpg")

def FindCircles(grayscale,dp,mindist,par1,par2,minrad,maxrad):
    circles = cv2.HoughCircles(grayscale, cv2.HOUGH_GRADIENT,dp,mindist,param1=par1,param2=par2,minRadius=minrad,maxRadius=maxrad)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
    return circles

def AddCircles(circles, image):
    if circles is not None:
        for (x, y, r) in circles:
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

def detection(image):
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dp = 1.4
    mindist = 400
    param1 = 80
    param2 = 80
    minrad = 40
    maxrad = 200
    circles = FindCircles(grayscale,dp,mindist,param1,param2,minrad,maxrad)
    AddCircles(circles, image)
    print("Circles found: ", circles)

    #show detected image
    cv2.imshow("Detected Image", image)
    cv2.waitKey(0)


def main():
    detection(image)

while True:
    main()

