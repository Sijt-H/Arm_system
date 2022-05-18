#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def talker():
    connected = []
    for i in range(8):
        cap = cv2.VideoCapture(i)
        if cap is None or not cap.isOpened():
            print('No Camera Connected: ', i)
        else:
            connected.append(i)

    print('---------------------------')
    print('Usable camera connections:')
    print(*connected, sep = ", ")
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

