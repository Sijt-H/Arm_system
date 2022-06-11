#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

loaddir = "/home/sijt/ISBEP/Arm_system/Testing/Hough Circle Parameter/Images/"
cap = cv2.imread(loaddir+"image_10.jpg")
bridge = CvBridge()
frame = cv2.imread(loaddir+"image_10.jpg")
def talker():
    pub = rospy.Publisher('/image', Image, queue_size = 1) #create topic
    rospy.init_node('webcam', anonymous = False) #create node
    rate = rospy.Rate(20) #rate [Hz]
    while not rospy.is_shutdown():
        #ret,frame = cap.read()
        #if not ret:
        #    break
        
        msg = bridge.cv2_to_imgmsg(frame, "bgr8") #convert image to ROS image
        pub.publish(msg) #publish image
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():
            cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
