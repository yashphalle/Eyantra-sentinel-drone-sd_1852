#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
def callback(data):
 
  # Used to convert between ROS and OpenCV images
        br = CvBridge()
        
        # Output debugging information to the terminal
        rospy.loginfo("receiving video frame")
        
        # Convert ROS Image message to OpenCV image
        current_frame = br.imgmsg_to_cv2(data)
        frame = current_frame[:,:,::-1]
        # Display image
        
        
        clean = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(clean, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(frame,frame, mask= mask)
        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for c in contours:
            area = cv2.contourArea(c)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(clean, (x, y), (x+w, y+h), (0, 0, 255), 3)
            print(int(x+(w/2)), int(y+(h/2)))
        cv2.imshow("camera",frame)
        cv2.imshow("cleaned",clean)
        cv2.imshow("res", res)
        cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
