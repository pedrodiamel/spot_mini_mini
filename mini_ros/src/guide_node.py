#!/usr/bin/env python
"""
DESCRIPTION: Based on camera input publishes direction of movement for MiniMini

SUBSCRIBERS: /spot_camera/image_raw

PUBLISHERS: /command
"""

#---- Importing Libraries and Packages ----#
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from mini_ros.msg import MiniCmd

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


#---- Initializing Node ----#

rospy.init_node('Guide', anonymous=True)


#---- Define Publishers ----#

pub = rospy.Publisher("/command", String, queue_size=10)


#---- Callback Functions ----#

def camera_cb(msg):
    """
    Receives camera image and processes for the ball
    chaser application. Based on the position of a
    white ball in the image, sends the direction
    command for the robot to chase the ball. The
    direction is defined by a character following
    this convention:
        F : move forward
        L : turn left
        R : turn right
        S : stop

    Input:
        Image msg: image sensor message
    
    Output:
        None
    """
    #-- Defining Parameters --#

    # Obtain the width of each decision block:
    treshold = msg.step/3
    # Ball color value:
    color = [255, 255, 255]
    # Flag to indicate if the ball is in the image:
    no_ball = True
    # Direction command:
    com = "S" # Stop by default

    #-- Analising Image --#

    # Setup a CvBridege object
    bridge = CvBridge()

    # Convert the image message to an OpenCV image 
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Define image reading parameters
    height, width = cv_image.shape[:2]  
    roi_size = 50 # (10x10)
    roi_values = cv_image[(height-roi_size)/2:(height+roi_size)/2,(width-roi_size)/2:(width+roi_size)/2]

    # Get the RGB values
    mean_blue = np.mean(roi_values[:,:,0])
    mean_green = np.mean(roi_values[:,:,1])
    mean_red = np.mean(roi_values[:,:,2])

    # print("R: {}  G: {}  B: {}").format(mean_red, mean_green, mean_blue) 
    

    # Iterate through all pixels:
    for i in range(msg.height * msg.step):
        # print(msg[i][i])
        if (mean_red == color[0] and mean_green == color[1] and mean_blue == color[2]): #ball is present
            
            # Calculate horizontal pixel position:
            j = i % msg.step

            # Check pixel position
            if (j < treshold): #pixel on the left side
                # Go left:
                com = "L"
                no_ball = False #ball was found

            elif (j >= treshold and j <= 2*treshold): #pixel is in the middle
                # Go foward:
                com = "F"
                no_ball = False #ball was found

            elif (j > 2*treshold): #pixel on the right side
                #Go right:
                com = "R"
                no_ball = False #ball was found

    #finished the loop:    
    #no_ball will still be true if white ball is not present in the image!
    if (no_ball): #no white ball in sight
        # Stop:
        com = "S"

    #-- Send desired command --#

    pub.publish(com) #publish command


#---- Define Subscribers ----#

sub = rospy.Subscriber("/spot_camera/image_raw", Image, camera_cb)

# sub = rospy.Subscriber('mini_cmd', MiniCmd, camera_cb)


#---- Main code of node ----#

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            # Keep the node alive:
            pass

    except rospy.ROSInterruptException:
        pass
