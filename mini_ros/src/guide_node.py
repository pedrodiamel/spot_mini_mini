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
    color = 255
    # Flag to indicate if the ball is in the image:
    no_ball = True
    # Direction command:
    com = "S" # Stop by default

    #-- Analising Image --#

    # Iterate through all pixels:
    for i in range(msg.height * msg.step):
        
        if (msg.data[i] == color): #ball is present

            # Calculate horizontal pixel position:
            j = i % img.step

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
