#!/usr/bin/env python
"""
DESCRIPTION: Based on camera input publishes direction of movement for MiniMini

SUBSCRIBERS: /camera/rgb/image_raw

PUBLISHERS: /command
"""

#---- Importing Libraries and Packages ----#
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from mini_ros.msg import MiniCmd

#---- guide_node Variables ----#
com = "F"

#---- Callback Functions ----#
def camera_cb(msg):
    # Process image 
    # rospy.loginfo(msg.data)
    # pub.publish(com)
    pass

#---- Initializing Node ----#
rospy.init_node('Guide', anonymous=True)

#---- Define Publishers ----#
pub = rospy.Publisher("/command", String, queue_size=10)

#---- Define Subscribers ----#
# sub = rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)
sub = rospy.Subscriber('mini_cmd', MiniCmd, camera_cb)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            pub.publish(com)

    except rospy.ROSInterruptException:
        pass
