#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge
from  sensor_msgs.msg  import Image

def camera_callback(msg):
    try:
        rospy.loginfo("successfully started callback")
        bridge = CvBridge()
        cv_array = bridge.imgmsg_to_cv2(msg)
        rospy.loginfo(cv_array)
 
    except :
        rospy.loginfo("somthing is wrong!!!")

if __name__ == '__main__':
    rospy.init_node('special_node_for_opencv')
    rospy.Subscriber('/assemble_quadrotors1/usb/image_raw', Image, camera_callback)
    rospy.loginfo("successfully started sub")

    rospy.spin()
