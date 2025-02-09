#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def valve_callback(msg):
    orientation = msg.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    
    rospy.loginfo("Euler angles: roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw)

def listener():
    rospy.init_node('valve_pose_listener', anonymous=True)
    
    rospy.Subscriber("/valve/mocap/pose", PoseStamped, valve_callback, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
