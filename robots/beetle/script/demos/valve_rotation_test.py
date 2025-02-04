#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
class MoveAndRotateValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
        self.pos_beetle_1_sub = rospy.Subscriber("/beetle1/mocap/pose", PoseStamped, self.beetle_1_callback, queue_size=1)
        self.pos_beetle_2_sub = rospy.Subscriber("/beetle2/mocap/pose", PoseStamped, self.beetle_2_callback, queue_size=1)
        self.pos_valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        self.pos_beetle_1 = PoseStamped()
        self.pos_beetle_2 = PoseStamped()
        self.start_x = 0.0  
        self.start_z = 1.0  
        self.valve_x = 10.0  
        self.valve_z = 2.0   
        self.valve_rotation_angle = 3.14  
        self.flag = 0

    def beetle_1_callback(self, msg):
        self.pos_beetle_1 = msg

    def beetle_2_callback(self, msg):
        self.pos_beetle_2 = msg
    
    def valve_callback(self, msg):
        self.pos_valve = msg


    def execute(self, userdata):
        rospy.loginfo("Moving to the valve position...")
        
        # move to the valve position
        pos_assembly = FlightNav()
        pos_assembly.target = 1
        pos_assembly.pos_xy_nav_mode = 2
        pos_assembly.target_pos_x = self.valve_x
        pos_assembly.target_pos_y = 0.0
        pos_assembly.pos_z_nav_mode = 2
        pos_assembly.target_pos_z = self.valve_z + 0.5
        
        self.pos_pub.publish(pos_assembly)
        time.sleep(5)

        rospy.loginfo("Arrived at the valve position. Starting rotation...")
        
        # valve rotation
        pos_assembly.yaw_nav_mode = 2
        pos_assembly.target_yaw = self.valve_rotation_angle
        self.pos_pub.publish(pos_assembly)
        time.sleep(5)

        rospy.loginfo("Valve rotation completed. Returning to start position...")

        # back to start position
        pos_assembly.pos_xy_nav_mode = 2
        pos_assembly.target_pos_x = self.start_x
        pos_assembly.target_pos_y = 0.0
        pos_assembly.pos_z_nav_mode = 2
        pos_assembly.target_pos_z = self.start_z
        pos_assembly.yaw_nav_mode = 2
        pos_assembly.target_yaw = 0.0
        
        self.pos_pub.publish(pos_assembly)
        time.sleep(5)

        rospy.loginfo("Returned to start position.")
        return 'succeeded'


def main():
    rospy.init_node('simple_valve_rotation_demo')
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])

    with sm:
        smach.StateMachine.add('MOVE_AND_ROTATE_VALVE', MoveAndRotateValveState(),
                               transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_FAILED'})

    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
