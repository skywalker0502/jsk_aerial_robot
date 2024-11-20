#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
import os
import sys
sys.path.append('/home/mzq0502/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/')
# from tasks.assembly_motion import * 
# from tasks.disassembly_motion import *
from beetle.assembly_api import *
from beetle.disassembly_api import *
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped
class valverotationDemo():
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("valverotation_demo", anonymous=True)
        # rospy.init_node("valverotation_demo", anonymous=True)DesireCoor
        self.pos_beetle_1_pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=1)
        self.pos_beetle_2_pub = rospy.Publisher("/beetle2/uav/nav", FlightNav, queue_size=1)
        self.pos_assembly_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
        self.rotation_pub = rospy.Publisher("/assembled/final_target_baselink_rot", DesireCoord, queue_size=10)
        self.pos_beetle_1_sub = rospy.Subscriber("/beetle1/mocap/pose", PoseStamped, self.beetle_1_callback, queue_size=1)
        self.pos_beetle_2_sub = rospy.Subscriber("/beetle2/mocap/pose", PoseStamped, self.beetle_2_callback, queue_size=1)
        self.pos_beetle_1 = PoseStamped()
        self.pos_beetle_2 = PoseStamped()
        self.beetle_1_y_path = 0.71
        self.beetle_2_y_path = -0.75
        self.valve_rotation_angle = 3.14
        self.valve_pos_z = 1.4
        self.valve_pos_x = 14.8
        self.valve_pos_y = 0.0
        self.assembing_diatance = 1.1
        self.maze_entrace_x = 7.0
        self.maze_exit_x = 25.0
        self.flag = 0

    def beetle_1_callback(self, msg):
        self.pos_beetle_1 = msg

    def beetle_2_callback(self, msg):
        self.pos_beetle_2 = msg

    def main(self):
        # Step 0: StandbyState
        try:
            # demo_assemble = AssemblyDemo(module_ids = "1,2", real_machine=False)
            demo_assemble = AssembleDemo()
            outcome=demo_assemble.main() 
            time.sleep(0.1) 
            rospy.loginfo("The first step is completed!")
            # if outcome == 'succeeded':
            rospy.loginfo("AssembleDemo succeeded, moving to next step.")
            self.flag = 1  
            # else:
            #     rospy.loginfo("AssembleDemo did not succeed, stopping.")
                # return 
        except rospy.ROSInterruptException: pass
        # Step 1: Moving to gate
        if self.flag == 1:
            pos_assembly = FlightNav()
            pos_assembly.target=1
            pos_assembly.pos_xy_nav_mode = 2
            pos_assembly.target_pos_x = self.maze_entrace_x - 2.1
            # pos_assembly.target_vel_x = 0.1
            self.pos_assembly_pub.publish(pos_assembly)
            time.sleep(7)
            rospy.loginfo("Reaching the entrance, moving to next step.")
            # if  self.pos_beetle_2.pose.position.x > self.maze_entrace_x - 2.2:
            self.flag = 2
        # Step 2: Disassemble to pass the path
        try:
            if self.flag == 2:
                # demo_disassemble = DisassemblyDemo(module_ids = "1,2", real_machine=False);
                demo_disassemble = DisassembleDemo();
                demo_disassemble.main()
                time.sleep(3)
                self.flag = 3
                rospy.loginfo("Disassemble at the entrance, moving to next step.")
        except rospy.ROSInterruptException: pass
        # Step 3: Moving to the valve
        if self.flag == 3:
            pos_beetle_1 = FlightNav()
            pos_beetle_1.pos_xy_nav_mode = 2
            pos_beetle_1.target_pos_x = self.maze_entrace_x
            pos_beetle_1.target_pos_y = self.beetle_1_y_path
            pos_beetle_2 = FlightNav()
            pos_beetle_2.pos_xy_nav_mode = 2
            pos_beetle_2.target_pos_x = self.maze_entrace_x
            pos_beetle_2.target_pos_y = self.beetle_2_y_path      
            self.pos_beetle_1_pub.publish(pos_beetle_1)
            self.pos_beetle_2_pub.publish(pos_beetle_2)
            time.sleep(17)
            pos_beetle_1.pos_xy_nav_mode = 2
            pos_beetle_1.target_pos_x = self.valve_pos_x-self.assembing_diatance
            pos_beetle_1.target_pos_y = self.beetle_1_y_path
            self.pos_beetle_1_pub.publish(pos_beetle_1)
            pos_beetle_2.pos_xy_nav_mode = 2
            pos_beetle_2.target_pos_x = self.valve_pos_x-self.assembing_diatance
            pos_beetle_2.target_pos_y = self.beetle_2_y_path
            self.pos_beetle_2_pub.publish(pos_beetle_2)
            time.sleep(28)

            self.flag = 4
            rospy.loginfo("Reaching the valve, moving to next step.")
        # Step 4: Assembly under the valve
        if self.flag == 4:
            try:
                pos_beetle_2.target_pos_y = self.valve_pos_y
                pos_beetle_2.target_pos_x = self.valve_pos_x
                # pos_beetle_2.target_pos_x = 14.5
                pos_beetle_1.target_pos_y = 0.0
                self.pos_beetle_1_pub.publish(pos_beetle_1)
                self.pos_beetle_2_pub.publish(pos_beetle_2)
                time.sleep(7)
                # demo_assemble = AssemblyDemo(module_ids = "1,2", real_machine=False)
                demo_assemble = AssembleDemo();
                outcome=demo_assemble.main()
                time.sleep(1)
                self.flag = 5
            except rospy.ROSInterruptException:
                pass
            rospy.loginfo("Assemble under the valve, moving to next step.")
        # Step 5: Excuting the valve rotation task
        if self.flag == 5:
            pos_assembly = FlightNav()
            pos_assembly.target=1
            pos_assembly.pos_z_nav_mode = 2
            pos_assembly.target_pos_z = self.valve_pos_z
            rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10).publish(pos_assembly)
            time.sleep(6)
            pos_assembly.yaw_nav_mode = 2
            pos_assembly.target_yaw =  self.valve_rotation_angle
            self.pos_assembly_pub.publish(pos_assembly)
            time.sleep(10)
            rospy.loginfo("The valve rotation task is completed! Moving to next step.")
            pos_assembly.target_pos_z = 1.0
            self.pos_assembly_pub.publish(pos_assembly)
            time.sleep(10)
            self.flag = 6
        # Step 6: Disassemble to pass the path
        if self.flag == 6:
            try:
                demo_disassemble.main()
                time.sleep(1)
                rospy.loginfo("Disassemble at the opposite, moving to next step.")

            except rospy.ROSInterruptException: pass
            pos_beetle_2.target_pos_y = self.valve_pos_y-0.4
            pos_beetle_1.target_pos_y = self.valve_pos_y+0.4
            pos_beetle_2.target_pos_x = self.maze_exit_x-7.5
            pos_beetle_1.target_pos_x = self.maze_exit_x-7.5
            self.pos_beetle_1_pub.publish(pos_beetle_1)
            self.pos_beetle_2_pub.publish(pos_beetle_2)
            time.sleep(7)
            pos_beetle_2.target_pos_y = self.beetle_2_y_path
            pos_beetle_1.target_pos_y = self.beetle_1_y_path
            self.pos_beetle_1_pub.publish(pos_beetle_1)
            self.pos_beetle_2_pub.publish(pos_beetle_2)
            time.sleep(12)
            self.flag = 7
        # Step 7: Moving to the opposite gate
        if self.flag == 7:
            pos_beetle_1.pos_xy_nav_mode = 2
            pos_beetle_1.target_pos_x = self.maze_exit_x
            pos_beetle_1.target_pos_y = self.beetle_1_y_path
            pos_beetle_2.pos_xy_nav_mode = 2
            pos_beetle_2.target_pos_x = self.maze_exit_x
            pos_beetle_2.target_pos_y = self.beetle_2_y_path           
            self.pos_beetle_1_pub.publish(pos_beetle_1)
            self.pos_beetle_2_pub.publish(pos_beetle_2)
            time.sleep(20)
            self.flag = 8
            rospy.loginfo("The whole task is completed!")

        rospy.spin()
if __name__ == '__main__':
    try:
        demo = valverotationDemo();
        demo.main()
    except rospy.ROSInterruptException: pass