#!/usr/bin/env python

import rospy
# import smach
# import smach_ros
import math
import numpy as np
from beetle.assembly_api import * 
from beetle.disassembly_api import *
from assembly_demo import *
from disassembly_demo import *
class valverotationDemo():
    def __init__(self):
        rospy.init_node("valverotation_demo")
        self.pos_beetle_1_pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=1)
        self.pos_beetle_2_pub = rospy.Publisher("/beetle2/uav/nav", FlightNav, queue_size=1)
        self.pos_assemly_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
        self.flag = 0
    def main(self):
        # Step 1: StandbyState
        try:
            demo = AssemblyDemo();
            demo.main()

        except rospy.ROSInterruptException: pass
        # Step 1: Moving to gate
        while self.flag == 0:
            pos = FlightNav()
            pos.pos_xy_nav_mode = 2
            pos.target_pos_x = 2.5
            self.pos_assemly_pub.publish(pos)
            rospy.sleep(10)
            self.flag = 1
        # Step 2: Disassemble to pass the path
        try:
            demo = DisassemblyDemo();
            demo.main()
        except rospy.ROSInterruptException: pass
        # Step 3: Moving to the valve
        while self.flag == 1:
            pos_beetel_1 = FlightNav()
            pos_beetel_1.pos_xy_nav_mode = 2
            pos_beetel_1.target_pos_x = 10.0
            pos_beetel_1.target_pos_y = 0.65
            pos_beetel_2 = FlightNav()
            pos_beetel_2.pos_xy_nav_mode = 2
            pos_beetel_2.target_pos_x = 10.0
            pos_beetel_2.target_pos_y = -0.65          
            self.pos_beetle_1_pub.publish(pos_beetel_1)
            self.pos_beetle_2_pub.publish(pos_beetel_2)
            rospy.sleep(10)
            self.flag = 2
        # Step 4: Assembly under the valve
        try:
            demo = AssemblyDemo();
            demo.main()
        except rospy.ROSInterruptException: pass
        # Step 5: Excuting the valve rotation task
        while self.flag == 2:
            pos.yaw_nav_mode = 2
            pos.target_yaw = 3.14
            self.pos_assemly_pub.publish(pos)
            rospy.sleep(10)
            self.flag = 3
        # Step 6: Disassemble to pass the path
        try:
            demo = DisassemblyDemo();
            demo.main()
        except rospy.ROSInterruptException: pass
        # Step 7: Moving to the opposite gate
        while self.flag == 3:
            pos_beetel_1.pos_xy_nav_mode = 2
            pos_beetel_1.target_pos_x = 15.0
            pos_beetel_1.target_pos_y = 0.65
            pos_beetel_2.pos_xy_nav_mode = 2
            pos_beetel_2.target_pos_x = 15.0
            pos_beetel_2.target_pos_y = -0.65           
            self.pos_beetle_1_pub.publish(pos_beetel_1)
            self.pos_beetle_2_pub.publish(pos_beetel_2)
            rospy.sleep(10)
            self.flag = 4
            rospy.loginfo("The valve rotation task is completed!")

        rospy.spin()
if __name__ == '__main__':
    try:
        demo = valverotationDemo();
        demo.main()
    except rospy.ROSInterruptException: pass
