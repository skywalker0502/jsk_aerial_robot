#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from beetle.assembly_api import AssembleDemo
from beetle.disassembly_api import DisassembleDemo
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped


class MoveToGateState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pos_pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=1)
        self.maze_entrance_x = 7.0

    def execute(self, userdata):
        rospy.loginfo("Moving to the gate...")
        pos_assembly = FlightNav()
        pos_assembly.target = 1
        pos_assembly.pos_xy_nav_mode = 2
        pos_assembly.target_pos_x = self.maze_entrance_x - 2.1
        self.pos_pub.publish(pos_assembly)
        time.sleep(5)
        rospy.loginfo("Reached the entrance.")
        return 'succeeded'

class DisassembleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.demo_disassemble = DisassembleDemo()

    def execute(self, userdata):
        rospy.loginfo("Disassembling at the entrance...")
        try:
            self.demo_disassemble.main()
            time.sleep(1)
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Disassemble State failed.")
            return 'failed'

class MoveToValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pos_beetle_1_pub = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=1)
        self.maze_entrance_x = 7.0
        self.valve_pos_x = 15.0
        self.valve_pos_z = 0.4
        self.beetle_1_y_path = 0.75
        self.assembling_distance = 1.1

    def execute(self, userdata):
        rospy.loginfo("Moving to the valve...")
        pos_beetle_1 = PoseStamped()
        pos_beetle_1.pose.position.x = self.maze_entrance_x
        pos_beetle_1.pose.position.y = self.beetle_1_y_path
        pos_beetle_1.pose.position.z = self.valve_pos_z + 0.6
        pos_beetle_1.pose.orientation.x = 1.0
        self.pos_beetle_1_pub.publish(pos_beetle_1)
        time.sleep(6)

        pos_beetle_1.pose.position.x = self.valve_pos_x - self.assembling_distance

        self.pos_beetle_1_pub.publish(pos_beetle_1)
        time.sleep(14)

        rospy.loginfo("Reached the valve.")
        return 'succeeded'

class RotateValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.demo_assemble = AssembleDemo()  
        self.pos_pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=10)
        self.pos_beetle_1_pub = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=1)
        self.valve_pos_x = 15.0
        self.valve_pos_y = 0.0
        self.valve_pos_z = 0.4
        self.valve_rotation_angle = 3.14
        self.assembling_distance = 1.1
    def execute(self, userdata):
        rospy.loginfo("Starting assembly for valve rotation...")
        pos_beetle_1 = PoseStamped()
        pos_beetle_1.pose.position.x = self.valve_pos_x-self.assembling_distance 
        pos_beetle_1.pose.position.z = self.valve_pos_z+1.0
        pos_beetle_1.pose.position.y = self.valve_pos_y
        self.pos_beetle_1_pub.publish(pos_beetle_1)
        time.sleep(6)
 
        rospy.loginfo("Starting valve rotation task...")
        pos_assembly = FlightNav()
        pos_assembly.target = 1
        pos_assembly.pos_z_nav_mode = 2
        pos_assembly.target_pos_z = self.valve_pos_z+0.46

        self.pos_pub.publish(pos_assembly)
        rospy.loginfo("Moving to valve height...")
        time.sleep(5)

        pos_assembly.yaw_nav_mode = 2
        pos_assembly.target_yaw = self.valve_rotation_angle/2 
        pos_assembly.target_pos_x = self.valve_pos_x +0.05
        self.pos_pub.publish(pos_assembly)
        rospy.loginfo("Rotating the valve...")
        time.sleep(10)

        pos_assembly.target_pos_z = 1.5
        self.pos_pub.publish(pos_assembly)
        rospy.loginfo("Lowering after rotation...")
        time.sleep(4)

        rospy.loginfo("Valve rotation task completed.")
        return 'succeeded'



class LeaveValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.demo_disassemble = DisassembleDemo()  # 调用 DisassemblyDemo
        self.pos_beetle_1_pub = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=1)
        self.maze_exit_x = 25.0
        self.beetle_1_y_path = -0.75

    def execute(self, userdata):
        rospy.loginfo("Starting disassembly to separate robots...")
        try:
            self.demo_disassemble.main()
            rospy.loginfo("Disassembly completed successfully.")
        except rospy.ROSInterruptException:
            rospy.logerr("Disassembly process failed.")
            return 'failed'

        rospy.loginfo("Robots are leaving the valve area...")

        pos_beetle_1 = PoseStamped()
        pos_beetle_1.pose.position.x = self.maze_exit_x-7.5
        pos_beetle_1.pose.position.y = self.beetle_1_y_path
        pos_beetle_1.pose.position.z = 1.0
        pos_beetle_1.pose.orientation.x = 1.0

        self.pos_beetle_1_pub.publish(pos_beetle_1)
        time.sleep(8)
        pos_beetle_1.pose.position.x = self.maze_exit_x
        pos_beetle_1.pose.position.y = self.beetle_1_y_path
        pos_beetle_1.pose.position.z = 1.0
        self.pos_beetle_1_pub.publish(pos_beetle_1)
        rospy.loginfo("Moving robots to exit...")
        time.sleep(10)

        rospy.loginfo("Robots successfully left the valve area.")
        return 'succeeded'


def main():
    rospy.init_node('valverotation_demo')
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])

    with sm:

        smach.StateMachine.add('MOVE_TO_GATE', MoveToGateState(),
                               transitions={'succeeded': 'DISASSEMBLE'})

        smach.StateMachine.add('DISASSEMBLE', DisassembleState(),
                               transitions={'succeeded': 'MOVE_TO_VALVE', 'failed': 'TASK_FAILED'})

        smach.StateMachine.add('MOVE_TO_VALVE', MoveToValveState(),
                               transitions={'succeeded': 'ROTATE_VALVE'})

        smach.StateMachine.add('ROTATE_VALVE', RotateValveState(),
                               transitions={'succeeded': 'LEAVE_VALVE', 'failed': 'TASK_FAILED'})

        smach.StateMachine.add('LEAVE_VALVE', LeaveValveState(),
                               transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_FAILED'})
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
