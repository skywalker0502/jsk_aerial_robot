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

class StandbyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.demo_assemble = AssembleDemo()

    def execute(self, userdata):
        rospy.loginfo("Executing Standby State...")
        try:
            self.demo_assemble.main()
            rospy.loginfo("AssembleDemo succeeded!")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Standby State failed.")
            return 'failed'

class MoveToGateState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
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
        self.pos_beetle_2_pub = rospy.Publisher("/beetle2/target_pose", PoseStamped, queue_size=1)
        self.maze_entrance_x = 7.0
        self.valve_pos_x = 15.0
        self.valve_pos_z = 1.4
        self.beetle_1_y_path = 0.75
        self.beetle_2_y_path = -0.75
        self.assembling_distance = 1.1

    def execute(self, userdata):
        rospy.loginfo("Moving to the valve...")
        pos_beetle_1 = PoseStamped()
        pos_beetle_2 = PoseStamped()
        pos_beetle_1.pose.position.x = self.maze_entrance_x
        pos_beetle_1.pose.position.y = self.beetle_1_y_path
        pos_beetle_1.pose.position.z = self.valve_pos_z - 0.4

        pos_beetle_2.pose.position.x = self.maze_entrance_x
        pos_beetle_2.pose.position.y = self.beetle_2_y_path
        pos_beetle_2.pose.position.z = self.valve_pos_z - 0.4

        self.pos_beetle_1_pub.publish(pos_beetle_1)
        self.pos_beetle_2_pub.publish(pos_beetle_2)
        time.sleep(6)

        pos_beetle_1.pose.position.x = self.valve_pos_x - self.assembling_distance
        pos_beetle_2.pose.position.x = self.valve_pos_x - self.assembling_distance

        self.pos_beetle_1_pub.publish(pos_beetle_1)
        self.pos_beetle_2_pub.publish(pos_beetle_2)
        time.sleep(15)

        rospy.loginfo("Reached the valve.")
        return 'succeeded'

class RotateValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.demo_assemble = AssembleDemo()  # 调用 AssemblyDemo
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
        self.pos_beetle_1_pub = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=1)
        self.pos_beetle_2_pub = rospy.Publisher("/beetle2/target_pose", PoseStamped, queue_size=1)
        self.valve_pos_x = 15.0
        self.valve_pos_y = 0.0
        self.valve_pos_z = 1.4
        self.valve_rotation_angle = 3.14
        self.assembling_distance = 1.1
    def execute(self, userdata):
        rospy.loginfo("Starting assembly for valve rotation...")
        pos_beetle_1 = PoseStamped()
        pos_beetle_2 = PoseStamped()
        pos_beetle_1.pose.position.x = self.valve_pos_x-self.assembling_distance 
        pos_beetle_2.pose.position.y = self.valve_pos_y
        pos_beetle_2.pose.position.z = self.valve_pos_z-0.5
        pos_beetle_1.pose.position.z = self.valve_pos_z-0.5
        pos_beetle_1.pose.position.y = self.valve_pos_y
        pos_beetle_2.pose.position.x = self.valve_pos_x+0.2
        self.pos_beetle_2_pub.publish(pos_beetle_2)
        time.sleep(1.5)
        self.pos_beetle_1_pub.publish(pos_beetle_1)
        time.sleep(6)
        # Step 1: 调用 AssemblyDemo 完成合体
        try:
            self.demo_assemble.main()
            rospy.loginfo("Assembly completed successfully.")
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly process failed.")
            return 'failed'

        # Step 2: 阀门旋转操作
        rospy.loginfo("Starting valve rotation task...")

        pos_assembly = FlightNav()
        pos_assembly.target = 1
        pos_assembly.pos_z_nav_mode = 2
        pos_assembly.target_pos_z = self.valve_pos_z

        # 提升至阀门高度
        self.pos_pub.publish(pos_assembly)
        rospy.loginfo("Moving to valve height...")
        time.sleep(6)

        # 执行旋转任务
        pos_assembly.yaw_nav_mode = 2
        pos_assembly.target_yaw = self.valve_rotation_angle
        self.pos_pub.publish(pos_assembly)
        rospy.loginfo("Rotating the valve...")
        time.sleep(7)

        # 降低高度
        pos_assembly.target_pos_z = 1.0
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
        self.pos_beetle_2_pub = rospy.Publisher("/beetle2/target_pose", PoseStamped, queue_size=1)
        self.maze_exit_x = 25.0
        self.beetle_1_y_path = 0.75
        self.beetle_2_y_path = -0.75

    def execute(self, userdata):
        rospy.loginfo("Starting disassembly to separate robots...")

        # Step 1: 调用 DisassemblyDemo 完成分离
        try:
            self.demo_disassemble.main()
            rospy.loginfo("Disassembly completed successfully.")
        except rospy.ROSInterruptException:
            rospy.logerr("Disassembly process failed.")
            return 'failed'

        # Step 2: 离开阀门区域
        rospy.loginfo("Robots are leaving the valve area...")

        pos_beetle_1 = PoseStamped()
        pos_beetle_2 = PoseStamped()

        # 设置机器人移动到出口位置
        pos_beetle_1.pose.position.x = self.maze_exit_x-7.5
        pos_beetle_1.pose.position.y = self.beetle_1_y_path
        pos_beetle_1.pose.position.z = 1.0

        pos_beetle_2.pose.position.x = self.maze_exit_x-7.5
        pos_beetle_2.pose.position.y = self.beetle_2_y_path
        pos_beetle_2.pose.position.z = 1.0

        # 发布目标位置
        self.pos_beetle_1_pub.publish(pos_beetle_1)
        self.pos_beetle_2_pub.publish(pos_beetle_2)
        time.sleep(4)
        pos_beetle_1.pose.position.x = self.maze_exit_x
        pos_beetle_1.pose.position.y = self.beetle_1_y_path
        pos_beetle_1.pose.position.z = 1.0

        pos_beetle_2.pose.position.x = self.maze_exit_x
        pos_beetle_2.pose.position.y = self.beetle_2_y_path
        pos_beetle_2.pose.position.z = 1.0
        self.pos_beetle_1_pub.publish(pos_beetle_1)
        self.pos_beetle_2_pub.publish(pos_beetle_2)
        rospy.loginfo("Moving robots to exit...")
        time.sleep(10)

        rospy.loginfo("Robots successfully left the valve area.")
        return 'succeeded'


def main():
    rospy.init_node('valverotation_demo')

    # 创建 SMACH 状态机
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])

    with sm:
        # 注册各个状态及其可能的跳转结果
        smach.StateMachine.add('STANDBY', StandbyState(),
                               transitions={'succeeded': 'MOVE_TO_GATE', 'failed': 'TASK_FAILED'})

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

    # 执行 SMACH 状态机
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
