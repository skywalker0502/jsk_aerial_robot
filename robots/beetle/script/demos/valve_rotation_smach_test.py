#!/usr/bin/env python
import rospy
import smach
import smach_ros
import time
import math
import threading
import numpy as np
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from beetle.assembly_api import AssembleDemo
from beetle.disassembly_api import DisassembleDemo

class PolynomialTrajectory:
    def __init__(self, duration):
        self.duration = duration
        self.coeffs_x = None
        self.coeffs_y = None
        self.coeffs_z = None
        self.coeffs_scalar = None  
        self.start_time = None
        self.is_scalar = False  

    def compute_coefficients(self, start, target):
        T = self.duration
        A = np.array([
            [0,         0,      0,    0,  0, 1],
            [T**5,     T**4,   T**3,  T**2, T, 1],
            [0,         0,      0,    0,  1, 0],
            [5*T**4,   4*T**3, 3*T**2, 2*T, 1, 0],
            [0,         0,      0,    2,   0, 0],
            [20*T**3, 12*T**2, 6*T,    2,   0, 0]
        ])
        B = np.array([start, target, 0, 0, 0, 0])
        return np.linalg.solve(A, B)

    def generate_trajectory(self, start_pos, target_pos):
        if isinstance(start_pos, (int, float)) and isinstance(target_pos, (int, float)):
            self.is_scalar = True
            self.coeffs_scalar = self.compute_coefficients(start_pos, target_pos)
        else:
            self.is_scalar = False
            self.coeffs_x = self.compute_coefficients(start_pos[0], target_pos[0])
            self.coeffs_y = self.compute_coefficients(start_pos[1], target_pos[1])
            self.coeffs_z = self.compute_coefficients(start_pos[2], target_pos[2])
        self.start_time = rospy.Time.now().to_sec()

    def evaluate(self):
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        if elapsed_time > self.duration:
            return None 
        T = np.array([elapsed_time**5, elapsed_time**4, elapsed_time**3, 
                      elapsed_time**2, elapsed_time, 1])
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T)
        else:
            return (
                np.dot(self.coeffs_x, T),
                np.dot(self.coeffs_y, T),
                np.dot(self.coeffs_z, T)
            )

def execute_poly_motion_pose_async(pub, start, target, avg_speed, rate_hz=20, delay_after=0):

    def motion():
        distance = math.sqrt((target[0]-start[0])**2 +
                             (target[1]-start[1])**2 +
                             (target[2]-start[2])**2)
        if avg_speed <= 0:
            speed = 0.1
        else:
            speed = avg_speed
        duration = distance / speed
        rospy.loginfo("Executing polynomial motion (PoseStamped) asynchronously: from {} to {} over {:.2f} s".format(start, target, duration))
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start, target)
        rate_obj = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            pt = traj.evaluate()
            if pt is None:
                break
            msg = PoseStamped()
            msg.pose.position.x = pt[0]
            msg.pose.position.y = pt[1]
            msg.pose.position.z = pt[2]
            pub.publish(msg)
            rate_obj.sleep()
        if delay_after > 0:
            time.sleep(delay_after)
    t = threading.Thread(target=motion)
    t.start()
    return t

def execute_poly_motion_nav(pub, start, target, avg_speed, rate_hz=20, delay_after=0):
    distance = math.sqrt((target[0]-start[0])**2 +
                         (target[1]-start[1])**2 +
                         (target[2]-start[2])**2)
    if avg_speed <= 0:
        avg_speed = 0.1
    duration = distance / avg_speed
    rospy.loginfo("Executing polynomial motion (FlightNav): from {} to {} over {:.2f} s".format(start, target, duration))
    traj = PolynomialTrajectory(duration)
    traj.generate_trajectory(start, target)
    rate_obj = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        pt = traj.evaluate()
        if pt is None:
            break
        msg = FlightNav()
        msg.target = 1
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x = pt[0]
        msg.target_pos_y = pt[1]
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.target_pos_z = pt[2]
        pub.publish(msg)
        rate_obj.sleep()
    if delay_after > 0:
        time.sleep(delay_after)

class SeparatedMoveToGateState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.beetle1_pub = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=1)
        self.beetle2_pub = rospy.Publisher("/beetle2/target_pose", PoseStamped, queue_size=1)
        self.maze_entrance_x = 7.0
        self.start_beetle1 = [0.0, 0.75, 1.0]
        self.start_beetle2 = [0.0, -0.75, 1.0]
        self.avg_speed = 0.2  
    def execute(self, userdata):
        rospy.loginfo("Separated: Moving to maze entrance using asynchronous polynomial trajectories...")
        target = [self.maze_entrance_x - 2.1, 0.0, 1.0]
        target_beetle1 = [target[0], 0.75, target[2]]
        target_beetle2 = [target[0], -0.75, target[2]]
        t1 = execute_poly_motion_pose_async(self.beetle1_pub, self.start_beetle1, target_beetle1, self.avg_speed)
        t2 = execute_poly_motion_pose_async(self.beetle2_pub, self.start_beetle2, target_beetle2, self.avg_speed)
        t1.join()
        t2.join()
        rospy.loginfo("Separated: Reached maze entrance.")
        return 'succeeded'

class SeparatedMoveToValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.beetle1_pub = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=1)
        self.beetle2_pub = rospy.Publisher("/beetle2/target_pose", PoseStamped, queue_size=1)
        self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_callback, queue_size=1)
        self.valve_pos_x = None
        self.valve_pos_z = None
        self.avg_speed = 0.2  
    def valve_callback(self, msg):
        self.valve_pos_x = msg.pose.pose.position.x
        self.valve_pos_z = msg.pose.pose.position.z - 0.17
    def execute(self, userdata):
        rospy.loginfo("Separated: Moving near the valve using asynchronous polynomial trajectories...")
        while self.valve_pos_x is None or self.valve_pos_z is None:
            rospy.logwarn("Valve position not received yet!")
            time.sleep(0.1)
        # 设定目标位置（这里假设期望无人机稍微前进）
        target_beetle1 = [self.valve_pos_x - 1.1 + 0.5, 0.75, self.valve_pos_z + 0.6]
        target_beetle2 = [self.valve_pos_x - 1.1 + 0.5, -0.75, self.valve_pos_z + 0.6]
        # 假设起始位置为当前设定的基本位置
        start_beetle1 = [self.valve_pos_x - 1.1, 0.75, self.valve_pos_z + 0.6]
        start_beetle2 = [self.valve_pos_x - 1.1, -0.75, self.valve_pos_z + 0.6]
        # 同时启动两个运动线程
        t1 = execute_poly_motion_pose_async(self.beetle1_pub, start_beetle1, target_beetle1, self.avg_speed)
        t2 = execute_poly_motion_pose_async(self.beetle2_pub, start_beetle2, target_beetle2, self.avg_speed)
        t1.join()
        t2.join()
        rospy.loginfo("Separated: Reached valve vicinity.")
        return 'succeeded'

class AssembleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.assemble_demo = AssembleDemo()
    def execute(self, userdata):
        rospy.loginfo("Assembling UAVs...")
        try:
            self.assemble_demo.main()
            time.sleep(1)
            rospy.loginfo("Assembly succeeded!")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly failed.")
            return 'failed'

class MoveAndRotateValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.module_ids = [1, 2]
        self.pos_beetle_2 = PoseStamped()
        self.pos_beetle_1 = PoseStamped()
        self.pos_valve = PoseStamped()
        self.pos_valve_sim = Odometry()
        self.is_simulation = rospy.get_param("~simulation", True)
        self.beetle_1_received = threading.Event()
        self.beetle_2_received = threading.Event()
        self.valve_received = threading.Event()
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
        self.pos_beetle_2_sub = rospy.Subscriber("/beetle2/mocap/pose", PoseStamped, self.beetle_2_callback, queue_size=1)
        self.pos_beetle_1_sub = rospy.Subscriber("/beetle1/mocap/pose", PoseStamped, self.beetle_1_callback, queue_size=1)
        if self.is_simulation:
            self.pos_valve_sim_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.pos_valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        self.pos_initialization = False 
        self.wait_for_initialization(timeout=10)
        self.z_offset_real = 0.47  
        self.z_offset_sim = 0.23
        self.yaw_offset = 0
        self.valve_rotation_angle_compenstation = 0.06
        self.valve_rotation_angle = math.pi / 2.0  + self.valve_rotation_angle_compenstation
        self.avg_speed = 0.1       
        self.avg_yaw_speed = 0.15
        self.avg_valve_rotation_speed = 0.3

    def beetle_2_callback(self, msg):
        self.pos_beetle_2 = msg
        self.beetle_2_received.set()

    def beetle_1_callback(self, msg):
        self.pos_beetle_1 = msg
        self.beetle_1_received.set()
    
    def valve_callback(self, msg):
        self.pos_valve = msg
        self.pos_valve_sub.unregister()
        self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        self.pos_valve_sim = msg
        self.pos_valve_sim_sub.unregister()
        self.valve_received.set()

    def wait_for_initialization(self, timeout=10):
        events = [self.beetle_1_received, self.beetle_2_received, self.valve_received]
        all_received = all(event.wait(timeout) for event in events)
        if all_received:
            rospy.loginfo("All positions received, initializing...")
            self.pos_initialize()
            self.pos_initialization = True
        else:
            rospy.logwarn("Timeout waiting for position messages.")
            self.pos_initialization = False

    def update_current_pos(self):
        self.current_pos = PoseStamped()
        self.current_pos.pose.position.x = (self.pos_beetle_2.pose.position.x + self.pos_beetle_1.pose.position.x) / len(self.module_ids)
        self.current_pos.pose.position.y = (self.pos_beetle_2.pose.position.y + self.pos_beetle_1.pose.position.y) / len(self.module_ids)
        self.current_pos.pose.position.z = (self.pos_beetle_2.pose.position.z + self.pos_beetle_1.pose.position.z) / len(self.module_ids)

    def pos_initialize(self):
        self.start_x = (self.pos_beetle_2.pose.position.x + self.pos_beetle_1.pose.position.x) / len(self.module_ids)
        self.start_y = (self.pos_beetle_2.pose.position.y + self.pos_beetle_1.pose.position.y) / len(self.module_ids)
        self.start_z = (self.pos_beetle_2.pose.position.z + self.pos_beetle_1.pose.position.z) / len(self.module_ids)
        if self.is_simulation:
            self.valve_x = self.pos_valve_sim.pose.pose.position.x
            self.valve_y = self.pos_valve_sim.pose.pose.position.y
            self.valve_z = self.pos_valve_sim.pose.pose.position.z
        else:
            self.valve_x = self.pos_valve.pose.position.x
            self.valve_y = self.pos_valve.pose.position.y
            self.valve_z = self.pos_valve.pose.position.z
        self.update_current_pos()
        rospy.loginfo(f"start position is [{self.start_x}, {self.start_y}, {self.start_z}]")
        rospy.loginfo(f"valve position is [{self.valve_x}, {self.valve_y}, {self.valve_z}]")
        self.pos_initialization = True
    
    def get_uav_yaw(self):
        orientation_q1 = self.pos_beetle_1.pose.orientation
        orientation_q2 = self.pos_beetle_2.pose.orientation
        quaternion1 = (orientation_q1.x, orientation_q1.y, orientation_q1.z, orientation_q1.w)
        quaternion2 = (orientation_q2.x, orientation_q2.y, orientation_q2.z, orientation_q2.w)
        _, _, yaw1 = euler_from_quaternion(quaternion1)
        _, _, yaw2 = euler_from_quaternion(quaternion2)
        return (yaw1 + yaw2) / 2.0

    def pos_check(self, tolerance=0.1):
        self.update_current_pos()  
        error_x = abs(self.current_pos.pose.position.x - self.start_x)
        error_y = abs(self.current_pos.pose.position.y - self.start_y)
        error_z = abs(self.current_pos.pose.position.z - self.start_z)
        rospy.loginfo(f"Checking position - Errors: x={error_x}, y={error_y}, z={error_z}")
        if error_x < tolerance and error_y < tolerance and error_z < tolerance:
            rospy.loginfo("Successfully went to the position.")
            return True
        else:
            rospy.logwarn("Failed to went to the position.")
            return False

    def move_to_target_poly(self, target_x, target_y, target_z, avg_speed=None):
        if avg_speed is None:
            avg_speed = self.avg_speed
        self.update_current_pos()
        start_pos = (
            self.current_pos.pose.position.x,
            self.current_pos.pose.position.y,
            self.current_pos.pose.position.z
        )
        target_pos = (target_x, target_y, target_z)
        distance = math.sqrt((target_x - start_pos[0])**2 +
                             (target_y - start_pos[1])**2 +
                             (target_z - start_pos[2])**2)
        if avg_speed <= 0:
            avg_speed = 0.1
        duration = distance / avg_speed
        rospy.loginfo(f"Moving from {start_pos} to {target_pos} over duration {duration:.2f} s (avg_speed = {avg_speed} m/s).")
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pos = traj.evaluate()
            if pos is None:
                break
            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_x = pos[0]
            pos_cmd.target_pos_y = pos[1]
            pos_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_z = pos[2]
            self.pos_pub.publish(pos_cmd)
            rate.sleep()
        rospy.loginfo("Reached target position.")
        time.sleep(3)

    def rotate_to_target_poly(self, target_yaw, avg_yaw_speed=None, fixed_x=None, fixed_y=None, fixed_z=None):
        if avg_yaw_speed is None:
            avg_yaw_speed = self.avg_yaw_speed
        current_yaw = self.get_uav_yaw()
        yaw_diff = target_yaw - current_yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        duration = abs(yaw_diff) / avg_yaw_speed if avg_yaw_speed > 0 else 0.1
        rospy.loginfo(f"Rotating from yaw {current_yaw:.4f} to {target_yaw:.4f} (diff={yaw_diff:.4f}) over duration {duration:.2f} s (avg_yaw_speed = {avg_yaw_speed} rad/s).")
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(current_yaw, target_yaw)
        rate = rospy.Rate(20)
        if fixed_x is None or fixed_y is None or fixed_z is None:
            self.update_current_pos()
            fixed_x = self.current_pos.pose.position.x
            fixed_y = self.current_pos.pose.position.y
            fixed_z = self.current_pos.pose.position.z
        while not rospy.is_shutdown():
            yaw_val = traj.evaluate()
            if yaw_val is None:
                break
            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_x = fixed_x
            pos_cmd.target_pos_y = fixed_y
            pos_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_z = fixed_z
            pos_cmd.yaw_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_yaw = yaw_val
            self.pos_pub.publish(pos_cmd)
            rate.sleep()
        rospy.loginfo("Rotation complete, reached target yaw.")
        time.sleep(2)

    def execute(self, userdata):
        if not self.pos_initialization:
            rospy.logwarn("Positions are not initialized. Aborting mission.")
            return 'failed'
        self.update_current_pos()
        rospy.loginfo("Initializing position with yaw 0...")
        self.rotate_to_target_poly(0, avg_yaw_speed=self.avg_yaw_speed,
                                     fixed_x=self.start_x, fixed_y=self.start_y, fixed_z=self.start_z)
        rospy.loginfo("Moving horizontally to the valve position...")
        self.move_to_target_poly(self.valve_x, self.valve_y, self.current_pos.pose.position.z, avg_speed=self.avg_speed)
        rospy.loginfo("Descending to the valve operation height...")
        target_z = self.valve_z + (self.z_offset_sim if self.is_simulation else self.z_offset_real)
        self.move_to_target_poly(self.valve_x, self.valve_y, target_z, avg_speed=self.avg_speed)
        rospy.loginfo("Arrived at the valve position. Starting valve rotation...")
        self.rotate_to_target_poly(self.valve_rotation_angle + self.yaw_offset, avg_yaw_speed=self.avg_valve_rotation_speed,
                                     fixed_x=self.valve_x, fixed_y=self.valve_y, fixed_z=target_z)
        rospy.loginfo("Valve rotation completed. Hovering at start altitude...")
        self.move_to_target_poly(self.valve_x, self.valve_y, self.start_z, avg_speed=self.avg_speed)
        self.rotate_to_target_poly(0.0, avg_yaw_speed=self.avg_yaw_speed,
                                   fixed_x=self.valve_x, fixed_y=self.valve_y, fixed_z=self.start_z)
        rospy.loginfo("Returning to start position...")
        self.move_to_target_poly(self.start_x, self.start_y, self.start_z, avg_speed=self.avg_speed)
        if self.pos_check():
            rospy.loginfo("Returned to start position successfully.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to return to start position.")
            return 'failed'

class AssembledLeaveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
        self.exit_target = [25.0, 0.0, 1.0]
    def execute(self, userdata):
        rospy.loginfo("Assembled: Leaving valve area using polynomial trajectory...")
        execute_poly_motion_nav(self.pos_pub, self.exit_target, self.exit_target, avg_speed=0.15, delay_after=10)
        rospy.loginfo("Assembled: Left valve area.")
        return 'succeeded'

def main():
    rospy.init_node('valve_task_state_machine')
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
    with sm:
        smach.StateMachine.add('SEPARATED_MOVE_GATE', SeparatedMoveToGateState(),
                                 transitions={'succeeded': 'SEPARATED_MOVE_VALVE'})
        smach.StateMachine.add('SEPARATED_MOVE_VALVE', SeparatedMoveToValveState(),
                                 transitions={'succeeded': 'ASSEMBLE'})
        smach.StateMachine.add('ASSEMBLE', AssembleState(),
                                 transitions={'succeeded': 'ASSEMBLED_VALVE_TASK',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('ASSEMBLED_VALVE_TASK', MoveAndRotateValveState(),
                                 transitions={'succeeded': 'ASSEMBLED_LEAVE',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('ASSEMBLED_LEAVE', AssembledLeaveState(),
                                 transitions={'succeeded': 'TASK_COMPLETED'})
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
