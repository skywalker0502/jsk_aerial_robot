#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import threading
from tf.transformations import euler_from_quaternion
import math
import numpy as np

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


class MoveAndRotateValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.module_ids = [1, 2]
        self.pos_beetle_2 = PoseStamped()
        self.pos_beetle_1 = PoseStamped()
        self.pos_valve = PoseStamped()
        self.pos_valve_sim = Odometry()
        self.is_simulation = rospy.get_param("~simulation", False)
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
        self.z_offset_real = 0.47#0.21
        self.z_offset_sim = 0.23
        self.yaw_offset = 0
        self.valve_rotation_angle = math.pi / 2.0  

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
        yaw_combined = (yaw1 + yaw2) / 2.0
        return yaw_combined

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

    def move_to_target_poly(self, target_x, target_y, target_z, duration=10.0):

        self.update_current_pos()
        start_pos = (
            self.current_pos.pose.position.x,
            self.current_pos.pose.position.y,
            self.current_pos.pose.position.z
        )
        target_pos = (target_x, target_y, target_z)
        rospy.loginfo(f"Moving from {start_pos} to {target_pos} over {duration} seconds.")
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

    def rotate_to_target_poly(self, target_yaw, duration=10.0, fixed_x=None, fixed_y=None, fixed_z=None):

        current_yaw = self.get_uav_yaw()
        rospy.loginfo(f"Rotating from yaw {current_yaw:.4f} to {target_yaw:.4f} over {duration} seconds.")
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
            pos_cmd.yaw_nav_mode = FlightNav.POS_MODE #POS_MODE
            pos_cmd.target_yaw = yaw_val
            # pos_cmd.target_omega_z = 0.15
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
        self.rotate_to_target_poly(0, duration=3.0, fixed_x=self.start_x, fixed_y=self.start_y, fixed_z=self.start_z)

        rospy.loginfo("Moving horizontally to the valve position...")
        self.move_to_target_poly(self.valve_x, self.valve_y, self.current_pos.pose.position.z, duration=20.0)

        rospy.loginfo("Descending to the valve operation height...")
        target_z = self.valve_z + (self.z_offset_sim if self.is_simulation else self.z_offset_real)
        self.move_to_target_poly(self.valve_x, self.valve_y, target_z, duration=10.0)

        rospy.loginfo("Arrived at the valve position. Starting valve rotation...")
        self.rotate_to_target_poly(self.valve_rotation_angle+self.yaw_offset, duration=10.0, fixed_x=self.valve_x, fixed_y=self.valve_y, fixed_z=target_z)

        rospy.loginfo("Valve rotation completed. Hovering at start altitude...")
        self.move_to_target_poly(self.valve_x, self.valve_y, self.start_z, duration=10.0)
        self.rotate_to_target_poly(0.0, duration=5.0, fixed_x=self.valve_x, fixed_y=self.valve_y, fixed_z=self.start_z)

        rospy.loginfo("Returning to start position...")
        self.move_to_target_poly(self.start_x, self.start_y, self.start_z, duration=10.0)

        if self.pos_check():
            rospy.loginfo("Returned to start position successfully.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to return to start position.")
            return 'failed'

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
