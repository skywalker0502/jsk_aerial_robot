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

class PIDController:
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0
        self.previous_error = 0

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)
        return output

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

        self.z_offset_real = 0.21
        self.z_offset_sim = 0.23
        self.valve_rotation_angle = 3.14  
        self.pid_x = PIDController(0.2, 0.0, 0.1, max_output=0.1, min_output=-0.1)
        self.pid_y = PIDController(0.2, 0.0, 0.1, max_output=0.1, min_output=-0.1)


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

    def approach_target_valve(self, target_x, target_y, target_z, tolerance=0.05, maintain_z=False):

        rate = rospy.Rate(20)
        step_size_z = 0.02  
        count_i = 0

        if maintain_z:
            fixed_z = self.current_pos.pose.position.z  
        else:
            fixed_z = target_z  

        self.pid_x.reset()
        self.pid_y.reset()

        while not rospy.is_shutdown():
            self.update_current_pos()

            current_x = self.current_pos.pose.position.x
            current_y = self.current_pos.pose.position.y
            current_z = self.current_pos.pose.position.z

            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = target_z - current_z  

            dt = 1.0 / 20.0
            vx = self.pid_x.compute(target_x, current_x, dt)
            vy = self.pid_y.compute(target_y, current_y, dt)

            if maintain_z:
                target_pos_z = fixed_z  
            else:
                if abs(error_z) > tolerance:
                    if target_z > current_z:
                        target_pos_z = min(current_z + step_size_z, target_z)  
                    else:
                        target_pos_z = max(current_z - step_size_z, target_z)  
                else:
                    target_pos_z = round(target_z, 4)  

            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = 1
            pos_cmd.target_vel_x = vx
            pos_cmd.target_vel_y = vy

            pos_cmd.pos_z_nav_mode = 2  
            pos_cmd.target_pos_z = target_pos_z
            count_i = count_i +1    
            if count_i % 20 == 0:
                rospy.loginfo(f"Publishing - vx: {vx:.4f}, vy: {vy:.4f}, target_pos_z: {target_pos_z:.4f}")
            self.pos_pub.publish(pos_cmd)
            rate.sleep()

            if abs(error_x) < tolerance and abs(error_y) < tolerance and abs(error_z) < tolerance:
                rospy.loginfo("Target position reached.")
                break



    def maintain_position_during_rotation(self, target_x, target_y, target_z, target_yaw, duration, yaw_tolerance=0.05):
        rate = rospy.Rate(20)  
        start_time = rospy.Time.now().to_sec()

        current_yaw = self.get_uav_yaw()  
        yaw_step = 0.05  

        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now().to_sec() - start_time
            if elapsed_time > duration:
                rospy.logwarn("Rotation timeout reached, stopping rotation.")
                break

            self.update_current_pos()
            current_yaw = self.get_uav_yaw()  

            yaw_error = target_yaw - current_yaw
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  

            if abs(yaw_error) > yaw_step:
                target_yaw_step = current_yaw + (yaw_step if yaw_error > 0 else -yaw_step)
            else:
                target_yaw_step = target_yaw  

            rospy.loginfo(f"Yaw error: {yaw_error:.4f}, target step: {target_yaw_step:.4f}, current: {current_yaw:.4f}")

            if abs(yaw_error) < yaw_tolerance:
                rospy.loginfo("Rotation complete: UAV reached target yaw.")
                break

            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = 1
            pos_cmd.target_vel_x = 0
            pos_cmd.target_vel_y = 0
            pos_cmd.pos_z_nav_mode = 1
            pos_cmd.target_vel_z = 0
            pos_cmd.yaw_nav_mode = 2
            pos_cmd.target_yaw = target_yaw_step 

            self.pos_pub.publish(pos_cmd)
            rate.sleep()


    def execute(self, userdata):
        if not self.pos_initialization:
            rospy.logwarn("Positions are not initialized. Aborting mission.")
            return 'failed'

        self.update_current_pos()

        rospy.loginfo("Moving horizontally to the valve position while maintaining current altitude...")
        self.approach_target_valve(self.valve_x, self.valve_y, self.current_pos.pose.position.z, maintain_z=True)

        rospy.loginfo("Descending to the valve operation height...")
        target_z = self.valve_z + (self.z_offset_sim if self.is_simulation else self.z_offset_real)
        self.approach_target_valve(self.valve_x, self.valve_y, target_z)

        rospy.loginfo("Arrived at the valve position. Starting rotation...")
        self.maintain_position_during_rotation(self.valve_x, self.valve_y, target_z, self.valve_rotation_angle, 20, yaw_tolerance=0.05)

        rospy.loginfo("Valve rotation completed. Hovering...")
        self.maintain_position_during_rotation(self.valve_x, self.valve_y, self.start_z, 0.0, 20, yaw_tolerance=0.05)  
        self.approach_target_valve(self.valve_x, self.valve_y, self.start_z)  

        rospy.loginfo(" Returning to start position...")
        self.approach_target_valve(self.start_x, self.start_y, self.start_z)

        if self.pos_check():
            rospy.loginfo("Returned to start position.")
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
