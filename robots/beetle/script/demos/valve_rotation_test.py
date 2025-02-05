#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import threading

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
        self.z_offset_sim = 0.443
        self.valve_rotation_angle = 3.14  
        self.pid_x = PIDController(1.0, 0.0, 0.1, max_output=1.0, min_output=-1.0)
        self.pid_y = PIDController(1.0, 0.0, 0.1, max_output=1.0, min_output=-1.0)
        self.pid_z = PIDController(1.0, 0.0, 0.1, max_output=1.0, min_output=-1.0)

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
    
    def pos_check(self, tolerance=0.09):

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


    def approach_target_valve(self, target_x, target_y, target_z, tolerance=0.05):
        rate = rospy.Rate(20)  
        while not rospy.is_shutdown():
            current_x = self.current_pos.pose.position.x
            current_y = self.current_pos.pose.position.y
            current_z = self.current_pos.pose.position.z

            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = target_z - current_z

            if abs(error_x) < tolerance and abs(error_y) < tolerance and abs(error_z) < tolerance:
                rospy.loginfo("Target position reached.")
                break

            dt = 1.0 / 20.0  
            vx = self.pid_x.compute(target_x, current_x, dt)
            vy = self.pid_y.compute(target_y, current_y, dt)
            vz = self.pid_z.compute(target_z, current_z, dt)

            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = 2  
            pos_cmd.target_vel_x = vx
            pos_cmd.target_vel_y = vy
            pos_cmd.pos_z_nav_mode = 2  
            pos_cmd.target_vel_z = vz

            self.pos_pub.publish(pos_cmd)
            rate.sleep()

    def maintain_position_during_rotation(self, target_x, target_y, target_z, target_yaw, duration):
        rate = rospy.Rate(20)
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now().to_sec() - start_time
            if elapsed_time > duration:
                break
            
            self.update_current_pos()
            vx = self.pid_x.compute(target_x, self.current_pos.pose.position.x, 1.0/20.0)
            vy = self.pid_y.compute(target_y, self.current_pos.pose.position.y, 1.0/20.0)
            vz = self.pid_z.compute(target_z, self.current_pos.pose.position.z, 1.0/20.0)

            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = 2
            pos_cmd.target_vel_x = vx
            pos_cmd.target_vel_y = vy
            pos_cmd.pos_z_nav_mode = 2
            pos_cmd.target_vel_z = vz
            pos_cmd.yaw_nav_mode = 2
            pos_cmd.target_yaw = target_yaw

            self.pos_pub.publish(pos_cmd)
            rate.sleep()

    def execute(self, userdata):
        if not self.pos_initialization:
            rospy.logwarn("Positions are not initialized. Aborting mission.")
            return 'failed'
        self.update_current_pos()
        
        rospy.loginfo("Moving horizontally to the valve position")
        self.approach_target_valve(self.valve_x, self.valve_y, self.current_pos.pose.position.z)  

        rospy.loginfo("Descending to the valve operation height...")
        target_z = self.valve_z + (self.z_offset_sim if self.is_simulation else self.z_offset_real)
        self.approach_target_valve(self.valve_x, self.valve_y, target_z)

        rospy.loginfo("Arrived at the valve position. Starting rotation")
        self.maintain_position_during_rotation(self.valve_x, self.valve_y, target_z, self.valve_rotation_angle, 5)

        rospy.loginfo("Valve rotation completed. Returning to start position...")
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
