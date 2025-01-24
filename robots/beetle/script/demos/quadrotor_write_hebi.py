#!/usr/bin/env python
import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped
import random
class CircTrajFollow():
  def __init__(self):
    self.period = rospy.get_param("~period", 40.0)
    self.yaw = rospy.get_param("~yaw", True)
    self.loop = rospy.get_param("~loop", False)
    self.nav_pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=1)
    self.control_sub = rospy.Subscriber("/beetle1/mocap/pose", PoseStamped, self.controlCb)
    self.center_pos_x = None
    self.center_pos_y = None
    self.center_pos_z = None
    #mocapの初期状態のみ
    self.x0 = None
    self.y0 = None
    self.z0 = None
    self.nav_rate = rospy.get_param("~nav_rate", 20.0) # hz
    self.nav_rate = 1 / self.nav_rate
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    # self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_xy_nav_mode = FlightNav.VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.VEL_MODE
    # signal.signal(signal.SIGINT, self.stopRequest)
    time.sleep(0.5)

  def controlCb(self, msg):
    #mocapからとってきている現在位置
    self.center_pos_x = msg.pose.position.x
    self.center_pos_y = msg.pose.position.y
    self.center_pos_z = msg.pose.position.z
    # rospy.loginfo("position from mocap is [%f, %f]", self.center_pos_x, self.center_pos_y)
    #rospy.loginfo("position from mocap is [%f, %f]", self.center_pos_x, self.center_pos_z)
    if self.x0 is None and self.z0 is None:
      self.x0 = self.center_pos_x
      self.y0 = self.center_pos_y
      self.z0 = self.center_pos_z
    #self.control_sub.unregister()

  def stopRequest(self, signal, frame):
    rospy.loginfo("stop following")
    self.flight_nav.target_vel_x = 0
    self.flight_nav.target_vel_y = 0
    self.flight_nav.target_omega_z = 0
    self.nav_pub.publish(self.flight_nav)
    sys.exit(0)

  def main(self):
    ###cnt = 0
      l = 0.4 #square size
      d = 0.4 #pen movement
      a = 0.02 #gosa
      # rospy.loginfo(f"self.x0")
      points = [(self.x0, self.y0, self.z0),
                (self.x0, self.y0-d, self.z0), (self.x0+l, self.y0-d, self.z0), (self.x0+l, self.y0+a, self.z0),
                (self.x0+l, self.y0-d, self.z0), (self.x0+l, self.y0-d, self.z0+l), (self.x0+l, self.y0+a, self.z0+l),
                (self.x0+l, self.y0-d, self.z0+l), (self.x0, self.y0-d, self.z0+l), (self.x0, self.y0+a, self.z0+l),
                (self.x0, self.y0-d, self.z0+l), (self.x0, self.y0-d, self.z0)]
      # points = [(self.x0, self.y0, self.z0),
      #           #D
      #           (self.x0, self.y0+d, self.z0), (self.x0, self.y0+d, self.z0+0.9), (self.x0+0.4, self.y0+d, self.z0+0.7), (self.x0+0.4, self.y0+d, self.z0+0.2), (self.x0, self.y0+d, self.z0), (self.x0, self.y0, self.z0),
      #           #R
      #           (self.x0+0.5, self.y0, self.z0), (self.x0+0.5, self.y0+d, self.z0), (self.x0+0.5, self.y0+d, self.z0+0.9), (self.x0+0.7, self.y0+d, self.z0+0.9), (self.x0+0.8, self.y0+d, self.z0+0.8), (self.x0+0.8, self.y0+d, self.z0+0.6), (self.x0+0.5, self.y0+d, self.z0+0.5), (self.x0+0.9, self.y0+d, self.z0), (self.x0+0.9, self.y0, self.z0),
      #           #A
      #           (self.x0+1.0, self.y0, self.z0), (self.x0+1.0, self.y0+d, self.z0), (self.x0+1.4, self.y0+d, self.z0+0.9), (self.x0+1.8, self.y0+d, self.z0), (self.x0+1.8, self.y0, self.z0), (self.x0+1.2, self.y0, self.z0+0.45), (self.x0+1.2, self.y0+d, self.z0+0.45), (self.x0+1.6, self.y0+d, self.z0+0.45), (self.x0+1.6, self.y0, self.z0+0.45),
      #           #G
      #           (self.x0+2.3, self.y0, self.z0+0.2)
      #           ]
      point_index = 0
      #rospy.loginfo("initial position is [%f, &f]", self.x0, self.y0)
      while not rospy.is_shutdown():
        ####if self.center_pos_x is None or self.center_pos_y is None:
        if self.center_pos_x is None or self.center_pos_y is None or self.center_pos_z is None:
          rospy.loginfo_throttle(1.0, "not yet receive the controller message")
          time.sleep(self.nav_rate)
          continue
        ####target_x, target_y = points[point_index]
        target_x, target_y, target_z = points[point_index]
        dx = target_x - self.center_pos_x
        dy = target_y - self.center_pos_y
        dz = target_z - self.center_pos_z
        norm = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        rospy.loginfo("norm is %f", norm)
        # speed = 0.1
        speed = norm/8 + 0.05
        self.flight_nav.target_vel_x = speed * dx / norm
        self.flight_nav.target_vel_y = speed * dy / norm
        ####self.flight_nav.target_vel_y = 0
        self.flight_nav.target_vel_z = speed * dz / norm
        self.nav_pub.publish(self.flight_nav)
        if norm < 0.01:
          point_index = (point_index + 1) % len(points)
          time.sleep(2)
        time.sleep(self.nav_rate)
if __name__ == "__main__":
  rospy.init_node("circle_trajectory_follow")
  Tracker = CircTrajFollow()
  Tracker.main()