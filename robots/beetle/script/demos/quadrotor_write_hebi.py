#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import random

class CircTrajFollow():
  def __init__(self):
    self.period = rospy.get_param("~period", 40.0)
    self.yaw = rospy.get_param("~yaw", True)
    self.loop = rospy.get_param("~loop", False)

    self.nav_pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=1)
    self.control_sub = rospy.Subscriber("/beetle1/mocap/pose", PoseStamped, self.controlCb)

    self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
    self.markers = MarkerArray()


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

      l = 1.2 #square size
      d = 0.4 #pen movement
      s = 1 #scale
      # rospy.loginfo(f"self.x0")

      # points = [(self.x0, self.y0, self.z0, 0),
      #           (self.x0+l, self.y0, self.z0, 0), (self.x0+l, self.y0+d, self.z0, 1), (self.x0+l, self.y0, self.z0, 0),
      #           (self.x0+l, self.y0, self.z0+l, 0), (self.x0+l, self.y0+d, self.z0+l, 1), (self.x0+l, self.y0, self.z0+l, 0),
      #           (self.x0, self.y0, self.z0+l, 0), (self.x0, self.y0+d, self.z0+l, 1), (self.x0, self.y0, self.z0+l, 0),
      #           (self.x0, self.y0, self.z0, 0), (self.x0, self.y0+d, self.z0, 1)]

      points = [(self.x0, self.y0, self.z0, 0), (self.x0, self.y0-d, self.z0, 0),
                #1画目 虫
                (self.x0+s*0.05, self.y0-d, self.z0+s*0.7, 0), (self.x0+s*0.05, self.y0, self.z0+s*0.7, 1), (self.x0+s*0.05, self.y0, self.z0+s*0.3, 1), (self.x0+s*0.05, self.y0-d, self.z0+s*0.3, 0),
                #2画目 虫
                (self.x0+s*0.05, self.y0-d, self.z0+s*0.7, 0), (self.x0+s*0.05, self.y0, self.z0+s*0.7, 1), (self.x0+s*0.35, self.y0, self.z0+s*0.7, 1), (self.x0+s*0.35, self.y0, self.z0+s*0.3, 1), (self.x0+s*0.35, self.y0-d, self.z0+s*0.3, 0),
                #3画目 虫
                (self.x0+s*0.05, self.y0-d, self.z0+s*0.35, 0), (self.x0+s*0.05, self.y0, self.z0+s*0.35, 1), (self.x0+s*0.35, self.y0, self.z0+s*0.35, 1), (self.x0+s*0.35, self.y0-d, self.z0+s*0.35, 0),
                #4画目 虫
                (self.x0+s*0.2, self.y0-d, self.z0+s*0.9, 0), (self.x0+s*0.2, self.y0, self.z0+s*0.9, 1), (self.x0+s*0.2, self.y0, self.z0+s*0.05, 1), (self.x0+s*0.2, self.y0-d, self.z0+s*0.05, 0),
                #5画目 虫
                (self.x0, self.y0-d, self.z0, 0), (self.x0, self.y0, self.z0, 1), (self.x0+s*0.4, self.y0, self.z0+s*0.1, 1), (self.x0+s*0.4, self.y0-d, self.z0+s*0.1, 0),
                #6画目 虫
                (self.x0+s*0.35, self.y0-d, self.z0+s*0.2, 0), (self.x0+s*0.35, self.y0, self.z0+s*0.2, 1), (self.x0+s*0.45, self.y0, self.z0, 1), (self.x0+s*0.45, self.y0-d, self.z0, 0),
                #7画目 ウ冠
                (self.x0+s*0.7, self.y0-d, self.z0+s*0.9, 0), (self.x0+s*0.7, self.y0, self.z0+s*0.9, 1), (self.x0+s*0.7, self.y0, self.z0+s*0.75, 1), (self.x0+s*0.7, self.y0-d, self.z0+s*0.75, 0),
                #8画目 ウ冠
                (self.x0+s*0.45, self.y0-d, self.z0+s*0.75, 0), (self.x0+s*0.45, self.y0, self.z0+s*0.75, 1), (self.x0+s*0.45, self.y0, self.z0+s*0.55, 1), (self.x0+s*0.45, self.y0-d, self.z0+s*0.55, 0),
                #9画目 ウ冠
                (self.x0+s*0.45, self.y0-d, self.z0+s*0.75, 0), (self.x0+s*0.45, self.y0, self.z0+s*0.75, 1), (self.x0+s*0.95, self.y0, self.z0+s*0.75, 1), (self.x0+s*0.95, self.y0, self.z0+s*0.55, 1), (self.x0+s*0.95, self.y0-d, self.z0+s*0.55, 0),
                #10画目 匕
                (self.x0+s*0.85, self.y0-d, self.z0+s*0.5, 0), (self.x0+s*0.85, self.y0, self.z0+s*0.5, 1), (self.x0+s*0.55, self.y0, self.z0+s*0.3, 1), (self.x0+s*0.55, self.y0-d, self.z0+s*0.3, 0),
                #11画目 匕
                (self.x0+s*0.55, self.y0-d, self.z0+s*0.6, 0), (self.x0+s*0.55, self.y0, self.z0+s*0.6, 1), (self.x0+s*0.55, self.y0, self.z0+s*0.05, 1), (self.x0+s*0.6, self.y0, self.z0, 1), (self.x0+s*0.85, self.y0, self.z0, 1),(self.x0+s*0.95, self.y0, self.z0+s*0.15, 1), (self.x0+s*0.95, self.y0-d, self.z0+s*0.15, 0)
                ]


      point_index = 0

      #rospy.loginfo("initial position is [%f, &f]", self.x0, self.y0)

      while not rospy.is_shutdown():
        ####if self.center_pos_x is None or self.center_pos_y is None:
        if self.center_pos_x is None or self.center_pos_y is None or self.center_pos_z is None:
          rospy.loginfo_throttle(1.0, "not yet receive the controller message")
          time.sleep(self.nav_rate)
          continue

        ####target_x, target_y = points[point_index]
        target_x, target_y, target_z, viz_flag = points[point_index]

        dx = target_x - self.center_pos_x
        dy = target_y - self.center_pos_y
        dz = target_z - self.center_pos_z
        norm = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        speed = 0.25
        self.flight_nav.target_vel_x = speed * dx / norm
        self.flight_nav.target_vel_y = speed * dy / norm
        ####self.flight_nav.target_vel_y = 0
        self.flight_nav.target_vel_z = speed * dz / norm

        # rospy.loginfo /rospy.logwarn / rospy.logerr
        
        self.nav_pub.publish(self.flight_nav)

        # check pose convergence
        if norm < 0.05: # default 0.2; TODO: rosparam
          prev_index = point_index
          point_index = (point_index + 1) % len(points)

          if viz_flag:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.seq = len(self.markers.markers)
            marker.header.stamp = rospy.Time.now()
            marker.id = len(self.markers.markers)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = self.center_pos_x
            marker.pose.position.y = self.center_pos_y
            marker.pose.position.z = self.center_pos_z

            marker.pose.orientation.w = 1.0

            # sphere radius
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            # Add the marker to the MarkerArray
            self.markers.markers.append(marker)

            # Publish the MarkerArray
            self.marker_pub.publish(self.markers)
          
        time.sleep(self.nav_rate)


if __name__ == "__main__":

  rospy.init_node("circle_trajectory_follow")

  Tracker = CircTrajFollow()
  Tracker.main()



