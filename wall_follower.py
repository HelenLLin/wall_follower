#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

desired_distance = .5 # desired distance from wall in meters

# sets the PID control values
class PID_control:
   # variables to start with
   def __init__(self,Kp,Ki,Kd):
      self.Kp = Kp
      self.Ki = Ki
      self.Kd = Kd
      self.curr_error = 0 # current error
      self.prev_error = 0  # previous error
      self.total_error = 0 # total error to calculate Ki
      self.diff_error = 0 # difference between previous and current error
      self.pid = 0 # Kp*curr_error + Ki*error_total + Kd*error_diff
   
   # updates the PID values
   def update_values(self,new_error):
      self.prev_error = self.curr_error
      self.curr_error = new_error
        
      self.total_error = self.total_error + self.curr_error # Ki
      self.diff_error = self.curr_error - self.prev_error # Kd

      # Kp*curr_error + Ki*error_total + Kd*error_diff
      self.pid = self.Kp * self.curr_error + self.Ki * self.total_error + self.Kp * self.diff_error

   def get_pid(self):
      return self.pid

# scan callback
def scan_cb(msg):

   error = 0 # the error between desired_distance and distance
   distance = 100000 # set to high value so it can be reset

   # finds the minimum distance from a direction
   for i in range(-90,90):
      if msg.ranges[i]<distance:
         distance = msg.ranges[i]

   # values for PID control
   error = desired_distance - distance
   pid.update_values(error)

# creates the node
rospy.init_node('wall_follower')

# creates new PID control object
pid  = PID_control(0.01,0.01,0.01)
#subscribers and publishers
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

# publishes to cmd_vel while not shutdown
while not rospy.is_shutdown():
   t = Twist()
   t.linear.x = 0.2
   t.angular.z = pid.get_pid()
   cmd_vel.publish(t)
   rate.sleep()
