#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
from figure_eight import *

class ClosedLoopControl():
  def __init__(self):
    self.fc = FilteredController({'N':4,'Wn':0.25},lemniscate_control,{'a_gain':5.0})

  def state_callback(self,msg):
    if not rospy.is_shutdown():
      state = numpy.array([msg.x,msg.y,msg.theta])
      cmd = self.fc.control(state)[0]
      cmd_msg = Twist()
      cmd_msg.linear.x = cmd[0]
      cmd_msg.angular.z = cmd[1]
      self.cmd_pub.publish(cmd_msg)

  def run(self):
    rospy.init_node('controller')
    self.cmd_pub = rospy.Pubslisher('/observer/cmd_vel',Twist,queue_size=1)
    rospy.Subscriber('/observer/ground_pose',Pose2D,self.state_callback,queue_size=1)

    while not rospy.is_shutdown():
      rospy.spin()

    # Send 0,0 command if stopped
    self.cmd_pub.publish(Twist())

if __name__ == "__main__":
  clc = ClosedLoopControl()
  clc.run()

