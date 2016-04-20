#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose2D
import tf
import numpy as np
from signal import signal, SIGINT
from figure_eight import *

t = Twist()
states = []
def shut_down(sig, var):
    t.linear.x = 0
    t.angular.z = 0
    for __ in range(4):
        zumy_pub.publish(t)
    exit(0)
#signal(SIGINT, shut_down)
   
def gather_data(data):
    # TODO: log all the data to a file by subscribing
    # to appropriate nodes
    pass

def drive(data, pub):
    state = np.array([data.x, data.y, data.theta])
    #states.append(state)
    cmd = lemniscate_control(state, 0, [-3.0,3.0], v=0.2)
    t.linear.x = cmd[0]
    t.angular.z = cmd[1]
    pub.publish(t)
    #if len(states) > 100:
    #plot_trajectory(np.array(states))
    print(state, cmd)

if __name__ == "__main__":
    rospy.init_node('figure_eight', anonymous=True)
    zumy_pub = rospy.Publisher('/observer/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/observer/ground_pose', Pose2D, drive, zumy_pub,queue_size=1)
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(20)
    rospy.spin()
    t.linear.x = 0
    t.angular.z = 0
    zumy_pub.publish(t)

