#!/usr/bin/python

import rospy
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

def pose_to_args(pose, tf_time=None):
  pos = [pose['transform']['translation'][n] for n in ['x','y','z']]
  ori = [pose['transform']['rotation'][n] for n in ['x','y','z','w']]
  if tf_time is None:
    tf_time = rospy.Time.now()
  child = pose['child_frame_id']
  parent = pose['header']['frame_id']
  return pos, ori, tf_time, child, parent

class StaticTransformPublisher:
  def __init__(self):
    rospy.init_node('static_tf_broadcaster')
    self.last_time = rospy.Time.now()

  def tf_callback(msg):
    if len(msg.transforms):
      self.last_time = msg.transforms[0].header.stamp
  
  def run(self):
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)

    poses = rospy.get_param('transforms')
    while not rospy.is_shutdown():
      for pose in poses:  
        br.sendTransform(*pose_to_args(pose))
      rate.sleep()
    
if __name__ == '__main__':
  stp = StaticTransformPublisher()
  stp.run()
