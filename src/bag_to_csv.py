#!/usr/bin/python

import sys
import numpy

from rosbag import *
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TransformStamped, Transform, Vector3, Quaternion
from tf2_msgs.msg import TFMessage

# time (s), gyro (m/s), accel (m/s^2)
imu_columns = ['time','gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z']

def imu_to_csv(imu_entry):
  _, msg, timestamp, start_time = imu_entry
  gyro = msg.angular_velocity
  accel = msg.linear_acceleration
  return [[timestamp.to_sec()-start_time, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z]]

TICKS_PER_RADIAN = 600.0/(2*numpy.pi)
# time (s), encoder (rad)

enc_columns = ['time', 'angle']
def enc_to_csv(enc_entry):
  _, msg, timestamp, start_time = enc_entry
  return [[timestamp.to_sec()-start_time, float(msg.data)/TICKS_PER_RADIAN]]

# time (s), linear (m/s), angular (m/s)
cmd_columns = ['time', 'linear', 'angular']
def cmd_to_csv(cmd_entry):
  _, msg, timestamp, start_time = cmd_entry
  linear = msg.linear
  angular = msg.angular
  return [[timestamp.to_sec()-start_time, msg.linear.x, msg.angular.z]]

# time (s), pos (m), orientation (qx, qy, qz, qw)
tf_columns = ['time', 'base', 'child', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
def tf_to_csv(tf_entry):
  _, msg, _, start_time = tf_entry
  entries = []
  for tf_stamped in msg.transforms:
    timestamp = tf_stamped.header.stamp
    base_frame = tf_stamped.header.frame_id
    child_frame = tf_stamped.child_frame_id
    
    tf_tr = tf_stamped.transform
    pos = tf_tr.translation
    ori = tf_tr.rotation
    entries.append([
      timestamp.to_sec()-start_time, base_frame, child_frame,
      pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w
    ])

  return entries

base_topics = [
  'tf'
]

robots = [
  'observer',
  'picket_1',
  'picket_2'
]

robot_topics = [
  'imu',
  'l_enc',
  'r_enc',
  'cmd_vel'
]

topic_formats = {
  'tf': (tf_columns, tf_to_csv),
  'imu': (imu_columns, imu_to_csv),
  'l_enc': (enc_columns, enc_to_csv),
  'r_enc': (enc_columns, enc_to_csv),
  'cmd_vel': (cmd_columns, cmd_to_csv)
}

def init_topics():
  topics = {}
  topic_names = []

  for base_topic in base_topics:
    topic_names.append('/' + base_topic)

  for robot in robots:
    for robot_topic in robot_topics:
      topic_names.append('/' + robot + '/' + robot_topic)
  
  for topic_name in topic_names:
    topic_columns, topic_formatter = topic_formats[topic_name.split('/')[-1]]
    
    file_name = '_'.join(topic_name.split('/')[1:]) + '.csv'
    
    csv_file = open(file_name, 'w')
    csv_file.write(', '.join(topic_columns) + '\n')
    topics[topic_name] = {
      'file':csv_file,
      'formatter':topic_formatter
    }

  return topics

def split_tf_file(filename):
  tf_file = open(filename)
  lines = tf_file.readlines()
  tf_file.close()
  filenames = {}
  split_columns = tf_columns[0:1] + tf_columns[3:]
  
  for line in lines[1:]:
    data = [s.strip(' \n/') for s in line.split(',')]
    base, child = data[1:3]
    filename = 'tf_' + base + '_' + child + '.csv'
    if filename not in filenames.keys():
      filenames[filename] = open(filename, 'w')
      filenames[filename].write(', '.join(split_columns) + '\n')
    filenames[filename].write(', '.join(data[0:1] + data[3:]) + '\n')
  
  for f in filenames.values():
    f.close()

def bag_to_csv(filename):
  bagfile = Bag(filename)
  topics = init_topics()
  
  start_time = None
  for topic, msg, timestamp in bagfile.read_messages(topics=topics.keys()):
    if start_time is None:
      start_time = timestamp.to_sec()
    formatter = topics[topic]['formatter']
    data_lines = formatter((topic,msg,timestamp,start_time))
    for data_line in data_lines:
      topics[topic]['file'].write(', '.join(map(str,data_line)) + '\n')

  for topic in topics.keys():
    topics[topic]['file'].close()

  bagfile.close()

if __name__ == '__main__':
  bag_to_csv(sys.argv[1])
  split_tf_file('tf.csv')

