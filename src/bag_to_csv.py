#!/usr/bin/python

import sys
import numpy

from rosbag import *
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TransformStamped, Transform, Vector3, Quaternion
from tf2_msgs.msg import TFMessage

# Import the desired topics from specified Python file

#from observer_picket_topics import *
from figure_eight_topics import *

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
    filename = 'tf-' + base + '-' + child + '.csv'
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

