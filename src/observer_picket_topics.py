from topic_parsers import *

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

