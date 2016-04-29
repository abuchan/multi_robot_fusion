from topic_parsers import *

base_topics = [
  'tf'
]

robots = [
  'observer',
]

robot_topics = [
  'imu',
  'l_enc',
  'r_enc',
  'l_vel',
  'r_vel',
  'voltage',
  'cmd_vel'
]

topic_formats = {
  'tf': (tf_columns, tf_to_csv),
  'imu': (imu_columns, imu_to_csv),
  'l_enc': (enc_columns, enc_to_csv),
  'r_enc': (enc_columns, enc_to_csv),
  'l_vel': (vel_columns, vel_to_csv),
  'r_vel': (vel_columns, vel_to_csv),
  'voltage': (volt_columns, volt_to_csv),
  'cmd_vel': (cmd_columns, cmd_to_csv)
}

