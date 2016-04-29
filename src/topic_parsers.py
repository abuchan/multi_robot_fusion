import numpy

from std_msgs.msg import Int16, Float32
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

# time (s) velocity (rad/s)
vel_columns = ['time','velocity']
def vel_to_csv(vel_entry):
  _, msg, timestamp, start_time = vel_entry
  return [[timestamp.to_sec()-start_time, msg.data]]

# time (s) voltage (volt)
volt_columns = ['time','voltage']
def volt_to_csv(vel_entry):
  _, msg, timestamp, start_time = vel_entry
  return [[timestamp.to_sec()-start_time, msg.data]]

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

