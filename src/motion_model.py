import numpy
from multi_robot_fusion.src.construct_poses import *

import scipy.signal
import matplotlib.pyplot as plt

def smooth_signal(x):
  b,a = scipy.signal.butter(5,0.2)
  return scipy.signal.filtfilt(b,a,x)

def merge_time(time_a, time_b):
  if time_a[0] > time_b[0]:
    a_start = 0
    b_start = numpy.search_sorted(time_a,time_b[0])
  else:
    a_start = numpy.search_sorted(time_b,time_a[0])
    b_start = 0
  time_len = min(len(time_a)-a_start,len(time_b)-b_start)
  time_out = (time_a[a_start:(a_start+time_len)] + time_b[b_start:(b_start+time_len)])/2
  return time_out, a_start, b_start, time_len

def wheel_to_body_velocity(left_wheel, right_wheel, r=0.02, w=0.1):
  t = (left_wheel['time'] + right_wheel['time'])/2
  lv, rv = left_wheel['velocity']*r,right_wheel['velocity']*r
  vx = (lv+rv)/2
  wz = (rv-lv)/(2*w)
  return numpy.array(zip(t, vx, wz),dtype=[('time','<f8'),('vx','<f8'),('wz','<f8')])

def pose_to_body_velocity(pose):
  H = [tpq_to_matrix(p) for p in pose]
  R = [h[:3,:3] for h in H]
  P = numpy.array([h[:3,3] for h in H])
  
  t = pose['time']
  dT = numpy.round(numpy.diff(t),2)
  dT[dT==0] = 0.01

  v_world = (numpy.diff(P,axis=0).T/dT).T
  dR = [numpy.linalg.inv(R[i]).dot(R[i+1]) for i in range(len(R)-1)]
  w = numpy.array([((dr-dr.T)/(2*dt))[(2,0,1),(1,2,0)] for dr,dt in zip(dR,dT)])
  v = numpy.array([r.T.dot(v_w) for r,v_w in zip(R,v_world)])
  
  t.shape = (t.shape[0],1)
  
  retval = numpy.hstack([t[:-1], w, v]).reshape(-1)
  retval.dtype = [
    ('time','<f8'),('wx','<f8'),('wy','<f8'),('wz','<f8'),
    ('vx','<f8'),('vy','<f8'),('vz','<f8')]
 
  return retval

def plot_angular_velocity(pose, wheels, imu, v = 0.1):
  pose_vel = pose_to_body_velocity(pose)[['time','wz']]
  wheel_vel = wheel_to_body_velocity(wheels[0],wheels[1])[['time','wz']]
  imu_vel = imu[['time','gyro_z']]

  plt.plot(pose_vel['time'],pose_vel['wz'],label='opti_wz')
  plt.plot(wheel_vel['time'],wheel_vel['wz'],label='wheel_wz')
  plt.plot(imu_vel['time'],imu_vel['gyro_z'],label='imu_wz')
  plt.grid(True)
  plt.legend()
  plt.title('Angular velocity comparison, v=%0.2f' % v)
  plt.xlabel('time (s)')
  plt.ylabel('Z angular velocity (rad/s)')
  plt.show()
