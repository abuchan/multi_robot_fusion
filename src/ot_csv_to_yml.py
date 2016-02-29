#!/usr/bin/python

from glob import glob
from tf.transformations import *
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_msgs.msg import TFMessage
import yaml
import copy

from multi_robot_fusion.src.construct_poses import *
from multi_robot_fusion.src.fuse_poses import *

def ori_from_zx(z_axis, x_axis):
  z_axis = numpy.array(z_axis)
  z_axis = z_axis/(sum(z_axis**2.0)**0.5)
  x_axis = numpy.array(x_axis)
  x_axis = x_axis/(sum(x_axis**2.0)**0.5)
  y_axis = numpy.cross(z_axis,x_axis)
  ori_mat = numpy.eye(4)
  ori_mat[0:3,0:3] = numpy.vstack([x_axis,y_axis,z_axis]).T
  return quaternion_from_matrix(ori_mat)

def plot_markers(markers):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  for i in range(markers.shape[1]):
    ax.plot([markers[0,i]],[markers[1,i]],[markers[2,i]],'o',label=str(i))
  ax.axis('equal')
  ax.grid(True)
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_zlabel('z')
  ax.legend()
  plt.show()

def picket_ori_from_markers(markers, tl_idx, tr_idx, bl_idx, z_angle = 0.0, up_axis = (0,1,0)):
  tl = markers[:,tl_idx]
  tr = markers[:,tr_idx]
  print (tr-tl)[0]
  bl = markers[:,bl_idx]
  down = bl-tl
  across = tr-tl
  normal = numpy.cross(down,across)
  normal = normal/(sum(normal**2)**0.5)
  angle = numpy.arccos(numpy.array(up_axis).dot(normal))
  z_quat = quaternion_about_axis(z_angle, (0,0,1))
  x_quat = quaternion_about_axis(angle, (1,0,0))
  return quaternion_multiply(x_quat, z_quat)

def pose_to_tf_msg(parent, child, position, orientation):
  ts = TransformStamped()
  ts.header.frame_id = parent
  ts.child_frame_id = child
  ts.transform.rotation = Quaternion(*orientation)
  ts.transform.translation = Vector3(*position)
  return ts

def parse_optitrack(filename):
  f = open(filename)
  lines = [[value.strip('\" ') for value in line.split(',')] for line in f.readlines()]
  f.close()

def pose_inverse(pos, ori):
  inv_ori = quaternion_inverse(ori)
  inv_pos = -quaternion_matrix(inv_ori)[:3,:3].dot(numpy.array(pos))
  return inv_pos, inv_ori

def pose_to_avg_tf(pose):
  avg_tf = pose_average(pose['pose'])
  return [pose['parent'],pose['child'],avg_tf[:3,3],quaternion_from_matrix(avg_tf)]

def flip_pose(pose):
  return {
    'parent': pose['child'],
    'child': pose['parent'],
    'pose': [numpy.linalg.inv(H) for H in pose['pose']],
    'time': pose['time']
  }

def robot_calibration(cal_dir, optitrack_frame, marker_numbers=None, 
  static_tf=mrf_dir+'/config/calibration.yml', camera=None):
  
  poses = load_poses(cal_dir, static_tf=static_tf)
  
  exp_frame = optitrack_frame + '_exp'

  poses.append({
    'parent': 'static_frame',
    'child': exp_frame,
    'time': [numpy.nan],
    'pose': [numpy.eye(4)]
  })

  if marker_numbers is None:
    marker_numbers = []

  marker_paths = [(exp_frame, 'ar_marker_%d' % mn) for mn in marker_numbers]
  
  print 'Marker paths:' + str(marker_paths)

  if camera is None:
    camera_paths = []
  else:
    camera_index = [p['parent'] for p in poses].index(camera)
    poses[camera_index] = flip_pose(poses[camera_index])
    camera_paths = [(exp_frame, camera)]

  print 'Camera paths:' + str(camera_paths)

  static_tfs = []

  static_poses = construct_poses(poses, marker_paths + camera_paths)
  
  # Correct Optitrack frame rotation to get expected frame of robot
  static_tfs.append((
    optitrack_frame, exp_frame,
    (0.0, 0.0, 0.0),quaternion_about_axis(numpy.pi/2,(1,0,0))
  ))

  for sp in static_poses:
    avg_pose = pose_to_avg_tf(sp)
    
    if sp['child'] == camera:
      avg_pose[1] = camera
      static_tfs.append(avg_pose)  
    else:
      avg_pose[1] = avg_pose[1] + '_exp'

      static_tfs.append(avg_pose)

      marker = sp['child']
      obs_pose = (
        (marker, optitrack_frame + '_obs_' + marker.split('_')[-1]) + 
        pose_inverse(*avg_pose[2:])
      )

      static_tfs.append(obs_pose)

  return static_tfs
  
if __name__ == '__main__':
  static_transforms = []
  
  static_transforms.extend(robot_calibration(
    mrf_dir + '/data/calibration/picket_1', 'picket_1', [1,2,3,6,7]
  ))

  static_transforms.extend(robot_calibration(
    mrf_dir + '/data/calibration/picket_2', 'picket_2', [4,5,9,8,11]
  ))

  camera_calibration = robot_calibration(
    mrf_dir + '/data/calibration/observer', 'observer', camera='lifecam_odroid6'
  )
 
  usb_cam_calibration = copy.deepcopy(camera_calibration[-1])
  usb_cam_calibration[1] = 'usb_cam'
  camera_calibration.append(usb_cam_calibration)
  static_transforms.extend(camera_calibration)
  print camera_calibration

  poses = TFMessage()
  poses.transforms.extend([pose_to_tf_msg(*pose) for pose in static_transforms])

  f = open(mrf_dir + '/config/static_tf.yml','w')
  f.write(str(poses))
  f.close()
