#!/usr/bin/python

from glob import glob
from tf.transformations import *
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_msgs.msg import TFMessage
import yaml

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

def pose_to_tf_msg(parent, child, pos, ori):
  ts = TransformStamped()
  ts.header.frame_id = parent
  ts.child_frame_id = child
  ts.transform.rotation = Quaternion(*ori)
  ts.transform.translation = Vector3(*pos)
  return ts

observer_exp_usb_cam_ori = quaternion_about_axis(-1.55, (1,0,0))
observer_exp_usb_cam_pos = (0.0,-0.02,0.12)

poses = TFMessage()
poses.transforms.append(pose_to_tf_msg(
  'observer_exp', 'usb_cam',
  observer_exp_usb_cam_pos, observer_exp_usb_cam_ori
))

# From TrackingData 2016-01-22 2.48pm.csv
picket_1_markers = numpy.array([-0.06380191, 0.02113091, 0.0234817, 0.03571201, 0.01110142, -0.08322999, -0.06455258, -0.05244154, 0.03752184, 0.09264249, 0.02020924, 0.02222648]).reshape(4,3).T
picket_2_markers = numpy.array([-0.06421539, -0.05119627, 0.01091343, -0.05635554, 0.02373023, -0.00827247, 0.08175805, 0.02328915, 0.06975526, 0.03881291, 0.00417688, -0.07239634]).reshape(4,3).T

def parse_optitrack(filename):
  f = open(filename)
  lines = [[value.strip('\" ') for value in line.split(',')] for line in f.readlines()]
  f.close()

def pose_inverse(pos, ori):
  inv_ori = quaternion_inverse(ori)
  inv_pos = -quaternion_matrix(inv_ori)[:3,:3].dot(numpy.array(pos))
  return inv_pos, inv_ori

left_ar_pos = numpy.array((-0.04, -0.06, 0.05))
right_ar_pos = numpy.array((0.04, -0.06, 0.05))

picket_1_ar_ori = picket_ori_from_markers(picket_1_markers, 0, 3, 2, numpy.pi)

poses.transforms.append(pose_to_tf_msg(
  'picket_1_exp', 'ar_marker_17_exp',
  left_ar_pos, picket_1_ar_ori
))

poses.transforms.append(pose_to_tf_msg(
  'ar_marker_17', 'picket_1_obs_17',
  *pose_inverse(left_ar_pos, picket_1_ar_ori)
))

poses.transforms.append(pose_to_tf_msg(
  'picket_1_exp','ar_marker_16_exp',
  right_ar_pos, picket_1_ar_ori
))

poses.transforms.append(pose_to_tf_msg(
  'ar_marker_16','picket_1_obs_16',
  *pose_inverse(right_ar_pos, picket_1_ar_ori)
))

picket_2_ar_ori = picket_ori_from_markers(picket_2_markers, 1, 2, 0)

poses.transforms.append(pose_to_tf_msg(
  'picket_2_exp', 'ar_marker_10_exp',
  left_ar_pos, picket_2_ar_ori
))

poses.transforms.append(pose_to_tf_msg(
  'ar_marker_10','picket_2_obs_10',
  *pose_inverse(left_ar_pos, picket_2_ar_ori)
))

poses.transforms.append(pose_to_tf_msg(
  'picket_2_exp', 'ar_marker_11_exp',
  right_ar_pos, picket_2_ar_ori
))

poses.transforms.append(pose_to_tf_msg(
  'ar_marker_11','picket_2_obs_11',
  *pose_inverse(right_ar_pos, picket_2_ar_ori)
))

poses.transforms.append(pose_to_tf_msg(
  'world', 'odom',
  (0,0,0),(0,0,0,1)
))

poses.transforms.append(pose_to_tf_msg(
  'picket_1', 'picket_1_exp',
  (0.02,0.0,-0.090358632808888978),quaternion_about_axis(-0.025,(0,0,1))
  #(0.01564444,0,-0.090358632808888978),quaternion_about_axis(-0.025,(0,0,1))
))

poses.transforms.append(pose_to_tf_msg(
  'picket_2', 'picket_2_exp',
  (0.03,-0.03,-0.092169086757290727),quaternion_about_axis(-0.5,(0,0,1))
  #(0.02,-0.02,-0.092169086757290727),quaternion_about_axis(-0.45,(0,0,1))
))

poses.transforms.append(pose_to_tf_msg(
  'observer', 'observer_exp',
  (0.02,0,-0.12055603725444798),quaternion_about_axis(-0.15,(0,0,1))
))

f = open('static_tf.yml','w')
f.write(str(poses))
f.close()
