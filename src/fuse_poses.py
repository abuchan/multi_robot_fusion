#!/usr/bin/python

import rospkg
import glob
import sys
import yaml
from tf.transformations import *
import collections

from multi_robot_fusion.src.load_csv_data import *
from multi_robot_fusion.src.construct_poses import *

def pose_average(poses):
  positions = [H[:3,3] for H in poses]
  orientations = [quaternion_from_matrix(H) for H in poses]
  mean_ori = numpy.array(orientations[0])
  ori_count = 1.0
  for ori in orientations[1:]:
    ori = numpy.array(ori)
    if mean_ori.dot(ori) < 0.0:
      ori = -ori
    mean_ori = quaternion_slerp(mean_ori, ori, ori_count/(ori_count+1.0))
    ori_count += 1.0
  H = quaternion_matrix(mean_ori)
  H[:3,3] = numpy.array(positions).mean(0)
  return H

def fuse_poses(poses, parent, child, time_thresh=0.01):
  
  times = [p['time'] for p in poses]
  in_poses = numpy.array([p['pose'] for p in poses])

  fused_pose = []
  fused_time = []
  idxs = numpy.array([0] * len(times))
  lens = numpy.array([len(t) for t in times])

  while any(idxs < lens):
    curr_times = numpy.array([t[i] if i<l else numpy.nan for t,i,l in zip(times, idxs, lens)])
    min_time = numpy.nanmin(curr_times)

    fuse_idxs = (curr_times - min_time) < time_thresh

    f_ps = in_poses[fuse_idxs]
    p_is = idxs[fuse_idxs]
    fused_pose.append(pose_average([f_p[p_i] for f_p,p_i in zip(f_ps,p_is)]))
    fused_time.append(min_time)
    idxs[fuse_idxs] += 1

  return {
    'parent': parent,
    'child': child,
    'time': numpy.array(fused_time),
    'pose': fused_pose
  }
  
if __name__ == '__main__':
  p1_obs = load_poses(filter_str = 'tf-observer_exp-picket_1_obs_*',static_tf=None)
  p2_obs = load_poses(filter_str = 'tf-observer_exp-picket_2_obs_*',static_tf=None)

  p1_avg = fuse_poses(p1_obs, 'observer_exp', 'picket_1_obs_avg')
  p2_avg = fuse_poses(p2_obs, 'observer_exp', 'picket_2_obs_avg')

  poses_to_csv(**p1_avg)
  poses_to_csv(**p2_avg)
