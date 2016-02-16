#!/usr/bin/python

import rospkg
import glob
import sys
import yaml
from tf.transformations import *
import collections

rospack = rospkg.RosPack()
mrf_dir = rospack.get_path('multi_robot_fusion')

sys.path.append(mrf_dir + '/src')

from load_csv_data import *

default_data = mrf_dir + '/data'
default_static = mrf_dir + '/config/static_tf.yml'

def tpq_to_matrix(tpq):
  H = quaternion_matrix(numpy.array(tpq)[['qx','qy','qz','qw']].tolist())
  H[:3,3] = numpy.array(tpq)[['x','y','z']].tolist()
  return H

def load_poses(data_dir=default_data, static_tf=default_static):
  poses = []
  
  tf_files = glob.glob(data_dir + '/tf-*.csv')
  for tf_file in tf_files:
    data = csv_to_numpy(tf_file)
    # not sure why, but need to catch null shape arrays when there is only one entry
    if data.shape is ():
      data.shape = (1,)
    _, parent, child = tf_file.strip('.csv').split('-')
    pose_data = [tpq_to_matrix(line) for line in data] 
    poses.append({
      'parent': parent,
      'child': child,
      'time': data['time'],
      'pose': pose_data
    })
  
  static_file = open(static_tf)
  static_transforms = yaml.load(static_file)['transforms']
  static_file.close()

  for transform in static_transforms:
    pos = [transform['transform']['translation'][n] for n in ['x','y','z']]
    ori = [transform['transform']['rotation'][n] for n in ['x','y','z','w']]
    H = quaternion_matrix(ori)
    H[:3,3] = pos
    poses.append({
      'parent': transform['header']['frame_id'],
      'child': transform['child_frame_id'],
      'time': [numpy.nan],
      'pose': [H]
    })

  return poses

def get_pose_at_index(pose, pose_idx):
  if pose['time'][0] == -1:
    return pose['pose'][0]
  else:
    return pose['pose'][pose_idx]
  
def get_pose_at_time(pose, pose_time):
  return get_pose_at_index(pose, numpy.searchsorted(pose['time'],pose_time))

def build_pose_trees(poses):
  edges = [(p['parent'],p['child']) for p in poses]
  trees = collections.defaultdict(dict)
  for parent, child in edges:
    trees[parent][child] = trees[child]
  parents, children = zip(*edges)
  roots = set(parents).difference(children)
  return {root: trees[root] for root in roots}
  
def get_root_path(edges, child):
  if child in edges.keys():
    return get_root_path(edges, edges[child]) + [child]
  else:
    return [child]

def get_pose_paths(poses, paths):
  edges = collections.OrderedDict([(p['child'],p['parent']) for p in poses])
  pose_paths = []
  for parent, child in paths:
    parent_path = get_root_path(edges, parent)
    child_path = get_root_path(edges, child)

    if parent_path[0] != child_path[0]:
      pose_paths.append([])
    else:
      shared_idx = 0
      while (shared_idx < min(len(parent_path),len(child_path)) and 
        parent_path[shared_idx] == child_path[shared_idx]):
        
        shared_idx += 1
      
      parent_idxs = [(edges.keys().index(name), True) for name in parent_path[shared_idx:]]
      child_idxs = [(edges.keys().index(name), False) for name in child_path[shared_idx:]]
      pose_paths.append(parent_idxs[::-1] + child_idxs)
  return pose_paths
  
def construct_poses(poses, paths):
  pose_paths = get_pose_paths(poses,paths)
  results = []

  for path in pose_paths:
    seq_idxs, flips = zip(*path)
    seqs = [poses[si]['pose'] for si in seq_idxs]
    for i, flip in zip(range(len(seqs)), flips):
      if flip:
        seqs[i] = [numpy.linalg.inv(pose) for pose in seqs[i]]

    times = [poses[si]['time'] for si in seq_idxs]
    lens = numpy.array([len(seq) for seq in seqs])

    try:
      min_next_time = numpy.nanmax([t[0] for t in times])
    except RuntimeWarning:
      pass
    
    pose_idxs = numpy.array([numpy.searchsorted(t,min_next_time)-1 for t in times])
    pose_idxs[pose_idxs == -1] = 0
    res_pose = [reduce(numpy.dot,[seq[pi] for seq,pi in zip(seqs,pose_idxs)])]
    res_time = [min_next_time]

    while any(pose_idxs + 1 < lens):
      next_idxs = [min(pi+1,le-1) for pi,le in zip(pose_idxs,lens)]
      next_times = numpy.array([t[i] for t,i in zip(times, next_idxs)])

      min_next_time = numpy.nanmin(next_times[pose_idxs != lens-1])
      pose_idxs[next_times == min_next_time] += 1 
      
      res_pose.append(reduce(numpy.dot, [seq[pi] for seq,pi in zip(seqs,pose_idxs)]))
      res_time.append(min_next_time)

    if flips[0]:
      parent = poses[seq_idxs[0]]['child']
    else:
      parent = poses[seq_idxs[0]]['parent']
    
    if flips[-1]:
      child = poses[seq_idxs[-1]]['parent']
    else:
      child = poses[seq_idxs[-1]]['child']

    results.append({
      'parent':parent,
      'child':child,
      'time':numpy.array(res_time),
      'pose':res_pose
    })
    print 'Calculated %d poses for %s to %s' % (len(res_pose),parent,child)
  
  return results

def poses_to_csv(parent,child,time,pose,**kwargs):
  filename = 'tf-%s-%s.csv' % (parent,child)
  f = open(filename, 'w')
  f.write('time, x, y, z, qx, qy, qz, qw\n')
  for t,p in zip(time, pose):
    pos = p[:3,3]
    ori = quaternion_from_matrix(p)
    line = numpy.hstack([t,pos,ori])
    f.write(', '.join([str(v) for v in line])+'\n')
  f.close()

if __name__ == '__main__':
  poses = load_poses()
  
  print 'Loaded %d poses' % len(poses)

  paths = [
    ('observer_exp','picket_1_exp'),
    ('observer_exp','picket_1_obs_16'), ('observer_exp','picket_1_obs_17'),
    ('observer_exp','picket_2_exp'),
    ('observer_exp','picket_2_obs_10'), ('observer_exp','picket_2_obs_11')
  ]
  
  results = construct_poses(poses, paths)

  for result in results:
    poses_to_csv(**result)
