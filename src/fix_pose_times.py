#!/usr/bin/python

import glob
from multi_robot_fusion.src.construct_poses import *
from multi_robot_fusion.src.fuse_poses import *

import numpy

TIME_THRESH = -(3600*24)

if __name__ == '__main__':
  poses = load_poses(static_tf=None)
  min_time = numpy.nanmin([p['time'][0] for p in poses])
  
  print 'Min time %f' % min_time

  if min_time < TIME_THRESH:
    for pose in poses:
      if pose['time'][0] < -(3600*24):
        pose['time'] -= min_time
        print 'Fixed %s to %s, new min time is %f' % (pose['parent'],pose['child'],pose['time'][0])
        poses_to_csv(**pose)  
