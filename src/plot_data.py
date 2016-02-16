#!/usr/bin/python

import numpy
import sys
import matplotlib.pyplot as plt
import matplotlib

from load_csv_data import *
from tf.transformations import *

def plot_trajectories(
  filenames=None, datas=None, names=None, 
  is_2d=True, time_color=True, legend=True):

  if filenames is not None:
    datas = [csv_to_numpy(filename) for filename in filenames]
    names = [filename.split('.')[-2] for filename in filenames]
  
  if datas is None:
    datas = []

  if names is None:
    names = ['traj_%d' % i for i in range(len(datas))]

  time_min = min([d['time'][0] for d in datas])
  time_max = max([d['time'][-1] for d in datas])
  normalizer = matplotlib.colors.Normalize(vmin=time_min, vmax=time_max, clip=False)

  for data, name in zip(datas,names):
    plt.plot(data['x'], data['y'], label=name)
    plt.scatter(
      data['x'], data['y'], c=data['time'],
      cmap=plt.get_cmap('jet'), norm=normalizer, marker='o',edgecolor='')

  if legend:
    plt.legend()

  plt.axis('equal')
  plt.grid(True)
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')
  plt.colorbar(label='time (s)')

  plt.show()

def plot_time(
  filenames=None, datas=None, names=None, 
  is_2d=True, time_color=True, legend=True):

  if filenames is not None:
    datas = [csv_to_numpy(filename) for filename in filenames]
    names = [filename.split('.')[-2] for filename in filenames]
  
  if datas is None:
    datas = []

  if names is None:
    names = ['traj_%d' % i for i in range(len(datas))]

  for data, name in zip(datas, names):
    theta = [euler_from_quaternion(list(q))[2] for q in data[['qx','qy','qz','qw']]] 
    plt.plot(data['time'], data['x'], label=name+'_x')
    plt.plot(data['time'], data['y'], label=name+'_y')
    plt.plot(data['time'], theta, label=name+'_theta')
  
  if legend:
    plt.legend()

  plt.grid(True)
  plt.xlabel('time (s)')

  plt.show()

if __name__ == '__main__':
  plot_trajectories(sys.argv[1:])

