#!/usr/bin/python

import glob
import numpy

def csv_to_numpy(filename):
  return numpy.genfromtxt(filename, dtype=float, delimiter=',', names=True)

def load_csv_data(path='.'):
  data = {}
  files = glob.glob(path + '/*.csv')
  for filename in files:
    series = filename.split('.')[-2].strip('/')
    data[series] = csv_to_numpy(filename)
  return data
