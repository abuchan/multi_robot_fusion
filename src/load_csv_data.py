#!/usr/bin/python

import glob
import numpy

def csv_to_numpy(filename):
  return numpy.genfromtxt(filename, dtype=float, delimiter=',', names=True)

def load_csv_data(path='.',filename=None):
  if filename is not None:
    files = [filename]
  else:
    files = glob.glob(path + '/*.csv')
  
  data = {}
  for filename in files:
    series = filename.split('.')[-2].split('/')[-1]
    data[series] = csv_to_numpy(filename)
 
  if filename is not None:
    return data.values()[0]
  else:
    return data
