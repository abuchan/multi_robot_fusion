import numpy

def wheel_to_body_velocity(left_wheel, right_wheel, r=0.02, w=0.1):
  t = (left_wheel['time'] + right_wheel['time'])/2
  lv, rv = left_wheel['velocity']*r,right_wheel['velocity']*r
  v = (lv+rv)/2
  a = (rv-lv)/(2*w)
  return numpy.array(zip(t, v, a),dtype=[('time','<f8'),('v','<f8'),('a','<f8')])

def pose_to_body_velocity(pose):
  pass
