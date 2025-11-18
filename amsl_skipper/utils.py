import math

def euler_from_quat(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
 
  return roll_x, pitch_y, yaw_z # in radians


def quat_from_euler(r,p,y):
  """
  Convert euler angles (roll, pitch, yaw) into a quaternion
  """
  ai = r / 2.0
  aj = p / 2.0
  ak = y / 2.0
  ci = math.cos(ai)
  si = math.sin(ai)
  cj = math.cos(aj)
  sj = math.sin(aj)
  ck = math.cos(ak)
  sk = math.sin(ak)
  cc = ci*ck
  cs = ci*sk
  sc = si*ck
  ss = si*sk

  q = [0.,0.,0.,0.]
  q[0] = cj*sc - sj*cs
  q[1] = cj*ss + sj*cc
  q[2] = cj*cs - sj*sc
  q[3] = cj*cc + sj*ss

  return q
