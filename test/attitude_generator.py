from math import pi, sin, cos, sqrt

class Vector:
  x = 0
  y = 0
  z = 0

  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

  def __str__(self):
    return "(%0.2f, %0.2f, %0.2f)" % (self.x, self.y, self.z)

  def mag(self):
    return sqrt(self.x*self.x + self.y*self.y + self.z*self.z)

# v is the vector to be rotated
# a = pitch (x-axis), b = roll (y-axis), c = yaw (z-axis)
# All angles in radians.
def rotate(v, a, b, c):
  # Heading is the opposite direction of right-hand rule.
  c = -c

  c1 = cos(a)
  c2 = cos(b)
  c3 = cos(c)
  s1 = sin(a)
  s2 = sin(b)
  s3 = sin(c)

  # print("c1=%0.2f c2=%0.2f c3=%0.2f s1=%0.2f s2=%0.2f s3=%0.2f"
  #   % (c1, c2, c3, s1, s2, s3))

  # R_zxy
  x = v.x * ( c2*c3 - s1*s2*s3) + v.y * (-c1*s3) + v.z * (s2*c3 + s1*c2*s3)
  y = v.x * ( c2*s3 + s1*s2*c3) + v.y * ( c1*c3) + v.z * (s2*s3 - s1*c2*c3)
  z = v.x * (-c1*s2           ) + v.y * ( s1   ) + v.z * (c1*c2           )

  # # R_xz
  # x = v.x * (c3)    - v.y * (s3)
  # y = v.x * (c1*s3) + v.y * (c1*c3) - v.z * (s1)
  # z = v.x * (s1*s3) + v.y * (s1*c3) + v.z * (c1)

  # # R_zx
  # x = v.x * (c3) - v.y * (c1*s3) + v.z * (s1*s3)
  # y = v.x * (s3) + v.y * (c1*c3) - v.z * (s1*c3)
  # z = v.x * ( 0) + v.y * (s1)    + v.z * (c1)

  return Vector(x, y, z)

MAG_INCLINIATION = -60 * pi/180

north_unit_vector = Vector(0, cos(MAG_INCLINIATION), sin(MAG_INCLINIATION))
print("magnetic vector in fixed coordinates: %s" % str(north_unit_vector))

YAW = 30 * pi/180
PITCH = 60 * pi/180
ROLL = 00 * pi/180

print("YAW=%.0f PITCH=%.0f ROLL=%.0f" % (YAW*180/pi, PITCH*180/pi, ROLL*180/pi))

rotated_vector = rotate(north_unit_vector, PITCH, ROLL, YAW)
print("magnetic vector in body coordinates: %s  mag: %0.2f" % (str(rotated_vector), rotated_vector.mag()))
