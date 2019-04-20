#ifndef ARDUINO
#include <math.h>

#include "common.h"
#endif

Quaternion::Quaternion() : w(0), x(0), y(0), z(0) {}

Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion::Quaternion(Vector const &v) :
  w(0), x(v.x), y(v.y), z(v.z) {}

Quaternion::Quaternion(float angle, Vector const &v)
{
  Vector scaledV = v.scale(sin(angle));
  w = cos(angle);
  x = scaledV.x;
  y = scaledV.y;
  z = scaledV.z;
}

bool Quaternion::operator==(Quaternion const &other) const {
  float const TOLERANCE = 0.01;
  
  return
    fabs(w - other.w) < TOLERANCE &&
    fabs(x - other.x) < TOLERANCE &&
    fabs(y - other.y) < TOLERANCE &&
    fabs(z - other.z) < TOLERANCE;
}

/*
 * Calculate the conjugate of this quaternion.
 */
Quaternion Quaternion::conjugate() const
{
  return Quaternion(w, -x, -y, -z);
}

/*
 * Calculate the product of this quaternion and another (this x other).
 */
Quaternion Quaternion::multiply(Quaternion const &other) const
{
  return Quaternion(
    (w * other.w) - (x * other.x) - (y * other.y) - (z * other.z),
    (x * other.w) + (w * other.x) - (z * other.y) + (y * other.z),
    (y * other.w) + (z * other.x) + (w * other.y) - (x * other.z),
    (z * other.w) - (y * other.x) + (x * other.y) + (w * other.z)
  );
}

float Quaternion::magnitude() const
{
  return sqrt((w * w) + (x * x) + (y * y) + (z * z));
}

Quaternion Quaternion::normalize() const
{
  float const m = magnitude();
  Quaternion q_out = *this;

  if (m != 0.0) {
    q_out.w /= m;
    q_out.x /= m;
    q_out.y /= m;
    q_out.z /= m;
  }

  return q_out;
}

Vector Quaternion::vector() const
{
  return Vector(x, y, z);
}

void Quaternion::print(int digits) const
{
#ifdef ARDUINO
  Serial.print(w, digits);
  Serial.print(",");
  Serial.print(x, digits);
  Serial.print(",");
  Serial.print(y, digits);
  Serial.print(",");
  Serial.print(z, digits);
#endif
}

void Quaternion::print() const
{
  print(2);
}

#ifndef ARDUINO
std::ostream& operator<<(std::ostream& os, Quaternion const &q)
{
  return os << "(" << q.w << ", " << q.x << "i, "  << q.y << "j, " << q.z << "k)";  
}
#endif
