#ifndef ARDUINO
#include <math.h>
#include "common.h"
#endif

quaternion_t::quaternion_t() : w(0), x(0), y(0), z(0) {}

quaternion_t::quaternion_t(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

quaternion_t::quaternion_t(Vector const &v) :
  w(0), x(v.x), y(v.y), z(v.z) {}

quaternion_t::quaternion_t(float angle, Vector const &v)
{
  Vector scaledV = v.scale(sin(angle));
  w = cos(angle);
  x = scaledV.x;
  y = scaledV.y;
  z = scaledV.z;
}

/*
 * Calculate the conjugate of this quaternion.
 */
quaternion_t quaternion_t::conjugate() const
{
  return quaternion_t(w, -x, -y, -z);
}

/*
 * Calculate the product of this quaternion and another (this x other).
 */
quaternion_t quaternion_t::multiply(quaternion_t const &other) const
{
  return quaternion_t(
    (w * other.w) - (x * other.x) - (y * other.y) - (z * other.z),
    (x * other.w) + (w * other.x) - (z * other.y) + (y * other.z),
    (y * other.w) + (z * other.x) + (w * other.y) - (x * other.z),
    (z * other.w) - (y * other.x) + (x * other.y) + (w * other.z)
  );
}

float quaternion_t::magnitude() const
{
  return sqrt((w * w) + (x * x) + (y * y) + (z * z));
}

quaternion_t quaternion_t::normalize() const
{
  float const m = magnitude();
  quaternion_t q_out = *this;

  if (m != 0.0) {
    q_out.w /= m;
    q_out.x /= m;
    q_out.y /= m;
    q_out.z /= m;
  }

  return q_out;
}

Vector quaternion_t::vector() const
{
  return Vector(x, y, z);
}

void quaternion_t::print(int digits) const
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

void quaternion_t::print() const
{
  print(2);
}
