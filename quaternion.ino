#ifndef ARDUINO
#include <iostream>
#include <math.h>

#include "common.h"
#endif

Quaternion::Quaternion() : w(0), x(0), y(0), z(0) {}

Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion::Quaternion(Vector const &v) :
  w(0), x(v.x), y(v.y), z(v.z) {}

float const Quaternion::GIMBAL_LOCK_TOL = 0.1 * M_PI/180.0;

// Create a quaternion based on axis-angle values.
Quaternion::Quaternion(float angle, Vector const &v)
{
  Vector scaledV = v.normalize().scale(sin(angle));
  w = cos(angle);
  x = scaledV.x;
  y = scaledV.y;
  z = scaledV.z;
}

Quaternion::Quaternion(Vector const &ihat, Vector const &jhat, Vector const &khat)
{
  float const tr = ihat.x + jhat.y + khat.z;

  if (tr > 0) { 
    float const S = sqrt(tr+1.0) * 2; // S=4*qw 
    w = 0.25 * S;
    x = (khat.y - jhat.z) / S;
    y = (ihat.z - khat.x) / S; 
    z = (jhat.x - ihat.y) / S; 
  } else if ((ihat.x > jhat.y)&(ihat.x > khat.z)) { 
    float const S = sqrt(1.0 + ihat.x - jhat.y - khat.z) * 2; // S=4*qx 
    w = (khat.y - jhat.z) / S;
    x = 0.25 * S;
    y = (ihat.y + jhat.x) / S; 
    z = (ihat.z + khat.x) / S; 
  } else if (jhat.y > khat.z) { 
    float const S = sqrt(1.0 + jhat.y - ihat.x - khat.z) * 2; // S=4*qy
    w = (ihat.z - khat.x) / S;
    x = (ihat.y + jhat.x) / S; 
    y = 0.25 * S;
    z = (jhat.z + khat.y) / S; 
  } else { 
    float const S = sqrt(1.0 + khat.z - ihat.x - jhat.y) * 2; // S=4*qz
    w = (jhat.x - ihat.y) / S;
    x = (ihat.z + khat.x) / S;
    y = (jhat.z + khat.y) / S;
    z = 0.25 * S;
  }
}

Quaternion::Quaternion(float yaw, float pitch, float roll)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    w = cy * cp * cr   +   sy * sp * sr;
    x = cy * cp * sr   -   sy * sp * cr;
    y = sy * cp * sr   +   cy * sp * cr;
    z = sy * cp * cr   -   cy * sp * sr;
}

bool Quaternion::operator==(Quaternion const &other) const {
  float const TOLERANCE = 0.000001;
  
  return
    fabs(w - other.w) < TOLERANCE &&
    fabs(x - other.x) < TOLERANCE &&
    fabs(y - other.y) < TOLERANCE &&
    fabs(z - other.z) < TOLERANCE;
}

bool Quaternion::equivalent(Quaternion const &other) const {
  return *this == other || negative() == other;
}

/*
 * Calculate the conjugate of this quaternion.
 */
Quaternion Quaternion::negative() const
{
  return Quaternion(-w, -x, -y, -z);
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

float Quaternion::yaw() const
{
  // TODO We can make this decision without invoking asin.
  float const p = pitch();

  if (fabs(p + M_PI/2) < GIMBAL_LOCK_TOL) {
    // Gimbal lock, down.
    return 2 * atan2(x,w);
  }
  
  if (fabs(p - M_PI/2) < GIMBAL_LOCK_TOL) {
    // Gimbal lock, up.
    return -2 * atan2(x,w);
  }

  float const siny_cosp = 2.0 * (w*z + x*y);
	float const cosy_cosp = 1.0 - 2.0 * (y*y + z*z);

  return atan2(siny_cosp, cosy_cosp);
}

float Quaternion::pitch() const
{
  float const sinp = 2 * (w*y - z*x);
  if (fabs(sinp) >= 1) {
    // Use 90 degrees if out of range.
    return copysign(M_PI / 2, sinp);
  }

  return asin(sinp);
}

float Quaternion::roll() const
{
  // TODO We can make this decision without invoking asin.
  float const p = pitch();

  if (fabs(p + M_PI/2) < GIMBAL_LOCK_TOL) {
    // Gimbal lock, down.
    return 0;
  }
  
  if (fabs(p - M_PI/2) < GIMBAL_LOCK_TOL) {
    // Gimbal lock, up.
    return 0;
  }

  float const sinr_cosp = 2.0 * (w*x + y*z);
  float const cosr_cosp = 1.0 - 2.0 * (x*x + y*y);

  return atan2(sinr_cosp, cosr_cosp);
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
  return os << "(" << q.x << "i, "  << q.y << "j, " << q.z << "k, " << q.w << ")";  
}
#endif
