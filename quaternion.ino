#ifndef ARDUINO
#include <iostream>
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

Quaternion::Quaternion(Vector const &ihat, Vector const &jhat, Vector const &khat)
{
	// w = sqrt(1.0 + ihat.x + jhat.y + khat.z) / 2.0;
	// float const w4 = (4.0 * w);
	// x = (jhat.z - khat.y) / w4 ;
	// y = (khat.x - ihat.z) / w4 ;
	// z = (ihat.y - jhat.x) / w4 ;

  float const tr = ihat.x + jhat.y + khat.z;

  if (tr > 0) { 
    std::cout << "Case0";
    float const S = sqrt(tr+1.0) * 2; // S=4*qw 
    w = 0.25 * S;
    x = (jhat.z - khat.y) / S;
    y = (khat.x - ihat.z) / S; 
    z = (ihat.y - jhat.x) / S; 
  } else if ((ihat.x > jhat.y)&(ihat.x > khat.z)) { 
    std::cout << "Case1";
    float const S = sqrt(1.0 + ihat.x - jhat.y - khat.z) * 2; // S=4*qx 
    w = (jhat.z - khat.y) / S;
    x = 0.25 * S;
    y = (jhat.x + ihat.y) / S; 
    z = (khat.x + ihat.z) / S; 
  } else if (jhat.y > khat.z) { 
    std::cout << "Case2";
    float const S = sqrt(1.0 + jhat.y - ihat.x - khat.z) * 2; // S=4*qy
    w = (khat.x - ihat.z) / S;
    x = (jhat.x + ihat.y) / S; 
    y = 0.25 * S;
    z = (khat.y + jhat.z) / S; 
  } else { 
    std::cout << "Case3";
    float const S = sqrt(1.0 + khat.z - ihat.x - jhat.y) * 2; // S=4*qz
    w = (ihat.y - jhat.x) / S;
    x = (khat.x + ihat.z) / S;
    y = (khat.y + jhat.z) / S;
    z = 0.25 * S;
  }
  std::cout << ": tr=" << tr << " " <<
    ihat << " " << jhat << " " << khat<< std::endl;
}

// Quaternion::Quaternion(float yaw, float pitch, float roll)
// {
//     float cy = cos(yaw * 0.5);
//     float sy = sin(yaw * 0.5);
//     float cp = cos(pitch * 0.5);
//     float sp = sin(pitch * 0.5);
//     float cr = cos(roll * 0.5);
//     float sr = sin(roll * 0.5);

//     w = cy * cp * cr + sy * sp * sr;
//     x = cy * cp * sr - sy * sp * cr;
//     y = sy * cp * sr + cy * sp * cr;
//     z = sy * cp * cr - cy * sp * sr;
// }

Quaternion::Quaternion(float yaw, float pitch, float roll)
{
    float cy = cos(-yaw * 0.5);
    float sy = sin(-yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    w = cy * cp * cr + sy * sp * sr;
    y = cy * cp * sr - sy * sp * cr;
    x = sy * cp * sr + cy * sp * cr;
    z = -(sy * cp * cr - cy * sp * sr);
}

bool Quaternion::operator==(Quaternion const &other) const {
  float const TOLERANCE = 0.000001;
  
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
